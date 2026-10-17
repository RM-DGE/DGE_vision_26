/**
 * @file auto_aim_test.cpp
 * @brief 离线视频回放的自瞄全流程测试（YOLO 检测 -> EKF 跟踪 -> 弹道瞄准 -> 可视化）
 *
 * 测试内容：
 *   1. 按帧读取 avi 视频及其同名 txt 中的云台姿态，完整跑一遍自瞄链路；
 *   2. 在图像上重投影装甲板和瞄准点，并把瞄准/观测器数据发送到 Plotter 绘图；
 *   3. 打印 yolo、tracker、aimer 各阶段耗时，便于性能分析。
 *
 * 输入文件（通过 @input-path 指定前缀，不含扩展名）：
 *   <input-path>.avi  测试视频
 *   <input-path>.txt  每行 "t w x y z"：视频时间戳 + 云台 IMU 四元数(w,x,y,z)
 *
 * 使用方法（先在 CMakeLists.txt 中取消 auto_aim_test 相关注释并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/auto_aim_test assets/demo/demo -c configs/demo.yaml -s 0 -e 500
 *   参数：-c 配置文件  -s 起始帧  -e 结束帧（0 表示播放到结尾）
 *
 * 操作：图像窗口按 q 或 Ctrl+C 退出。
 */

#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | configs/demo.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                 | 视频起始帧下标    }"
  "{end-index e    | 0                 | 视频结束帧下标    }"
  "{@input-path    | assets/demo/demo  | avi和txt文件的路径}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Plotter plotter;
  tools::Exiter exiter;  // 监听 Ctrl+C，用于安全退出主循环

  // 视频与姿态数据同名，仅扩展名不同
  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  // 依次构造自瞄各模块，均从同一份 yaml 配置读取参数
  auto_aim::YOLO yolo(config_path);              // 神经网络检测装甲板
  auto_aim::Solver solver(config_path);          // 世界系/相机系解算与重投影
  auto_aim::Tracker tracker(config_path, solver);  // 目标跟踪与 EKF 状态估计
  auto_aim::Aimer aimer(config_path);            // 弹道解算并生成云台指令

  cv::Mat img, drawing;
  auto t0 = std::chrono::steady_clock::now();

  auto_aim::Target last_target;
  io::Command last_command;
  double last_t = -1;

  // 视频和 txt 都跳到起始帧，保证图像与姿态数据对齐
  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;  // 到达结束帧则停止

    video.read(img);
    if (img.empty()) break;  // 视频读完

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    // txt 中是相对视频开头的秒数，这里换算成 steady_clock 时间点
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    /// 自瞄核心逻辑

    // 更新云台到世界系的旋转（四元数顺序 w,x,y,z）
    solver.set_R_gimbal2world({w, x, y, z});

    // 1) 检测装甲板；2) 跟踪并估计目标状态；3) 解算云台指令
    auto yolo_start = std::chrono::steady_clock::now();
    auto armors = yolo.detect(img, frame_count);

    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, timestamp);

    auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, timestamp, 27, false);  // 27 为弹速 m/s

    // 简化版开火判断：有有效瞄准点且瞄准角变化小于 2 度时开火
    if (
      !targets.empty() && aimer.debug_aim_point.valid &&
      std::abs(command.yaw - last_command.yaw) * 57.3 < 2)
      command.shoot = true;

    if (command.control) last_command = command;
    /// 调试输出

    // 各阶段耗时（毫秒），用于评估整条链路的实时性
    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3,
      tools::delta_time(finish, aimer_start) * 1e3);

    tools::draw_text(
      img,
      fmt::format(
        "command is {},{:.2f},{:.2f},shoot:{}", command.control, command.yaw * 57.3,
        command.pitch * 57.3, command.shoot),
      {10, 60}, {154, 50, 205});

    // 当前云台 yaw（度），与瞄准指令一起显示，方便观察跟随效果
    Eigen::Quaternion gimbal_q = {w, x, y, z};
    tools::draw_text(
      img,
      fmt::format(
        "gimbal yaw{:.2f}", (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]),
      {10, 90}, {255, 255, 255});

    nlohmann::json data;

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
      data["armor_center_x"] = armor.center_norm.x;
      data["armor_center_y"] = armor.center_norm.y;
    }

    Eigen::Quaternion q{w, x, y, z};
    auto yaw = tools::eulers(q, 2, 1, 0)[0];
    data["gimbal_yaw"] = yaw * 57.3;
    data["cmd_yaw"] = command.yaw * 57.3;
    data["shoot"] = command.shoot;

    if (!targets.empty()) {
      auto target = targets.front();

      // 第一帧只记录基准，避免用初始状态做无意义的重投影
      if (last_t == -1) {
        last_target = target;
        last_t = t;
        continue;
      }

      std::vector<Eigen::Vector4d> armor_xyza_list;

      // 当前帧target更新后的四块装甲板(x,y,z,yaw)，重投影为绿色点
      armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // aimer 选定的瞄准位置，重投影为红色点
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});

      // 观测器内部状态：位置、速度、yaw/角速度、半径、装甲板高度等
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;

      // 卡方检验数据：残差、NIS/NEES 及失败计数，用于判断滤波器是否发散
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
    }

    // 发送到 Plotter（如 Foxglove）进行曲线可视化
    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(30);  // 约 33 fps 的播放速度
    if (key == 'q') break;
  }

  return 0;
}