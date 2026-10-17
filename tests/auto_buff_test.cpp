/**
 * @file auto_buff_test.cpp
 * @brief 离线视频回放的打符（能量机关）全流程测试
 *
 * 测试内容：
 *   1. 逐帧读取 avi 视频和同名 txt 中的云台姿态；
 *   2. 运行能量机关检测、位姿解算、目标预测与瞄准，输出云台指令；
 *   3. 在图像上绘制检测点、当前 buff 位置（绿色）和预测瞄准位置（蓝色）；
 *   4. 通过 Plotter 输出 buff 观测值、滤波器状态和云台响应曲线。
 *
 * 输入文件（@input-path 为不含扩展名的路径）：
 *   <input-path>.avi  测试视频
 *   <input-path>.txt  每行 "t w x y z"：时间戳 + 云台四元数
 *
 * 使用方法（先在 CMakeLists.txt 中取消 auto_buff_test 相关注释并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/auto_buff_test assets/buff/buff -c configs/sentry.yaml
 *   参数：-c 配置文件  -s 起始帧  -e 结束帧（0 表示到结尾）
 *
 * 操作：图像窗口按 q 退出；按空格暂停，暂停时再按 q 退出。
 */

#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{config-path c  | configs/sentry.yaml    | yaml配置文件的路径}"
  "{start-index s  | 0                      | 视频起始帧下标    }"
  "{end-index e    | 0                      | 视频结束帧下标    }"
  "{@input-path    |                        | avi和txt文件的路径}";

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
  tools::Exiter exiter;

  // 视频与姿态文件同名，扩展名分别为 .avi / .txt
  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  auto_buff::Buff_Detector detector(config_path);  // 传统视觉检测能量机关
  auto_buff::Solver solver(config_path);           // 像素/世界坐标解算
  // auto_buff::SmallTarget target;                // 小符目标模型
  auto_buff::BigTarget target;                     // 大符目标模型（含旋转预测）
  auto_buff::Aimer aimer(config_path);             // 弹道解算与瞄准

  cv::Mat img, drawing;
  auto t0 = std::chrono::steady_clock::now();

  io::Command last_command;
  double last_t = -1;

  // 跳到起始帧，保持视频和姿态数据同步
  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    video.read(img);
    if (img.empty()) break;

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    /// 自瞄核心逻辑

    solver.set_R_gimbal2world({w, x, y, z});  // 更新云台姿态

    auto power_runes = detector.detect(img);  // 检测能量机关扇叶

    solver.solve(power_runes);                // 解算到世界坐标系

    target.get_target(power_runes, timestamp);  // 更新目标模型/预测状态

    auto target_copy = target;  // 保存当前状态，供瞄准预测使用

    auto command = aimer.aim(target_copy, timestamp, 22, false);  // 22 为弹速 m/s

    // cboard.send(command);

    // -------------- 调试输出 --------------

    nlohmann::json data;

    // data["bullet_speed"] = cboard.bullet_speed;

    // buff原始观测数据
    if (power_runes.has_value()) {
      const auto & p = power_runes.value();
      data["buff_R_yaw"] = p.ypd_in_world[0];
      data["buff_R_pitch"] = p.ypd_in_world[1];
      data["buff_R_dis"] = p.ypd_in_world[2];
      data["buff_yaw"] = p.ypr_in_world[0] * 57.3;
      data["buff_pitch"] = p.ypr_in_world[1] * 57.3;
      data["buff_roll"] = p.ypr_in_world[2] * 57.3;
    }

    if (!target.is_unsolve()) {
      auto & p = power_runes.value();

      // 显示：扇叶四个角点、检测中心、能量机关旋转中心
      for (int i = 0; i < 4; i++) tools::draw_point(img, p.target().points[i]);
      tools::draw_point(img, p.target().center, {0, 0, 255}, 3);
      tools::draw_point(img, p.r_center, {0, 0, 255}, 3);

      // 当前帧target更新后buff，重投影为绿色
      auto Rxyz_in_world_now = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
      auto image_points =
        solver.reproject_buff(Rxyz_in_world_now, target.ekf_x()[4], target.ekf_x()[5]);
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4), {0, 255, 0});
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin() + 4, image_points.end()), {0, 255, 0});

      // buff瞄准位置（提前预测），重投影为蓝色
      double dangle = target.ekf_x()[5] - target_copy.ekf_x()[5];
      auto Rxyz_in_world_pre = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
      image_points =
        solver.reproject_buff(Rxyz_in_world_pre, target_copy.ekf_x()[4], target_copy.ekf_x()[5]);
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4), {255, 0, 0});
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin() + 4, image_points.end()), {255, 0, 0});

      // 观测器内部数据：旋转中心、角速度、当前角度/角速度等
      Eigen::VectorXd x = target.ekf_x();
      data["R_yaw"] = x[0];
      data["R_V_yaw"] = x[1];
      data["R_pitch"] = x[2];
      data["R_dis"] = x[3];
      data["yaw"] = x[4] * 57.3;

      data["angle"] = x[5] * 57.3;
      data["spd"] = x[6] * 57.3;
      if (x.size() >= 10) {
        data["spd"] = x[6];
        data["a"] = x[7];
        data["w"] = x[8];
        data["fi"] = x[9];
        data["spd0"] = target.spd;
      }
    }

    // 云台响应情况：把当前姿态转换成 yaw/pitch 角度
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = -ypr[1] * 57.3;

    // 只有瞄准器给出有效控制指令时才记录期望角度
    if (command.control) {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
    }

    plotter.plot(data);

    cv::imshow("result", img);

    int key = cv::waitKey(1);
    if (key == 'q') break;
    while (key == ' ') {  // 空格暂停，暂停期间按 q 可退出
      int y = cv::waitKey(30);
      if (y == 'q') break;
    }
  }
  cv::destroyAllWindows();
  text.close();  // 关闭文件

  return 0;
}