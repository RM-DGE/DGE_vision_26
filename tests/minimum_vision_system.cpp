/**
 * @file minimum_vision_system.cpp
 * @brief 最小可用视觉系统：相机 + 多线程检测 + DM IMU + 跟踪/瞄准/开火
 *
 * 测试内容：
 *   1. 独立线程从 io::Camera 取图并推入 MultiThreadDetector；
 *   2. 主线程取出检测结果，结合 DM IMU 姿态完成坐标解算；
 *   3. 依次运行 Tracker、Aimer、Shooter，形成完整自瞄闭环；
 *   4. 在画面上重投影装甲板/瞄准点，并通过 Plotter 输出状态与卡方数据。
 *
 * 使用方法（先在 CMakeLists.txt 中取消 minimum_vision_system 相关注释并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/minimum_vision_system configs/standard.yaml
 *
 * 操作：画面窗口按 q 或 Ctrl+C 退出，退出时会等待检测线程结束。
 */

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/dm_imu/dm_imu.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>("@config-path");

  tools::Exiter exiter;
  tools::Plotter plotter;
  io::Camera camera(config_path);
  io::DM_IMU dm_imu;

  // 初始化自瞄各模块，与正式比赛程序保持一致
  auto_aim::multithread::MultiThreadDetector detector(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  // 采集 + 检测线程：持续取图并送入多线程检测器
  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!exiter.exit()) {
      camera.read(img, t);
      detector.push(img, t);
    }
  });

  auto last_t = std::chrono::steady_clock::now();
  nlohmann::json data;

  while (!exiter.exit()) {
    // 阻塞等待一帧检测结果（图像、装甲板、时间戳）
    auto [img, armors, t] = detector.debug_pop();

    // 用该帧时间戳查询 IMU，保证姿态与图像同步
    Eigen::Quaterniond q = dm_imu.imu_at(t);

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    // 跟踪 -> 瞄准 -> 开火判定
    auto targets = tracker.track(armors, t);

    auto command = aimer.aim(targets, t, 22);

    shooter.shoot(command, aimer, targets, gimbal_pos);

    // 统计端到端帧率
    auto dt = tools::delta_time(t, last_t);
    last_t = t;

    data["dt"] = dt;
    data["fps"] = 1 / dt;
    plotter.plot(data);
    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      auto min_x = 1e10;
      auto & armor = armors.front();
      // 取最左侧装甲板做单板解算，便于和跟踪结果对照
      for (auto & a : armors) {
        if (a.center.x < min_x) {
          min_x = a.center.x;
          armor = a;
        }
      }  //always left
      solver.solve(armor);
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
    }

    if (!targets.empty()) {
      auto target = targets.front();
      // 左上角显示跟踪器状态（idle/tracking/temp_lost 等）
      tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});

      // 当前帧target更新后：重投影四块装甲板为绿色
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // aimer瞄准位置：红色表示有效瞄准点，蓝色表示无效/预测点
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255});  // red
      else
        tools::draw_points(img, image_points, {255, 0, 0});  // blue

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
      data["distance"] = std::sqrt(x[0] * x[0] + x[2] * x[2] + x[4] * x[4]);

      // 卡方检验数据：判断滤波器是否因异常观测而发散
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
    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  detect_thread.join();  // 等待采集/检测线程退出

  return 0;
}