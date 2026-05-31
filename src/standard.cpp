#include <fmt/core.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/plotter.hpp"
#include <nlohmann/json.hpp>
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

//ros2 run plotjuggler plotjuggler
const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::CBoard cboard(config_path); 
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  //当前空闲
  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  auto last_time = std::chrono::steady_clock::now();
  double fps = 0.0;

  while (!exiter.exit()) 
  {
    // 1. 读取图像
    camera.read(img, t);
    if (img.empty()) {
        std::this_thread::sleep_for(10ms);
        continue;
    }
    cboard.mode = io::Mode::auto_aim;//强制自瞄
    cboard.bullet_speed = 24.5; // 强制弹速

    Eigen::Quaterniond q = cboard.imu_at(t - 1ms);

    solver.set_R_gimbal2world(q);

    // 3. 识别与解算
    auto armors = detector.detect(img);
    auto targets = tracker.track(armors, t);
    auto command = aimer.aim(targets, t, cboard.bullet_speed);

    // 射击逻辑
    Eigen::Vector3d current_ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    command.shoot = shooter.shoot(command, aimer, targets, current_ypr);

    // 4. 发送数据
    cboard.send(command);

    
    //可视化
    for(const auto& armor : armors) {
        for(int i=0; i<4; i++) 
            cv::line(img, armor.points[i], armor.points[(i+1)%4], {0,255,0}, 2);
    }

    // //红框
    // if (!targets.empty() && aimer.debug_aim_point.valid) {
    //     auto target = targets.front();
    //     Eigen::Vector4d aim_xyza = aimer.debug_aim_point.xyza;

    //     // 调用 solver 将世界坐标系下的 3D 预测点和 Yaw 角，重投影回 2D 像素坐标
    //     auto image_points = solver.reproject_armor(
    //         aim_xyza.head(3), 
    //         aim_xyza[3], 
    //         target.armor_type, 
    //         target.name
    //     );

    //     // 使用红色 (0, 0, 255) 连线绘制预测框
    //     for(int i = 0; i < 4; i++) {
    //         cv::line(img, image_points[i], image_points[(i+1)%4], {0, 0, 255}, 2);
    //     }
    // }

    cv::resize(img, img, {}, 0.5, 0.5);
    // --- 绘制文字 ---
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.7; // 字体大小
    int thickness = 1;  // 字体粗细
    
    // 1. 第一行：FPS (青色)
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = current_time - last_time;
    fps = 1.0 / diff.count();
    last_time = current_time;
    std::string text_fps = fmt::format("FPS: {:.1f}", fps);
    cv::putText(img, text_fps, cv::Point(10, 20), font, scale, cv::Scalar(255, 255, 255), thickness);
    // 2. 第二行：Yaw 和 Pitch (绿色)
    std::string text_angles = fmt::format("Yaw: {:.2f}  Pitch: {:.2f}", command.yaw, command.pitch);
    cv::putText(img, text_angles, cv::Point(10, 40), font, scale, cv::Scalar(0, 255, 0), thickness);
    // 3. 第三行：Shoot 0/1 (红色)
    // Shoot 状态通过三目运算符转换为 1 或 0
    std::string text_shoot = fmt::format("Shoot: {}", command.shoot ? 1 : 0);
    cv::putText(img, text_shoot, cv::Point(10, 60), font, scale, cv::Scalar(0, 0, 255), thickness);

    cv::imshow("Standard Test", img);
    if (cv::waitKey(1) == 'q') break;

    
    // //曲线可视化
    // nlohmann::json debug_data;
    // // 1. 记录变量
    // if (!targets.empty()) 
    // {
    //     auto target = targets.front();
    //     debug_data["kf_y"] = target.ekf_x()[2]; // EKF预测的X
    //     debug_data["kf_vy"] = target.ekf_x()[3]; // EKF预测的VX
    // }
    //   // 2. 记录控制指令
    // debug_data["cmd_yaw"] = command.yaw;
    // // debug_data["cmd_pitch"] = command.pitch;
    //   // 3. 发送！
    // plotter.plot(debug_data);
  }

return 0;
}