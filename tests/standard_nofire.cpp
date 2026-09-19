#include <fmt/core.h>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
// #include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

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
  // tools::Recorder recorder;

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  // Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;   //空闲
  auto last_mode = io::Mode::idle;

  auto last_time = std::chrono::steady_clock::now();
  double fps = 0.0;


  // ===================== 主循环：单帧自瞄闭环 =====================
  while (!exiter.exit()) {
    camera.read(img, t);  //读取当前帧图像与时间戳
    if (img.empty()) {
        std::this_thread::sleep_for(10ms);
        continue;
    }
    cboard.mode = io::Mode::auto_aim;//强制自瞄
    cboard.bullet_speed = 24.5; // 强制弹速

    // --------------------- 获取 IMU 姿态并更新解算基准 ---------------------
    // 这里读取当前时刻的四元数姿态，并稍微回溯 1ms 来补偿延迟，避免控制时序偏差。
    Eigen::Quaterniond q = cboard.imu_at(t - 1ms);

    solver.set_R_gimbal2world(q); 

    // --------------------- 检测、跟踪与瞄准计算 ---------------------
    auto armors = detector.detect(img); //检测
    auto targets = tracker.track(armors, t);  //预测和滤波
    auto command = aimer.aim(targets, t, cboard.bullet_speed);

    // --------------------- 3.4 射击判定与下发控制 ---------------------
    // 通过当前云台欧拉角和目标瞄准结果决定是否发射，并将控制指令发送到底层控制器。
    Eigen::Vector3d current_ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    command.shoot = false;

    cboard.send(command);

    
    // ===================== 4. 可视化与调试显示 =====================
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
