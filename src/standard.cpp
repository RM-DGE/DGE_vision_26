#include <fmt/core.h>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#ifdef ENABLE_TALOS_MODE
#include "io/talos/talos.hpp"
#endif
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
#ifdef ENABLE_FOXGLOVE
#include "tools/foxglove_publisher.hpp"
#endif

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

#ifdef ENABLE_FOXGLOVE
  std::unique_ptr<tools::FoxglovePublisher> foxglove;
  try {
    foxglove = std::make_unique<tools::FoxglovePublisher>(8765, "sp25_learn", config_path);
  } catch (const std::exception & e) {
    std::cerr << "[FOXGLOVE] disabled: " << e.what() << std::endl;
  }
#endif

#ifdef ENABLE_TALOS_MODE
  io::Talos talos;
#else
  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);
#endif

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  // Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto last_time = std::chrono::steady_clock::now();
  double fps = 0.0;


  // ===================== 主循环：单帧自瞄闭环 =====================
   while (!exiter.exit()) {
#ifdef ENABLE_TALOS_MODE
     if (!talos.read_image(img, t)) img.release();
#else
     camera.read(img, t);
#endif
      if (img.empty()) {
         std::this_thread::sleep_for(10ms);
         continue;
     }

     // 调试：打印图像尺寸和帧数（每 30 帧打一次）
     static int frame_count = 0;
     if (++frame_count % 30 == 0) {
       std::cerr << "[DEBUG] Image: " << img.size() << " channels: " << img.channels() 
                 << " frames: " << frame_count << std::endl;
     }

     // --------------------- 获取 IMU 姿态并更新解算基准 ---------------------
     // 这里读取当前时刻的四元数姿态，并稍微回溯 1ms 来补偿延迟，避免控制时序偏差。
#ifdef ENABLE_TALOS_MODE
      Eigen::Quaterniond q = talos.get_gimbal_quaternion(t - 1ms);
#else
     Eigen::Quaterniond q = gimbal.q(t - 1ms);
#endif

     solver.set_R_gimbal2world(q); 

     // --------------------- 检测、跟踪与瞄准计算 ---------------------
      auto armors = detector.detect(img); //检测
      auto targets = tracker.track(armors, t);  //预测和滤波
#ifdef ENABLE_TALOS_MODE
      io::GimbalState gimbal_state{.bullet_speed = 25.0};
#else
     auto gimbal_state = gimbal.state();
#endif
      auto command = aimer.aim(targets, t, gimbal_state.bullet_speed);

     // --------------------- 3.4 射击判定与下发控制 ---------------------
     // 通过当前云台欧拉角和目标瞄准结果决定是否发射，并将控制指令发送到底层控制器。
     Eigen::Vector3d current_ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
     command.shoot = shooter.shoot(command, aimer, targets, current_ypr);

#ifdef ENABLE_TALOS_MODE
     std::cerr << fmt::format(
       "[TALOS-DBG] frame={} control={} shoot={} yaw={:.4f} pitch={:.4f} "
       "aim_valid={} targets={} tracker_state={}\n",
       frame_count, command.control, command.shoot, command.yaw, command.pitch,
       aimer.debug_aim_point.valid, targets.size(), tracker.state());
#endif

     // 发送控制命令给Gimbal（包含前馈速度和加速度）
#ifdef ENABLE_TALOS_MODE
      const auto aim_point = aimer.debug_aim_point;
      const auto aim_delta = aim_point.xyza.head<3>();
      const auto target_distance = aim_point.valid ? aim_delta.norm() : 0.0;
      talos.send_gimbal_command(command.control, command.shoot, command.yaw,
                                command.yaw_vel, command.yaw_acc, command.pitch,
                                command.pitch_vel, command.pitch_acc,
                                static_cast<float>(target_distance));
#else
     gimbal.send(command.control, command.shoot, command.yaw, 
                 command.yaw_vel, command.yaw_acc, command.pitch, 
                 command.pitch_vel, command.pitch_acc);
#endif

    
    // ===================== 4. 可视化与调试显示 =====================
    for(const auto& armor : armors) {
        for(int i=0; i<4; i++) 
            cv::line(img, armor.points[i], armor.points[(i+1)%4], {0,255,0}, 2);
    }

    auto current_time = std::chrono::steady_clock::now();
    fps = 1.0 / std::chrono::duration<double>(current_time - last_time).count();
    last_time = current_time;

#ifdef ENABLE_FOXGLOVE
    if (foxglove) {
      foxglove->observe(talos);
      cv::putText(img, fmt::format("hit rate {:.0f}%", foxglove->total_hit_rate() * 100.0),
                  {10, 45}, cv::FONT_HERSHEY_SIMPLEX, 1.4, {0, 255, 255}, 3);

      int label_index = 0;
      for (auto & armor : armors) {
        solver.solve(armor);
        const std::string armor_name = auto_aim::ARMOR_NAMES[armor.name];
        const std::string label = fmt::format("{} {:.2f}", armor_name, armor.confidence);
        cv::putText(img, label, {10, 100 + label_index * 40}, cv::FONT_HERSHEY_SIMPLEX, 1.1,
                    {0, 255, 0}, 2);

        float box_bottom = armor.points.front().y;
        float box_left = armor.points.front().x;
        for (const auto & point : armor.points) {
          if (point.y > box_bottom) box_bottom = point.y;
          if (point.x < box_left) box_left = point.x;
        }
        const std::string rate_text =
          fmt::format("{:.0f}%", foxglove->armor_hit_rate(armor_name) * 100.0);
        cv::putText(img, rate_text,
                    cv::Point(static_cast<int>(box_left), static_cast<int>(box_bottom) + 34),
                    cv::FONT_HERSHEY_SIMPLEX, 1.1, {0, 255, 0}, 2);
        ++label_index;
      }

      const Eigen::Quaterniond gimbal_world_q(solver.R_gimbal2world());
      const Eigen::Vector3d gimbal_ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      foxglove->publish(
        img, command, fps, armors, targets, aimer.debug_aim_point.valid,
        aimer.debug_aim_point.xyza, gimbal_world_q, gimbal_ypr);
    }
#else

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
#endif

    // 保存每 300 帧一张截图用于调试
    static int save_count = 0;
    if (++save_count % 300 == 0) {
      std::string filename = fmt::format("/tmp/frame_{:06d}.jpg", save_count);
      cv::imwrite(filename, img);
      std::cerr << "[DEBUG] Saved screenshot to " << filename << std::endl;
    }

    
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
