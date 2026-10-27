/**
 * @file sentry_DGE.cpp
 * @brief DGE 哨兵主程序：工业相机主自瞄 + 两路 USB 并行全向感知
 *
 * 结构参考 sp25 的 sentry_multithread.cpp：
 *   - 工业相机负责正常检测、跟踪、瞄准和开火；
 *   - 两路 USB 相机由 Perceptron 各开一个线程，持续检测并写入 DetectionResult 队列；
 *   - 主相机丢失目标时，用队列中的结果把云台转向目标；
 *   - 主相机跟踪到低优先级目标、而 USB 发现更高优先级目标时，Tracker 进入
 *     switching 状态，先把云台转过去，等主相机重新捕获后再切回 Aimer；
 *   - USB 全向感知阶段只发送 control，不发送 shoot。
 *
 * 相机角色：
 *   io::Camera      前向工业相机（主相机，随云台转动，0°）
 *   io::USBCamera   左 USB（device_name = "left"，YAML 配置，默认 +120°）
 *   io::USBCamera   右 USB（device_name = "right"，YAML 配置，默认 -120°）
 *
 * 使用方法：
 *   ./build/sentry_DGE configs/sentry.yaml
 *   ./build/sentry_DGE configs/sentry.yaml -d    # 打开可视化窗口
 *
 * 注意：
 *   - 配置中需要提供 camera_matrix、R_camera2gimbal、t_camera2gimbal、
 *     fov_h/fov_v、new_fov_h/new_fov_v、mode，以及工业相机/USB 相机参数；
 *   - 真机使用 io::CBoard（串口），仿真时在 yaml 中写 cboard_name: daedalus；
 *   - ROS2 通信暂时关闭：当前调试阶段先保证纯自瞄链路可运行。
 *     恢复时参考 io/ros2/mcu_ros_bridge.* 和 doc/RobotModel4_v0.40.md；
 *   - USB 设备名当前写死为 video2/video4（两路外接相机），不使用内置摄像头；
 *     按实际设备修改。
 */

#include <fmt/core.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/Daedalus/daedalus_cboard.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/visualizer.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tasks/omniperception/visualizer.hpp"
#include "tools/Foxglove/foxglove.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }"
  "{d display      |                     | 显示主相机和全向感知画面       }";


int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  if (config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  auto display = cli.has("display");

  tools::Exiter exiter;

  // 图像-IMU 时间补偿：真机需要按实际链路延迟调整，仿真默认 1 ms
  double imu_delay_ms = 1.0;
  std::string usb_left_device = "video2";
  std::string usb_right_device = "video4";
  {
    auto yaml = YAML::LoadFile(config_path);
    if (yaml["imu_delay_ms"]) imu_delay_ms = yaml["imu_delay_ms"].as<double>();
    if (yaml["usb_left_device"])
      usb_left_device = yaml["usb_left_device"].as<std::string>();
    if (yaml["usb_right_device"])
      usb_right_device = yaml["usb_right_device"].as<std::string>();
  }

  // 真机使用 CBoard（串口），仿真使用 DaedalusCBoard（共享内存）
  std::unique_ptr<io::CBoardBase> cboard_ptr;
  {
    auto yaml = YAML::LoadFile(config_path);
    auto cboard_name = yaml["cboard_name"] ? yaml["cboard_name"].as<std::string>() : "cboard";
    if (cboard_name == "daedalus")
      cboard_ptr = std::make_unique<io::DaedalusCBoard>(config_path);
    else
      cboard_ptr = std::make_unique<io::CBoard>(config_path);
  }
  auto & cboard = *cboard_ptr;

  // 工业相机：主自瞄，同时随云台转向全向相机发现的目标
  io::Camera camera(config_path);

  // 两路 USB：并行全向感知
  // video0/video1 是电脑内置摄像头，这里不使用；
  // 前向工业相机为 0°；两路外接 USB 相机分别放在左/右约 ±120°。
  // 用 /dev/v4l/by-path 的稳定路径，避免插拔/重启后 videoN 变化。
  io::USBCamera usbcam1(usb_left_device, config_path);
  io::USBCamera usbcam2(usb_right_device, config_path);

  // USBCamera 默认根据 sharpness 判断左右，这里显式指定，避免硬件差异
  usbcam1.device_name = "left";
  usbcam2.device_name = "right";

  // 自瞄各模块
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  // 全向感知：两路 USB 各一个线程、各一个 YOLO 实例
  omniperception::Decider decider(config_path);
  omniperception::Perceptron perceptron(&usbcam1, &usbcam2, config_path);

  // Foxglove：主相机图像、识别框、TF、3D 场景和调试曲线
  tools::FoxgloveServer foxglove(8765);
  foxglove.configure_camera(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_time = std::chrono::steady_clock::now();
  double fps = 0.0;

  while (!exiter.exit()) {
    // 1. 工业相机取图
    camera.read(img, timestamp);
    if (img.empty()) {
      std::this_thread::sleep_for(10ms);
      continue;
    }

    // 2. 取图像时刻的云台姿态
    Eigen::Quaterniond q = cboard.imu_at(
      timestamp - std::chrono::microseconds(static_cast<int>(imu_delay_ms * 1000)));
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    // 帧率统计，供 Foxglove 调试曲线使用
    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last_time);
    last_time = now;
    if (dt > 1e-6) fps = 1.0 / dt;

    /// 自瞄核心逻辑
    auto armors = yolo.detect(img);

    // ROS 通信暂时关闭：导航无敌目标/优先目标不再注入。
    // 过滤非敌方、5 号、前哨站和无敌目标，并写入优先级
    decider.armor_filter(armors);
    decider.set_priority(armors);

    // 3. 取出两路 USB 的并行检测结果，并按优先级排序
    auto detection_queue = perceptron.get_detection_queue();
    decider.sort(detection_queue);

    // 4. 主相机跟踪 + 全向目标切换
    auto [switch_target, targets] = tracker.track(detection_queue, armors, timestamp);

    io::Command command{false, false, 0, 0};

    /// 全向感知逻辑
    if (tracker.state() == "switching") {
      // 全向相机发现更高优先级目标：先把云台转过去，等主相机重新捕获
      command.control = switch_target.armors.empty() ? false : true;
      command.shoot = false;
      command.pitch = tools::limit_rad(switch_target.delta_pitch);
      command.yaw = tools::limit_rad(switch_target.delta_yaw + gimbal_pos[0]);
    }

    else if (tracker.state() == "lost") {
      // 主相机丢失目标：用两路 USB 的检测结果补盲
      command = decider.decide(detection_queue);
      command.yaw = tools::limit_rad(command.yaw + gimbal_pos[0]);
    }

    else {
      // 主相机正常跟踪：使用 Aimer 预测和弹道解算
      command = aimer.aim(targets, timestamp, cboard.bullet_speed);
    }

    /// 发射逻辑
    // switching/lost 阶段 targets 为空，Shooter 会返回 false，不会误开火
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    /// 下发云台/发射指令
    /// ROS 通信暂时关闭，云台指令直接写串口。
    cboard.send(command);

    /// Foxglove：图像、识别框、TF、3D 场景和调试曲线
    const bool aim_valid = aimer.debug_aim_point.valid;
    const Eigen::Vector4d aim_xyza =
      aim_valid ? aimer.debug_aim_point.xyza : Eigen::Vector4d::Zero();

    foxglove.publish_image(img);
    foxglove.publish_annotations(armors);
    foxglove.publish_number_image(armors);
    foxglove.publish_rmvision(armors, targets, aim_xyza);
    foxglove.publish_detector_markers(armors);
    foxglove.publish_tracker_markers(targets);
    foxglove.publish_tf(q);
    foxglove.publish_scene(armors, targets, aim_valid, aim_xyza, q);
    foxglove.publish_prediction(targets);
    foxglove.publish_ground_truth(camera.ground_truth());
    foxglove.publish_debug(fps, command, targets, armors, aim_valid, aim_xyza, gimbal_pos);

    /// 可视化
    if (display) {
      cv::Mat main_view = img.clone();
      auto_aim::draw_main_view(
        main_view, armors, targets, solver, aimer.debug_aim_point, tracker.state(), command,
        detection_queue.size());

      cv::Mat radar;
      omniperception::draw_omni_radar(radar, detection_queue, gimbal_pos[0], command, tracker.state());

      cv::resize(main_view, main_view, {}, 0.5, 0.5);
      cv::imshow("sentry_DGE", main_view);
      cv::imshow("omni", radar);

      // 两路 USB 原始画面
      auto usb_frames = perceptron.get_debug_frames();
      for (int i = 0; i < 2; ++i) {
        if (usb_frames[i].img.empty()) continue;
        cv::Mat usb_view;
        const auto title = (i == 0) ? "usb_left" : "usb_right";
        omniperception::draw_usb_view(usb_view, usb_frames[i], title);
        cv::imshow(title, usb_view);
      }

      if (cv::waitKey(1) == 'q') break;
    }
  }

  if (display) cv::destroyAllWindows();

  return 0;
}
