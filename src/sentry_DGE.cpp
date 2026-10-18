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
 *   io::Camera      工业相机（主相机，随云台转动）
 *   io::USBCamera   左 USB（device_name = "left"）
 *   io::USBCamera   右 USB（device_name = "right"）
 *
 * 使用方法：
 *   ./build/sentry_DGE configs/sentry.yaml
 *   ./build/sentry_DGE configs/sentry.yaml -d    # 打开可视化窗口
 *
 * 注意：
 *   - 配置中需要提供 camera_matrix、R_camera2gimbal、t_camera2gimbal、
 *     fov_h/fov_v、new_fov_h/new_fov_v、mode，以及工业相机/USB 相机参数；
 *   - 真机使用 io::CBoard（串口），仿真时在 yaml 中写 cboard_name: daedalus；
 *   - USB 设备名当前写死为 video0/video2，按实际设备修改。
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
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }"
  "{d display      |                     | 显示主相机和全向感知画面       }";

namespace
{
cv::Point polar_to_screen(const cv::Point & center, double yaw_deg, double radius)
{
  const auto a = yaw_deg * CV_PI / 180.0;
  // 屏幕上方为 0°，yaw 正方向向左
  return {
    static_cast<int>(center.x - radius * std::sin(a)),
    static_cast<int>(center.y - radius * std::cos(a))};
}

// 极坐标全向感知图：中心为云台，0° 朝前，正 yaw 向左
void draw_omni_radar(
  cv::Mat & radar, const std::vector<omniperception::DetectionResult> & detection_queue,
  double gimbal_yaw, const io::Command & command, const std::string & state)
{
  radar = cv::Mat(600, 600, CV_8UC3, cv::Scalar(30, 30, 30));
  const cv::Point center(300, 300);

  // 距离环和坐标轴
  for (double r : {80.0, 160.0, 240.0}) {
    cv::circle(radar, center, static_cast<int>(r), {80, 80, 80}, 1);
  }
  cv::line(radar, {40, 300}, {560, 300}, {60, 60, 60}, 1);
  cv::line(radar, {300, 40}, {300, 560}, {60, 60, 60}, 1);

  tools::draw_text(radar, "0", {292, 30}, {200, 200, 200});
  tools::draw_text(radar, "90", {530, 290}, {200, 200, 200});
  tools::draw_text(radar, "-90", {40, 290}, {200, 200, 200});
  tools::draw_text(radar, "180", {280, 575}, {200, 200, 200});

  // 云台当前朝向（0° 为车体/云台零位，图上的上方向）
  cv::arrowedLine(radar, center, {300, 60}, {0, 255, 0}, 2);

  // 当前云台指令方向（相对当前 yaw）
  if (command.control) {
    const auto cmd_yaw = (command.yaw - gimbal_yaw) * 57.3;
    cv::arrowedLine(radar, center, polar_to_screen(center, cmd_yaw, 240), {0, 0, 255}, 2);
  }

  // 两路 USB 的检测结果
  int index = 0;
  for (const auto & dr : detection_queue) {
    if (dr.armors.empty()) continue;

    const auto yaw_deg = dr.delta_yaw * 57.3;
    const auto point = polar_to_screen(center, yaw_deg, 180);

    const cv::Scalar color = (index++ % 2 == 0) ? cv::Scalar(255, 180, 0)
                                                : cv::Scalar(0, 180, 255);
    cv::circle(radar, point, 7, color, -1);
    tools::draw_text(
      radar, fmt::format("{:.0f}", yaw_deg), point + cv::Point(8, -8), color, 0.5, 1);
    tools::draw_text(
      radar, auto_aim::ARMOR_NAMES[dr.armors.front().name], point + cv::Point(8, 18), color, 0.5,
      1);
  }

  tools::draw_text(radar, fmt::format("state: {}", state), {10, 25}, {255, 255, 255}, 0.7, 1);
  tools::draw_text(
    radar, fmt::format("usb: {}", detection_queue.size()), {10, 50}, {255, 255, 255}, 0.7, 1);

  if (detection_queue.empty()) {
    tools::draw_text(radar, "no USB target", {220, 300}, {180, 180, 180}, 0.7, 1);
  }
}

// USB 相机画面：原始图像 + 本帧检测框 + 相对偏角
void draw_usb_view(
  cv::Mat & view, const omniperception::DebugFrame & frame, const std::string & title)
{
  if (frame.img.empty()) return;

  view = frame.img.clone();
  for (const auto & armor : frame.armors) {
    tools::draw_points(view, armor.points, {0, 255, 0});
    tools::draw_text(
      view, auto_aim::ARMOR_NAMES[armor.name],
      cv::Point(static_cast<int>(armor.center.x), static_cast<int>(armor.center.y)), {0, 255, 0},
      0.6, 1);
  }

  tools::draw_text(
    view, fmt::format("{}  yaw {:.1f}", title, frame.delta_yaw * 57.3), {10, 30},
    {0, 255, 255});
  cv::resize(view, view, {}, 0.5, 0.5);
}

// 主相机画面：装甲板、跟踪状态、瞄准点、云台指令
void draw_main_view(
  cv::Mat & canvas, const std::list<auto_aim::Armor> & armors,
  const std::list<auto_aim::Target> & targets, auto_aim::Solver & solver,
  const auto_aim::AimPoint & aim_point, const std::string & state, const io::Command & command,
  std::size_t usb_count)
{
  for (const auto & armor : armors) {
    tools::draw_points(canvas, armor.points, {0, 255, 0});
    tools::draw_text(
      canvas, auto_aim::ARMOR_NAMES[armor.name],
      cv::Point(static_cast<int>(armor.center.x), static_cast<int>(armor.center.y)), {0, 255, 0},
      0.6, 1);
  }

  if (!targets.empty()) {
    auto target = targets.front();
    for (const auto & xyza : target.armor_xyza_list()) {
      auto image_points =
        solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
      tools::draw_points(canvas, image_points, {255, 255, 0});
    }

    if (aim_point.valid) {
      auto aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(canvas, image_points, {0, 0, 255});
    }
  }

  tools::draw_text(canvas, fmt::format("[{}]", state), {10, 30}, {255, 255, 255});
  tools::draw_text(
    canvas,
    fmt::format(
      "cmd yaw {:.1f} pitch {:.1f} shoot {}", command.yaw * 57.3, command.pitch * 57.3,
      command.shoot),
    {10, 65}, {0, 255, 255});
  tools::draw_text(canvas, fmt::format("usb {}", usb_count), {10, 100}, {0, 255, 255});
}
}  // namespace

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
  io::USBCamera usbcam1("video0", config_path);
  io::USBCamera usbcam2("video2", config_path);

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

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  while (!exiter.exit()) {
    // 1. 工业相机取图
    camera.read(img, timestamp);
    if (img.empty()) {
      std::this_thread::sleep_for(10ms);
      continue;
    }

    // 2. 取图像时刻的云台姿态
    Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    /// 自瞄核心逻辑
    auto armors = yolo.detect(img);

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
    cboard.send(command);

    /// 可视化
    if (display) {
      cv::Mat main_view = img.clone();
      draw_main_view(
        main_view, armors, targets, solver, aimer.debug_aim_point, tracker.state(), command,
        detection_queue.size());

      cv::Mat radar;
      draw_omni_radar(radar, detection_queue, gimbal_pos[0], command, tracker.state());

      cv::resize(main_view, main_view, {}, 0.5, 0.5);
      cv::imshow("sentry_DGE", main_view);
      cv::imshow("omni", radar);

      // 两路 USB 原始画面
      auto usb_frames = perceptron.get_debug_frames();
      for (int i = 0; i < 2; ++i) {
        if (usb_frames[i].img.empty()) continue;
        cv::Mat usb_view;
        const auto title = (i == 0) ? "usb_left" : "usb_right";
        draw_usb_view(usb_view, usb_frames[i], title);
        cv::imshow(title, usb_view);
      }

      if (cv::waitKey(1) == 'q') break;
    }
  }

  if (display) cv::destroyAllWindows();

  return 0;
}
