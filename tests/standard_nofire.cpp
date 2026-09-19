#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{@config-path   | configs/standard3.yaml | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  cv::Mat img;
  steady_clock::time_point timestamp;

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    if (img.empty()) {
      std::this_thread::sleep_for(10ms);
      continue;
    }

    cboard.mode = io::Mode::auto_aim;
    cboard.bullet_speed = 24.5;

    auto q = cboard.imu_at(timestamp - 1ms);
    solver.set_R_gimbal2world(q);

    auto armors = detector.detect(img);
    auto targets = tracker.track(armors, timestamp);
    auto command = aimer.aim(targets, timestamp, cboard.bullet_speed);

    // Keep the complete aiming loop, but never request a shot.
    command.shoot = false;
    cboard.send(command);

    for (const auto & armor : armors) {
      for (int i = 0; i < 4; ++i) {
        cv::line(img, armor.points[i], armor.points[(i + 1) % 4], {0, 255, 0}, 2);
      }
    }

    cv::putText(
      img, fmt::format("Shoot: 0  Targets: {}", targets.size()), {10, 30},
      cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
    cv::putText(
      img, fmt::format("Yaw: {:.2f}  Pitch: {:.2f}", command.yaw, command.pitch), {10, 60},
      cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 255, 255}, 2);

    cv::imshow("Standard No Fire", img);
    if (cv::waitKey(1) == 'q') break;
  }

  return 0;
}
