#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{a amplitude    | 5.0                 | 摆动幅度(度)}"
  "{p period       | 4.0                 | 摆动周期(秒)}"
  "{axis           | yaw                 | 摆动轴: yaw / pitch / both}"
  "{hold           | 0                   | 非0则固定输出该角度(度)，不摆动}"
  "{@config-path   | configs/standard3.yaml | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>("@config-path");
  auto amplitude = cli.get<double>("a") / 57.3;
  auto period = cli.get<double>("p");
  auto axis = cli.get<std::string>("axis");
  auto hold = cli.get<double>("hold") / 57.3;

  tools::Exiter exiter;
  io::CBoard cboard(config_path);

  tools::logger()->info("等待IMU数据...");
  Eigen::Quaterniond q0;
  auto ready = false;
  for (int i = 0; i < 300 && !exiter.exit(); i++) {
    std::this_thread::sleep_for(10ms);
    q0 = cboard.imu_at(std::chrono::steady_clock::now());
    if (std::abs(q0.norm() - 1.0) < 0.05) {
      ready = true;
      break;
    }
  }
  if (!ready) {
    tools::logger()->error("收不到有效IMU数据，检查串口连接");
    return 1;
  }

  auto ypr0 = tools::eulers(q0, 2, 1, 0);
  tools::logger()->info(
    "起始姿态: yaw={:.2f}deg pitch={:.2f}deg  下位机mode={} bullet_speed={:.2f}", ypr0[0] * 57.3,
    ypr0[1] * 57.3, int(cboard.mode), cboard.bullet_speed);
  tools::logger()->warn("即将驱动云台，3秒后开始，Ctrl-C 可随时停止");
  std::this_thread::sleep_for(3s);

  auto t0 = std::chrono::steady_clock::now();
  auto log_stamp = t0;

  while (!exiter.exit()) {
    auto t = std::chrono::steady_clock::now();
    auto elapsed = tools::delta_time(t, t0);

    double yaw_cmd = ypr0[0];
    double pitch_cmd = ypr0[1];

    if (hold != 0.0) {
      if (axis == "pitch")
        pitch_cmd = hold;
      else
        yaw_cmd = hold;
    } else {
      auto offset = amplitude * std::sin(2 * M_PI * elapsed / period);
      if (axis == "yaw" || axis == "both") yaw_cmd = ypr0[0] + offset;
      if (axis == "pitch" || axis == "both") pitch_cmd = ypr0[1] + offset;
    }

    cboard.send({true, false, yaw_cmd, pitch_cmd});

    if (tools::delta_time(t, log_stamp) > 0.2) {
      log_stamp = t;
      auto ypr = tools::eulers(cboard.imu_at(t), 2, 1, 0);
      tools::logger()->info(
        "发送 yaw={:+7.2f} pitch={:+7.2f} | 实测 yaw={:+7.2f} pitch={:+7.2f} | 误差 "
        "yaw={:+6.2f} pitch={:+6.2f} | mode={} bs={:.2f}",
        yaw_cmd * 57.3, pitch_cmd * 57.3, ypr[0] * 57.3, ypr[1] * 57.3,
        (ypr[0] - yaw_cmd) * 57.3, (ypr[1] - pitch_cmd) * 57.3, int(cboard.mode),
        cboard.bullet_speed);
    }

    std::this_thread::sleep_for(10ms);
  }

  cboard.send({false, false, ypr0[0], ypr0[1]});
  tools::logger()->info("已发送停止指令");

  return 0;
}
