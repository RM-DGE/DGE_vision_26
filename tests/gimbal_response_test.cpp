/**
 * @file gimbal_response_test.cpp
 * @brief 云台响应特性测试：向 C 板发送不同激励信号并记录跟随曲线
 *
 * 支持的信号模式（-m/--signal-mode）：
 *   triangle_wave  三角波：每周期把 yaw 或 pitch 递增 delta_angle，用于测跟随误差
 *   step           阶跃：每 300 帧把目标角增加 delta_angle，用于测阶跃响应
 *   circle         圆周：按正弦规律同时摆动 yaw/pitch，用于测动态跟踪
 *
 * 使用方法（先在 CMakeLists.txt 中取消 gimbal_response_test 相关注释并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/gimbal_response_test configs/sentry.yaml -a 8 -c 0.2 -m triangle_wave -x yaw
 *   ./build/gimbal_response_test configs/sentry.yaml -m step       -x pitch
 *   ./build/gimbal_response_test configs/sentry.yaml -m circle
 *
 * 参数：-a delta角(度)  -c 周期(s)  -m 信号模式  -x 测试轴(yaw/pitch)
 * 操作：Ctrl+C 退出；数据通过 Plotter（Foxglove）查看。
 */

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{delta-angle a  |          8          | yaw轴delta角}"
  "{circle      c  |         0.2         | delta_angle的切片数}"
  "{signal-mode m  |     triangle_wave   | 发送信号的模式}"
  "{axis        x  |         yaw         | 发送信号的轴}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

// 圆周模式下的 yaw 激励：幅度 7 度、周期 4 s 的正弦
double yaw_cal(double t)
{
  double A = 7;
  double T = 4;  // s

  return A * std::sin(2 * M_PI * t / T);  // 31是云台yaw初始角度，单位为度
}

// 圆周模式下的 pitch 激励：相位超前 90 度并叠加 18 度初始角
double pitch_cal(double t)
{
  double A = 7;
  double T = 4;  // s

  return A * std::sin(2 * M_PI * t / T + M_PI / 2) + 18;  // 18是云台pitch初始角度，单位为度
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  auto delta_angle = cli.get<double>("delta-angle");
  auto circle = cli.get<double>("circle");
  auto signal_mode = cli.get<std::string>("signal-mode");
  auto axis = cli.get<std::string>("axis");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::CBoard cboard(config_path);

  auto init_angle = 0;
  double slice = circle * 100;  //切片数=周期*帧率（按 100 fps 估算）
  auto dangle = delta_angle / slice;  // 每个控制周期的角度增量
  double cmd_angle = init_angle;

  int axis_index = axis == "yaw" ? 0 : 1;  // 0 for yaw, 1 for pitch

  double error = 0;
  int count = 0;

  // 先发送归零指令，并等待云台稳定
  io::Command init_command{1, 0, 0, 0};
  cboard.send(init_command);
  std::this_thread::sleep_for(5s);  //等待云台归零

  io::Command command{0};
  io::Command last_command{0};

  double t = 0;
  auto last_t = t;
  double dt = 0.005;  // 5ms, 模拟200fps

  auto t0 = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    nlohmann::json data;
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(1ms);

    Eigen::Quaterniond q = cboard.imu_at(timestamp);

    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0);  // 当前云台姿态（弧度）

    // ---- 模式一：三角波，逐周期累加角度 ----
    if (signal_mode == "triangle_wave") {
      if (count == slice) {
        // 到达峰值后回到初始角，重新开始下一周期
        cmd_angle = init_angle;
        command = {1, 0, 0, 0};
        if (axis_index == 0)
          command.yaw = cmd_angle / 57.3;
        else
          command.pitch = cmd_angle / 57.3;
        count = 0;

      } else {
        // 每个周期内线性增加目标角
        cmd_angle += dangle;
        if (axis_index == 0)
          command.yaw = cmd_angle / 57.3;
        else
          command.pitch = cmd_angle / 57.3;
        count++;
      }

      cboard.send(command);
      // 记录指令角、上一条指令和实际云台角，便于对比响应
      if (axis_index == 0) {
        data["cmd_yaw"] = command.yaw * 57.3;
        data["last_cmd_yaw"] = last_command.yaw * 57.3;
        data["gimbal_yaw"] = eulers[0] * 57.3;
      } else {
        data["cmd_pitch"] = command.pitch * 57.3;
        data["last_cmd_pitch"] = last_command.pitch * 57.3;
        data["gimbal_pitch"] = eulers[1] * 57.3;
      }
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
      last_command = command;
      plotter.plot(data);
      std::this_thread::sleep_for(8ms);  //模拟自瞄100fps
    }

    // ---- 模式二：阶跃，每 300 帧跳变一次 ----
    else if (signal_mode == "step") {
      if (count == 300) {
        cmd_angle += delta_angle;
        count = 0;
      }
      command = {1, 0, tools::limit_rad(cmd_angle / 57.3), 0};
      count++;

      cboard.send(command);
      data["cmd_yaw"] = command.yaw * 57.3;
      data["last_cmd_yaw"] = last_command.yaw * 57.3;
      data["gimbal_yaw"] = eulers[0] * 57.3;
      last_command = command;
      plotter.plot(data);
      std::this_thread::sleep_for(8ms);  //模拟自瞄100fps
    }

    // ---- 模式三：yaw/pitch 正弦圆周运动 ----
    else if (signal_mode == "circle") {
      std::cout << "t: " << t << std::endl;
      command.yaw = yaw_cal(t) / 57.3;    // 度 -> 弧度
      command.pitch = pitch_cal(t) / 57.3;
      command.control = 1;
      command.shoot = 0;
      t += dt;
      if (t - last_t > 2) {
        t += 2.4;  // 跳过一段，模拟目标遮挡/丢失后的重新捕获
        last_t = t;
      }
      cboard.send(command);

      data["t"] = t;
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
      data["gimbal_yaw"] = eulers[0] * 57.3;
      data["gimbal_pitch"] = eulers[1] * 57.3;
      plotter.plot(data);
      std::this_thread::sleep_for(9ms);
    }
  }
  return 0;
}