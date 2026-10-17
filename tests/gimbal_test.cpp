/**
 * @file gimbal_test.cpp
 * @brief 云台状态监视与发弹测试
 *
 * 测试内容：
 *   1. 连接 io::Gimbal，持续读取模式、角度、角速度、弹速和弹丸计数；
 *   2. 模式变化时打印日志；
 *   3. 带 -f 时按 0.2 s 开火 / 1 s 停火的节奏循环发弹，并统计首发延迟；
 *   4. 通过 Plotter 输出云台状态与发弹信号曲线。
 *
 * 使用方法（先在 CMakeLists.txt 中取消 gimbal_test 相关注释并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/gimbal_test configs/standard.yaml        # 只监视状态
 *   ./build/gimbal_test configs/standard.yaml -f     # 同时测试发弹
 *
 * 注意：带 -f 会真实发弹，请在安全环境下使用；Ctrl+C 退出时会发送停止指令。
 */

#include "io/gimbal/gimbal.hpp"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{f              | | 是否开火}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto test_fire = cli.get<bool>("f");
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::Gimbal gimbal(config_path);

  auto t0 = std::chrono::steady_clock::now();
  auto last_mode = gimbal.mode();
  uint16_t last_bullet_count = 0;  // 上一帧弹丸计数，用于判断是否发弹

  auto fire = false;
  auto fire_count = 0;
  auto fire_stamp = std::chrono::steady_clock::now();
  auto first_fired = false;  // 本轮发弹是否已经检测到

  while (!exiter.exit()) {
    auto mode = gimbal.mode();

    // 云台模式变化（如初始化、正常控制、离线）时打印提示
    if (mode != last_mode) {
      tools::logger()->info("Gimbal mode changed: {}", gimbal.str(mode));
      last_mode = mode;
    }

    auto t = std::chrono::steady_clock::now();
    auto state = gimbal.state();
    auto q = gimbal.q(t);
    auto ypr = tools::eulers(q, 2, 1, 0);

    // 弹丸计数增加说明本帧成功发弹
    auto fired = state.bullet_count > last_bullet_count;
    last_bullet_count = state.bullet_count;

    if (!first_fired && fired) {
      first_fired = true;
      tools::logger()->info("Gimbal first fired after: {:.3f}s", tools::delta_time(t, fire_stamp));
    }

    // 发弹节奏：开火约 0.2 s 后停，停约 1 s 后再开火
    if (fire && fire_count > 20) {
      // 0.2 s
      fire = false;
      fire_count = 0;
    } else if (!fire && fire_count > 100) {
      // 1s
      fire = true;
      fire_count = 0;
      fire_stamp = t;
      first_fired = false;
    }
    fire_count++;

    // control, fire, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc
    gimbal.send(true, test_fire && fire, 1, 0, 0, 0, 0, 0);

    nlohmann::json data;
    // 四元数姿态与底层反馈角度/角速度对照，检查解算是否一致
    data["q_yaw"] = ypr[0];
    data["q_pitch"] = ypr[1];
    data["yaw"] = state.yaw;
    data["vyaw"] = state.yaw_vel;
    data["pitch"] = state.pitch;
    data["vpitch"] = state.pitch_vel;
    data["bullet_speed"] = state.bullet_speed;
    data["bullet_count"] = state.bullet_count;
    data["fired"] = fired ? 1 : 0;
    data["fire"] = test_fire && fire ? 1 : 0;
    data["t"] = tools::delta_time(t, t0);
    plotter.plot(data);

    std::this_thread::sleep_for(9ms);  // 约 100 Hz 查询
  }

  // 退出前发送停止指令，避免云台保持最后一条指令
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}