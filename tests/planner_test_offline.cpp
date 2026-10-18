/**
 * @file planner_test_offline.cpp
 * @brief MPC 规划器离线测试：无需云台，只验证规划输出
 *
 * 测试内容：
 *   1. 按命令行给定的距离 d 和角速度 w 构造虚拟圆周运动目标；
 *   2. 每 10 ms 预测一次目标并调用 Planner（固定弹速 22 m/s）；
 *   3. 通过 Plotter 输出目标角度与规划的 yaw/pitch、角速度、角加速度曲线，
 *      适合在没有硬件时调试 MPC 参数。
 *
 * 使用方法（先在 CMakeLists.txt 中取消 planner_test_offline 相关注释并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/planner_test_offline configs/standard.yaml -d=3.0 -w=5.0
 *
 * 参数：-d 目标距离(m)  -w 目标角速度(rad/s)
 * 操作：Ctrl+C 退出。
 */

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |     | 输出命令行参数说明    }"
  "{d              | 3.0 | Target距离(m)       }"
  "{w              | 5.0 | Target角速度(rad/s) }"
  "{@config-path   |     | yaml配置文件路径     }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  auto d = cli.get<double>("d");
  auto w = cli.get<double>("w");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  auto_aim::Planner planner(config_path);
  // 虚拟目标：距离 d、角速度 w、半径 0.2 m、装甲板高度 0.1 m
  auto_aim::Target target(d, w, 0.2, 0.1);

  auto t0 = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    target.predict(0.01);  // 按 10 ms 步长推进目标状态

    // 固定弹速 22 m/s，不依赖真实云台反馈
    auto plan = planner.plan(target, 22);

    nlohmann::json data;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

    // 目标角度与规划角度
    data["target_yaw"] = plan.target_yaw;
    data["target_pitch"] = plan.target_pitch;

    data["plan_yaw"] = plan.yaw;
    data["plan_yaw_vel"] = plan.yaw_vel;
    data["plan_yaw_acc"] = plan.yaw_acc;

    data["plan_pitch"] = plan.pitch;
    data["plan_pitch_vel"] = plan.pitch_vel;
    data["plan_pitch_acc"] = plan.pitch_acc;

    plotter.plot(data);

    std::this_thread::sleep_for(10ms);  // 模拟 100 Hz 控制周期
  }

  return 0;
}