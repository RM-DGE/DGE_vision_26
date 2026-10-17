/**
 * @file planner_test.cpp
 * @brief MPC 规划器在线测试：构造虚拟目标并驱动真实云台跟踪
 *
 * 测试内容：
 *   1. 按命令行给定的距离 d 和角速度 w 构造一个做圆周运动的虚拟目标；
 *   2. 每个控制周期预测目标位置，由 Planner 计算 yaw/pitch 及其速度、加速度；
 *   3. 把规划结果发送给 io::Gimbal，同时通过 Plotter 对比目标、规划与云台实际状态。
 *
 * 使用方法（先在 CMakeLists.txt 中取消 planner_test 相关注释并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/planner_test configs/standard.yaml -d 3.0 -w 5.0
 *
 * 参数：-d 目标距离(m)  -w 目标角速度(rad/s)
 * 注意：会驱动真实云台，请确保云台周围安全；按 q 或 Ctrl+C 退出并停止云台。
 */

#include "tasks/auto_aim/planner/planner.hpp"

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/gimbal/gimbal.hpp"
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

  io::Gimbal gimbal(config_path);
  auto_aim::Planner planner(config_path);
  // 虚拟目标：距离 d、角速度 w、半径 0.2 m、装甲板高度 0.1 m
  auto_aim::Target target(d, w, 0.2, 0.1);

  auto t0 = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    target.predict(0.01);  // 按 10 ms 步长推进目标状态

    auto gs = gimbal.state();
    // 用当前弹速计算弹道补偿，得到云台规划指令
    auto plan = planner.plan(target, gs.bullet_speed);

    gimbal.send(
      plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
      plan.pitch_acc);

    nlohmann::json data;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

    // 云台实际反馈
    data["gimbal_yaw"] = gs.yaw;
    data["gimbal_yaw_vel"] = gs.yaw_vel;
    data["gimbal_pitch"] = gs.pitch;
    data["gimbal_pitch_vel"] = gs.pitch_vel;

    // 目标角度与规划角度，用于对比跟踪误差
    data["target_yaw"] = plan.target_yaw;
    data["target_pitch"] = plan.target_pitch;

    data["plan_yaw"] = plan.yaw;
    data["plan_yaw_vel"] = plan.yaw_vel;
    data["plan_yaw_acc"] = plan.yaw_acc;

    data["plan_pitch"] = plan.pitch;
    data["plan_pitch_vel"] = plan.pitch_vel;
    data["plan_pitch_acc"] = plan.pitch_acc;

    plotter.plot(data);

    std::this_thread::sleep_for(10ms);  // 100 Hz 控制周期
  }

  // 退出前停止云台
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}