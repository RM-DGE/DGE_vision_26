/**
 * @file fire_test.cpp
 * @brief 云台发弹测试：周期性发送开火指令并检查是否成功发弹
 *
 * 测试内容：
 *   1. 连接 io::Gimbal 云台；
 *   2. 每约 1.6 s 切换一次 mode=2（开火），其余时间 mode=1（正常控制）；
 *   3. 通过 Plotter 输出开火信号，配合录像检查发弹机构是否正常。
 *
 * 使用方法（先在 CMakeLists.txt 中取消 fire_test 相关注释并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/fire_test configs/standard.yaml
 *
 * 注意：会真实触发发弹，务必在安全环境下运行。
 * 操作：按 q 或 Ctrl+C 退出。
 */

#include <fmt/format.h>

#include "io/gimbal/gimbal.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/trajectory.hpp"

// 定义命令行参数
const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // 初始化绘图器、录制器、退出器
  tools::Plotter plotter;
  tools::Recorder recorder;
  tools::Exiter exiter;

  // 初始化云台
  io::Gimbal gimbal(config_path);
  io::VisionToGimbal plan;  // 视觉发给云台的控制量
  auto last_t = std::chrono::steady_clock::now();
  // 初始目标角与角速度、角加速度全部置零
  plan.yaw = 0;
  plan.yaw_vel = 0;
  plan.yaw_acc = 0;
  plan.pitch = 0;
  plan.pitch_vel = 0;
  plan.pitch_acc = 0;

  while (!exiter.exit()) {
    auto now = std::chrono::steady_clock::now();
    auto gs = gimbal.state();
    // 每 1.6 s 发一次开火命令，持续一个控制周期
    if(tools::delta_time(now, last_t) > 1.600) {
        plan.mode = 2;  // 2 表示开火
        tools::logger()->debug("fire!");
        last_t = now;
    } else plan.mode = 1;  // 1 表示正常控制

    gimbal.send(plan);

    // -------------- 调试输出 --------------

    nlohmann::json data;

    // 只有非零模式才记录开火状态，避免曲线在空闲时跳变
    if (plan.mode != 0) {
      data["shoot"] = plan.mode == 2 ? 1 : 0;
    }

    plotter.plot(data);

    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}
