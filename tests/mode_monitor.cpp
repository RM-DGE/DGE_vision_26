#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/cboard.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>("@config-path");

  tools::Exiter exiter;
  io::CBoard cboard(config_path);

  tools::logger()->info("=== 只读监视，不驱动云台。请拨动遥控器拨杆，观察 mode 是否变化 ===");

  auto last_mode = -1;
  auto last_shoot_mode = -1;
  auto last_bs = -1.0;
  auto stamp = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    std::this_thread::sleep_for(20ms);

    auto mode = int(cboard.mode);
    auto changed = (mode != last_mode) || (cboard.shoot_mode != last_shoot_mode) ||
                   (std::abs(cboard.bullet_speed - last_bs) > 0.01);

    auto now = std::chrono::steady_clock::now();
    auto elapsed = tools::delta_time(now, stamp);

    if (changed) {
      tools::logger()->warn(
        ">>> 变化! mode={} ({})  shoot_mode={}  bullet_speed={:.2f}", mode,
        mode >= 0 && mode < int(io::MODES.size()) ? io::MODES[mode] : "非法值",
        cboard.shoot_mode, cboard.bullet_speed);
      last_mode = mode;
      last_shoot_mode = cboard.shoot_mode;
      last_bs = cboard.bullet_speed;
      stamp = now;
    }

    else if (elapsed >= 2.0) {
      tools::logger()->info(
        "保持 mode={} ({})  shoot_mode={}  bullet_speed={:.2f}  ft_angle={:.2f}deg", mode,
        mode >= 0 && mode < int(io::MODES.size()) ? io::MODES[mode] : "非法值",
        cboard.shoot_mode, cboard.bullet_speed, cboard.ft_angle * 57.3);
      stamp = now;
    }
  }

  return 0;
}
