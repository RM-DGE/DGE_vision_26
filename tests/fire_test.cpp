#include <fmt/format.h>

#include "io/cboard.hpp"
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
  io::CBoard cboard(config_path);
  auto last_t = std::chrono::steady_clock::now();
  bool fire = false;

  while (!exiter.exit()) {
    auto now = std::chrono::steady_clock::now();
    if(tools::delta_time(now, last_t) > 1.600) {
        fire = true;
        tools::logger()->debug("fire!");
        last_t = now;
    } else fire = false;


    cboard.send(io::Command{true, fire, 0, 0});

    // -------------- 调试输出 --------------

    nlohmann::json data;

    data["shoot"] = fire ? 1 : 0;

    plotter.plot(data);

    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}
