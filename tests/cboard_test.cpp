/**
 * @file cboard_test.cpp
 * @brief 下位机 C 板通信测试：读取 IMU 姿态与弹速
 *
 * 测试内容：
 *   1. 通过 io::CBoard 与下位机建立串口/网口通信；
 *   2. 以约 1 kHz 频率查询 IMU 四元数并转换为 z/y/x 欧拉角（度）打印；
 *   3. 同时打印下位机回传的弹丸速度，用于检查通信是否正常。
 *
 * 使用方法（先在 CMakeLists.txt 中取消 cboard_test 相关注释并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/cboard_test configs/standard.yaml
 *
 * 操作：Ctrl+C 退出。若日志长时间不刷新，请检查串口设备与配置文件。
 */

#include "io/cboard.hpp"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                       | 输出命令行参数说明}"
  "{@config-path   | configs/standard.yaml | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  tools::Exiter exiter;

  io::CBoard cboard(config_path);

  while (!exiter.exit()) {
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(1ms);  // 约 1 kHz 查询

    Eigen::Quaterniond q = cboard.imu_at(timestamp);

    // 四元数转 z-y-x 欧拉角，再换算成度
    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0) * 57.3;
    tools::logger()->info("z{:.2f} y{:.2f} x{:.2f} degree", eulers[0], eulers[1], eulers[2]);
    tools::logger()->info("bullet speed {:.2f} m/s", cboard.bullet_speed);  // 弹速反馈
  }

  return 0;
}