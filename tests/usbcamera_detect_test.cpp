/**
 * @file usbcamera_detect_test.cpp
 * @brief USB 相机 + YOLO 检测的帧率测试
 *
 * 测试内容：
 *   1. 按 -n/--name 指定的设备名打开 io::USBCamera（默认 video0）；
 *   2. 每帧调用 YOLO 检测装甲板（本测试不关心检测结果，只测耗时）；
 *   3. 打印取图循环的 fps，可选 -d/--display 显示画面。
 *
 * 使用方法（先在 CMakeLists.txt 中按需添加该测试目标并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/usbcamera_detect_test configs/sentry.yaml -n=video0 -d
 *
 * 参数：-n 设备名  -d 显示画面
 * 操作：显示模式下按 q 或 Ctrl+C 退出。
 */

#include <opencv2/opencv.hpp>
#include <thread>

#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{name n         |        video0          | 端口名称 }"
  "{@config-path   | configs/sentry.yaml    | 位置参数，yaml配置文件路径 }"
  "{d display      |                        | 显示视频流       }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  tools::Exiter exiter;

  auto config_path = cli.get<std::string>(0);
  auto device_name = cli.get<std::string>("name");
  auto display = cli.has("display");

  io::USBCamera usbcam(device_name, config_path);

  auto_aim::YOLO yolo(config_path, true);  // debug 模式，保留可视化信息

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_stamp = std::chrono::steady_clock::now();
  while (!exiter.exit()) {
    usbcam.read(img, timestamp);
    yolo.detect(img);  // 结果丢弃，仅用于测量检测耗时

    auto dt = tools::delta_time(timestamp, last_stamp);
    last_stamp = timestamp;

    tools::logger()->info("{:.2f} fps", 1 / dt);
    std::this_thread::sleep_for(10ms);  // 避免日志刷得过快

    if (!display) continue;
    cv::imshow("img", img);
    if (cv::waitKey(1) == 'q') break;
  }
}