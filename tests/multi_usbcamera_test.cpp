/**
 * @file multi_usbcamera_test.cpp
 * @brief 多相机同时取图测试：两路 USB 相机 + 一路主相机
 *
 * 测试内容：
 *   1. 分别打开 /dev/video0、/dev/video2 两个 USB 相机以及配置文件中 camera_name 指定的主相机；
 *   2. 顺序读取三路图像，统计整体循环的 fps；
 *   3. 可选 -d/--display 同屏显示三路画面，检查设备索引与画面是否正确。
 *
 * 使用方法（先在 CMakeLists.txt 中按需添加该测试目标并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/multi_usbcamera_test configs/sentry.yaml -d
 *
 * 注意：设备名 "video0"/"video2" 写死在代码中，若实际设备不同请修改；
 *      主相机使用命令行传入的配置文件。按 q 或 Ctrl+C 退出。
 */

#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
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
  auto display = cli.has("display");

  io::USBCamera usbcam1("video0", config_path);  // 第一路 USB 相机
  io::USBCamera usbcam2("video2", config_path);  // 第二路 USB 相机
  io::Camera camera(config_path);                // 主相机（配置中的 camera_name）

  cv::Mat img1, img2, img3;
  std::chrono::steady_clock::time_point timestamp;
  auto last_stamp = std::chrono::steady_clock::now();
  while (!exiter.exit()) {
    // 三路顺序读取；总耗时反映多相机同时工作的开销
    usbcam1.read(img1, timestamp);
    usbcam2.read(img2, timestamp);
    camera.read(img3, timestamp);

    auto dt = tools::delta_time(timestamp, last_stamp);
    last_stamp = timestamp;

    tools::logger()->info("{:.2f} fps", 1 / dt);

    if (!display) continue;
    cv::imshow("img1", img1);
    cv::imshow("img2", img2);
    cv::imshow("img3", img3);

    if (cv::waitKey(1) == 'q') break;
  }
}