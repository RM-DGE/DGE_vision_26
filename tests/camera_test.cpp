/**
 * @file camera_test.cpp
 * @brief 实车相机取图与帧率测试
 *
 * 测试内容：
 *   1. 按配置打开 io::Camera；
 *   2. 循环取图并根据帧时间戳计算 fps 打印到日志；
 *   3. 可选 -d/--display 打开窗口实时显示原始画面。
 *
 * 使用方法（先在 CMakeLists.txt 中取消 camera_test 相关注释并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/camera_test -c configs/camera.yaml          # 只打印帧率
 *   ./build/camera_test -c configs/camera.yaml -d       # 同时显示画面
 *
 * 操作：显示模式下按 q 或 Ctrl+C 退出。
 */

#include "io/camera.hpp"

#include <opencv2/opencv.hpp>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/camera.yaml | yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");  // 是否显示图像
  io::Camera camera(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_stamp = std::chrono::steady_clock::now();
  while (!exiter.exit()) {
    camera.read(img, timestamp);

    // 用相邻两帧时间戳之差估算实际采集帧率
    auto dt = tools::delta_time(timestamp, last_stamp);
    last_stamp = timestamp;

    tools::logger()->info("{:.2f} fps", 1 / dt);

    if (!display) continue;
    cv::imshow("img", img);
    if (cv::waitKey(1) == 'q') break;
  }
}