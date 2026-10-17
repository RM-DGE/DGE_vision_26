/**
 * @file camera_detect_test.cpp
 * @brief 实车相机 + 装甲板检测的帧率测试（对比传统视觉与 YOLO）
 *
 * 测试内容：
 *   1. 从 io::Camera 持续取图；
 *   2. 按 -t/--tradition 选择传统视觉 Detector 或 YOLO 进行检测；
 *   3. 统计并打印每帧检测耗时对应的 fps，用于评估检测器性能。
 *
 * 使用方法（先在 CMakeLists.txt 中取消 camera_detect_test 相关注释并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/camera_detect_test configs/sentry.yaml          # 默认使用 YOLO
 *   ./build/camera_detect_test configs/sentry.yaml -t       # 使用传统视觉
 *
 * 操作：按 q 或 Ctrl+C 退出。注意本测试不显示图像，只输出日志。
 */

#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/sentry.yaml    | yaml配置文件的路径}"
  "{tradition t    |  false                 | 是否使用传统方法识别}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto use_tradition = cli.get<bool>("tradition");

  tools::Exiter exiter;

  // 两个检测器都初始化，实际使用哪个由命令行开关决定
  io::Camera camera(config_path);
  auto_aim::Detector detector(config_path, true);  // 传统视觉检测器（debug 模式）
  auto_aim::YOLO yolo(config_path, true);          // 神经网络检测器（debug 模式）

  std::chrono::steady_clock::time_point timestamp;

  while (!exiter.exit()) {
    cv::Mat img;
    std::list<auto_aim::Armor> armors;

    camera.read(img, timestamp);

    if (img.empty()) break;

    auto last = std::chrono::steady_clock::now();

    // 根据参数选择检测方式
    if (use_tradition)
      armors = detector.detect(img);
    else
      armors = yolo.detect(img);

    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last);
    tools::logger()->info("{:.2f} fps", 1 / dt);  // 单帧检测耗时对应的帧率

    auto key = cv::waitKey(33);
    if (key == 'q') break;
  }

  return 0;
}