/**
 * @file detector_video_test.cpp
 * @brief 离线视频上的装甲板检测测试（传统视觉 / YOLO）
 *
 * 测试内容：
 *   1. 读取指定 avi 视频，可在指定帧区间内逐帧处理；
 *   2. 用 -t/--tradition 选择传统视觉 Detector 或 YOLO 检测装甲板；
 *   3. 把第一个装甲板四个角点的像素坐标发送给 Plotter，便于分析检测抖动；
 *   4. 按 q 可提前结束。
 *
 * 使用方法（先在 CMakeLists.txt 中取消 detector_video_test 相关注释并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/detector_video_test assets/demo/demo.avi -c=configs/sentry.yaml -s=0 -e=300
 *   ./build/detector_video_test assets/demo/demo.avi -t      # 使用传统视觉
 *
 * 参数：-c 配置文件  -s 起始帧  -e 结束帧（0 表示到结尾）  -t 使用传统方法
 */

#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{config-path c  | configs/sentry.yaml    | yaml配置文件的路径}"
  "{start-index s  | 0                      | 视频起始帧下标    }"
  "{end-index e    | 0                      | 视频结束帧下标    }"
  "{@video_path    |                        | avi路径}"
  "{tradition t    |  false                 | 是否使用传统方法识别}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto video_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");
  auto use_tradition = cli.get<bool>("tradition");

  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::VideoCapture video(video_path);

  auto_aim::Detector detector(config_path);  // 传统视觉检测器
  auto_aim::YOLO yolo(config_path);          // 神经网络检测器

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);  // 定位到起始帧

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    cv::Mat img;
    std::list<auto_aim::Armor> armors;
    video.read(img);
    if (img.empty()) break;
    // cv::GaussianBlur(img, img, cv::Size(5, 5), 0, 0, cv::BORDER_DEFAULT);

    // 按参数选择检测器
    if (use_tradition)
      armors = detector.detect(img, frame_count);
    else
      armors = yolo.detect(img, frame_count);

    // 只记录每帧第一个装甲板的四个角点，用于观察检测是否稳定
    if (!armors.empty()) {
      nlohmann::json data;
      auto armor = armors.front();

      data["armor_0_pixel_x"] = armor.points[0].x;
      data["armor_0_pixel_y"] = armor.points[0].y;
      data["armor_1_pixel_x"] = armor.points[1].x;
      data["armor_1_pixel_y"] = armor.points[1].y;
      data["armor_2_pixel_x"] = armor.points[2].x;
      data["armor_2_pixel_y"] = armor.points[2].y;
      data["armor_3_pixel_x"] = armor.points[3].x;
      data["armor_3_pixel_y"] = armor.points[3].y;
      plotter.plot(data);
    }

    auto key = cv::waitKey(33);  // 约 30 fps 播放
    if (key == 'q') break;
  }

  return 0;
}