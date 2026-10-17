/**
 * @file camera_thread_test.cpp
 * @brief 相机采集 + 多 YOLO 实例 + 线程池的吞吐量测试
 *
 * 测试内容：
 *   1. 主线程持续从 io::Camera 取图，并把检测任务投递到 ThreadPool；
 *   2. 线程池中同时运行 num_yolo_thread 个 YOLO 实例，空闲实例分配给新帧；
 *   3. 检测结果封装成 tools::Frame 放入 OrderedQueue，由独立处理线程按序取出；
 *   4. 打印相机 fps，Plotter 输出每帧检测到的装甲板数量。
 *
 * 使用方法（先在 CMakeLists.txt 中取消 camera_thread_test 相关注释并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/camera_thread_test configs/ascento.yaml
 *
 * 注意：
 *   - 需要与 assets 中模型匹配的配置文件；
 *   - 按 q 或 Ctrl+C 退出（处理线程会随 exiter 结束）。
 */

#include <fmt/core.h>
#include <unistd.h>

#include <chrono>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/thread_pool.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/ascento.yaml | 位置参数，yaml配置文件路径 }";

// 检测结果队列：检测线程写入，处理线程按帧号顺序读取
tools::OrderedQueue frame_queue;

// 处理一帧的检测任务：调用指定 YOLO 实例，然后带着结果入队
void detect_frame(tools::Frame && frame, auto_aim::YOLO & yolo)
{
  frame.armors = yolo.detect(frame.img);
  frame_queue.enqueue(frame);
}

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  // tools::Recorder recorder(100);  // 需要录制时打开

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // 固定的下游处理线程：从有序队列取帧，做后续逻辑或可视化
  auto process_thread = std::thread([&]() {
    tools::Frame process_frame;
    while (!exiter.exit()) {
      process_frame = frame_queue.dequeue();
      auto img = process_frame.img;
      auto armors = process_frame.armors;
      auto t = process_frame.t;

      nlohmann::json data;
      data["armor_num"] = armors.size();

      plotter.plot(data);
      // cv::resize(img, img, {}, 0.5, 0.5);
      // cv::imshow("reprojection", img);
    }
  });

  io::Camera camera(config_path);
  int num_yolo_thread = 8;  // YOLO 实例数量，约等于并行检测线程数
  auto yolos = tools::create_yolov8s(config_path, num_yolo_thread, true);
  // auto yolos = tools::create_yolo11s(config_path, num_yolo_thread, true);
  std::vector<bool> yolo_used(num_yolo_thread, false);  // 标记各实例是否正在使用
  tools::ThreadPool thread_pool(num_yolo_thread);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  std::chrono::steady_clock::time_point last_t = std::chrono::steady_clock::now();

  int frame_id = 0;

  while (!exiter.exit()) {
    camera.read(img, t);
    auto dt = tools::delta_time(t, last_t);
    last_t = t;

    // tools::logger()->info("{:.2f} fps", 1 / dt);
    // tools::draw_text(img, fmt::format("{:.2f} fps", 1/dt), {10, 60}, {255, 255, 255});
    nlohmann::json data;
    data["fps"] = 1 / dt;

    frame_id++;

    // 将本帧检测任务提交到线程池，取一个空闲 YOLO 实例处理
    std::mutex yolo_mutex;
    thread_pool.enqueue([&, frame_id, t] {
      auto_aim::YOLO * yolo = nullptr;
      int yolo_id = -1;
      for (int i = 0; i < num_yolo_thread; i++) {
        if (!yolo_used[i]) {
          yolo_used[i] = true;
          yolo = &yolos[i];
          yolo_id = i;
          break;
        }
      }
      if (yolo) {
        tools::Frame frame{frame_id, img.clone(), t};
        detect_frame(std::move(frame), *yolo);

        yolo_used[yolo_id] = false;  // 用完归还实例
      }
    });
    plotter.plot(data);

    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}
