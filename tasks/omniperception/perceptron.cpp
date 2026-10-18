/**
 * @file perceptron.cpp
 * @brief 多相机并行感知实现：两路 USB 相机 + 两个 YOLO 实例 + 结果队列
 *
 * 工作方式：
 *   1. 构造时为每路相机创建一个 YOLO 实例，避免模型对象并发调用；
 *   2. 每路相机一个线程，循环执行 read -> detect -> delta_angle -> enqueue；
 *   3. 上层通过 get_detection_queue() 一次性取走当前积压的结果。
 *
 * 注意：
 *   - 队列只保存“检测到目标”的帧，没有目标时不会入队；
 *   - 多路相机之间没有时间同步/融合，只按到达顺序排队；
 *   - decider_ 在这里只用于 delta_angle()，不做完整决策。
 */

#include "perceptron.hpp"

#include <chrono>
#include <memory>
#include <thread>

#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

namespace omniperception
{
Perceptron::Perceptron(
  io::USBCamera * usbcam1, io::USBCamera * usbcam2, const std::string & config_path)
: detection_queue_(10), decider_(config_path), stop_flag_(false)
{
  cameras_ = {usbcam1, usbcam2};

  // 初始化 YOLO 模型：每个线程独立一个实例，防止并发调用同一个模型
  yolo_parallel1_ = std::make_shared<auto_aim::YOLO>(config_path, false);
  yolo_parallel2_ = std::make_shared<auto_aim::YOLO>(config_path, false);

  // 等待相机/模型初始化稳定后再开始推理
  std::this_thread::sleep_for(std::chrono::seconds(2));
  // 创建两个线程进行并行推理
  // 注意：相机指针必须按值捕获，按引用捕获会在构造函数返回后悬空
  threads_.emplace_back([this, usbcam1] { parallel_infer(usbcam1, yolo_parallel1_); });
  threads_.emplace_back([this, usbcam2] { parallel_infer(usbcam2, yolo_parallel2_); });

  tools::logger()->info("Perceptron initialized.");
}

Perceptron::~Perceptron()
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    stop_flag_ = true;  // 设置退出标志
  }
  condition_.notify_all();  // 唤醒所有等待的线程（当前循环未使用 wait，保留以便扩展）

  // 等待线程结束
  for (auto & t : threads_) {
    if (t.joinable()) {
      t.join();
    }
  }
  tools::logger()->info("Perceptron destructed.");
}

// 非阻塞地取走队列中所有结果；队列为空时返回空 vector
std::vector<DetectionResult> Perceptron::get_detection_queue()
{
  std::vector<DetectionResult> result;
  DetectionResult temp;

  // 注意：这里的 pop 不阻塞（假设队列为空时会报错或忽略）
  while (!detection_queue_.empty()) {
    detection_queue_.pop(temp);
    result.push_back(std::move(temp));
  }

  return result;
}

// 取两路相机最近一帧，供 sentry_DGE 的 -d 可视化使用。
std::vector<DebugFrame> Perceptron::get_debug_frames() const
{
  std::lock_guard<std::mutex> lock(debug_mutex_);
  return {debug_frames_[0], debug_frames_[1]};
}

// 单路相机的推理循环：取图 -> 检测 -> 计算偏角 -> 入队
void Perceptron::parallel_infer(
  io::USBCamera * cam, std::shared_ptr<auto_aim::YOLO> & yolov8_parallel)
{
  if (!cam) {
    tools::logger()->error("Camera pointer is null!");
    return;
  }
  try {
    while (true) {
      cv::Mat usb_img;
      std::chrono::steady_clock::time_point ts;

      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_flag_) break;  // 检查是否需要退出
      }

      cam->read(usb_img, ts);
      if (usb_img.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        continue;
      }

      std::list<auto_aim::Armor> armors;
      armors = yolov8_parallel->detect(usb_img);

      Eigen::Vector2d delta_angle{0, 0};
      if (!armors.empty()) {
        // 用相机安装位置把画面中心坐标换算成相对该相机的偏角（度）
        delta_angle = decider_.delta_angle(armors, cam->device_name);
      }

      // 保存最近一帧，供可视化使用；img 是浅拷贝，不额外复制像素
      {
        int index = (cam == cameras_[1]) ? 1 : 0;
        std::lock_guard<std::mutex> lock(debug_mutex_);
        auto & frame = debug_frames_[index];
        frame.img = usb_img;
        frame.armors = armors;
        frame.name = cam->device_name;
        frame.delta_yaw = delta_angle[0] / 57.3;
        frame.delta_pitch = delta_angle[1] / 57.3;
      }

      if (!armors.empty()) {
        DetectionResult dr;
        dr.armors = std::move(armors);
        dr.timestamp = ts;
        dr.delta_yaw = delta_angle[0] / 57.3;    // 度 -> 弧度
        dr.delta_pitch = delta_angle[1] / 57.3;
        detection_queue_.push(dr);  // 推入线程安全队列
      }
    }
  } catch (const cv::Exception & e) {
    tools::logger()->error("cv::Exception in parallel_infer: {}", e.what());
  } catch (const std::exception & e) {
    tools::logger()->error("std::exception in parallel_infer: {}", e.what());
  }
}

}  // namespace omniperception
