/**
 * @file perceptron.hpp
 * @brief 多相机并行感知：每个相机一个线程 + 一个 YOLO 实例
 *
 * 数据流：
 *   USBCamera 取图 -> YOLO 检测 -> Decider::delta_angle 计算偏角
 *   -> DetectionResult 入队 -> 上层 get_detection_queue() 取走
 *
 * 说明：
 *   - 相机指针由外部创建并管理，Perceptron 不负责析构；
 *   - 两路 USB 分别作为 left / right 补盲相机；
 *   - 每个相机的 device_name 必须是 "left" / "right"，否则 Decider 会按后置相机处理；
 *   - 当前 sentry_DGE 使用本类：工业主相机 + 两路 USB 全向感知。
 */

#ifndef OMNIPERCEPTION__PERCEPTRON_HPP
#define OMNIPERCEPTION__PERCEPTRON_HPP

#include <array>
#include <chrono>
#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <string>

#include "decider.hpp"
#include "detection.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tools/thread_pool.hpp"
#include "tools/thread_safe_queue.hpp"

namespace omniperception
{

/// 可视化用的单路 USB 调试帧：原始图像 + 本帧检测结果
struct DebugFrame
{
  cv::Mat img;
  std::list<auto_aim::Armor> armors;
  std::string name;
  double delta_yaw = 0;    // rad
  double delta_pitch = 0;  // rad
};

// 多相机并行感知器：为每路相机维护独立线程和 YOLO 实例。
class Perceptron
{
public:
  /**
   * @brief 构造并启动两路 USB 相机的推理线程
   * @param usbcam1..2  相机指针，非拥有；调用者需保证生命周期长于 Perceptron
   * @param config_path yaml 配置路径（相机、模型、FOV 等）
   */
  Perceptron(
    io::USBCamera * usbcam1, io::USBCamera * usbcam2, const std::string & config_path);

  /// 停止标志置位并 join 所有推理线程
  ~Perceptron();

  /// 取出当前队列中的全部检测结果（非阻塞，队列为空时返回空 vector）
  std::vector<DetectionResult> get_detection_queue();

  /// 取两路相机最近一帧的原始图像和检测结果，仅供可视化使用
  std::vector<DebugFrame> get_debug_frames() const;

  /// 单路相机的推理循环：取图 -> 检测 -> 计算偏角 -> 入队
  void parallel_infer(io::USBCamera * cam, std::shared_ptr<auto_aim::YOLO> & yolo_parallel);

private:
  std::vector<std::thread> threads_;  // 每个相机一个推理线程
  tools::ThreadSafeQueue<DetectionResult> detection_queue_;  // 检测结果队列

  // 每路相机独立的 YOLO 实例，避免同一个模型对象被多线程并发调用
  std::shared_ptr<auto_aim::YOLO> yolo_parallel1_;
  std::shared_ptr<auto_aim::YOLO> yolo_parallel2_;

  Decider decider_;             // 仅复用其 delta_angle 角度换算
  bool stop_flag_;              // 退出标志，受 mutex_ 保护
  mutable std::mutex mutex_;
  std::condition_variable condition_;  // 预留：当前实现用轮询等待

  std::array<io::USBCamera *, 2> cameras_ {nullptr, nullptr};  // 用于调试帧定位
  std::array<DebugFrame, 2> debug_frames_;  // 两路相机最近一帧
  mutable std::mutex debug_mutex_;
};

}  // namespace omniperception
#endif