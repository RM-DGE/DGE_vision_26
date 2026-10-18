/**
 * @file detection.hpp
 * @brief 多相机检测结果的统一数据结构
 *
 * Perceptron 的每个相机线程检测到装甲板后，会把结果连同时间戳和
 * “目标相对该相机光轴的角度”一起封装成 DetectionResult，放入线程安全队列，
 * 供 Decider 或上层控制逻辑消费。
 *
 * 角度单位约定：delta_yaw / delta_pitch 均为弧度。
 */

#ifndef OMNIPERCEPTION__DETECTION_HPP
#define OMNIPERCEPTION__DETECTION_HPP

#include <chrono>
#include <list>

#include "tasks/auto_aim/armor.hpp"

namespace omniperception
{
// 一个识别结果可能包含多个armor,需要排序和过滤。armors, timestamp, delta_yaw, delta_pitch
struct DetectionResult
{
  std::list<auto_aim::Armor> armors;          // 该相机本帧检测到的装甲板（未排序/未过滤）
  std::chrono::steady_clock::time_point timestamp;  // 图像时间戳
  double delta_yaw;    // 目标相对该相机光轴的 yaw 偏角（rad）
  double delta_pitch;  // 目标相对该相机光轴的 pitch 偏角（rad）

  // Assignment operator
  DetectionResult & operator=(const DetectionResult & other)
  {
    if (this != &other) {
      armors = other.armors;
      timestamp = other.timestamp;
      delta_yaw = other.delta_yaw;
      delta_pitch = other.delta_pitch;
    }
    return *this;
  }
};
}  // namespace omniperception

#endif