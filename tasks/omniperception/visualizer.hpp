#ifndef OMNIPERCEPTION__VISUALIZER_HPP
#define OMNIPERCEPTION__VISUALIZER_HPP

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

#include "detection.hpp"
#include "io/command.hpp"
#include "perceptron.hpp"

namespace omniperception
{

// 全向感知调试画面：极坐标雷达图 + 两路 USB 原始画面。
// 只消费 Perceptron/Decider 的结果，不参与检测和决策。
void draw_omni_radar(
  cv::Mat & radar, const std::vector<DetectionResult> & detection_queue, double gimbal_yaw,
  const io::Command & command, const std::string & state);

void draw_usb_view(cv::Mat & view, const DebugFrame & frame, const std::string & title);

}  // namespace omniperception

#endif  // OMNIPERCEPTION__VISUALIZER_HPP
