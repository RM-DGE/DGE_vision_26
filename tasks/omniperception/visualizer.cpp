#include "visualizer.hpp"

#include <fmt/core.h>

#include <cmath>

#include "tools/img_tools.hpp"

namespace omniperception
{
namespace
{
cv::Point polar_to_screen(const cv::Point & center, double yaw_deg, double radius)
{
  const auto a = yaw_deg * CV_PI / 180.0;
  // 屏幕上方为 0°，yaw 正方向向左
  return {
    static_cast<int>(center.x - radius * std::sin(a)),
    static_cast<int>(center.y - radius * std::cos(a))};
}
}  // namespace

// 极坐标全向感知图：中心为云台，0° 朝前，正 yaw 向左
void draw_omni_radar(
  cv::Mat & radar, const std::vector<DetectionResult> & detection_queue, double gimbal_yaw,
  const io::Command & command, const std::string & state)
{
  radar = cv::Mat(600, 600, CV_8UC3, cv::Scalar(30, 30, 30));
  const cv::Point center(300, 300);

  // 距离环和坐标轴
  for (double r : {80.0, 160.0, 240.0}) {
    cv::circle(radar, center, static_cast<int>(r), {80, 80, 80}, 1);
  }
  cv::line(radar, {40, 300}, {560, 300}, {60, 60, 60}, 1);
  cv::line(radar, {300, 40}, {300, 560}, {60, 60, 60}, 1);

  tools::draw_text(radar, "0", {292, 30}, {200, 200, 200});
  tools::draw_text(radar, "-90", {530, 290}, {200, 200, 200});
  tools::draw_text(radar, "90", {40, 290}, {200, 200, 200});
  tools::draw_text(radar, "180", {280, 575}, {200, 200, 200});

  // 云台当前朝向（0° 为车体/云台零位，图上的上方向）
  cv::arrowedLine(radar, center, {300, 60}, {0, 255, 0}, 2);

  // 当前云台指令方向（相对当前 yaw）
  if (command.control) {
    const auto cmd_yaw = (command.yaw - gimbal_yaw) * 57.3;
    cv::arrowedLine(radar, center, polar_to_screen(center, cmd_yaw, 240), {0, 0, 255}, 2);
  }

  // 两路 USB 的检测结果
  int index = 0;
  for (const auto & dr : detection_queue) {
    if (dr.armors.empty()) continue;

    const auto yaw_deg = dr.delta_yaw * 57.3;
    const auto point = polar_to_screen(center, yaw_deg, 180);

    const cv::Scalar color = (index++ % 2 == 0) ? cv::Scalar(255, 180, 0)
                                                : cv::Scalar(0, 180, 255);
    cv::circle(radar, point, 7, color, -1);
    tools::draw_text(
      radar, fmt::format("{:.0f}", yaw_deg), point + cv::Point(8, -8), color, 0.5, 1);
    tools::draw_text(
      radar, auto_aim::ARMOR_NAMES[dr.armors.front().name], point + cv::Point(8, 18), color, 0.5,
      1);
  }

  tools::draw_text(radar, fmt::format("state: {}", state), {10, 25}, {255, 255, 255}, 0.7, 1);
  tools::draw_text(
    radar, fmt::format("usb: {}", detection_queue.size()), {10, 50}, {255, 255, 255}, 0.7, 1);

  if (detection_queue.empty()) {
    tools::draw_text(radar, "no USB target", {220, 300}, {180, 180, 180}, 0.7, 1);
  }
}

// USB 相机画面：原始图像 + 本帧检测框 + 相对偏角
void draw_usb_view(cv::Mat & view, const DebugFrame & frame, const std::string & title)
{
  if (frame.img.empty()) return;

  view = frame.img.clone();
  for (const auto & armor : frame.armors) {
    tools::draw_points(view, armor.points, {0, 255, 0});
    tools::draw_text(
      view, auto_aim::ARMOR_NAMES[armor.name],
      cv::Point(static_cast<int>(armor.center.x), static_cast<int>(armor.center.y)), {0, 255, 0},
      0.6, 1);
  }

  tools::draw_text(
    view, fmt::format("{}  yaw {:.1f}", title, frame.delta_yaw * 57.3), {10, 30},
    {0, 255, 255});
  cv::resize(view, view, {}, 0.5, 0.5);
}

}  // namespace omniperception
