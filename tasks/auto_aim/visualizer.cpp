#include "visualizer.hpp"

#include <fmt/core.h>

#include "tools/img_tools.hpp"

namespace auto_aim
{

void draw_main_view(
  cv::Mat & canvas, const std::list<Armor> & armors, const std::list<Target> & targets,
  Solver & solver, const AimPoint & aim_point, const std::string & state,
  const io::Command & command, std::size_t usb_count)
{
  for (const auto & armor : armors) {
    tools::draw_points(canvas, armor.points, {0, 255, 0});
    tools::draw_text(
      canvas, ARMOR_NAMES[armor.name],
      cv::Point(static_cast<int>(armor.center.x), static_cast<int>(armor.center.y)), {0, 255, 0},
      0.6, 1);
  }

  if (!targets.empty()) {
    auto target = targets.front();
    for (const auto & xyza : target.xyza_list()) {
      auto image_points =
        solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
      tools::draw_points(canvas, image_points, {255, 255, 0});
    }

    if (aim_point.valid) {
      auto aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(canvas, image_points, {0, 0, 255});
    }
  }

  tools::draw_text(canvas, fmt::format("[{}]", state), {10, 30}, {255, 255, 255});
  tools::draw_text(
    canvas,
    fmt::format(
      "cmd yaw {:.1f} pitch {:.1f} shoot {}", command.yaw * 57.3, command.pitch * 57.3,
      command.shoot),
    {10, 65}, {0, 255, 255});
  tools::draw_text(canvas, fmt::format("usb {}", usb_count), {10, 100}, {0, 255, 255});
}

}  // namespace auto_aim
