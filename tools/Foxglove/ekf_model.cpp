#include "ekf_model.hpp"

namespace tools {
namespace {
using namespace foxglove::messages;
Vector3 vec(const Eigen::Vector3d& v) { return {v.x(), v.y(), v.z()}; }
Pose pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q = Eigen::Quaterniond::Identity()) {
  Pose result;
  result.position = vec(p);
  result.orientation = Quaternion{q.x(), q.y(), q.z(), q.w()};
  return result;
}
LinePrimitive line(const Eigen::Vector3d& a, const Eigen::Vector3d& b, Color color) {
  LinePrimitive result;
  result.type = LinePrimitive::LineType::LINE_STRIP;
  result.pose = pose(Eigen::Vector3d::Zero());
  result.thickness = .003;
  result.color = color;
  result.points = {{a.x(), a.y(), a.z()}, {b.x(), b.y(), b.z()}};
  return result;
}
}

EkfModelView build_ekf_model(
  const auto_aim::Target* target, const auto_aim::AimPoint& aim, Timestamp timestamp) {
  EkfModelView view;
  view.selection = {{"tracking", target != nullptr}, {"frame_id", "odom"},
    {"selected_id", -1}, {"last_observed_id", -1}, {"aim_valid", false},
    {"aim_position", nullptr}, {"plates", nlohmann::json::array()}};
  if (!target) {
    SceneEntityDeletion deletion;
    deletion.timestamp = timestamp;
    deletion.type = SceneEntityDeletion::SceneEntityDeletionType::ALL;
    view.scene.deletions.push_back(deletion);
    return view;
  }
  const auto state = target->state();
  const auto plates = target->full();
  const Eigen::Vector3d center(state.x, state.y, state.z + state.height_lateral * .5);
  const double width = target->armor_type == auto_aim::big
    ? rmcs_ekf::kLargeArmorWidth : rmcs_ekf::kSmallArmorWidth;
  const double height = rmcs_ekf::kLightBarHeight;
  const int selected = aim.valid && aim.xyza.allFinite() && aim.armor_id >= 0 && aim.armor_id < 4
    ? aim.armor_id : -1;
  const Color green{.15, 1., .35, 1.}, cyan{0., .85, 1., 1.}, amber{1., .68, .12, 1.};
  const Color muted{.45, .55, .65, .7};
  const auto entity = [&](std::string id) {
    SceneEntity result;
    result.timestamp = timestamp;
    result.frame_id = "odom";
    result.id = std::move(id);
    // Expire if the producer stops without an orderly reset.
    result.lifetime = Duration{0, 500000000};
    return result;
  };
  FrameTransform tf;
  tf.timestamp = timestamp;
  tf.parent_frame_id = "odom";
  tf.child_frame_id = "ekf_target";
  tf.translation = vec(center);
  tf.rotation = Quaternion{0, 0, 0, 1};
  view.target_tf = tf;
  view.selection["selected_id"] = selected;
  view.selection["last_observed_id"] = target->last_id();
  view.selection["aim_valid"] = selected >= 0;
  view.selection["width_m"] = width;
  view.selection["height_m"] = height;
  view.selection["radius_forward_m"] = state.radius_forward;
  view.selection["radius_lateral_m"] = state.radius_lateral;
  view.selection["height_lateral_m"] = state.height_lateral;
  view.selection["rotation_speed_rad_s"] = state.rotation_speed;
  view.selection["target_name"] = auto_aim::ARMOR_NAMES.at(static_cast<size_t>(target->name));
  view.selection["center"] = {{"x", center.x()}, {"y", center.y()}, {"z", center.z()}};
  for (const auto& plate : plates) {
    const int id = plate.id;
    const auto& p = plate.translation;
    const bool picked = id == selected;
    const bool observed = id == target->last_id();
    const Color color = picked ? green : amber;
    auto e = entity("armor_" + std::to_string(id));
    CubePrimitive cube;
    cube.pose = pose(p, plate.orientation);
    // Thickness is for visibility only. Width/height are the EKF lightbar rectangle.
    cube.size = Vector3{.008, width, height};
    cube.color = color;
    e.cubes.push_back(cube);
    LinePrimitive outline;
    outline.type = LinePrimitive::LineType::LINE_LOOP;
    outline.pose = pose(p, plate.orientation);
    outline.thickness = observed ? .005 : .002;
    outline.color = observed ? cyan : color;
    outline.points = {{0, width/2, height/2}, {0, -width/2, height/2},
                      {0, -width/2, -height/2}, {0, width/2, -height/2}};
    e.lines.push_back(outline);
    e.lines.push_back(line(center, p, muted));
    e.lines.push_back(line(p, p + plate.orientation * Eigen::Vector3d(.10, 0, 0), color));
    e.metadata = {{"armor_id", std::to_string(id)}, {"selected", picked ? "true" : "false"},
      {"last_observed", observed ? "true" : "false"}, {"width_m", std::to_string(width)},
      {"height_m", std::to_string(height)}, {"thickness", "8mm display only"}};
    view.scene.entities.push_back(std::move(e));
    view.selection["plates"].push_back({{"id", id}, {"selected", picked}, {"last_observed", observed},
      {"position", {{"x", p.x()}, {"y", p.y()}, {"z", p.z()}}},
      {"orientation", {{"x", plate.orientation.x()}, {"y", plate.orientation.y()},
        {"z", plate.orientation.z()}, {"w", plate.orientation.w()}}}});
  }
  auto future = entity("selected_at_impact");
  if (selected >= 0) {
    const Eigen::Vector3d p = aim.xyza.head<3>();
    view.selection["aim_position"] = {{"x", p.x()}, {"y", p.y()}, {"z", p.z()}};
    const Color predicted{1., .25, .8, .85};
    SpherePrimitive point;
    point.pose = pose(p);
    point.size = Vector3{.012, .012, .012};
    point.color = predicted;
    future.spheres.push_back(point);
    future.lines.push_back(line(plates[selected].translation, p, predicted));
    future.metadata = {{"selected_id", std::to_string(selected)}, {"meaning", "predicted impact pose"}};
  }
  view.scene.entities.push_back(std::move(future));
  auto body = entity("ekf_center");
  SpherePrimitive point;
  point.pose = pose(center);
  point.size = Vector3{.035, .035, .035};
  point.color = cyan;
  body.spheres.push_back(point);
  for (int row = 0; row < 2; ++row) {
    const double r = row == 0 ? state.radius_forward : state.radius_lateral;
    const double z = state.z + row * state.height_lateral;
    LinePrimitive ring;
    ring.type = LinePrimitive::LineType::LINE_LOOP;
    ring.pose = pose(Eigen::Vector3d::Zero());
    ring.thickness = .002;
    ring.color = row == 0 ? cyan : amber;
    for (int i = 0; i < 64; ++i) {
      const double a = 2 * rmcs_ekf::kPi * i / 64.;
      ring.points.push_back({state.x + r * std::cos(a), state.y + r * std::sin(a), z});
    }
    body.lines.push_back(std::move(ring));
  }
  view.scene.entities.push_back(std::move(body));
  return view;
}
}
