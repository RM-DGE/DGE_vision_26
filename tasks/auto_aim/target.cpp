#include "target.hpp"

#include <chrono>
#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{

namespace
{

auto to_armor2d(const Armor & armor) -> rmcs_ekf::Armor2d
{
  rmcs_ekf::Armor2d result;
  result.large = (armor.type == ArmorType::big);
  result.tl = armor.points[0];
  result.tr = armor.points[1];
  result.br = armor.points[2];
  result.bl = armor.points[3];
  return result;
}

}  // namespace

Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t,
  const rmcs_ekf::Config & ekf_config, const rmcs_ekf::CameraFeature & camera,
  const rmcs_ekf::RecoverySettings & recovery_settings)
: name(armor.name)
, armor_type(armor.type)
, priority(armor.priority)
, t_(t)
, recovery_(recovery_settings)
{
  color_=armor.color;
  if (armor.points.size() < 4) {
    tools::logger()->warn(
      "[Target] armor has {} corners, expected at least 4", armor.points.size());
    return;
  }

  rmcs_model_.configure(ekf_config);

  const auto observation=to_armor2d(armor);
  const bool initialized=armor.pose_valid && rmcs_model_.initialize_from_pose(
    camera,armor.xyz_in_world,rmcs_ekf::euler_to_quaternion(
      armor.ypr_in_world[0],armor.ypr_in_world[1],armor.ypr_in_world[2]),
    armor.type==ArmorType::big,&observation);
  init_source_=initialized ? 1 : 0; // unified constrained pixel pose

  if (!initialized) {
    tools::logger()->warn("[Target] rmcs EKF init failed");
    return;
  }

  isinit = true;
  recovery_.reset(std::chrono::duration<double>(t_.time_since_epoch()).count());
}

auto Target::synthetic(double x, double vyaw, double radius, double h) -> Target
{
  Target target;
  target.name = ArmorName::not_armor;
  target.armor_type = ArmorType::small;
  target.priority = ArmorPriority::fifth;

  rmcs_ekf::State state;
  state.x = x;
  state.y = 0.0;
  state.z = 0.0;
  state.vx = 0.0;
  state.vy = 0.0;
  state.vz = 0.0;
  state.rotation_angle = 0.0;
  state.rotation_speed = vyaw;
  state.radius_forward = radius;
  state.radius_lateral = radius;
  state.height_lateral = h;

  target.rmcs_model_.initialize_synthetic(state);
  target.isinit = true;
  return target;
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  frame_valid_=false;
  const auto dt = tools::delta_time(t, t_);
  if (!std::isfinite(dt) || dt <= 0.0) {
    tools::logger()->debug("[Target] ignoring non-positive predict dt: dt={:.4f}s", dt);
    return;
  }
  rmcs_model_.predict(dt);
  prediction_=rmcs_model_;
  frame_valid_=true;
  t_ = t;
}

auto Target::at(double dt) const -> Target
{
  Target copy = *this;
  copy.rmcs_model_ = rmcs_model_.extrapolated(dt);
  copy.t_ += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(dt));
  return copy;
}

void Target::update(const std::list<Armor> & armors, const rmcs_ekf::CameraFeature & camera)
{
  if(!frame_valid_) return;
  frame_valid_=false;
  std::vector<rmcs_ekf::Armor2d> armors2d;
  armors2d.reserve(armors.size());
  for (const auto & armor : armors) {
    if (armor.points.size() < 4 || armor.color!=color_ || armor.name!=name || armor.type!=armor_type) continue;
    armors2d.push_back(to_armor2d(armor));
  }

  rmcs_model_.correct(armors2d, {}, camera);
  // Conservatively reject same-class mixtures that cannot form one four-plate vehicle.
  const bool identity_consistent=armors2d.size()<=2;
  recovery_.process(rmcs_model_,prediction_,armors2d,camera,
    std::chrono::duration<double>(t_.time_since_epoch()).count(),identity_consistent);
}

auto Target::state() const -> rmcs_ekf::State { return rmcs_model_.state(); }

auto Target::full() const -> std::array<rmcs_ekf::Armor3d, 4> { return rmcs_model_.full(); }

auto Target::xyza_list() const -> std::vector<Eigen::Vector4d>
{
  const auto armors = full();

  std::vector<Eigen::Vector4d> result;
  result.reserve(armors.size());
  for (std::size_t i = 0; i < armors.size(); ++i) {
    const auto ypr = rmcs_ekf::eulers(armors[i].orientation, 2, 1, 0);
    result.emplace_back(
      armors[i].translation.x(), armors[i].translation.y(), armors[i].translation.z(), ypr[0]);
  }
  return result;
}

auto Target::converge() const -> bool { return reliable() && rmcs_model_.converge(); }

auto Target::diverged() const -> bool { return rmcs_model_.diverged(); }

auto Target::jumped() const -> bool { return rmcs_model_.jumped(); }

auto Target::last_id() const -> int { return rmcs_model_.last_observed_id(); }

auto Target::armor_count() const -> std::size_t { return 4; }

auto Target::diagnostics() const -> rmcs_ekf::Diagnostics { return rmcs_model_.diagnostics(); }

}  // namespace auto_aim
