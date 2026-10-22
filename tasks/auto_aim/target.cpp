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
  const rmcs_ekf::Config & ekf_config, const rmcs_ekf::CameraFeature & camera)
: name(armor.name)
, armor_type(armor.type)
, priority(armor.priority)
, t_(t)
{
  if (armor.points.size() < 4) {
    tools::logger()->warn(
      "[Target] armor has {} corners, expected at least 4", armor.points.size());
    return;
  }

  rmcs_model_.configure(ekf_config);

  // v0.32: 两条初始化路径。Solver 位姿已经过 sp_vision 的 yaw 重投影优化，
  // 并与 DGE 的 R/t_camera2gimbal 外参一致，默认优先；RobustPnp 作兜底。
  const auto init_from_solver = [&]() {
    const auto orientation = rmcs_ekf::euler_to_quaternion(
      armor.ypr_in_world[0], armor.ypr_in_world[1], armor.ypr_in_world[2]);
    return rmcs_model_.initialize_from_pose(
      camera, armor.xyz_in_world, orientation, armor.type == ArmorType::big);
  };
  const auto init_from_pnp = [&]() {
    return rmcs_model_.initialize(camera, { to_armor2d(armor) });
  };

  bool initialized = false;
  if (ekf_config.init_prefer_solver_pose) {
    initialized = init_from_solver();
    init_source_ = initialized ? 1 : 0;
    if (!initialized && init_from_pnp()) {
      initialized = true;
      init_source_ = 2;
    }
  } else {
    initialized = init_from_pnp();
    init_source_ = initialized ? 2 : 0;
    if (!initialized && init_from_solver()) {
      initialized = true;
      init_source_ = 1;
    }
  }

  if (!initialized) {
    tools::logger()->warn("[Target] rmcs EKF init failed");
    return;
  }

  isinit = true;
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  const auto dt = tools::delta_time(t, t_);
  if (!std::isfinite(dt) || dt < 0.0) {
    tools::logger()->debug("[Target] ignoring out-of-order predict timestamp: dt={:.4f}s", dt);
    return;
  }
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  if (!std::isfinite(dt) || dt <= 0.0) return;
  rmcs_model_.predict(dt);
}

void Target::update(const std::list<Armor> & armors, const rmcs_ekf::CameraFeature & camera)
{
  std::vector<rmcs_ekf::Armor2d> armors2d;
  armors2d.reserve(armors.size());
  for (const auto & armor : armors) {
    if (armor.points.size() < 4) continue;
    armors2d.push_back(to_armor2d(armor));
  }

  rmcs_model_.correct(armors2d, {}, camera);
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

auto Target::converge() const -> bool { return rmcs_model_.converge(); }

auto Target::diverged() const -> bool { return rmcs_model_.diverged(); }

auto Target::jumped() const -> bool { return rmcs_model_.jumped(); }

auto Target::last_id() const -> int { return rmcs_model_.last_observed_id(); }

auto Target::armor_count() const -> std::size_t { return 4; }

auto Target::diagnostics() const -> rmcs_ekf::Diagnostics { return rmcs_model_.diagnostics(); }

}  // namespace auto_aim
