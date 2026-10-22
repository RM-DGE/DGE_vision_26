#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <array>
#include <chrono>
#include <cstddef>
#include <list>
#include <vector>

#include <Eigen/Dense>

#include "armor.hpp"
#include "rmcs_ekf/rmcs_ekf.hpp"

namespace auto_aim
{

/// 4-armor target backed only by the rmcs pixel-domain EKF.
///
/// DGE no longer keeps the legacy sp_vision 3D EKF for 2/3-armor targets
/// (balance / outpost / base). Tracker drops those detections instead.
class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool isinit = false;

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t,
    const rmcs_ekf::Config & ekf_config, const rmcs_ekf::CameraFeature & camera);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);

  // Bulk update: the rmcs model needs all observations in a frame to do
  // anchor matching and sequential lightbar updates.
  void update(const std::list<Armor> & armors, const rmcs_ekf::CameraFeature & camera);

  auto state() const -> rmcs_ekf::State;
  auto full() const -> std::array<rmcs_ekf::Armor3d, 4>;
  auto xyza_list() const -> std::vector<Eigen::Vector4d>;
  auto converge() const -> bool;
  auto diverged() const -> bool;
  auto jumped() const -> bool;
  auto last_id() const -> int;
  auto armor_count() const -> std::size_t;
  // v0.32: 4 板 EKF 的初始化来源。0 = 未初始化，1 = Solver 位姿，2 = RobustPnp
  auto init_source() const -> int { return init_source_; }

  // 统一 EKF 健康状态，供 Foxglove / PlotJuggler 使用。
  auto diagnostics() const -> rmcs_ekf::Diagnostics;

  bool checkinit() const { return isinit; }

private:
  std::chrono::steady_clock::time_point t_;

  // 4-armor pixel-domain EKF (rmcs port)
  int init_source_ = 0;
  rmcs_ekf::RobotModel rmcs_model_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
