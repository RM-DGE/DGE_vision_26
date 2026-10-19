#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <cstddef>
#include <list>
#include <optional>
#include <vector>

#include "armor.hpp"
#include "rmcs_ekf/rmcs_ekf.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool isinit = false;

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig, const rmcs_ekf::Config & ekf_config,
    const rmcs_ekf::CameraFeature & camera);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);

  // Bulk update: the rmcs model needs all observations in a frame to do
  // anchor matching and sequential lightbar updates.
  void update(const std::list<Armor> & armors, const rmcs_ekf::CameraFeature & camera);

  // rmcs-style interface
  auto state() const -> rmcs_ekf::State;
  auto full() const -> std::array<rmcs_ekf::Armor3d, 4>;
  auto xyza_list() const -> std::vector<Eigen::Vector4d>;
  auto converge() const -> bool;
  auto diverged() const -> bool;
  auto jumped() const -> bool;
  auto last_id() const -> int;
  auto armor_count() const -> std::size_t;
  auto uses_rmcs() const -> bool { return use_rmcs_; }

  // 统一 EKF 健康状态：4 板 rmcs 像素 EKF 与 2/3 板 legacy 3D EKF
  // 都映射到同一个 Diagnostics 结构，便于 Foxglove/PlotJuggler 使用。
  auto diagnostics() const -> rmcs_ekf::Diagnostics;

  bool checkinit() const { return isinit; }

private:
  int armor_num_ = 4;
  int switch_count_ = 0;
  int update_count_ = 0;

  bool is_switch_ = false;
  mutable bool is_converged_ = false;

  bool jumped_ = false;
  int last_id_ = 0;
  double last_dt_ = 0.0;

  std::chrono::steady_clock::time_point t_;

  // 4-armor pixel-domain EKF (rmcs port)
  bool use_rmcs_ = false;
  rmcs_ekf::RobotModel rmcs_model_;

  // Legacy sp_vision EKF (2/3-armor targets: balance / outpost / base)
  tools::ExtendedKalmanFilter ekf_;

  void update_legacy(const Armor & armor);
  void update_ypda(const Armor & armor, int id);

  auto h_armor_xyz(const Eigen::VectorXd & x, int id) const -> Eigen::Vector3d;
  auto h_jacobian(const Eigen::VectorXd & x, int id) const -> Eigen::MatrixXd;

  auto diverged_legacy() const -> bool;
  auto convergened_legacy() const -> bool;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
