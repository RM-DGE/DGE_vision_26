#include "target.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

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

constexpr double kPi = 3.14159265358979323846;

}  // namespace

Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig, const rmcs_ekf::Config & ekf_config,
  const rmcs_ekf::CameraFeature & camera)
: name(armor.name)
, armor_type(armor.type)
, priority(armor.priority)
, armor_num_(armor_num)
, t_(t)
{
  // 4-armor targets: use the rmcs pixel-domain EKF.
  if (armor_num_ == 4 && armor.points.size() >= 4) {
    rmcs_model_.configure(ekf_config);

    auto initialized = rmcs_model_.initialize(camera, { to_armor2d(armor) });
    if (!initialized) {
      // Fallback to the solver's already-computed armor pose when robust PnP
      // is too picky (small/far plate, noisy corners).
      const auto orientation = rmcs_ekf::euler_to_quaternion(
        armor.ypr_in_world[0], armor.ypr_in_world[1], armor.ypr_in_world[2]);
      initialized = rmcs_model_.initialize_from_pose(
        camera, armor.xyz_in_world, orientation, armor.type == ArmorType::big);
    }

    if (initialized) {
      use_rmcs_ = true;
      isinit = true;
      return;
    }
    tools::logger()->warn("[Target] rmcs EKF init failed, falling back to sp_vision EKF");
  }

  // Legacy sp_vision EKF (2/3-armor targets).
  const auto & xyz = armor.xyz_in_world;
  const auto & ypr = armor.ypr_in_world;

  const auto center_x = xyz[0] + radius * std::cos(ypr[0]);
  const auto center_y = xyz[1] + radius * std::sin(ypr[0]);
  const auto center_z = xyz[2];

  Eigen::VectorXd x0{ { center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, radius, 0, 0 } };
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);

  jumped_ = false;
  last_id_ = 0;
  update_count_ = 0;
  is_switch_ = false;
  is_converged_ = false;
  switch_count_ = 0;
}

Target::Target(double x, double vyaw, double radius, double h)
: name(ArmorName::not_armor)
, armor_type(ArmorType::small)
, priority(ArmorPriority::fifth)
, armor_num_(4)
{
  Eigen::VectorXd x0{ { x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h } };
  Eigen::VectorXd P0_dig{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } };
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
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

  last_dt_ = dt;
  if (use_rmcs_) {
    rmcs_model_.predict(dt);
    return;
  }

  // Legacy sp_vision 11-dim EKF.
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  double v1, v2;
  v1 = 100;
  v2 = 400;

  const auto a = dt * dt * dt * dt / 4;
  const auto b = dt * dt * dt / 2;
  const auto c = dt * dt;
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  ekf_.predict(F, Q, f);
}

void Target::update(const std::list<Armor> & armors, const rmcs_ekf::CameraFeature & camera)
{
  if (use_rmcs_) {
    std::vector<rmcs_ekf::Armor2d> armors2d;
    armors2d.reserve(armors.size());
    for (const auto & armor : armors) {
      if (armor.points.size() < 4) continue;
      armors2d.push_back(to_armor2d(armor));
    }

    rmcs_model_.correct(armors2d, {}, camera);
    jumped_ = rmcs_model_.jumped();
    last_id_ = rmcs_model_.last_observed_id();
    return;
  }

  for (const auto & armor : armors) update_legacy(armor);
}

void Target::update_legacy(const Armor & armor)
{
  int id = 0;
  auto min_angle_error = 1e10;
  const auto armor_xyzas = xyza_list();

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++) {
    xyza_i_list.push_back({ armor_xyzas[i], i });
  }

  std::sort(
    xyza_i_list.begin(), xyza_i_list.end(),
    [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
      Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
      Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
      return ypd1[2] < ypd2[2];
    });

  for (int i = 0; i < std::min(3, static_cast<int>(xyza_i_list.size())); i++) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
    auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3]))
      + std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }

  if (id != 0) jumped_ = true;

  is_switch_ = (id != last_id_);
  if (is_switch_) switch_count_++;

  last_id_ = id;
  update_count_++;

  update_ypda(armor, id);
}

void Target::update_ypda(const Armor & armor, int id)
{
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  Eigen::VectorXd R_dig{ { 4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1,
    log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2 } };

  Eigen::MatrixXd R = R_dig.asDiagonal();

  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    return { ypd[0], ypd[1], ypd[2], angle };
  };

  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{ { ypd[0], ypd[1], ypd[2], ypr[0] } };

  ekf_.update(z, H, R, h, z_subtract);
}

auto Target::state() const -> rmcs_ekf::State
{
  if (use_rmcs_) return rmcs_model_.state();

  const auto & x = ekf_.x;
  rmcs_ekf::State result;
  result.x = x[0];
  result.y = x[2];
  result.z = x[4];
  result.vx = x[1];
  result.vy = x[3];
  result.vz = x[5];
  result.rotation_angle = rmcs_ekf::normalize_angle(x[6] + kPi);
  result.rotation_speed = x[7];
  result.radius_forward = x[8];
  result.radius_lateral = x[8] + x[9];
  result.height_lateral = x[10];
  return result;
}

auto Target::full() const -> std::array<rmcs_ekf::Armor3d, 4>
{
  if (use_rmcs_) return rmcs_model_.full();

  std::array<rmcs_ekf::Armor3d, 4> result{};
  const auto & x = ekf_.x;

  const auto center = Eigen::Vector3d{ x[0], x[2], x[4] };
  const auto angle0 = x[6];
  const auto radius = x[8];
  const auto radius_delta = x[9];
  const auto height_delta = x[10];
  const auto pitch = (name == ArmorName::outpost) ? rmcs_ekf::kPredictedOutpostArmorPitch
                                                  : rmcs_ekf::kPredictedOtherArmorPitch;

  for (int i = 0; i < armor_num_; ++i) {
    const auto angle = rmcs_ekf::normalize_angle(angle0 + i * 2 * CV_PI / armor_num_);
    const auto use_l_h = (armor_num_ == 4) && (i == 1 || i == 3);
    const auto r = use_l_h ? radius + radius_delta : radius;
    const auto z = use_l_h ? center.z() + height_delta : center.z();

    result[i].id = i;
    result[i].translation =
      Eigen::Vector3d{ center.x() - r * std::cos(angle), center.y() - r * std::sin(angle), z };
    result[i].orientation = rmcs_ekf::euler_to_quaternion(angle, pitch, 0.0);
  }

  for (int i = armor_num_; i < 4; ++i) result[i].id = -1;
  return result;
}

auto Target::xyza_list() const -> std::vector<Eigen::Vector4d>
{
  const auto armors = full();
  const auto count = armor_count();

  std::vector<Eigen::Vector4d> result;
  result.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto ypr = rmcs_ekf::eulers(armors[i].orientation, 2, 1, 0);
    result.emplace_back(
      armors[i].translation.x(), armors[i].translation.y(), armors[i].translation.z(), ypr[0]);
  }
  return result;
}

auto Target::converge() const -> bool
{
  return use_rmcs_ ? rmcs_model_.converge() : convergened_legacy();
}

auto Target::diverged() const -> bool
{
  return use_rmcs_ ? rmcs_model_.diverged() : diverged_legacy();
}

auto Target::jumped() const -> bool
{
  return use_rmcs_ ? rmcs_model_.jumped() : jumped_;
}

auto Target::last_id() const -> int
{
  return use_rmcs_ ? rmcs_model_.last_observed_id() : last_id_;
}

auto Target::armor_count() const -> std::size_t
{
  return use_rmcs_ ? 4 : static_cast<std::size_t>(armor_num_);
}

auto Target::diagnostics() const -> rmcs_ekf::Diagnostics
{
  if (use_rmcs_) return rmcs_model_.diagnostics();

  rmcs_ekf::Diagnostics result;
  if (ekf_.x.size() < 11 || ekf_.P.rows() < 11) return result;
  result.state = state();

  // legacy 状态顺序: [x, vx, y, vy, z, vz, a, w, r, l, h]
  // Diagnostics 顺序: [x, y, z, vx, vy, vz, a, w, rf, rl, hl]
  const auto & P = ekf_.P;
  result.variance[0] = P(0, 0);   // x
  result.variance[1] = P(2, 2);   // y
  result.variance[2] = P(4, 4);   // z
  result.variance[3] = P(1, 1);   // vx
  result.variance[4] = P(3, 3);   // vy
  result.variance[5] = P(5, 5);   // vz
  result.variance[6] = P(6, 6);   // a
  result.variance[7] = P(7, 7);   // w
  result.variance[8] = P(8, 8);   // rf
  result.variance[9] = P(8, 8) + P(9, 9) + 2.0 * P(8, 9);  // rl = r + l
  result.variance[10] = P(10, 10);  // hl

  result.covariance_xy << P(0, 0), P(0, 2), P(2, 0), P(2, 2);

  const auto data_or = [&](const char * key, double fallback = 0.0) {
    const auto it = ekf_.data.find(key);
    return it == ekf_.data.end() ? fallback : it->second;
  };

  result.nis = data_or("nis");
  result.nis_threshold = ekf_.nis_threshold;
  result.nis_fail_rate = data_or("recent_nis_failures");
  result.residual_norm = data_or("residual_norm");
  result.dt = last_dt_;
  result.update_count = static_cast<std::size_t>(update_count_);
  result.frame_update_count = 1;
  result.last_observed_id = last_id_;
  result.converged = convergened_legacy();
  result.diverged = diverged_legacy();
  result.last_update_accepted = ekf_.last_update_accepted;
  return result;
}

auto Target::diverged_legacy() const -> bool
{
  const auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  const auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

auto Target::convergened_legacy() const -> bool
{
  if (name != ArmorName::outpost && update_count_ > 3 && !diverged_legacy()) {
    is_converged_ = true;
  }
  return is_converged_;
}

auto Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const -> Eigen::Vector3d
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

  return { armor_x, armor_y, armor_z };
}

auto Target::h_jacobian(const Eigen::VectorXd & x, int id) const -> Eigen::MatrixXd
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

}  // namespace auto_aim
