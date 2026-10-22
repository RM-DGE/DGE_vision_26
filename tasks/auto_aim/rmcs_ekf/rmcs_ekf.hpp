#pragma once

// Port of rmcs_auto_aim_v2's 4-armor RobotModel EKF for sp_vision / DGE.
//
// Key differences from sp_vision's 11-dim EKF:
//   - State order: [x, y, z, vx, vy, vz, a, w, rf, rl, hl]
//   - Observation is per-lightbar pixel coordinates (4D per lightbar),
//     using the full camera projection model.
//   - Initialization uses rmcs' robust IPPE + reprojection-yaw search.
//   - Convergence / divergence checks use posterior covariance and state bounds.
//
// The code is intentionally self-contained and C++17-only so it can live in
// DGE_vision_26 without pulling rmcs' utility tree.

#include <array>
#include <chrono>
#include <cstddef>
#include <deque>
#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <yaml-cpp/yaml.h>

namespace rmcs_ekf
{

constexpr double kPi = 3.14159265358979323846;

constexpr double kLightBarHeight = 0.056;
constexpr double kLargeArmorWidth = 0.230;
constexpr double kSmallArmorWidth = 0.135;

constexpr double kPredictedOtherArmorPitch = +15.0 * kPi / 180.0;
constexpr double kPredictedOutpostArmorPitch = -15.0 * kPi / 180.0;

// ---------------------------------------------------------------------------
// Basic types
// ---------------------------------------------------------------------------

struct CameraFeature
{
  // Row-major 3x3 camera intrinsic matrix.
  std::array<std::array<double, 3>, 3> camera_matrix{};
  // k1, k2, p1, p2, k3
  std::array<double, 5> distort_coeff{};

  // camera -> odom (ROS convention: x forward, y left, z up)
  Eigen::Quaterniond orientation{ Eigen::Quaterniond::Identity() };
  Eigen::Vector3d translation{ Eigen::Vector3d::Zero() };

  auto intrinsic() const -> cv::Mat;
  auto distortion() const -> cv::Mat;
};

struct Armor2d
{
  bool large = false;
  cv::Point2f tl{};
  cv::Point2f tr{};
  cv::Point2f br{};
  cv::Point2f bl{};
};

struct Lightbar2d
{
  Eigen::Vector2d upper{ Eigen::Vector2d::Zero() };
  Eigen::Vector2d lower{ Eigen::Vector2d::Zero() };
};

struct Lightbar3d
{
  Eigen::Vector3d upper{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d lower{ Eigen::Vector3d::Zero() };
};

struct Armor3d
{
  int id = -1;
  Eigen::Vector3d translation{ Eigen::Vector3d::Zero() };
  Eigen::Quaterniond orientation{ Eigen::Quaterniond::Identity() };
};

struct State
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;

  double rotation_angle = 0.0; // center -> front armor (rmcs convention)
  double rotation_speed = 0.0;

  double radius_forward = 0.2;
  double radius_lateral = 0.2;
  double height_lateral = 0.0;
};

// ---------------------------------------------------------------------------
// Math helpers (ported from rmcs utility/math)
// ---------------------------------------------------------------------------

inline auto normalize_angle(double angle) -> double
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle <= -kPi) angle += 2.0 * kPi;
  return angle;
}

inline auto opencv2ros_position(const Eigen::Vector3d & p) -> Eigen::Vector3d
{
  return Eigen::Vector3d{ p.z(), -p.x(), -p.y() };
}

inline auto opencv2ros_rotation(const Eigen::Matrix3d & r) -> Eigen::Matrix3d
{
  static const Eigen::Matrix3d kCoordTransform{
    { +0, +0, +1 },
    { -1, +0, +0 },
    { +0, -1, +0 },
  };
  return kCoordTransform * r * kCoordTransform.transpose();
}

inline auto ros2opencv_position(const Eigen::Vector3d & p) -> Eigen::Vector3d
{
  return Eigen::Vector3d{ -p.y(), -p.z(), p.x() };
}

inline auto ros2opencv_rotation(const Eigen::Matrix3d & r) -> Eigen::Matrix3d
{
  static const Eigen::Matrix3d kCoordTransform{
    { +0, +0, +1 },
    { -1, +0, +0 },
    { +0, -1, +0 },
  };
  return kCoordTransform.transpose() * r * kCoordTransform;
}

inline auto euler_to_quaternion(double yaw, double pitch, double roll) -> Eigen::Quaterniond
{
  return Eigen::Quaterniond{
    Eigen::AngleAxisd{ yaw, Eigen::Vector3d::UnitZ() }
    * Eigen::AngleAxisd{ pitch, Eigen::Vector3d::UnitY() }
    * Eigen::AngleAxisd{ roll, Eigen::Vector3d::UnitX() } };
}

inline auto eulers(const Eigen::Quaterniond & q, int axis0 = 2, int axis1 = 1, int axis2 = 0,
  bool extrinsic = false) -> Eigen::Vector3d
{
  if (!extrinsic) std::swap(axis0, axis2);

  auto i = axis0, j = axis1, k = axis2;
  auto is_proper = (i == k);
  if (is_proper) k = 3 - i - j;
  const auto sign = (i - j) * (j - k) * (k - i) / 2;

  double a, b, c, d;
  const Eigen::Vector4d xyzw = q.coeffs();
  if (is_proper) {
    a = xyzw[3];
    b = xyzw[i];
    c = xyzw[j];
    d = xyzw[k] * sign;
  } else {
    a = xyzw[3] - xyzw[j];
    b = xyzw[i] + xyzw[k] * sign;
    c = xyzw[j] + xyzw[3];
    d = xyzw[k] * sign - xyzw[i];
  }

  const auto n2 = a * a + b * b + c * c + d * d;
  Eigen::Vector3d result;
  result[1] = std::acos(2 * (a * a + b * b) / n2 - 1);

  const auto half_sum = std::atan2(b, a);
  const auto half_diff = std::atan2(-d, c);

  constexpr auto eps = 1e-7;
  const auto safe1 = std::abs(result[1]) >= eps;
  const auto safe2 = std::abs(result[1] - kPi) >= eps;
  const auto safe = safe1 && safe2;
  if (safe) {
    result[0] = half_sum + half_diff;
    result[2] = half_sum - half_diff;
  } else {
    if (!extrinsic) {
      result[0] = 0;
      if (!safe1) result[2] = 2 * half_sum;
      if (!safe2) result[2] = -2 * half_diff;
    } else {
      result[2] = 0;
      if (!safe1) result[0] = 2 * half_sum;
      if (!safe2) result[0] = 2 * half_diff;
    }
  }

  for (int n = 0; n < 3; ++n) result[n] = normalize_angle(result[n]);

  if (!is_proper) {
    result[2] *= sign;
    result[1] -= kPi / 2;
  }

  if (!extrinsic) std::swap(result[0], result[2]);
  return result;
}

// ---------------------------------------------------------------------------
// 4-armor geometry (port of rmcs RobotSolution)
// ---------------------------------------------------------------------------

struct RobotSolution
{
  struct Input
  {
    Eigen::Vector3d center{ Eigen::Vector3d::Zero() };
    Eigen::Quaterniond toward{ Eigen::Quaterniond::Identity() };
    double radius_forward = 0.2;
    double radius_lateral = 0.2;
    double height_lateral = 0.0;
    bool large = false;
  } input;

  auto solve_armors() const -> std::array<Armor3d, 4>;
  auto solve_lightbars() const -> std::array<Lightbar3d, 8>;
};

// ---------------------------------------------------------------------------
// Robust PnP (port of rmcs RobustPnpSolution, 4-armor only)
// ---------------------------------------------------------------------------

struct RobustPnp
{
  CameraFeature camera;
  Armor2d armor;

  bool fixed_outpost_pitch = false;
  bool fixed_normal_pitch = false;

  auto solve(Armor3d & output) const -> bool;
};

// ---------------------------------------------------------------------------
// EKF configuration
// ---------------------------------------------------------------------------

struct Config
{
  // Process noise (diagonal Q)
  double noise_x = 1e-4;
  double noise_y = 1e-4;
  double noise_z = 1e-8;

  double noise_vx = 2e-1;
  double noise_vy = 2e-1;
  double noise_vz = 1e-8;

  double noise_rotation_angle = 1e-3;
  double noise_rotation_speed = 1e0;

  double noise_radius_forward = 1e-8;
  double noise_radius_lateral = 1e-8;
  double noise_height_lateral = 1e-8;

  // Observation noise (pixel^2)
  double noise_observation = 40.0;

  // Rate-aware process noise. When true, the noise_* diagonal above is
  // interpreted as a per-second density and scaled by dt inside predict().
  // false reproduces rmcs' per-frame behavior (kept for A/B tests).
  bool q_rate_aware = true;

  // State clamps
  double radius_forward_min = 0.1;
  double radius_forward_max = 0.4;
  double radius_lateral_min = 0.1;
  double radius_lateral_max = 0.4;
  double height_lateral_min = -0.2;
  double height_lateral_max = +0.2;
  double velocity_min = -10.0;
  double velocity_max = +10.0;
  double rotation_speed_limit = 8.0 * kPi;  // |w| clamp, rad/s; <= 0 disables

  // Visibility model (full plate < yaw_full, partial < yaw_part)
  double yaw_full_max = 70.0 * kPi / 180.0;
  double yaw_part_max = 90.0 * kPi / 180.0;

  // PnP pitch prior
  bool fixed_outpost_pitch = false;
  bool fixed_normal_pitch = false;

  // Convergence
  int converge_min_update = 10;
  double converge_max_cov_xy = 0.04;   // x/y variance, m^2
  double converge_max_cov_v = 1.0;     // vx/vy variance, (m/s)^2
  double converge_max_cov_w = 1.0;     // w variance, (rad/s)^2
  int converge_max_no_update_frames = 5;
  double converge_delay_s = 0.1;

  // Divergence
  double diverge_cov = 150.0;
  double diverge_xy = 15.0;
  double diverge_z = 2.0;
  double diverge_w = 10.0 * kPi;
  double diverge_v = 5.0;
  double diverge_vz = 1.0;

  // Initial covariance diagonal (state order). The defaults are tighter than
  // rmcs' 64/64/0.4/100 because the state already comes from a solved PnP:
  // a loose position prior lets one noisy plate drag the center by meters.
  double initial_cov_position = 0.04;       // x, y (m^2)
  double initial_cov_z = 0.01;              // m^2
  double initial_cov_velocity = 4.0;        // vx, vy ((m/s)^2)
  double initial_cov_vz = 0.01;             // (m/s)^2
  double initial_cov_angle = 0.1;           // rad^2
  double initial_cov_rotation_speed = 4.0;  // (rad/s)^2
  double initial_cov_radius = 1e-3;         // rf, rl (m^2)
  double initial_cov_height = 1e-3;         // hl (m^2)

  // Association / recovery
  // Gross-error gate on the anchor plate: mean corner error in pixels, and a
  // scale-aware floor relative to the predicted plate width. <= 0 disables.
  double anchor_max_corner_error = 40.0;
  double anchor_max_relative_error = 0.5;

  // If detections keep arriving but no lightbar is accepted for this many
  // frames, re-initialize from the current detections (0 disables).
  int auto_reinit_after_no_update = 10;

  // Hard divergence: too many frames without an accepted update (0 disables).
  int max_no_update_frames = 30;

  // Consistency monitoring (NIS, 4-D pixel innovation)
  // chi-square(4) 95% quantile = 9.488; 99% = 13.277.
  double nis_threshold = 9.488;
  int nis_window = 100;

  // Innovation gating. Disabled by default to preserve rmcs behavior.
  // When enabled, an update whose NIS exceeds nis_gate_threshold is rejected
  // instead of being allowed to pull the state. A threshold <= 0 falls back
  // to nis_threshold.
  bool nis_gate_enabled = false;
  double nis_gate_threshold = 0.0;
  int nis_gate_warmup_updates = 10;
  int nis_max_consecutive_rejects = 5;
  // Added to the position/velocity/angle diagonal after too many consecutive
  // rejections, so the filter can re-open its gate instead of staying stuck.
  double nis_reject_cov_inflation = 0.01;

  // ---- v0.32 观测侧开关（全部可在 YAML 关闭做 A/B）----
  // 初始化优先使用 sp_vision Solver 已解算的世界系板位姿（与 DGE 外参一致）；
  // false = 优先 RobustPnp（v0.3 行为）。
  bool init_prefer_solver_pose = true;
  // 跟踪阶段 4 板目标跳过逐板 Solver::solve（像素 EKF 不需要）。会让 Foxglove
  // detections 方块失去世界系位姿，默认 false。
  bool skip_tracking_pnp = false;
};

auto load_config(const YAML::Node & node) -> Config;

// ---------------------------------------------------------------------------
// 11-dim EKF
// ---------------------------------------------------------------------------

/// 滤波器健康状态，供 Foxglove / PlotJuggler / 控制台使用。
/// variance 的顺序与 StateVector 一致：
/// [x, y, z, vx, vy, vz, a, w, rf, rl, hl]
struct Diagnostics
{
  State state;
  Eigen::Matrix<double, 11, 1> variance{ Eigen::Matrix<double, 11, 1>::Zero() };
  Eigen::Matrix2d covariance_xy{ Eigen::Matrix2d::Zero() };

  double nis = 0.0;                // 当前帧单条最大 NIS（4 维；与 9.488 同口径）
  double nis_threshold = 9.488;    // chi-square(4) 95%
  bool nis_gate_enabled = false;   // 是否启用 NIS 门控
  double nis_gate_threshold = 0.0; // 门控阈值；<=0 时使用 nis_threshold
  double nis_fail_rate = 0.0;      // 最近 nis_window 次更新的失败比例
  double nis_mean = 0.0;           // 当前帧 NIS 均值（仅参考，阈值判读用 nis）
  double anchor_error = 0.0;       // 当前帧 anchor 平均角点误差（像素）
  double residual_norm = 0.0;      // 当前帧新息向量平均范数（像素）
  double dt = 0.0;                 // 最近一次 predict 的 dt

  std::size_t update_count = 0;    // 累计 correct 帧数
  std::size_t frame_update_count = 0;  // 当前帧被接受的灯条数
  std::size_t frame_rejected_count = 0;  // 当前帧因 NIS/投影无效被拒绝的灯条数
  std::size_t nis_rejected_count = 0;    // 累计 NIS 拒绝次数
  int nis_consecutive_rejects = 0;       // 连续 NIS 拒绝次数
  int no_update_frames = 0;              // 连续“有检测但 0 次接受更新”的帧数
  int reinit_count = 0;                  // 累计自动重初始化次数
  int last_observed_id = -1;
  bool converged = false;
  bool diverged = false;
  bool last_update_accepted = false;
};

class RobotModel
{
public:
  using StateVector = Eigen::Matrix<double, 11, 1>;
  using Covariance = Eigen::Matrix<double, 11, 11>;
  using ProcessNoise = Eigen::Matrix<double, 11, 11>;
  using ObservationNoise = Eigen::Matrix<double, 4, 4>;
  using KalmanGain = Eigen::Matrix<double, 11, 4>;

  static constexpr int kStateX = 0;
  static constexpr int kStateY = 1;
  static constexpr int kStateZ = 2;
  static constexpr int kStateVx = 3;
  static constexpr int kStateVy = 4;
  static constexpr int kStateVz = 5;
  static constexpr int kStateA = 6;
  static constexpr int kStateW = 7;
  static constexpr int kStateRF = 8;
  static constexpr int kStateRL = 9;
  static constexpr int kStateHL = 10;

  explicit RobotModel(const Config & config = Config{});

  auto configure(const Config & config) -> void;

  auto initialize(const CameraFeature & camera, const std::vector<Armor2d> & armors) -> bool;

  // Fallback: initialize from an already-solved armor pose (sp_vision Solver).
  // Useful when the detector's PnP is too noisy for the robust-PnP branch.
  auto initialize_from_pose(const CameraFeature & camera, const Eigen::Vector3d & translation,
    const Eigen::Quaterniond & orientation, bool large) -> bool;

  auto predict(double dt) -> void;

  auto correct(const std::vector<Armor2d> & armors, const std::vector<Lightbar2d> & lightbars,
    const CameraFeature & camera) -> void;

  auto state() const -> State;
  auto full() const -> std::array<Armor3d, 4>;

  auto converge() const -> bool;
  auto diverged() const -> bool;

  auto jumped() const -> bool { return jumped_; }
  auto last_observed_id() const -> int { return last_observed_id_; }
  auto update_count() const -> std::size_t { return update_count_; }

  /// 后验协方差（外部只读；用于可视化和一致性分析）。
  auto covariance() const -> const Covariance & { return covariance_; }

  /// 一次性取回状态 + 协方差 + NIS + 收敛/发散标志。
  auto diagnostics() const -> Diagnostics;

private:
  struct Observable
  {
    CameraFeature camera;
    bool large = false;

    std::array<Eigen::Vector2d, 8> upper2d{};
    std::array<Eigen::Vector2d, 8> lower2d{};
    std::array<Eigen::Vector3d, 8> upper3d{};
    std::array<Eigen::Vector3d, 8> lower3d{};
    std::array<bool, 8> visible{};
    // Projection validity: a lightbar can be geometrically "visible" but its
    // endpoints may be behind the camera or fail projectPoints. Such a
    // measurement must not be fed into the EKF as (0, 0).
    std::array<bool, 8> upper_valid{};
    std::array<bool, 8> lower_valid{};

    double yaw_full_max = 70.0 * kPi / 180.0;
    double yaw_part_max = 90.0 * kPi / 180.0;

    auto update(const StateVector & state) -> void;
    auto visible_armors() const -> std::size_t;
  };

  auto reset_covariance() -> void;
  auto set_noise() -> void;
  auto finish_initialize(
    const CameraFeature & camera, const Eigen::Vector3d & center, double yaw, bool large) -> bool;
  auto maybe_reinitialize(const CameraFeature & camera, const std::vector<Armor2d> & armors)
    -> bool;

  auto make_observation_jacobian(int lightbar_id) const -> Eigen::Matrix<double, 4, 11>;
  auto update_one(const Eigen::Vector2d & upper, const Eigen::Vector2d & lower, int lightbar_id)
    -> void;

  Config config_;
  StateVector state_{ StateVector::Zero() };
  Covariance covariance_{ Covariance::Identity() };
  ProcessNoise process_noise_{ ProcessNoise::Zero() };
  ObservationNoise observation_noise_{ ObservationNoise::Zero() };

  Observable observable_;

  bool large_ = false;
  bool jumped_ = false;
  int last_observed_id_ = -1;
  std::size_t update_count_ = 0;
  std::chrono::steady_clock::time_point init_timestamp_{ std::chrono::steady_clock::now() };

  // Consistency diagnostics
  double last_dt_ = 0.0;
  double last_nis_ = 0.0;
  double last_residual_norm_ = 0.0;
  double last_frame_nis_ = 0.0;
  double last_frame_nis_mean_ = 0.0;
  double last_frame_residual_norm_ = 0.0;
  std::size_t last_frame_update_count_ = 0;
  std::size_t last_frame_rejected_count_ = 0;
  bool last_update_accepted_ = false;

  double frame_nis_sum_ = 0.0;
  double frame_nis_max_ = 0.0;
  double frame_residual_sum_ = 0.0;
  std::size_t frame_nis_count_ = 0;
  std::size_t frame_update_count_ = 0;
  std::size_t frame_rejected_count_ = 0;

  std::size_t nis_rejected_count_ = 0;
  int consecutive_nis_rejects_ = 0;

  // Association / recovery diagnostics
  double last_anchor_error_ = 0.0;
  int no_update_frames_ = 0;
  int reinit_count_ = 0;

  std::deque<int> recent_nis_failures_;
};

} // namespace rmcs_ekf
