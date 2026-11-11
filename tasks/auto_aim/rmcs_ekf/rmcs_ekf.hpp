#pragma once

// Port of rmcs_auto_aim_v2's 4-armor RobotModel EKF for sp_vision / DGE.
//
// Key differences from sp_vision's 11-dim EKF:
//   - State order: [x, y, z, vx, vy, vz, a, w, rf, rl, hl]
//   - Observation is per-lightbar pixel coordinates (4D per lightbar),
//     using the full camera projection model.
//   - Initialization uses one constrained IPPE pixel-pose solver.
//   - Convergence / divergence checks use posterior covariance and state bounds.
//
// The code is intentionally self-contained and C++17-only so it can live in
// DGE_vision_26 without pulling rmcs' utility tree.

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>
#include <limits>
#include <vector>
#include <string>

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
  if (!std::isfinite(angle)) return std::numeric_limits<double>::quiet_NaN();
  angle = std::remainder(angle, 2.0 * kPi);
  if (angle <= -kPi) angle += 2.0 * kPi;
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
// Shared constrained IPPE pose solver (4-armor model only)
// ---------------------------------------------------------------------------

struct RobustPnp
{
  CameraFeature camera;
  Armor2d armor;

  double noise_observation = 40.0;
  double max_rms_px = 8.0;
  auto solve(Armor3d & output, std::string * reason=nullptr, double * rms=nullptr) const -> bool;
};

// ---------------------------------------------------------------------------
// EKF configuration
// ---------------------------------------------------------------------------

struct Config
{
  // Process noise (diagonal Q, q*dt random walk; radii/height have no
  // velocity state, so they are driven directly by these rates).
  double noise_radius_forward = 1e-8;
  double noise_radius_lateral = 1e-8;
  double noise_height_lateral = 1e-8;

  // Observation noise (pixel^2)
  double noise_observation = 40.0;

  // Continuous white-noise acceleration (CWNA) intensity, exactly discretized:
  //   Q_pp = q*dt^3/3, Q_pv = q*dt^2/2, Q_vv = q*dt
  // q_accel_xy / q_accel_z in (m/s^2)^2*s, q_alpha in (rad/s^2)^2*s.
  double q_accel_xy = 1.0;
  double q_accel_z = 1e-4;
  double q_alpha = 10.0;

  // State validity bounds; a violating posterior is rejected without clipping.
  double radius_forward_min = 0.1;
  double radius_forward_max = 0.4;
  double radius_lateral_min = 0.1;
  double radius_lateral_max = 0.4;
  double height_lateral_min = -0.2;
  double height_lateral_max = +0.2;
  double velocity_min = -10.0;
  double velocity_max = +10.0;
  double rotation_speed_limit = 8.0 * kPi;  // |w| bound, rad/s; <= 0 disables

  // Visibility model (full plate < yaw_full, partial < yaw_part)
  double yaw_full_max = 70.0 * kPi / 180.0;
  double yaw_part_max = 90.0 * kPi / 180.0;

  // Shared pose quality
  double init_max_rms_px = 8.0;  // startup pixel RMS ceiling; validate against recorded data

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

  double association_miss_penalty = 30.0;
  double association_clutter_penalty = 30.0;
  double association_ambiguity_gap = 2.0;
  double nis_gate_warmup_s = 0.1;

  // Association / recovery
  // Gross-error gate on the anchor plate: mean corner error in pixels, and a
  // scale-aware floor relative to the predicted plate width. <= 0 disables.
  double anchor_max_corner_error = 40.0;
  double anchor_max_relative_error = 0.5;

  // Consistency monitoring (NIS, 4-D pixel innovation)
  // chi-square(4) 95% quantile = 9.488; 99% = 13.277.
  double nis_threshold = 9.488;
  int nis_window = 100;

  // Innovation gating is enabled by default.
  // When enabled, an update whose NIS exceeds nis_gate_threshold is rejected
  // instead of being allowed to pull the state. A threshold <= 0 falls back
  // to nis_threshold.
  bool nis_gate_enabled = true;
  double nis_gate_threshold = 13.277;
  int nis_gate_warmup_updates = 10;

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
// Fixed model endpoint-pair IDs; a slot is valid only for this correct() call.
struct PixelInnovation
{
  bool valid = false;
  std::array<double, 4> residual{};  // upper u/v, lower u/v, before this sequential update
  double nis = 0.0;
};

enum class RejectReason { InvalidObservation, DuplicateObservation, CandidateLimit,
  UnsupportedCamera, NoAssociation, AmbiguousAssociation, Projection, InnovationSystem,
  NisGate, StateBounds, Covariance, UnsupportedLightbar };
struct UpdateReport {
  std::array<int,4> matched_observation_by_plate{{-1,-1,-1,-1}};
  std::array<bool,4> matched_swapped_by_plate{};
  double association_ms=0, update_ms=0; // Performance telemetry only, never filter time.
  std::size_t detection_count=0, legal_armor_count=0, associated_armor_count=0;
  std::size_t nis_count=0, accepted_lightbar_count=0, rejected_lightbar_count=0;
  std::size_t accepted_frame_count=0, trimmed_count=0, duplicate_count=0;
  bool nis_valid=false, ambiguous=false, estimated_quality=false;
  double association_cost=0, association_second_cost=0;
  double prediction_rms=std::numeric_limits<double>::quiet_NaN();
  std::vector<RejectReason> reject_reasons;
};

struct Diagnostics
{
  std::array<PixelInnovation, 8> pixel_innovations{};
  State state;
  UpdateReport report;
  bool nis_valid = false;
  std::size_t nis_count = 0;
  Eigen::Matrix<double, 11, 1> variance{ Eigen::Matrix<double, 11, 1>::Zero() };
  Eigen::Matrix2d covariance_xy{ Eigen::Matrix2d::Zero() };

  double nis = 0.0;                // 当前帧单条最大 NIS；最大值不服从单条卡方分布
  double nis_threshold = 9.488;    // chi-square(4) 95%
  bool nis_gate_enabled = true;   // 是否启用 NIS 门控
  double nis_gate_threshold = 13.277; // 门控阈值；<=0 时使用 nis_threshold
  double nis_fail_rate = 0.0;      // 最近 nis_window 次更新的失败比例
  double nis_mean = 0.0;           // 当前帧 NIS 均值（仅参考，阈值判读用 nis）
  double anchor_error = 0.0;       // 当前帧 anchor 平均角点误差（像素）
  double residual_norm = 0.0;      // 当前帧新息向量平均范数（像素）
  double dt = 0.0;                 // 最近一次 predict 的 dt

  std::size_t update_count = 0;    // 累计至少接受一根灯条的帧数
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

  // Test/utility: initialize directly from a synthetic state without a camera.
  auto initialize_synthetic(const State & state) -> void;

  // Initialize from the validated shared pose; optionally verify its measured corners.
  // Useful when the detector's PnP is too noisy for the robust-PnP branch.
  auto initialize_from_pose(const CameraFeature & camera, const Eigen::Vector3d & translation,
    const Eigen::Quaterniond & orientation, bool large, const Armor2d * observation=nullptr) -> bool;

  auto predict(double dt) -> void;

  /// 只做状态外推，不修改协方差/时间/observable；dt 可以为负。
  auto extrapolate(double dt) const -> State;

  /// 返回一个“状态已外推”的轻量副本；协方差和观测缓存保持不变。
  /// 供 aimer / planner / Foxglove 做未来时刻查询。
  auto extrapolated(double dt) const -> RobotModel;

  auto correct(const std::vector<Armor2d> & armors, const std::vector<Lightbar2d> & lightbars,
    const CameraFeature & camera) -> UpdateReport;

  const Config & config() const { return config_; }
  bool is_large() const { return large_; }
  double elapsed() const { return elapsed_s_; }
  const StateVector & state_vector() const { return state_; }
  const UpdateReport & last_report() const { return report_; }
  bool import_snapshot(const StateVector & x, const Covariance & P, bool large, double elapsed=0);
  bool valid_state(const StateVector & x) const;
  static Covariance transition(double dt);
  ProcessNoise process_covariance(double dt) const;
  Covariance future_covariance(double dt) const;

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
  auto set_observation_noise() -> void;
  auto extrapolate_state(const StateVector & state, double dt) const -> StateVector;
  auto propagate_covariance(double dt) -> void;
  auto refresh_observable() const -> void;
  auto state_from_vector(const StateVector & state) const -> State;
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
  ObservationNoise observation_noise_{ ObservationNoise::Zero() };

  mutable Observable observable_;
  mutable bool observable_dirty_ = true;

  bool large_ = false;
  bool jumped_ = false;
  int last_observed_id_ = -1;
  std::size_t update_count_ = 0;
  double elapsed_s_ = 0.0;
  UpdateReport report_;

  std::array<PixelInnovation, 8> pixel_innovations_{};

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
