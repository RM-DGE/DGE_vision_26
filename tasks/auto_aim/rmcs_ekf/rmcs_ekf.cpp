#include "rmcs_ekf.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

#include <opencv2/core/eigen.hpp>

#include "tools/logger.hpp"

namespace rmcs_ekf
{

namespace
{

constexpr double kEpsilon = 1e-9;

template <typename T>
auto read_or(const YAML::Node & node, const char * key, T fallback) -> T
{
  if (node && node[key]) return node[key].as<T>();
  return fallback;
}

auto squared_distance(const Eigen::Vector2d & a, const Eigen::Vector2d & b) -> double
{
  const auto dx = a.x() - b.x();
  const auto dy = a.y() - b.y();
  return dx * dx + dy * dy;
}

auto armor_shape_points(bool large) -> std::array<cv::Point3f, 4>
{
  const auto width = large ? kLargeArmorWidth : kSmallArmorWidth;
  const auto half_w = static_cast<float>(0.5 * width);
  const auto half_h = static_cast<float>(0.5 * kLightBarHeight);
  return { {
    cv::Point3f{ -half_w, -half_h, 0.0F }, // tl
    cv::Point3f{ -half_w, +half_h, 0.0F }, // bl
    cv::Point3f{ +half_w, +half_h, 0.0F }, // br
    cv::Point3f{ +half_w, -half_h, 0.0F }, // tr
  } };
}

auto project_camera_point(const Eigen::Vector3d & point_camera_opencv,
  const cv::Mat & intrinsic, const cv::Mat & distortion) -> std::optional<Eigen::Vector2d>
{
  const std::vector<cv::Point3f> object_points{
    cv::Point3f{ static_cast<float>(point_camera_opencv.x()),
      static_cast<float>(point_camera_opencv.y()),
      static_cast<float>(point_camera_opencv.z()) },
  };
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(object_points, cv::Vec3d{ 0, 0, 0 }, cv::Vec3d{ 0, 0, 0 }, intrinsic,
    distortion, image_points);
  if (image_points.empty()) return std::nullopt;
  return Eigen::Vector2d{ image_points[0].x, image_points[0].y };
}

struct ArmorPose
{
  Eigen::Vector3d translation{ Eigen::Vector3d::Zero() };
  Eigen::Quaterniond orientation{ Eigen::Quaterniond::Identity() };
};

auto compute_armor_poses(const RobotSolution::Input & input) -> std::array<ArmorPose, 4>
{
  const auto q = input.toward;
  const auto c = input.center;

  const auto px_dir = Eigen::Vector3d{ q * Eigen::Vector3d::UnitX() };
  const auto py_dir = Eigen::Vector3d{ q * Eigen::Vector3d::UnitY() };

  const auto pitch = kPredictedOtherArmorPitch;
  const auto make_orientation = [&q, pitch](double delta) {
    return Eigen::Quaterniond{ Eigen::AngleAxisd{ delta, Eigen::Vector3d::UnitZ() } * q
      * Eigen::AngleAxisd{ pitch, Eigen::Vector3d::UnitY() } };
  };

  const auto fr = input.radius_forward;
  const auto lr = input.radius_lateral;
  const auto lh = input.height_lateral;
  const auto vz = Eigen::Vector3d::UnitZ();

  return { {
    ArmorPose{ c + fr * px_dir, make_orientation(kPi) },
    ArmorPose{ c + lr * py_dir + vz * lh, make_orientation(-0.5 * kPi) },
    ArmorPose{ c - fr * px_dir, make_orientation(0.0) },
    ArmorPose{ c - lr * py_dir + vz * lh, make_orientation(0.5 * kPi) },
  } };
}

} // namespace

// ---------------------------------------------------------------------------
// CameraFeature
// ---------------------------------------------------------------------------

auto CameraFeature::intrinsic() const -> cv::Mat
{
  cv::Mat matrix(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      matrix.at<double>(row, col) = camera_matrix[row][col];
    }
  }
  return matrix;
}

auto CameraFeature::distortion() const -> cv::Mat
{
  cv::Mat matrix(1, 5, CV_64F);
  for (int col = 0; col < 5; ++col) {
    matrix.at<double>(0, col) = distort_coeff[col];
  }
  return matrix;
}

// ---------------------------------------------------------------------------
// RobotSolution
// ---------------------------------------------------------------------------

auto RobotSolution::solve_armors() const -> std::array<Armor3d, 4>
{
  const auto poses = compute_armor_poses(input);

  return { {
    Armor3d{ 0, poses[0].translation, poses[0].orientation },
    Armor3d{ 1, poses[1].translation, poses[1].orientation },
    Armor3d{ 2, poses[2].translation, poses[2].orientation },
    Armor3d{ 3, poses[3].translation, poses[3].orientation },
  } };
}

auto RobotSolution::solve_lightbars() const -> std::array<Lightbar3d, 8>
{
  const auto poses = compute_armor_poses(input);

  const auto half_w = (input.large ? kLargeArmorWidth : kSmallArmorWidth) * 0.5;
  const auto half_h = kLightBarHeight * 0.5;

  std::array<Lightbar3d, 8> result{};
  for (int i = 0; i < 4; ++i) {
    const auto y_axis = Eigen::Vector3d{ poses[i].orientation * Eigen::Vector3d::UnitY() };
    const auto z_axis = Eigen::Vector3d{ poses[i].orientation * Eigen::Vector3d::UnitZ() };

    const auto center = poses[i].translation;
    const auto even_point = center + y_axis * half_w;
    const auto odd_point = center - y_axis * half_w;

    result[i * 2 + 0] = Lightbar3d{ even_point + z_axis * half_h, even_point - z_axis * half_h };
    result[i * 2 + 1] = Lightbar3d{ odd_point + z_axis * half_h, odd_point - z_axis * half_h };
  }
  return result;
}

// ---------------------------------------------------------------------------
// RobustPnP
// ---------------------------------------------------------------------------

auto RobustPnp::solve(Armor3d & output) const -> bool
{
  const auto intrinsic = camera.intrinsic();
  const auto distortion = camera.distortion();

  const auto shape = armor_shape_points(armor.large);
  const std::vector<cv::Point3f> object_points{ shape.begin(), shape.end() };
  const std::vector<cv::Point2f> image_points{ armor.tl, armor.bl, armor.br, armor.tr };

  std::vector<cv::Vec3d> rotation_vectors;
  std::vector<cv::Vec3d> translation_vectors;
  std::vector<double> errors;

  const auto success = cv::solvePnPGeneric(object_points, image_points, intrinsic, distortion,
    rotation_vectors, translation_vectors, false, cv::SOLVEPNP_IPPE, cv::noArray(),
    cv::noArray(), errors);

  if (!success || rotation_vectors.empty() || rotation_vectors.size() != translation_vectors.size())
    return false;

  const auto q_odom_camera = camera.orientation;
  const auto t_odom_camera = camera.translation;

  std::optional<Armor3d> best_armor;
  auto best_error = std::numeric_limits<double>::max();

  for (std::size_t index = 0; index < rotation_vectors.size(); ++index) {
    if (translation_vectors[index][2] <= 0.0) continue;

    cv::Mat rotation_cv;
    cv::Rodrigues(rotation_vectors[index], rotation_cv);

    Eigen::Matrix3d rotation_opencv{};
    cv::cv2eigen(rotation_cv, rotation_opencv);

    Eigen::Vector3d translation_opencv{};
    cv::cv2eigen(translation_vectors[index], translation_opencv);

    const auto q_camera_armor =
      Eigen::Quaterniond{ opencv2ros_rotation(rotation_opencv) }.normalized();
    const auto t_camera_armor = opencv2ros_position(translation_opencv);

    const auto q_odom_armor = Eigen::Quaterniond{ q_odom_camera * q_camera_armor }.normalized();
    const auto t_odom_armor = Eigen::Vector3d{ q_odom_camera * t_camera_armor + t_odom_camera };

    // Reject armor plates facing away from camera.
    const auto back_direction = q_camera_armor * Eigen::Vector3d::UnitX();
    const auto camera_to_armor = t_camera_armor.normalized();
    if (back_direction.dot(camera_to_armor) <= 0.0) continue;

    // Reject implausible pitch.
    const auto expected_pitch = kPredictedOtherArmorPitch;
    const auto pitch = eulers(q_odom_armor, 2, 1, 0)[1];
    if (std::abs(pitch - expected_pitch) > 40.0 * kPi / 180.0) continue;

    const auto error = index < errors.size() ? errors[index] : 0.0;
    if (error >= best_error) continue;

    best_error = error;
    best_armor = Armor3d{ 0, t_odom_armor, q_odom_armor };
  }

  if (!best_armor) return false;

  // Reprojection yaw search, matching rmcs RobustPnpSolution.
  constexpr auto kYawOptimizeRangeDeg = 60.0;
  constexpr auto kYawOptimizeStepDeg = 1.0;

  const auto ypr = eulers(best_armor->orientation, 2, 1, 0);
  const auto origin_yaw = ypr[0];
  auto origin_pitch = ypr[1];

  if (fixed_normal_pitch) origin_pitch = kPredictedOtherArmorPitch;
  if (fixed_outpost_pitch) origin_pitch = kPredictedOutpostArmorPitch;

  const auto t_odom_armor = best_armor->translation;
  const auto q_camera_odom = q_odom_camera.conjugate();

  const auto evaluate = [&](double yaw, double & error) -> bool {
    const auto q_odom_armor = euler_to_quaternion(yaw, origin_pitch, 0.0).normalized();
    const auto q_camera_armor = Eigen::Quaterniond{ q_camera_odom * q_odom_armor }.normalized();
    const auto t_camera_armor = Eigen::Vector3d{ q_camera_odom * (t_odom_armor - t_odom_camera) };

    const Eigen::Matrix3d rotation_opencv = ros2opencv_rotation(q_camera_armor.toRotationMatrix());
    const auto translation_opencv = ros2opencv_position(t_camera_armor);

    error = 0.0;
    for (int point_index = 0; point_index < 4; ++point_index) {
      const auto object = shape[point_index];
      // v0.32: 必须求值成具体向量。`const auto` 会得到 Eigen 表达式模板，
      // 引用已析构的临时 Vector3d（未定义行为）。实测 -O2 下每个候选 yaw 的误差都
      // 相同，搜索总停在窗口左边界：初始化 yaw 恒偏 -30°、中心偏 10.35 cm。
      const Eigen::Vector3d object_point{ object.x, object.y, object.z };
      const Eigen::Vector3d point = rotation_opencv * object_point + translation_opencv;

      const auto projected = project_camera_point(point, intrinsic, distortion);
      if (!projected) return false;

      const auto detected = Eigen::Vector2d{ image_points[point_index].x,
        image_points[point_index].y };
      error += (projected->head<2>() - detected).norm();
    }
    return true;
  };

  const auto half_range = kYawOptimizeRangeDeg * 0.5 * kPi / 180.0;
  const auto step = kYawOptimizeStepDeg * kPi / 180.0;
  const auto step_count = static_cast<int>(std::round((half_range * 2.0) / step));

  auto optimized_yaw = origin_yaw;
  auto optimized_error = std::numeric_limits<double>::max();

  for (int index = 0; index <= step_count; ++index) {
    const auto yaw = origin_yaw - half_range + index * step;
    double error = 0.0;
    if (!evaluate(yaw, error)) continue;
    if (error < optimized_error) {
      optimized_error = error;
      optimized_yaw = yaw;
    }
  }

  double error_lower = 0.0;
  double error_upper = 0.0;
  if (evaluate(optimized_yaw - step, error_lower)
    && evaluate(optimized_yaw + step, error_upper)) {
    const auto denominator =
      error_lower - 2.0 * optimized_error + error_upper + kEpsilon;
    const auto interpolated_yaw =
      optimized_yaw + step * 0.5 * (error_lower - error_upper) / denominator;

    double interpolated_error = 0.0;
    if (evaluate(interpolated_yaw, interpolated_error)
      && interpolated_error < optimized_error) {
      optimized_yaw = interpolated_yaw;
      optimized_error = interpolated_error;
    }
  }

  output = *best_armor;
  output.orientation = euler_to_quaternion(optimized_yaw, origin_pitch, 0.0).normalized();
  return true;
}

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

auto load_config(const YAML::Node & node) -> Config
{
  Config config;

  config.noise_x = read_or(node, "noise_x", config.noise_x);
  config.noise_y = read_or(node, "noise_y", config.noise_y);
  config.noise_z = read_or(node, "noise_z", config.noise_z);

  config.noise_vx = read_or(node, "noise_vx", config.noise_vx);
  config.noise_vy = read_or(node, "noise_vy", config.noise_vy);
  config.noise_vz = read_or(node, "noise_vz", config.noise_vz);

  config.noise_rotation_angle = read_or(node, "noise_rotation_angle", config.noise_rotation_angle);
  config.noise_rotation_speed = read_or(node, "noise_rotation_speed", config.noise_rotation_speed);

  config.noise_radius_forward = read_or(node, "noise_radius_forward", config.noise_radius_forward);
  config.noise_radius_lateral = read_or(node, "noise_radius_lateral", config.noise_radius_lateral);
  config.noise_height_lateral = read_or(node, "noise_height_lateral", config.noise_height_lateral);

  config.noise_observation = read_or(node, "noise_observation", config.noise_observation);
  config.q_rate_aware = read_or(node, "q_rate_aware", config.q_rate_aware);

  config.radius_forward_min = read_or(node, "radius_forward_min", config.radius_forward_min);
  config.radius_forward_max = read_or(node, "radius_forward_max", config.radius_forward_max);
  config.radius_lateral_min = read_or(node, "radius_lateral_min", config.radius_lateral_min);
  config.radius_lateral_max = read_or(node, "radius_lateral_max", config.radius_lateral_max);
  config.height_lateral_min = read_or(node, "height_lateral_min", config.height_lateral_min);
  config.height_lateral_max = read_or(node, "height_lateral_max", config.height_lateral_max);

  config.velocity_min = read_or(node, "velocity_min", config.velocity_min);
  config.velocity_max = read_or(node, "velocity_max", config.velocity_max);
  config.rotation_speed_limit =
    read_or(node, "rotation_speed_limit", config.rotation_speed_limit);

  config.yaw_full_max = read_or(node, "yaw_full_max_deg", 70.0) * kPi / 180.0;
  config.yaw_part_max = read_or(node, "yaw_part_max_deg", 90.0) * kPi / 180.0;

  config.fixed_outpost_pitch = read_or(node, "fixed_outpost_pitch", config.fixed_outpost_pitch);
  config.fixed_normal_pitch = read_or(node, "fixed_normal_pitch", config.fixed_normal_pitch);

  config.converge_min_update = read_or(node, "converge_min_update", config.converge_min_update);
  config.converge_max_cov_xy = read_or(node, "converge_max_cov_xy", config.converge_max_cov_xy);
  config.converge_max_cov_v = read_or(node, "converge_max_cov_v", config.converge_max_cov_v);
  config.converge_max_cov_w = read_or(node, "converge_max_cov_w", config.converge_max_cov_w);
  config.converge_max_no_update_frames = read_or(
    node, "converge_max_no_update_frames", config.converge_max_no_update_frames);
  config.converge_delay_s = read_or(node, "converge_delay_s", config.converge_delay_s);

  config.diverge_cov = read_or(node, "diverge_cov", config.diverge_cov);
  config.diverge_xy = read_or(node, "diverge_xy", config.diverge_xy);
  config.diverge_z = read_or(node, "diverge_z", config.diverge_z);
  config.diverge_w = read_or(node, "diverge_w", config.diverge_w);
  config.diverge_v = read_or(node, "diverge_v", config.diverge_v);
  config.diverge_vz = read_or(node, "diverge_vz", config.diverge_vz);

  config.initial_cov_position = read_or(node, "initial_cov_position", config.initial_cov_position);
  config.initial_cov_z = read_or(node, "initial_cov_z", config.initial_cov_z);
  config.initial_cov_velocity = read_or(node, "initial_cov_velocity", config.initial_cov_velocity);
  config.initial_cov_vz = read_or(node, "initial_cov_vz", config.initial_cov_vz);
  config.initial_cov_angle = read_or(node, "initial_cov_angle", config.initial_cov_angle);
  config.initial_cov_rotation_speed =
    read_or(node, "initial_cov_rotation_speed", config.initial_cov_rotation_speed);
  config.initial_cov_radius = read_or(node, "initial_cov_radius", config.initial_cov_radius);
  config.initial_cov_height = read_or(node, "initial_cov_height", config.initial_cov_height);

  config.anchor_max_corner_error =
    read_or(node, "anchor_max_corner_error", config.anchor_max_corner_error);
  config.anchor_max_relative_error =
    read_or(node, "anchor_max_relative_error", config.anchor_max_relative_error);
  config.auto_reinit_after_no_update =
    read_or(node, "auto_reinit_after_no_update", config.auto_reinit_after_no_update);
  config.max_no_update_frames =
    read_or(node, "max_no_update_frames", config.max_no_update_frames);

  config.nis_threshold = read_or(node, "nis_threshold", config.nis_threshold);
  config.nis_window = read_or(node, "nis_window", config.nis_window);

  config.nis_gate_enabled = read_or(node, "nis_gate_enabled", config.nis_gate_enabled);
  config.nis_gate_threshold = read_or(node, "nis_gate_threshold", config.nis_gate_threshold);
  config.nis_gate_warmup_updates =
    read_or(node, "nis_gate_warmup_updates", config.nis_gate_warmup_updates);
  config.nis_max_consecutive_rejects =
    read_or(node, "nis_max_consecutive_rejects", config.nis_max_consecutive_rejects);
  config.nis_reject_cov_inflation =
    read_or(node, "nis_reject_cov_inflation", config.nis_reject_cov_inflation);

  config.init_prefer_solver_pose =
    read_or(node, "init_prefer_solver_pose", config.init_prefer_solver_pose);
  config.skip_tracking_pnp = read_or(node, "skip_tracking_pnp", config.skip_tracking_pnp);

  return config;
}

// ---------------------------------------------------------------------------
// RobotModel
// ---------------------------------------------------------------------------

RobotModel::RobotModel(const Config & config)
{
  configure(config);
  reset_covariance();
}

auto RobotModel::configure(const Config & config) -> void
{
  config_ = config;
  observable_.yaw_full_max = config_.yaw_full_max;
  observable_.yaw_part_max = config_.yaw_part_max;
  set_noise();
}

auto RobotModel::set_noise() -> void
{
  process_noise_.setZero();
  process_noise_.diagonal() << config_.noise_x, config_.noise_y, config_.noise_z,
    config_.noise_vx, config_.noise_vy, config_.noise_vz,
    config_.noise_rotation_angle, config_.noise_rotation_speed,
    config_.noise_radius_forward, config_.noise_radius_lateral, config_.noise_height_lateral;

  observation_noise_.setZero();
  observation_noise_.diagonal() << config_.noise_observation, config_.noise_observation,
    config_.noise_observation, config_.noise_observation;
}

auto RobotModel::reset_covariance() -> void
{
  StateVector diagonal;
  diagonal << config_.initial_cov_position, config_.initial_cov_position, config_.initial_cov_z,
    config_.initial_cov_velocity, config_.initial_cov_velocity, config_.initial_cov_vz,
    config_.initial_cov_angle, config_.initial_cov_rotation_speed,
    config_.initial_cov_radius, config_.initial_cov_radius, config_.initial_cov_height;
  // Never allow a zero variance: it would make the covariance singular.
  for (Eigen::Index index = 0; index < diagonal.size(); ++index) {
    diagonal[index] = std::max(diagonal[index], 1e-12);
  }
  covariance_ = diagonal.asDiagonal();
}

auto RobotModel::initialize(const CameraFeature & camera, const std::vector<Armor2d> & armors)
  -> bool
{
  if (armors.empty()) return false;

  const Armor2d * best_armor = nullptr;
  auto best_width = 0.0;
  for (const auto & armor : armors) {
    const auto width = std::abs(armor.tr.x - armor.tl.x);
    if (width <= best_width) continue;
    best_width = width;
    best_armor = &armor;
  }
  if (!best_armor) return false;

  large_ = best_armor->large;

  RobustPnp pnp;
  pnp.camera = camera;
  pnp.armor = *best_armor;
  pnp.fixed_outpost_pitch = config_.fixed_outpost_pitch;
  pnp.fixed_normal_pitch = config_.fixed_normal_pitch;

  Armor3d armor3d;
  if (!pnp.solve(armor3d)) return false;

  const auto orientation = armor3d.orientation;
  const auto armor_to_center = Eigen::Vector3d{
    Eigen::AngleAxisd{ -kPredictedOtherArmorPitch, orientation * Eigen::Vector3d::UnitY() }
    * (orientation * Eigen::Vector3d::UnitX()) };

  const auto yaw = normalize_angle(std::atan2(-armor_to_center.y(), -armor_to_center.x()));
  const auto center = Eigen::Vector3d{ armor3d.translation + 0.2 * armor_to_center };

  return finish_initialize(camera, center, yaw, large_);
}

auto RobotModel::initialize_from_pose(const CameraFeature & camera,
  const Eigen::Vector3d & translation, const Eigen::Quaterniond & orientation, bool large) -> bool
{
  large_ = large;

  const auto armor_to_center = Eigen::Vector3d{
    Eigen::AngleAxisd{ -kPredictedOtherArmorPitch, orientation * Eigen::Vector3d::UnitY() }
    * (orientation * Eigen::Vector3d::UnitX()) };

  const auto yaw = normalize_angle(std::atan2(-armor_to_center.y(), -armor_to_center.x()));
  const auto center = Eigen::Vector3d{ translation + 0.2 * armor_to_center };

  return finish_initialize(camera, center, yaw, large);
}

auto RobotModel::finish_initialize(
    const CameraFeature & camera, const Eigen::Vector3d & center, double yaw, bool large) -> bool
{
  state_.setZero();
  state_[kStateX] = center.x();
  state_[kStateY] = center.y();
  state_[kStateZ] = center.z();
  state_[kStateRF] = 0.2;
  state_[kStateRL] = 0.2;
  state_[kStateHL] = 0.0;

  observable_.camera = camera;
  observable_.large = large;

  // Verify the PnP yaw by counting visible armor plates.
  state_[kStateA] = yaw;
  observable_.update(state_);
  const auto original_count = observable_.visible_armors();

  const auto alternate_yaw = normalize_angle(yaw + kPi);
  state_[kStateA] = alternate_yaw;
  observable_.update(state_);
  const auto alternate_count = observable_.visible_armors();

  if (alternate_count > original_count) yaw = alternate_yaw;

  state_[kStateA] = yaw;
  observable_.update(state_);

  set_noise();
  reset_covariance();

  init_timestamp_ = std::chrono::steady_clock::now();
  jumped_ = false;
  last_observed_id_ = -1;
  update_count_ = 0;
  last_frame_update_count_ = 0;
  last_frame_rejected_count_ = 0;
  frame_update_count_ = 0;
  frame_rejected_count_ = 0;
  nis_rejected_count_ = 0;
  consecutive_nis_rejects_ = 0;
  recent_nis_failures_.clear();

  last_frame_nis_ = 0.0;
  last_frame_nis_mean_ = 0.0;
  last_frame_residual_norm_ = 0.0;
  frame_nis_max_ = 0.0;
  frame_nis_sum_ = 0.0;
  frame_nis_count_ = 0;
  last_anchor_error_ = 0.0;
  no_update_frames_ = 0;
  return true;
}

auto RobotModel::predict(double dt) -> void
{
  // Out-of-order or non-finite timestamps must not poison the filter.
  if (!std::isfinite(dt) || dt <= 0.0) {
    last_dt_ = 0.0;
    return;
  }

  last_dt_ = dt;
  auto next = state_;
  next[kStateX] += next[kStateVx] * dt;
  next[kStateY] += next[kStateVy] * dt;
  next[kStateZ] += next[kStateVz] * dt;
  next[kStateA] = normalize_angle(next[kStateA] + next[kStateW] * dt);
  if (config_.rotation_speed_limit > 0.0) {
    next[kStateW] = std::clamp(
      next[kStateW], -config_.rotation_speed_limit, config_.rotation_speed_limit);
  }

  Covariance jacobian = Covariance::Identity();
  jacobian(kStateX, kStateVx) = dt;
  jacobian(kStateY, kStateVy) = dt;
  jacobian(kStateZ, kStateVz) = dt;
  jacobian(kStateA, kStateW) = dt;

  state_ = next;

  // rmcs adds the same Q every predict (per-frame noise). DGE runs at a
  // different frame rate, so q_rate_aware interprets the configured diagonal
  // as a per-second density and scales it by dt. Set q_rate_aware=false to
  // reproduce the original per-frame behavior for A/B tests.
  auto q = process_noise_;
  if (config_.q_rate_aware) q *= dt;
  covariance_ = jacobian * covariance_ * jacobian.transpose() + q;

  observable_.update(state_);
}

auto RobotModel::correct(const std::vector<Armor2d> & armors,
  const std::vector<Lightbar2d> & lightbars, const CameraFeature & camera) -> void
{
  observable_.camera = camera;
  observable_.large = large_;
  observable_.update(state_);

  frame_nis_sum_ = 0.0;
  frame_nis_max_ = 0.0;
  frame_residual_sum_ = 0.0;
  frame_nis_count_ = 0;
  frame_update_count_ = 0;
  frame_rejected_count_ = 0;
  last_frame_update_count_ = 0;
  last_frame_rejected_count_ = 0;
  last_anchor_error_ = 0.0;
  last_update_accepted_ = false;

  if (armors.empty() && lightbars.empty()) {
    last_frame_nis_ = 0.0;
    last_frame_nis_mean_ = 0.0;
    last_frame_residual_norm_ = 0.0;
    return;
  }

  const auto mark_no_update = [this]() {
    ++frame_rejected_count_;
    last_frame_update_count_ = 0;
    last_frame_rejected_count_ = frame_rejected_count_;
    last_frame_nis_ = 0.0;
    last_frame_nis_mean_ = 0.0;
    last_frame_residual_norm_ = 0.0;
    ++no_update_frames_;
  };

  struct SortedEntry
  {
    int assigned_id = -1;
    Eigen::Vector2d upper{ Eigen::Vector2d::Zero() };
    Eigen::Vector2d lower{ Eigen::Vector2d::Zero() };
  };

  // Step 1: find the full armor with the smallest corner error as anchor.
  // sp_vision's detector orders corners by image-left/right; rmcs' model
  // indexes lightbars by robot-left/right. Try both orientations so a
  // back/side view does not shift the lightbar ids by one.
  const Armor2d * anchor_armor = nullptr;
  auto anchor_armor_id = -1;
  auto anchor_swap = false;
  auto best_error_sum = std::numeric_limits<double>::max();

  for (const auto & armor : armors) {
    for (int armor_id = 0; armor_id < 4; ++armor_id) {
      const auto left_id = armor_id * 2;
      const auto right_id = armor_id * 2 + 1;
      if (!observable_.visible[left_id] || !observable_.visible[right_id]) continue;

      for (const auto swap : { false, true }) {
        const auto & left_upper = swap ? armor.tr : armor.tl;
        const auto & left_lower = swap ? armor.br : armor.bl;
        const auto & right_upper = swap ? armor.tl : armor.tr;
        const auto & right_lower = swap ? armor.bl : armor.br;

        const auto error_0 = squared_distance(
          Eigen::Vector2d{ left_upper.x, left_upper.y }, observable_.upper2d[left_id]);
        const auto error_1 = squared_distance(
          Eigen::Vector2d{ left_lower.x, left_lower.y }, observable_.lower2d[left_id]);
        const auto error_2 = squared_distance(
          Eigen::Vector2d{ right_upper.x, right_upper.y }, observable_.upper2d[right_id]);
        const auto error_3 = squared_distance(
          Eigen::Vector2d{ right_lower.x, right_lower.y }, observable_.lower2d[right_id]);
        const auto sum = error_0 + error_1 + error_2 + error_3;

        if (sum >= best_error_sum) continue;
        best_error_sum = sum;
        anchor_armor_id = armor_id;
        anchor_armor = &armor;
        anchor_swap = swap;
      }
    }
  }

  if (!anchor_armor) {
    // No geometrically plausible plate at all (e.g. the state drifted behind
    // the camera). Treat the frame as rejected and let the recovery logic run.
    mark_no_update();
    maybe_reinitialize(camera, armors);
    return;
  }

  last_anchor_error_ = std::sqrt(best_error_sum / 4.0);
  {
    // Gross-error gate: a wrong plate / wrong id must not pull the state.
    // The threshold is scale-aware so a close, fast-spinning plate is not
    // rejected only because its pixel prediction lags by a few dozen pixels.
    const auto left_id = anchor_armor_id * 2;
    const auto right_id = left_id + 1;
    const auto plate_width = (observable_.upper2d[left_id] - observable_.upper2d[right_id]).norm();
    const auto max_error = std::max(
      config_.anchor_max_corner_error, config_.anchor_max_relative_error * plate_width);
    if (max_error > 0.0 && last_anchor_error_ > max_error) {
      mark_no_update();
      maybe_reinitialize(camera, armors);
      return;
    }
  }

  // Step 2: collect all observed lightbars (2 per full armor + extras).
  std::vector<SortedEntry> sorted;
  sorted.reserve(armors.size() * 2 + lightbars.size());

  for (const auto & armor : armors) {
    const auto & left_upper = anchor_swap ? armor.tr : armor.tl;
    const auto & left_lower = anchor_swap ? armor.br : armor.bl;
    const auto & right_upper = anchor_swap ? armor.tl : armor.tr;
    const auto & right_lower = anchor_swap ? armor.bl : armor.br;

    sorted.push_back(SortedEntry{
      -1,
      Eigen::Vector2d{ left_upper.x, left_upper.y },
      Eigen::Vector2d{ left_lower.x, left_lower.y },
    });
    sorted.push_back(SortedEntry{
      -1,
      Eigen::Vector2d{ right_upper.x, right_upper.y },
      Eigen::Vector2d{ right_lower.x, right_lower.y },
    });
  }
  for (const auto & lightbar : lightbars) {
    sorted.push_back(SortedEntry{ -1, lightbar.upper, lightbar.lower });
  }

  std::sort(sorted.begin(), sorted.end(), [](const SortedEntry & a, const SortedEntry & b) {
    return a.upper.x() < b.upper.x();
  });

  // Step 3: anchor the anchor armor, then propagate ids left/right.
  {
    std::size_t anchor_left = 0;
    std::size_t anchor_right = 0;
    auto min_distance_left = std::numeric_limits<double>::max();
    auto min_distance_right = std::numeric_limits<double>::max();

    const auto anchor_left_x = anchor_swap ? anchor_armor->tr.x : anchor_armor->tl.x;
    const auto anchor_right_x = anchor_swap ? anchor_armor->tl.x : anchor_armor->tr.x;

    for (std::size_t index = 0; index < sorted.size(); ++index) {
      const auto distance_left = std::abs(sorted[index].upper.x() - anchor_left_x);
      const auto distance_right = std::abs(sorted[index].upper.x() - anchor_right_x);
      if (distance_left < min_distance_left) {
        min_distance_left = distance_left;
        anchor_left = index;
      }
      if (distance_right < min_distance_right) {
        min_distance_right = distance_right;
        anchor_right = index;
      }
    }

    sorted[anchor_left].assigned_id = anchor_armor_id * 2;
    sorted[anchor_right].assigned_id = anchor_armor_id * 2 + 1;

    for (std::size_t index = anchor_left + 1; index < sorted.size(); ++index) {
      sorted[index].assigned_id = (sorted[index - 1].assigned_id + 1) % 8;
    }
    for (std::size_t index = anchor_left; index > 0; --index) {
      sorted[index - 1].assigned_id = (sorted[index].assigned_id - 1 + 8) % 8;
    }
  }

  // Step 4: sequential EKF updates.
  for (const auto & entry : sorted) {
    update_one(entry.upper, entry.lower, entry.assigned_id);
  }

  last_observed_id_ = anchor_armor_id;
  if (anchor_armor_id != 0) jumped_ = true;
  ++update_count_;

  last_frame_update_count_ = frame_update_count_;
  last_frame_rejected_count_ = frame_rejected_count_;
  if (frame_nis_count_ > 0) {
    // nis 取单条最大值，和 chi-square(4) 阈值同口径；均值另发字段。
    last_frame_nis_ = frame_nis_max_;
    last_frame_nis_mean_ = frame_nis_sum_ / static_cast<double>(frame_nis_count_);
    last_frame_residual_norm_ = frame_residual_sum_ / static_cast<double>(frame_nis_count_);
  } else {
    last_frame_nis_ = 0.0;
    last_frame_nis_mean_ = 0.0;
    last_frame_residual_norm_ = 0.0;
  }

  if (frame_update_count_ == 0) {
    // Detections arrived but every lightbar was gated/projection-invalid.
    ++no_update_frames_;
    maybe_reinitialize(camera, armors);
  } else {
    no_update_frames_ = 0;
  }
}

auto RobotModel::maybe_reinitialize(
    const CameraFeature & camera, const std::vector<Armor2d> & armors) -> bool
{
  const auto threshold = config_.auto_reinit_after_no_update;
  if (threshold <= 0 || armors.empty()) return false;
  if (no_update_frames_ < threshold) return false;

  // Retry every `threshold` frames while the state stays unusable, instead of
  // running another robust PnP on every single frame.
  if (no_update_frames_ % threshold != 0) return false;

  const auto frames = no_update_frames_;
  if (!initialize(camera, armors)) {
    tools::logger()->warn(
      "[rmcs_ekf] auto re-init failed after {} frames without accepted update", frames);
    return false;
  }

  ++reinit_count_;
  tools::logger()->warn(
    "[rmcs_ekf] auto re-init after {} frames without accepted update (total={})", frames,
    reinit_count_);
  return true;
}

auto RobotModel::update_one(const Eigen::Vector2d & upper, const Eigen::Vector2d & lower,
  int lightbar_id) -> void
{
  if (lightbar_id < 0 || lightbar_id >= 8) return;
  if (!observable_.visible[lightbar_id]) {
    last_update_accepted_ = false;
    return;
  }

  // Skip measurements whose predicted endpoints are behind the camera or
  // failed to project. Feeding (0, 0) into the EKF would corrupt the state.
  if (!observable_.upper_valid[lightbar_id] || !observable_.lower_valid[lightbar_id]) {
    last_update_accepted_ = false;
    ++frame_rejected_count_;
    return;
  }

  const auto & predicted_upper = observable_.upper2d[lightbar_id];
  const auto & predicted_lower = observable_.lower2d[lightbar_id];

  Eigen::Matrix<double, 4, 1> innovation;
  innovation << upper.x() - predicted_upper.x(), upper.y() - predicted_upper.y(),
    lower.x() - predicted_lower.x(), lower.y() - predicted_lower.y();

  const auto jacobian = make_observation_jacobian(lightbar_id);
  if (jacobian.hasNaN() || !jacobian.allFinite()) {
    last_update_accepted_ = false;
    ++frame_rejected_count_;
    return;
  }

  // v0.32: 显式类型，避免 Eigen 惰性表达式在每次使用时重复求值（含求逆）。
  const Eigen::Matrix4d innovation_covariance =
    jacobian * covariance_ * jacobian.transpose() + observation_noise_;
  const Eigen::Matrix4d innovation_covariance_inv = innovation_covariance.inverse();
  if (innovation_covariance_inv.hasNaN() || !innovation_covariance_inv.allFinite()) {
    last_update_accepted_ = false;
    ++frame_rejected_count_;
    return;
  }

  // NIS: r^T S^-1 r，自由度 = 观测维数 4。
  // 注意：这里用的是更新前的 covariance_ 和 innovation，符合标准 NIS 定义。
  const double nis = (innovation.transpose() * innovation_covariance_inv * innovation)(0, 0);
  last_nis_ = nis;
  last_residual_norm_ = innovation.norm();

  // Diagnostics count every valid measurement attempt, including rejected ones.
  frame_nis_sum_ += nis;
  frame_nis_max_ = std::max(frame_nis_max_, nis);
  frame_residual_sum_ += last_residual_norm_;
  ++frame_nis_count_;

  const auto gate_active =
    config_.nis_gate_enabled
    && update_count_ >= static_cast<std::size_t>(std::max(0, config_.nis_gate_warmup_updates));
  const auto gate_threshold =
    config_.nis_gate_threshold > 0.0 ? config_.nis_gate_threshold : config_.nis_threshold;

  if (gate_active && nis > gate_threshold) {
    // Reject the outlier: a wrong association or a sudden occlusion must not
    // pull the state. Grow the position/velocity/angle uncertainty after a run
    // of rejections so the filter can re-open its gate instead of staying stuck.
    last_update_accepted_ = false;
    ++frame_rejected_count_;
    ++nis_rejected_count_;
    ++consecutive_nis_rejects_;

    if (consecutive_nis_rejects_ >= std::max(1, config_.nis_max_consecutive_rejects)) {
      if (config_.nis_reject_cov_inflation > 0.0) {
        for (int index = kStateX; index <= kStateW; ++index) {
          covariance_(index, index) += config_.nis_reject_cov_inflation;
        }
      }
      consecutive_nis_rejects_ = 0;
    }

    recent_nis_failures_.push_back(1);
    const auto window = static_cast<std::size_t>(std::max(1, config_.nis_window));
    while (recent_nis_failures_.size() > window) recent_nis_failures_.pop_front();
    return;
  }

  consecutive_nis_rejects_ = 0;

  const KalmanGain kalman_gain =
    covariance_ * jacobian.transpose() * innovation_covariance_inv;

  auto posterior_state = StateVector{ state_ + kalman_gain * innovation };
  if (posterior_state.hasNaN() || !posterior_state.allFinite()) {
    last_update_accepted_ = false;
    ++frame_rejected_count_;
    return;
  }

  posterior_state[kStateA] = normalize_angle(posterior_state[kStateA]);

  posterior_state[kStateRF] = std::clamp(
    posterior_state[kStateRF], config_.radius_forward_min, config_.radius_forward_max);
  posterior_state[kStateRL] = std::clamp(
    posterior_state[kStateRL], config_.radius_lateral_min, config_.radius_lateral_max);
  posterior_state[kStateHL] = std::clamp(
    posterior_state[kStateHL], config_.height_lateral_min, config_.height_lateral_max);

  posterior_state[kStateVx] =
    std::clamp(posterior_state[kStateVx], config_.velocity_min, config_.velocity_max);
  posterior_state[kStateVy] =
    std::clamp(posterior_state[kStateVy], config_.velocity_min, config_.velocity_max);
  posterior_state[kStateVz] =
    std::clamp(posterior_state[kStateVz], config_.velocity_min, config_.velocity_max);
  if (config_.rotation_speed_limit > 0.0) {
    posterior_state[kStateW] = std::clamp(
      posterior_state[kStateW], -config_.rotation_speed_limit, config_.rotation_speed_limit);
  }

  const Covariance complement = Covariance::Identity() - kalman_gain * jacobian;
  Covariance posterior_covariance =
    complement * covariance_ * complement.transpose();
  posterior_covariance += kalman_gain * observation_noise_ * kalman_gain.transpose();
  posterior_covariance =
    0.5 * (posterior_covariance + posterior_covariance.transpose());

  state_ = posterior_state;
  covariance_ = posterior_covariance;

  observable_.update(state_);

  last_update_accepted_ = true;
  ++frame_update_count_;

  recent_nis_failures_.push_back(last_nis_ > config_.nis_threshold ? 1 : 0);
  const auto window = static_cast<std::size_t>(std::max(1, config_.nis_window));
  while (recent_nis_failures_.size() > window) recent_nis_failures_.pop_front();
}

auto RobotModel::diagnostics() const -> Diagnostics
{
  Diagnostics result;
  result.state = state();
  result.variance = covariance_.diagonal();
  result.covariance_xy = covariance_.topLeftCorner<2, 2>();

  result.nis = last_frame_nis_;
  result.nis_mean = last_frame_nis_mean_;
  result.anchor_error = last_anchor_error_;
  result.nis_threshold = config_.nis_threshold;
  result.nis_gate_enabled = config_.nis_gate_enabled;
  result.nis_gate_threshold =
    config_.nis_gate_threshold > 0.0 ? config_.nis_gate_threshold : config_.nis_threshold;
  result.residual_norm = last_frame_residual_norm_;
  result.dt = last_dt_;

  result.update_count = update_count_;
  result.frame_update_count = last_frame_update_count_;
  result.frame_rejected_count = last_frame_rejected_count_;
  result.nis_rejected_count = nis_rejected_count_;
  result.nis_consecutive_rejects = consecutive_nis_rejects_;
  result.no_update_frames = no_update_frames_;
  result.reinit_count = reinit_count_;
  result.last_observed_id = last_observed_id_;
  result.converged = converge();
  result.diverged = diverged();
  result.last_update_accepted = last_update_accepted_;

  if (!recent_nis_failures_.empty()) {
    const auto failures =
      std::count(recent_nis_failures_.begin(), recent_nis_failures_.end(), 1);
    result.nis_fail_rate =
      static_cast<double>(failures) / static_cast<double>(recent_nis_failures_.size());
  }
  return result;
}

auto RobotModel::make_observation_jacobian(int lightbar_id) const
  -> Eigen::Matrix<double, 4, 11>
{
  constexpr auto kFiniteDifferenceStep = 1e-4;

  Eigen::Matrix<double, 4, 11> jacobian;
  jacobian.setZero();

  if (state_.hasNaN() || !state_.allFinite()) return jacobian;

  const auto theta = state_[kStateA];
  const auto fx = observable_.camera.camera_matrix[0][0];
  const auto fy = observable_.camera.camera_matrix[1][1];

  const auto r_cam2odom =
    observable_.camera.orientation.conjugate().toRotationMatrix();

  const auto projection_jacobian =
    [=](const Eigen::Vector3d & point) -> std::optional<Eigen::Matrix<double, 2, 3>> {
    constexpr auto kMinDepth = 0.1;

    const auto point_camera = ros2opencv_position(point);
    const auto depth = point_camera.z();
    if (depth < kMinDepth) return std::nullopt;

    Eigen::Matrix<double, 2, 3> jacobian_pixel_camera;
    jacobian_pixel_camera << fx / depth, 0.0, -fx * point_camera.x() / (depth * depth), 0.0,
      fy / depth, -fy * point_camera.y() / (depth * depth);

    Eigen::Matrix3d jacobian_camera_ros;
    jacobian_camera_ros << 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0;

    return jacobian_pixel_camera * jacobian_camera_ros;
  };

  const auto theta_jacobian = [&](int id, bool is_upper) -> Eigen::Vector3d {
    const auto compute = [&](double angle) -> Eigen::Vector3d {
      RobotSolution solution;
      solution.input.center =
        Eigen::Vector3d{ state_[kStateX], state_[kStateY], state_[kStateZ] };
      solution.input.toward =
        Eigen::Quaterniond{ Eigen::AngleAxisd{ angle, Eigen::Vector3d::UnitZ() } };
      solution.input.radius_forward = state_[kStateRF];
      solution.input.radius_lateral = state_[kStateRL];
      solution.input.height_lateral = state_[kStateHL];
      solution.input.large = large_;

      const auto lightbars = solution.solve_lightbars();
      return is_upper ? lightbars[id].upper : lightbars[id].lower;
    };

    const auto position_plus = compute(theta + kFiniteDifferenceStep);
    const auto position_minus = compute(theta - kFiniteDifferenceStep);
    if (position_plus.hasNaN() || position_minus.hasNaN()) return Eigen::Vector3d::Zero();
    return (position_plus - position_minus) / (2.0 * kFiniteDifferenceStep);
  };

  for (int endpoint = 0; endpoint < 2; ++endpoint) {
    const auto row = endpoint * 2;
    const auto is_upper = (endpoint == 0);
    const auto & point_ros =
      is_upper ? observable_.upper3d[lightbar_id] : observable_.lower3d[lightbar_id];

    const auto jacobian_projection = projection_jacobian(point_ros);
    if (!jacobian_projection) continue;

    Eigen::Matrix<double, 3, 11> jacobian_position;
    jacobian_position.setZero();

    jacobian_position(0, kStateX) = 1.0;
    jacobian_position(1, kStateY) = 1.0;
    jacobian_position(2, kStateZ) = 1.0;
    jacobian_position.block<3, 1>(0, kStateA) = theta_jacobian(lightbar_id, is_upper);

    const auto armor_id = lightbar_id / 2;
    const auto direction_forward =
      Eigen::Vector3d{ std::cos(theta), std::sin(theta), 0.0 };
    const auto direction_right =
      Eigen::Vector3d{ -std::sin(theta), std::cos(theta), 0.0 };

    auto jacobian_forward = jacobian_position.block<3, 1>(0, kStateRF);
    auto jacobian_lateral = jacobian_position.block<3, 1>(0, kStateRL);

    if (armor_id == 0) jacobian_forward = direction_forward;
    if (armor_id == 2) jacobian_forward = -direction_forward;

    if (armor_id == 1) jacobian_lateral = direction_right;
    if (armor_id == 3) jacobian_lateral = -direction_right;

    if (armor_id == 1 || armor_id == 3) jacobian_position(2, kStateHL) = 1.0;

    jacobian.block<2, 11>(row, 0).noalias() =
      *jacobian_projection * r_cam2odom * jacobian_position;
  }

  return jacobian;
}

auto RobotModel::state() const -> State
{
  State result;
  result.x = state_[kStateX];
  result.y = state_[kStateY];
  result.z = state_[kStateZ];
  result.vx = state_[kStateVx];
  result.vy = state_[kStateVy];
  result.vz = state_[kStateVz];
  result.rotation_angle = state_[kStateA];
  result.rotation_speed = state_[kStateW];
  result.radius_forward = state_[kStateRF];
  result.radius_lateral = state_[kStateRL];
  result.height_lateral = state_[kStateHL];
  return result;
}

auto RobotModel::full() const -> std::array<Armor3d, 4>
{
  RobotSolution solution;
  solution.input.center = Eigen::Vector3d{ state_[kStateX], state_[kStateY], state_[kStateZ] };
  solution.input.toward =
    Eigen::Quaterniond{ Eigen::AngleAxisd{ state_[kStateA], Eigen::Vector3d::UnitZ() } };
  solution.input.radius_forward = state_[kStateRF];
  solution.input.radius_lateral = state_[kStateRL];
  solution.input.height_lateral = state_[kStateHL];
  solution.input.large = large_;

  return solution.solve_armors();
}

auto RobotModel::converge() const -> bool
{
  if (update_count_ < static_cast<std::size_t>(std::max(0, config_.converge_min_update)))
    return false;

  // A filter that has not accepted any measurement for a few frames is not
  // converged, no matter how small its covariance looks.
  if (no_update_frames_ > std::max(0, config_.converge_max_no_update_frames)) return false;

  if (covariance_(kStateX, kStateX) > config_.converge_max_cov_xy) return false;
  if (covariance_(kStateY, kStateY) > config_.converge_max_cov_xy) return false;
  if (covariance_(kStateVx, kStateVx) > config_.converge_max_cov_v) return false;
  if (covariance_(kStateVy, kStateVy) > config_.converge_max_cov_v) return false;
  if (covariance_(kStateW, kStateW) > config_.converge_max_cov_w) return false;

  const auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - init_timestamp_);
  if (elapsed.count() < config_.converge_delay_s) return false;

  return true;
}

auto RobotModel::diverged() const -> bool
{
  if (state_.hasNaN() || !state_.allFinite() || covariance_.hasNaN()
    || !covariance_.allFinite())
    return true;

  // A deadlock where detections keep arriving but nothing is accepted is just
  // as bad as a state blow-up; let the tracker drop and re-acquire the target.
  if (config_.max_no_update_frames > 0 && no_update_frames_ > config_.max_no_update_frames)
    return true;

  // Let the filter settle for a few frames after initialization; the first
  // sequential updates can legitimately produce large velocity corrections.
  if (update_count_ < 5) return false;

  return covariance_(kStateX, kStateX) > config_.diverge_cov
    || covariance_(kStateY, kStateY) > config_.diverge_cov
    || std::abs(state_[kStateX]) > config_.diverge_xy
    || std::abs(state_[kStateY]) > config_.diverge_xy
    || std::abs(state_[kStateZ]) > config_.diverge_z
    || std::abs(state_[kStateW]) > config_.diverge_w
    || std::abs(state_[kStateVx]) > config_.diverge_v
    || std::abs(state_[kStateVy]) > config_.diverge_v
    || std::abs(state_[kStateVz]) > config_.diverge_vz;
}

auto RobotModel::Observable::update(const StateVector & state) -> void
{
  const auto q_cam_from_odom = camera.orientation.conjugate();
  const auto camera_position = camera.translation;

  std::fill(upper2d.begin(), upper2d.end(), Eigen::Vector2d::Zero());
  std::fill(lower2d.begin(), lower2d.end(), Eigen::Vector2d::Zero());
  std::fill(upper3d.begin(), upper3d.end(), Eigen::Vector3d::Zero());
  std::fill(lower3d.begin(), lower3d.end(), Eigen::Vector3d::Zero());
  std::fill(visible.begin(), visible.end(), false);
  std::fill(upper_valid.begin(), upper_valid.end(), false);
  std::fill(lower_valid.begin(), lower_valid.end(), false);

  RobotSolution solution;
  solution.input.center =
    Eigen::Vector3d{ state[RobotModel::kStateX], state[RobotModel::kStateY],
      state[RobotModel::kStateZ] };
  solution.input.toward = Eigen::Quaterniond{
    Eigen::AngleAxisd{ state[RobotModel::kStateA], Eigen::Vector3d::UnitZ() } };
  solution.input.radius_forward = state[RobotModel::kStateRF];
  solution.input.radius_lateral = state[RobotModel::kStateRL];
  solution.input.height_lateral = state[RobotModel::kStateHL];
  solution.input.large = large;

  const auto lightbars = solution.solve_lightbars();
  const auto armors = solution.solve_armors();

  const auto intrinsic = camera.intrinsic();
  const auto distortion = camera.distortion();

  for (int index = 0; index < 8; ++index) {
    const auto upper_odom = lightbars[index].upper;
    const auto lower_odom = lightbars[index].lower;

    const auto upper_camera = q_cam_from_odom * (upper_odom - camera_position);
    const auto lower_camera = q_cam_from_odom * (lower_odom - camera_position);

    upper3d[index] = upper_camera;
    lower3d[index] = lower_camera;

    const auto upper_opencv = ros2opencv_position(upper_camera);
    const auto lower_opencv = ros2opencv_position(lower_camera);

    // A point behind the camera can still be "projected" by cv::projectPoints,
    // producing a plausible-looking but physically invalid pixel.
    constexpr auto kMinDepth = 0.1;
    const auto upper_in_front = upper_opencv.z() > kMinDepth;
    const auto lower_in_front = lower_opencv.z() > kMinDepth;

    if (upper_in_front) {
      if (const auto upper_projected =
            project_camera_point(upper_opencv, intrinsic, distortion)) {
        upper2d[index] = *upper_projected;
        upper_valid[index] = true;
      }
    }
    if (lower_in_front) {
      if (const auto lower_projected =
            project_camera_point(lower_opencv, intrinsic, distortion)) {
        lower2d[index] = *lower_projected;
        lower_valid[index] = true;
      }
    }
  }

  for (int index = 0; index < 4; ++index) {
    const auto orientation = armors[index].orientation;
    const auto position = armors[index].translation;

    const auto armor_to_center = orientation * Eigen::Vector3d::UnitX();
    const auto armor_face = -armor_to_center;
    const auto armor_to_camera = (camera_position - position).normalized();
    const auto yaw =
      std::acos(std::clamp(armor_face.dot(armor_to_camera), -1.0, 1.0));

    if (yaw >= yaw_part_max) continue;

    const auto left_id = index * 2;
    const auto right_id = index * 2 + 1;

    if (yaw < yaw_full_max) {
      visible[left_id] = true;
      visible[right_id] = true;
    } else {
      const auto y_axis = Eigen::Vector3d{ orientation * Eigen::Vector3d::UnitY() };
      const auto on_plus_y = armor_to_camera.dot(y_axis) > 0.0;
      visible[on_plus_y ? right_id : left_id] = true;
    }
  }
}

auto RobotModel::Observable::visible_armors() const -> std::size_t
{
  std::size_t count = 0;
  for (int index = 0; index < 4; ++index) {
    if (visible[index * 2] && visible[index * 2 + 1]) ++count;
  }
  return count;
}

} // namespace rmcs_ekf
