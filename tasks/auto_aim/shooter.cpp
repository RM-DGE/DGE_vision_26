#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "solver.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path)
{
  const auto yaml = YAML::LoadFile(config_path);
  width_small_m_ = yaml["shoot_width_small_m"].as<double>(0.12);
  width_large_m_ = yaml["shoot_width_large_m"].as<double>(0.23);
  height_m_ = yaml["shoot_height_m"].as<double>(0.08);
  max_imu_age_ms_ = yaml["shoot_max_imu_age_ms"].as<double>(20.0);
  auto_fire_ = yaml["auto_fire"].as<bool>();
  for (double value : {width_small_m_, width_large_m_, height_m_, max_imu_age_ms_})
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("Shooter dimensions and IMU age must be finite and positive");
}

bool Shooter::shoot(
  const io::Command & command, const Aimer & aimer, const std::list<Target> & targets,
  const tools::time_sync::ImuQuery & attitude, const Solver & solver)
{
  if (
    !attitude.valid() || attitude.left_time != attitude.right_time ||
    !attitude.q.coeffs().allFinite() || std::abs(attitude.q.squaredNorm() - 1.) > .1)
    return false;
  const double age_ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - attitude.right_time)
                          .count();
  if (age_ms < 0 || age_ms > max_imu_age_ms_) return false;
  const auto ypr = tools::eulers(solver.R_gimbal2world(attitude.q.normalized()), 2, 1, 0);
  return shoot(command, aimer, targets, ypr);
}

bool Shooter::shoot(
  const io::Command & command, const Aimer & aimer, const std::list<Target> & targets,
  const Eigen::Vector3d & gimbal_pos)
{
  if (
    !command.control || targets.empty() || !auto_fire_ || !aimer.debug_aim_point.valid ||
    !gimbal_pos.allFinite() || !std::isfinite(command.yaw) || !std::isfinite(command.pitch))
    return false;
  const auto & aim = aimer.debug_aim_point.xyza;
  if (!aim.allFinite()) return false;
  const double distance = std::hypot(aim.x(), aim.y());
  if (distance < .01) return false;
  const double bearing = std::atan2(aim.y(), aim.x());
  const double facing = std::cos(tools::limit_rad(aim.w() - bearing));
  if (facing <= 1e-6) return false;

  // Actual horizontal edge rays of the predicted plate; retain the smaller
  // half-window to avoid enlarging the asymmetric projection of an oblique plate.
  const double half_width =
    .5 * (targets.front().armor_type == ArmorType::big ? width_large_m_ : width_small_m_);
  const Eigen::Vector2d edge(-std::sin(aim.w()) * half_width, std::cos(aim.w()) * half_width);
  const double left =
    tools::limit_rad(std::atan2(aim.y() + edge.y(), aim.x() + edge.x()) - bearing);
  const double right =
    tools::limit_rad(std::atan2(aim.y() - edge.y(), aim.x() - edge.x()) - bearing);
  if (left * right >= 0) return false;
  const double yaw_window = std::min(std::abs(left), std::abs(right));
  const double yaw_error = std::abs(tools::limit_rad(gimbal_pos[0] - command.yaw));
  if (yaw_error >= yaw_window) return false;

  // Preserve the existing conservative oblique-plate height reduction.
  // No distance cap: a fixed physical region gets smaller in angle at longer range.
  const double half_height = .5 * height_m_ * facing;
  const double center_pitch = -std::atan2(aim.z(), distance);
  const double lower = -std::atan2(aim.z() + half_height, distance) - center_pitch;
  const double upper = -std::atan2(aim.z() - half_height, distance) - center_pitch;
  const double pitch_error = gimbal_pos[1] - command.pitch;
  return pitch_error >= lower && pitch_error <= upper;
}
}  // namespace auto_aim
