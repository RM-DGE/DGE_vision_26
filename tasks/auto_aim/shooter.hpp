#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <string>

#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tools/imu_time_buffer.hpp"

namespace auto_aim
{
class Solver;
class Shooter
{
public:
  Shooter(const std::string & config_path);

  // Runtime entry: latest real IMU sample, with age and calibration checks.
  bool shoot(
    const io::Command & command, const Aimer & aimer, const std::list<Target> & targets,
    const tools::time_sync::ImuQuery & attitude, const Solver & solver);

  // Geometry-only entry for offline tests / callers responsible for attitude timing.
  bool shoot(
    const io::Command & command, const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos);

private:
  double width_small_m_, width_large_m_, height_m_, max_imu_age_ms_;
  bool auto_fire_;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP
