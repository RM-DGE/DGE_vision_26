#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "target.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
  // Index in Target::full()/xyza_list(), from the final selection iteration.
  int armor_id = -1;
};

class Aimer
{
public:
  AimPoint debug_aim_point{false, Eigen::Vector4d::Zero()};
  explicit Aimer(const std::string & config_path);
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true, double observation_age_s = 0.005);

  // io::Command aim(
  //   std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  //   io::ShootMode shoot_mode, bool to_now = true);

private:
  double yaw_offset_;
  // std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double lock_id_ = -1;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double decision_speed_;

  AimPoint choose_aim_point(const Target & target);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP
