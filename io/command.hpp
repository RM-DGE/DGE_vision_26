#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double yaw_vel = 0.0;        // 前馈速度
  double yaw_acc = 0.0;        // 前馈加速度
  double pitch_vel = 0.0;      // 前馈速度
  double pitch_acc = 0.0;      // 前馈加速度
  double horizon_distance = 0; // 无人机专有
};

}  // namespace io

#endif  // IO__COMMAND_HPP