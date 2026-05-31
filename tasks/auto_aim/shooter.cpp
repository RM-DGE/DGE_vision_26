#include "shooter.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <algorithm> // 引入 std::min

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  if (!command.control || targets.empty() || !auto_fire_ || !aimer.debug_aim_point.valid) {
    last_command_ = command;
    return false;
  }

  auto target = targets.front();
  auto target_x = target.ekf_x()[0];
  auto target_y = target.ekf_x()[2];
  
  // 计算目标距离
  double distance = std::sqrt(tools::square(target_x) + tools::square(target_y));
  
  // 获取目标的旋转角速度 (omega)
  double omega = target.ekf_x()[7]; 
  double abs_omega = std::abs(omega);

  // 1. 基础宽容度判定
  double tolerance = distance > judge_distance_ ? second_tolerance_ : first_tolerance_;

  // 2. 小陀螺动态容差放大
  if (abs_omega > 1.5) {
    double multiplier = 1.0 + 0.4 * (abs_omega - 1.5);
    multiplier = std::min(multiplier, 2); // 为了安全，最高放大2.5倍即可
    tolerance *= multiplier;
  }

  // 3. 计算云台当前绝对 Yaw 与 预测指令 Yaw 的最短角度误差
  double yaw_error = std::abs(tools::limit_rad(gimbal_pos[0] - command.yaw));
  
  // 4. 计算两次指令的突变差值 (当前预测点 vs 上一帧预测点)
  double command_diff = std::abs(tools::limit_rad(last_command_.yaw - command.yaw));

  // --- 新增：物理防走火极限 ---
  // 设定小陀螺切换装甲板允许的最大合法物理跳变（18度 = 0.314 弧度）
  // 超过这个值，说明是切换到了另一个机器人，或者是目标刚刚进入视野，绝对禁止开火！
  double max_plate_switch_jump = 18.0 / 57.3; 

  bool should_shoot = false;

  // 逻辑 A: 常规平移目标
  // 要求极高的连续性（严苛的 command_diff），防止手抖或检测框微小跳动
  bool normal_shoot = (command_diff < tolerance * 2) && (yaw_error < tolerance);

  // 逻辑 B: 应对小陀螺/前哨站
  // 允许跳变，但跳变必须小于装甲板切换的物理极限 (max_plate_switch_jump)
  // 且云台枪管当前扫过了被放大的判定窗口 (yaw_error < tolerance)
  bool spin_shoot = (abs_omega > 1.5) && 
                    (command_diff < max_plate_switch_jump) && 
                    (yaw_error < tolerance);

  if (normal_shoot || spin_shoot) {
    should_shoot = true;
  }

  last_command_ = command;
  return should_shoot;
}

}  // namespace auto_aim


// #include "shooter.hpp"

// #include <yaml-cpp/yaml.h>

// #include "tools/logger.hpp"
// #include "tools/math_tools.hpp"

// namespace auto_aim
// {
// Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
// {
//   auto yaml = YAML::LoadFile(config_path);
//   first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
//   second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
//   judge_distance_ = yaml["judge_distance"].as<double>();
//   auto_fire_ = yaml["auto_fire"].as<bool>();
// }

// bool Shooter::shoot(
//   const io::Command & command, const auto_aim::Aimer & aimer,
//   const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
// {
//   if (!command.control || targets.empty() || !auto_fire_) return false;

//   auto target_x = targets.front().ekf_x()[0];
//   auto target_y = targets.front().ekf_x()[2];
//   auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
//                      ? second_tolerance_
//                      : first_tolerance_;
//   // tools::logger()->debug("d(command.yaw) is {:.4f}", std::abs(last_command_.yaw - command.yaw));
//   if (
//     std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
//     std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
//     aimer.debug_aim_point.valid) {
//     last_command_ = command;
//     return true;
//   }

//   last_command_ = command;
//   return false;
// }

// }  // namespace auto_aim