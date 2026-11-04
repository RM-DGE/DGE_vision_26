#ifndef IO__DAEDALUS_CBOARD_HPP
#define IO__DAEDALUS_CBOARD_HPP

#include <algorithm>
#include <cmath>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "io/cboard.hpp"
#include "io/Daedalus/shared.hpp"
#include "tools/math_tools.hpp"

namespace io
{

// CBoard 的仿真替身：不连串口，通过共享内存与 Daedalus 仿真器通信。
// - imu_query(): 安全查询 IMU 缓存（由 DaedalusCamera::read() 按帧填充）
// - send():   把 DGE 世界系的绝对 yaw/pitch (rad) 换算成度发给仿真器，
//             fire_advice 来自 command.shoot
class DaedalusCBoard : public CBoardBase
{
public:
  explicit DaedalusCBoard(const std::string & config_path)
  {
    auto & shared = DaedalusShared::instance(config_path);  // 确保单例已初始化
    if (auto yaml = YAML::LoadFile(config_path); yaml["daedalus"]) {
      auto d = yaml["daedalus"];
      if (d["smooth_alpha"]) smooth_alpha_ = std::clamp(d["smooth_alpha"].as<double>(), 0.05, 1.0);
      // 零位修正（deg）：自瞄对静止目标稳定偏向一侧时标定。
      // 云台偏左 → yaw_trim 取正把指令往右拉；先从 ±1deg 试起。
      if (d["yaw_trim"]) yaw_trim_ = d["yaw_trim"].as<double>() / 57.3;
      if (d["pitch_trim"]) pitch_trim_ = d["pitch_trim"].as<double>() / 57.3;
    }
  }

  ~DaedalusCBoard() override { set_imu_observer({}); }

  tools::time_sync::ImuQuery imu_query(
    std::chrono::steady_clock::time_point timestamp, double max_gap_ms = 20,
    double max_wait_ms = 12) override
  {
    auto & shared = DaedalusShared::instance("");
    if (!std::isfinite(max_wait_ms) || max_wait_ms < 0 || max_wait_ms > 1000) {
      tools::time_sync::ImuQuery result;
      result.status = tools::time_sync::ImuQueryStatus::invalid_sample;
      return result;
    }
    // Camera::read produces the samples on this same thread. Waiting cannot
    // produce a future right sample; fail immediately and let the next read run.
    return shared.imu_buffer.query(timestamp, max_gap_ms);
  }

  void set_imu_observer(ImuObserver observer) override
  {
    auto & shared = DaedalusShared::instance("");
    std::lock_guard<std::mutex> lock(shared.imu_observer_mutex);
    if (!observer) { shared.imu_observer = {}; return; }
    shared.imu_observer = [callback = std::move(observer)](
      const Eigen::Quaterniond& q, tools::time_sync::TimePoint timestamp) {
      callback(ImuSample{q, timestamp});
    };
  }

  void send(Command command) const override
  {
    auto & shared = DaedalusShared::instance("");

    // 无有效目标时不要驱动仿真器云台。
    // 仿真器 GimbalCmd 虽没有 control 使能位，但它用 distance_m == -1 表示
    // “solver 无解”：收到后 remove::<GimbalAimTracker>()，云台不再被 PID 驱动、
    // 原地保持（见 talos/plugin.rs process_subscription）。
    // 旧逻辑在 control=0 时回发“当前云台姿态”，但那条路要经过 tools::eulers 反解、
    // yaw_sign/pitch_sign、再被仿真器 from_solver_degrees 做 -90°，两者约定不同，
    // 根本不是恒等映射 —— 结果是把云台拉到固定的错误姿态（按 F5 后“头突然摆向
    // 一侧”就是这个）。正确做法是直接发无解哨兵，让仿真器松手。
    if (!command.control) {
      smoothed_ = false;
      shared.client->send_gimbal_cmd(0.0f, 0.0f, -1.0f, false);
      return;
    }

    if (smoothed_) {
      // 真机云台 PID 天然滤掉指令噪声；仿真器是绝对角直通，需自行加一级
      // 低通（EMA）抑制 EKF/选板抖动。相邻指令突变超过跳变阈值时视为
      // 真实的装甲板切换，直通不拖延（小陀螺打板不滞后）。
      constexpr double kJumpThreshold = 10.0 * M_PI / 180.0;
      if (std::abs(tools::limit_rad(command.yaw - smooth_yaw_)) < kJumpThreshold)
        command.yaw = smooth_yaw_ + smooth_alpha_ * tools::limit_rad(command.yaw - smooth_yaw_);
      if (std::abs(command.pitch - smooth_pitch_) < kJumpThreshold)
        command.pitch = smooth_pitch_ + smooth_alpha_ * (command.pitch - smooth_pitch_);
    }
    smooth_yaw_ = command.yaw;
    smooth_pitch_ = command.pitch;
    smoothed_ = true;

    command.yaw += yaw_trim_;
    command.pitch += pitch_trim_;

    constexpr double kRadToDeg = 180.0 / M_PI;
    shared.client->send_gimbal_cmd(
      static_cast<float>(shared.yaw_sign * command.yaw * kRadToDeg),
      static_cast<float>(shared.pitch_sign * command.pitch * kRadToDeg),
      static_cast<float>(command.horizon_distance), command.shoot);
  }

private:
  // 指令低通滤波状态（send 为 const，成员标记 mutable）
  mutable double smooth_alpha_ = 0.5;
  mutable double smooth_yaw_ = 0.0;
  mutable double smooth_pitch_ = 0.0;
  mutable bool smoothed_ = false;
  // 零位修正（rad），补偿指令/反馈两条通路的恒定失配
  double yaw_trim_ = 0.0;
  double pitch_trim_ = 0.0;
};

}  // namespace io

#endif  // IO__DAEDALUS_CBOARD_HPP
