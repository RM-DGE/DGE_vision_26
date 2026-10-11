#ifndef IO__DAEDALUS_CBOARD_HPP
#define IO__DAEDALUS_CBOARD_HPP

#include <algorithm>
#include <cmath>

#include "io/cboard.hpp"
#include "io/Daedalus/shared.hpp"

namespace io
{

// CBoard 的仿真替身：不连串口，通过共享内存与 Daedalus 仿真器通信。
// - imu_at(): 从 IMU 队列插值（队列由 DaedalusCamera::read() 按帧填充）
// - send():   把 DGE 世界系的绝对 yaw/pitch (rad) 换算成度发给仿真器，
//             fire_advice 来自 command.shoot
class DaedalusCBoard : public CBoardBase
{
public:
  explicit DaedalusCBoard(const std::string & config_path)
  {
    DaedalusShared::instance(config_path);  // 确保单例已初始化
  }

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp) override
  {
    auto & shared = DaedalusShared::instance("");

    DaedalusImuSample data_ahead = shared.last_imu_sample;
    DaedalusImuSample data_behind = shared.last_imu_sample;

    while (true) {
      DaedalusImuSample temp;
      bool has_data = false;
      {
        std::lock_guard<std::mutex> lock(shared.imu_mutex);
        if (!shared.imu_queue.empty()) {
          temp = shared.imu_queue.front();
          shared.imu_queue.pop_front();
          has_data = true;
        }
      }
      if (!has_data) break;

      data_behind = temp;
      if (data_behind.timestamp > timestamp) break;
      data_ahead = data_behind;
    }
    shared.last_imu_sample = data_ahead;

    // slerp 插值（与 CBoard::imu_at 相同的逻辑）
    const Eigen::Quaterniond & q_a = data_ahead.q;
    const Eigen::Quaterniond & q_b = data_behind.q;
    auto t_a = data_ahead.timestamp;
    auto t_b = data_behind.timestamp;
    auto t_c = timestamp;

    if (t_a == t_b) return q_a;

    double k = double((t_c - t_a).count()) / double((t_b - t_a).count());
    k = std::max(0.0, std::min(1.0, k));
    return q_a.slerp(k, q_b);
  }

  void send(Command command) const override
  {
    auto & shared = DaedalusShared::instance("");
    constexpr double kRadToDeg = 180.0 / M_PI;
    shared.client->send_gimbal_cmd(
      static_cast<float>(shared.yaw_sign * command.yaw * kRadToDeg),
      static_cast<float>(shared.pitch_sign * command.pitch * kRadToDeg),
      static_cast<float>(command.horizon_distance), command.shoot);
  }
};

}  // namespace io

#endif  // IO__DAEDALUS_CBOARD_HPP
