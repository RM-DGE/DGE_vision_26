#ifndef IO__DAEDALUS__SHARED_HPP
#define IO__DAEDALUS__SHARED_HPP

// DaedalusCamera 与 DaedalusCBoard 共享的进程级单例：
// 一个 ShmClient 客户端 + 非消耗式 IMU 缓存 + 符号/坐标系映射参数。
// 图像与位姿都由 DaedalusCamera::read() 驱动接收，DaedalusCBoard 查询缓存、发指令。

#include <Eigen/Geometry>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "shm_client.hpp"
#include "tools/logger.hpp"
#include "tools/imu_time_buffer.hpp"
#include <yaml-cpp/yaml.h>

namespace io
{

struct DaedalusShared
{
  std::shared_ptr<ipc::ShmClient> client;
  tools::time_sync::ImuTimeBuffer imu_buffer;
  std::mutex imu_observer_mutex;
  std::function<void(const Eigen::Quaterniond&, tools::time_sync::TimePoint)> imu_observer;

  // command.yaw/pitch (rad, DGE世界系) → GimbalCmd (deg, 仿真器世界系) 的符号映射
  double yaw_sign = 1.0;
  double pitch_sign = -1.0;  // talos 同款：仿真器 pitch_deg 与 DGE pitch 反号

  // Bevy(Y-up) → DGE(Z-up, X-forward) 坐标系变换: q' = left * q * right.conjugate()
  // 默认 left = right = (0.5, 0.5, 0.5, 0.5)，即绕 (1,1,1) 轴 120° 的循环轴变换
  Eigen::Quaterniond imu_left = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5);
  Eigen::Quaterniond imu_right = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5);

  // 最近一次真实收到的云台位姿；只有新位姿可形成新的时间样本。
  Eigen::Quaterniond last_gimbal_q = Eigen::Quaterniond::Identity();
  std::optional<std::uint64_t> pending_gimbal_frame;

  // 最新一帧敌方真值（DaedalusCamera::read 时刷新；与主循环同线程，无需加锁）
  ipc::GroundTruthBatch last_ground_truth{};

  static DaedalusShared & instance(const std::string & config_path)
  {
    static DaedalusShared shared;
    static bool initialized = false;
    if (!initialized) {
      initialized = true;
      try {
        shared.client = std::make_shared<ipc::ShmClient>();
      } catch (const std::exception & e) {
        tools::logger()->error("[Daedalus] {}", e.what());
        exit(1);
      }
      tools::logger()->info("[Daedalus] shm connected (producer alive: {})",
                            shared.client->is_producer_alive());

      auto yaml = YAML::LoadFile(config_path);
      if (yaml["daedalus"]) {
        auto d = yaml["daedalus"];
        if (d["yaw_sign"]) shared.yaw_sign = d["yaw_sign"].as<double>();
        if (d["pitch_sign"]) shared.pitch_sign = d["pitch_sign"].as<double>();
        if (d["imu_left"]) {
          auto v = d["imu_left"].as<std::vector<double>>();
          shared.imu_left = Eigen::Quaterniond(v[0], v[1], v[2], v[3]).normalized();
        }
        if (d["imu_right"]) {
          auto v = d["imu_right"].as<std::vector<double>>();
          shared.imu_right = Eigen::Quaterniond(v[0], v[1], v[2], v[3]).normalized();
        }
      }
    }
    return shared;
  }

  void stage_gimbal_pose(const Eigen::Quaterniond & q_bevy, std::uint64_t frame)
  {
    last_gimbal_q = q_bevy;
    pending_gimbal_frame = frame;
  }

  bool publish_frame_imu(std::uint64_t frame, tools::time_sync::TimePoint timestamp)
  {
    if (!pending_gimbal_frame) return false;
    if (*pending_gimbal_frame < frame) { pending_gimbal_frame.reset(); return false; }
    if (*pending_gimbal_frame != frame) return false;
    pending_gimbal_frame.reset();
    return push_imu(last_gimbal_q, timestamp);
  }

  bool push_imu(const Eigen::Quaterniond & q_bevy, tools::time_sync::TimePoint timestamp)
  {
    const Eigen::Quaterniond q = imu_left * q_bevy * imu_right.conjugate();
    if (!imu_buffer.push({timestamp, q})) return false;
    // Same frame-origin timestamp as the image. No synthetic asynchronous IMU.
    // Holding this lock also makes unregister wait for any in-flight callback.
    std::lock_guard<std::mutex> lock(imu_observer_mutex);
    if (imu_observer) imu_observer(q, timestamp);
    return true;
  }
};

}  // namespace io

#endif  // IO__DAEDALUS__SHARED_HPP
