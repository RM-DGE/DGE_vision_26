#ifndef IO__DAEDALUS__SHARED_HPP
#define IO__DAEDALUS__SHARED_HPP

// DaedalusCamera 与 DaedalusCBoard 共享的进程级单例：
// 一个 ShmClient 客户端 + IMU 四元数队列 + 符号/坐标系映射参数。
// 图像与位姿都由 DaedalusCamera::read() 驱动接收，DaedalusCBoard 只读队列、发指令。

#include <Eigen/Geometry>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "shm_client.hpp"
#include "tools/logger.hpp"
#include <yaml-cpp/yaml.h>

namespace io
{

struct DaedalusImuSample
{
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point timestamp;
};

struct DaedalusShared
{
  std::shared_ptr<ipc::ShmClient> client;
  std::mutex imu_mutex;
  std::deque<DaedalusImuSample> imu_queue;
  const size_t imu_queue_capacity = 5000;

  // command.yaw/pitch (rad, DGE世界系) → GimbalCmd (deg, 仿真器世界系) 的符号映射
  double yaw_sign = 1.0;
  double pitch_sign = -1.0;  // talos 同款：仿真器 pitch_deg 与 DGE pitch 反号

  // Bevy(Y-up) → DGE(Z-up, X-forward) 坐标系变换: q' = left * q * right.conjugate()
  // 默认 left = right = (0.5, 0.5, 0.5, 0.5)，即绕 (1,1,1) 轴 120° 的循环轴变换
  Eigen::Quaterniond imu_left = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5);
  Eigen::Quaterniond imu_right = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5);

  // 最近一次收到的云台位姿（Bevy 系原始四元数），位姿通道无新数据时沿用
  Eigen::Quaterniond last_gimbal_q = Eigen::Quaterniond::Identity();

  // imu_at() 的游标状态（与 CBoard 的 data_ahead_/data_behind_ 等价）
  DaedalusImuSample last_imu_sample{
    Eigen::Quaterniond::Identity(), std::chrono::steady_clock::time_point::min()};

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

  void push_imu(const Eigen::Quaterniond & q_bevy)
  {
    Eigen::Quaterniond q =
      (imu_left * q_bevy * imu_right.conjugate()).normalized();
    std::lock_guard<std::mutex> lock(imu_mutex);
    if (imu_queue.size() >= imu_queue_capacity) imu_queue.pop_front();
    imu_queue.push_back({q, std::chrono::steady_clock::now()});
  }
};

}  // namespace io

#endif  // IO__DAEDALUS__SHARED_HPP
