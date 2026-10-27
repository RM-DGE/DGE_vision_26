#ifndef IO__ROS2__MCU_ROS_BRIDGE_HPP
#define IO__ROS2__MCU_ROS_BRIDGE_HPP

#include <Eigen/Dense>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sp_msgs/msg/auto_aim_target.hpp"
#include "sp_msgs/msg/autoaim_target_msg.hpp"
#include "sp_msgs/msg/enemy_status_msg.hpp"
#include "sp_msgs/msg/gimbal_cmd.hpp"
#include "sp_msgs/msg/mcu_status.hpp"
#include "std_msgs/msg/string.hpp"

#include "io/cboard.hpp"

namespace io
{

// 串口/共享内存 <-> ROS2 的唯一桥接层。
//
// 设计原则（与 sp25 的 ROS2 包装保持一致，但把“单 owner、最新值、带时间戳”
// 落实到 CBoard 的观察者回调和独立发布线程上）：
//   - 串口仍由 CBoard 独占，本类不重复打开设备；
//   - CBoard 读线程只把样本塞进 mailbox，绝不在读线程里 publish；
//   - IMU 事件驱动发布最新样本；状态按固定频率发布最新样本；
//   - 导航下发只更新 latest-value，主循环读取时做超时判断；
//   - 可选 /gimbal/cmd 远程下发，默认关闭，避免和 sentry 主循环重复发送。
class McuRosBridge
{
public:
  struct Config
  {
    bool enabled = true;
    bool enable_remote_gimbal_cmd = false;
    std::string node_name = "dge_mcu_bridge";
    std::string imu_topic = "/mcu/imu";
    std::string status_topic = "/mcu/status";
    std::string gimbal_cmd_topic = "/gimbal/cmd";
    std::string nav_enemy_status_topic = "/enemy_status";
    std::string nav_autoaim_target_topic = "/autoaim_target";
    std::string target_pos_topic = "/auto_aim_target_pos";
    std::string target_info_topic = "/auto_aim_target_info";
    std::string imu_frame_id = "imu_link";
    double imu_publish_rate = 200.0;    // Hz，<=0 表示每帧都发
    double status_publish_rate = 20.0;  // Hz
    double max_cmd_age_ms = 20.0;
    double nav_timeout_ms = 1500.0;
    double status_online_timeout_ms = 200.0;
  };

  struct Stats
  {
    uint64_t imu_published = 0;
    uint64_t status_published = 0;
    uint64_t gimbal_cmd_received = 0;
    uint64_t gimbal_cmd_sent = 0;
    uint64_t gimbal_cmd_dropped = 0;
    double last_cmd_age_ms = 0.0;
    double last_send_us = 0.0;
    double last_imu_age_ms = 0.0;
  };

  McuRosBridge(CBoardBase & board, const std::string & config_path);
  ~McuRosBridge();

  McuRosBridge(const McuRosBridge &) = delete;
  McuRosBridge & operator=(const McuRosBridge &) = delete;

  static Config load_config(const std::string & config_path);
  static bool enabled_in_config(const std::string & config_path);

  bool remote_gimbal_enabled() const { return config_.enable_remote_gimbal_cmd; }

  // 兼容旧接口：发布 "x,y,z,id" 到 /auto_aim_target_pos。
  void publish(const Eigen::Vector4d & target_pos);

  // 结构化目标：position/velocity 使用 gimbal 系。
  // stamp 传图像/解算时刻的 steady_clock；默认空表示用当前时间。
  // 导航端应使用 header.stamp 做 TF 查询和目标年龄判断。
  void publish_target(
    const Eigen::Vector3d & position, const Eigen::Vector3d & velocity, int8_t id, bool tracking,
    const std::string & frame_id = "gimbal", double yaw = 0.0, double v_yaw = 0.0,
    std::chrono::steady_clock::time_point stamp = {});

  // 导航 -> 自瞄：超时后返回空 vector，调用方按“无导航信息”处理。
  std::vector<int8_t> subscribe_enemy_status();
  std::vector<int8_t> subscribe_autoaim_target();

  // 自瞄 -> 网关：把指令发布到 /gimbal/cmd，由订阅回调真正写串口。
  // 只有 config.enable_remote_gimbal_cmd=true 时才会写串口；
  // 默认生产模式仍由 sentry 主循环直接 cboard.send()，避免多一次 ROS 往返。
  void send_command(const Command & command);

  // 测试/监控用：每次 /gimbal/cmd 真正写串口后回调。
  // 参数：从 header.stamp 到回调的年龄（ms），以及 board.send() 调用耗时（us）。
  using CmdLatencyObserver = std::function<void(double age_ms, double send_us)>;
  void set_cmd_latency_observer(CmdLatencyObserver observer);

  Stats stats() const;

private:
  void on_imu(const ImuSample & sample);
  void on_status(const StatusSample & sample);
  void imu_publish_loop();
  void status_publish_loop();
  void publish_imu(const ImuSample & sample);
  void publish_status(const StatusSample & sample);

  void enemy_status_callback(const sp_msgs::msg::EnemyStatusMsg::SharedPtr msg);
  void autoaim_target_callback(const sp_msgs::msg::AutoaimTargetMsg::SharedPtr msg);
  void gimbal_cmd_callback(const sp_msgs::msg::GimbalCmd::SharedPtr msg);

  rclcpp::Time to_ros_time(std::chrono::steady_clock::time_point steady_time) const;

  CBoardBase & board_;
  Config config_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sp_msgs::msg::McuStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr legacy_pub_;
  rclcpp::Publisher<sp_msgs::msg::AutoAimTarget>::SharedPtr target_pub_;
  rclcpp::Publisher<sp_msgs::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;

  rclcpp::Subscription<sp_msgs::msg::EnemyStatusMsg>::SharedPtr enemy_status_sub_;
  rclcpp::Subscription<sp_msgs::msg::AutoaimTargetMsg>::SharedPtr autoaim_target_sub_;
  rclcpp::Subscription<sp_msgs::msg::GimbalCmd>::SharedPtr gimbal_cmd_sub_;

  // IMU mailbox
  std::mutex imu_mutex_;
  std::condition_variable imu_cv_;
  ImuSample imu_mailbox_;
  bool imu_has_new_ = false;
  std::thread imu_thread_;

  // 状态 mailbox：按固定频率发布“最新值”，不在每个状态帧到达时立即发布。
  std::mutex status_mutex_;
  std::condition_variable status_cv_;
  StatusSample status_mailbox_;
  std::thread status_thread_;

  // 导航信息 latest-value
  mutable std::mutex nav_mutex_;
  std::vector<int8_t> invincible_enemy_ids_;
  std::vector<int8_t> autoaim_target_ids_;
  std::chrono::steady_clock::time_point enemy_status_time_{};
  std::chrono::steady_clock::time_point autoaim_target_time_{};

  std::atomic<bool> quit_{false};
  bool owns_context_ = false;

  mutable std::mutex observer_cb_mutex_;
  CmdLatencyObserver cmd_latency_observer_;

  mutable std::mutex stats_mutex_;
  Stats stats_;

  uint32_t status_seq_ = 0;
  uint32_t cmd_seq_ = 0;
};

}  // namespace io

#endif  // IO__ROS2__MCU_ROS_BRIDGE_HPP
