#include "io/ros2/mcu_ros_bridge.hpp"

#include <yaml-cpp/yaml.h>

#include <utility>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "tools/logger.hpp"

namespace io
{
namespace
{
double get_double(const YAML::Node & node, const char * key, double fallback)
{
  return node[key] ? node[key].as<double>() : fallback;
}

std::string get_string(const YAML::Node & node, const char * key, const std::string & fallback)
{
  return node[key] ? node[key].as<std::string>() : fallback;
}

bool get_bool(const YAML::Node & node, const char * key, bool fallback)
{
  return node[key] ? node[key].as<bool>() : fallback;
}
}  // namespace

McuRosBridge::Config McuRosBridge::load_config(const std::string & config_path)
{
  Config config;

  YAML::Node yaml;
  try {
    yaml = YAML::LoadFile(config_path);
  } catch (const std::exception & e) {
    tools::logger()->warn("[ROS2] cannot read {}: {}", config_path, e.what());
    return config;
  }

  const auto node = yaml["ros2"];
  if (!node) return config;

  config.enabled = get_bool(node, "enabled", config.enabled);
  config.enable_remote_gimbal_cmd =
    get_bool(node, "enable_remote_gimbal_cmd", config.enable_remote_gimbal_cmd);
  config.node_name = get_string(node, "node_name", config.node_name);
  config.imu_topic = get_string(node, "imu_topic", config.imu_topic);
  config.status_topic = get_string(node, "status_topic", config.status_topic);
  config.gimbal_cmd_topic = get_string(node, "gimbal_cmd_topic", config.gimbal_cmd_topic);
  config.nav_enemy_status_topic =
    get_string(node, "nav_enemy_status_topic", config.nav_enemy_status_topic);
  config.nav_autoaim_target_topic =
    get_string(node, "nav_autoaim_target_topic", config.nav_autoaim_target_topic);
  config.target_pos_topic = get_string(node, "target_pos_topic", config.target_pos_topic);
  config.target_info_topic = get_string(node, "target_info_topic", config.target_info_topic);
  config.imu_frame_id = get_string(node, "imu_frame_id", config.imu_frame_id);
  config.imu_publish_rate = get_double(node, "imu_publish_rate", config.imu_publish_rate);
  config.status_publish_rate = get_double(node, "status_publish_rate", config.status_publish_rate);
  config.max_cmd_age_ms = get_double(node, "max_cmd_age_ms", config.max_cmd_age_ms);
  config.nav_timeout_ms = get_double(node, "nav_timeout_ms", config.nav_timeout_ms);
  config.status_online_timeout_ms =
    get_double(node, "status_online_timeout_ms", config.status_online_timeout_ms);

  return config;
}

bool McuRosBridge::enabled_in_config(const std::string & config_path)
{
  return load_config(config_path).enabled;
}

McuRosBridge::McuRosBridge(CBoardBase & board, const std::string & config_path)
: board_(board), config_(load_config(config_path))
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
    owns_context_ = true;
  }

  node_ = std::make_shared<rclcpp::Node>(config_.node_name);

  // 高频 IMU 用 SensorDataQoS（best_effort + keep_last），
  // 状态/目标用 reliable + keep_last(1)，避免旧消息排队。
  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
    config_.imu_topic, rclcpp::SensorDataQoS().keep_last(5));
  status_pub_ = node_->create_publisher<sp_msgs::msg::McuStatus>(
    config_.status_topic, rclcpp::QoS(1).reliable());
  legacy_pub_ = node_->create_publisher<std_msgs::msg::String>(
    config_.target_pos_topic, rclcpp::QoS(1).reliable());
  target_pub_ = node_->create_publisher<sp_msgs::msg::AutoAimTarget>(
    config_.target_info_topic, rclcpp::QoS(1).reliable());
  gimbal_cmd_pub_ = node_->create_publisher<sp_msgs::msg::GimbalCmd>(
    config_.gimbal_cmd_topic, rclcpp::QoS(1).best_effort());

  enemy_status_sub_ = node_->create_subscription<sp_msgs::msg::EnemyStatusMsg>(
    config_.nav_enemy_status_topic, rclcpp::QoS(1).reliable(),
    [this](sp_msgs::msg::EnemyStatusMsg::SharedPtr msg) { enemy_status_callback(msg); });
  autoaim_target_sub_ = node_->create_subscription<sp_msgs::msg::AutoaimTargetMsg>(
    config_.nav_autoaim_target_topic, rclcpp::QoS(1).reliable(),
    [this](sp_msgs::msg::AutoaimTargetMsg::SharedPtr msg) { autoaim_target_callback(msg); });

  if (config_.enable_remote_gimbal_cmd) {
    gimbal_cmd_sub_ = node_->create_subscription<sp_msgs::msg::GimbalCmd>(
      config_.gimbal_cmd_topic, rclcpp::QoS(1).best_effort(),
      [this](sp_msgs::msg::GimbalCmd::SharedPtr msg) { gimbal_cmd_callback(msg); });
  }

  // 串口读线程只更新 mailbox。
  board_.set_imu_observer([this](const ImuSample & sample) { on_imu(sample); });
  board_.set_status_observer([this](const StatusSample & sample) { on_status(sample); });

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {
    try {
      executor_->spin();
    } catch (const std::exception & e) {
      tools::logger()->warn("[ROS2] executor stopped: {}", e.what());
    }
  });

  imu_thread_ = std::thread(&McuRosBridge::imu_publish_loop, this);
  status_thread_ = std::thread(&McuRosBridge::status_publish_loop, this);

  tools::logger()->info(
    "[ROS2] bridge ready: imu={} status={} cmd={} remote_cmd={}", config_.imu_topic,
    config_.status_topic, config_.gimbal_cmd_topic, config_.enable_remote_gimbal_cmd);
}

McuRosBridge::~McuRosBridge()
{
  quit_ = true;
  imu_cv_.notify_all();
  status_cv_.notify_all();

  if (executor_) executor_->cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
  if (imu_thread_.joinable()) imu_thread_.join();
  if (status_thread_.joinable()) status_thread_.join();

  board_.set_imu_observer(nullptr);
  board_.set_status_observer(nullptr);

  if (owns_context_ && rclcpp::ok()) rclcpp::shutdown();
}

void McuRosBridge::on_imu(const ImuSample & sample)
{
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_mailbox_ = sample;
    imu_has_new_ = true;
  }
  imu_cv_.notify_one();
}

void McuRosBridge::on_status(const StatusSample & sample)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_mailbox_ = sample;
}

void McuRosBridge::imu_publish_loop()
{
  using namespace std::chrono_literals;

  auto next_publish = std::chrono::steady_clock::now();
  const auto period = config_.imu_publish_rate > 0.0
                        ? std::chrono::duration<double>(1.0 / config_.imu_publish_rate)
                        : std::chrono::duration<double>::zero();

  while (!quit_.load()) {
    ImuSample sample;
    {
      std::unique_lock<std::mutex> lock(imu_mutex_);
      imu_cv_.wait(lock, [this]() { return quit_.load() || imu_has_new_; });
      if (quit_.load()) break;
      sample = imu_mailbox_;
      imu_has_new_ = false;
    }

    publish_imu(sample);

    if (period.count() > 0.0) {
      next_publish += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      auto now = std::chrono::steady_clock::now();
      if (next_publish < now) next_publish = now;
      std::this_thread::sleep_until(next_publish);
    }
  }
}

void McuRosBridge::status_publish_loop()
{
  using clock = std::chrono::steady_clock;

  const auto period = config_.status_publish_rate > 0.0
                        ? std::chrono::duration<double>(1.0 / config_.status_publish_rate)
                        : std::chrono::duration<double>(1.0);
  const auto period_steady = std::chrono::duration_cast<clock::duration>(period);

  auto next_publish = clock::now();
  while (!quit_.load()) {
    StatusSample sample;
    {
      std::unique_lock<std::mutex> lock(status_mutex_);
      // 固定频率发布最新值：状态源可能是 175 Hz，但导航只需要 20 Hz。
      // 即使下位机停止发帧，也继续发布“最后一帧 + online=false”，
      // 让导航能靠 status 本身判断掉线。
      status_cv_.wait_until(lock, next_publish, [this]() { return quit_.load(); });
      if (quit_.load()) break;
      sample = status_mailbox_;
    }

    // 还没收到过状态帧时不要发 epoch 时间戳的假消息。
    if (sample.timestamp.time_since_epoch().count() != 0) publish_status(sample);

    next_publish += period_steady;
    if (next_publish < clock::now()) next_publish = clock::now() + period_steady;
  }
}

rclcpp::Time McuRosBridge::to_ros_time(std::chrono::steady_clock::time_point steady_time) const
{
  // steady_clock 不能直接转 ROS 时间：用“当前 system/steady 双读”估计样本的
  // 系统时间。误差在几十微秒量级，导航端用它计算 age 足够。
  const auto system_now = std::chrono::system_clock::now();
  const auto steady_now = std::chrono::steady_clock::now();

  auto age = steady_now - steady_time;
  if (age < std::chrono::steady_clock::duration::zero()) {
    age = std::chrono::steady_clock::duration::zero();
  }

  const auto system_stamp = system_now - age;
  const auto ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(system_stamp.time_since_epoch()).count();
  return rclcpp::Time(ns, RCL_SYSTEM_TIME);
}

void McuRosBridge::publish_imu(const ImuSample & sample)
{
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = to_ros_time(sample.timestamp);
  msg.header.frame_id = config_.imu_frame_id;

  const auto q = sample.q.normalized();
  msg.orientation.w = q.w();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();

  // 串口协议只有四元数，没有角速度/线加速度，按 REP-145 置 -1 表示未知。
  msg.orientation_covariance.fill(0.0);
  msg.orientation_covariance[0] = 1e-4;
  msg.orientation_covariance[4] = 1e-4;
  msg.orientation_covariance[8] = 1e-4;
  msg.angular_velocity_covariance.fill(0.0);
  msg.angular_velocity_covariance[0] = -1.0;
  msg.linear_acceleration_covariance.fill(0.0);
  msg.linear_acceleration_covariance[0] = -1.0;

  imu_pub_->publish(msg);

  const auto age_ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - sample.timestamp)
                        .count();
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.imu_published++;
    stats_.last_imu_age_ms = age_ms;
  }
}

void McuRosBridge::publish_status(const StatusSample & sample)
{
  sp_msgs::msg::McuStatus msg;
  msg.header.stamp = to_ros_time(sample.timestamp);
  msg.header.frame_id = "base_link";
  msg.seq = status_seq_++;
  msg.bullet_speed = static_cast<float>(sample.bullet_speed);
  msg.mode = static_cast<int16_t>(sample.mode);
  msg.shoot_mode = static_cast<int16_t>(sample.shoot_mode);
  msg.ft_angle = static_cast<float>(sample.ft_angle);

  const auto now = std::chrono::steady_clock::now();
  msg.online =
    std::chrono::duration<double, std::milli>(now - sample.timestamp).count() <
    config_.status_online_timeout_ms;

  double imu_age_ms = 1e9;
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (imu_mailbox_.timestamp.time_since_epoch().count() != 0) {
      imu_age_ms =
        std::chrono::duration<double, std::milli>(now - imu_mailbox_.timestamp).count();
    }
  }
  msg.imu_age_ms = static_cast<float>(imu_age_ms);

  status_pub_->publish(msg);

  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.status_published++;
  }
}

void McuRosBridge::publish(const Eigen::Vector4d & target_pos)
{
  auto message = std::make_shared<std_msgs::msg::String>();
  message->data = std::to_string(target_pos[0]) + "," + std::to_string(target_pos[1]) + "," +
                  std::to_string(target_pos[2]) + "," + std::to_string(target_pos[3]);
  legacy_pub_->publish(*message);
}

void McuRosBridge::publish_target(
  const Eigen::Vector3d & position, const Eigen::Vector3d & velocity, int8_t id, bool tracking,
  const std::string & frame_id, double yaw, double v_yaw,
  std::chrono::steady_clock::time_point stamp)
{
  sp_msgs::msg::AutoAimTarget msg;
  msg.header.stamp = stamp.time_since_epoch().count() == 0 ? node_->now() : to_ros_time(stamp);
  msg.header.frame_id = frame_id;
  msg.tracking = tracking;
  msg.id = id;
  msg.position.x = position.x();
  msg.position.y = position.y();
  msg.position.z = position.z();
  msg.velocity.x = velocity.x();
  msg.velocity.y = velocity.y();
  msg.velocity.z = velocity.z();
  msg.yaw = yaw;
  msg.v_yaw = v_yaw;

  target_pub_->publish(msg);
}

void McuRosBridge::send_command(const Command & command)
{
  sp_msgs::msg::GimbalCmd msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "gimbal";
  msg.seq = cmd_seq_++;
  msg.control = command.control;
  msg.shoot = command.shoot;
  msg.yaw = static_cast<float>(command.yaw);
  msg.pitch = static_cast<float>(command.pitch);
  msg.horizon_distance = static_cast<float>(command.horizon_distance);

  gimbal_cmd_pub_->publish(msg);
}

void McuRosBridge::enemy_status_callback(const sp_msgs::msg::EnemyStatusMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(nav_mutex_);
  invincible_enemy_ids_ = msg->invincible_enemy_ids;
  enemy_status_time_ = std::chrono::steady_clock::now();
}

void McuRosBridge::autoaim_target_callback(const sp_msgs::msg::AutoaimTargetMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(nav_mutex_);
  autoaim_target_ids_ = msg->target_ids;
  autoaim_target_time_ = std::chrono::steady_clock::now();
}

void McuRosBridge::gimbal_cmd_callback(const sp_msgs::msg::GimbalCmd::SharedPtr msg)
{
  double age_ms = 0.0;
  if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
    const rclcpp::Time stamp(msg->header.stamp);
    age_ms = (node_->now() - stamp).seconds() * 1000.0;
    if (age_ms < 0.0) age_ms = 0.0;
  }

  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.gimbal_cmd_received++;
    stats_.last_cmd_age_ms = age_ms;
  }

  if (age_ms > config_.max_cmd_age_ms) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.gimbal_cmd_dropped++;
    return;
  }

  Command command;
  command.control = msg->control;
  command.shoot = msg->shoot;
  command.yaw = msg->yaw;
  command.pitch = msg->pitch;
  command.horizon_distance = msg->horizon_distance;

  const auto send_start = std::chrono::steady_clock::now();
  board_.send(command);
  const auto send_end = std::chrono::steady_clock::now();

  const double send_us =
    std::chrono::duration<double, std::micro>(send_end - send_start).count();

  CmdLatencyObserver observer;
  {
    std::lock_guard<std::mutex> lock(observer_cb_mutex_);
    observer = cmd_latency_observer_;
  }
  if (observer) observer(age_ms, send_us);

  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_.gimbal_cmd_sent++;
  stats_.last_send_us = send_us;
}

void McuRosBridge::set_cmd_latency_observer(CmdLatencyObserver observer)
{
  std::lock_guard<std::mutex> lock(observer_cb_mutex_);
  cmd_latency_observer_ = std::move(observer);
}

std::vector<int8_t> McuRosBridge::subscribe_enemy_status()
{
  std::lock_guard<std::mutex> lock(nav_mutex_);
  if (enemy_status_time_.time_since_epoch().count() == 0) return {};
  const auto age_ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - enemy_status_time_)
                        .count();
  if (age_ms > config_.nav_timeout_ms) return {};
  return invincible_enemy_ids_;
}

std::vector<int8_t> McuRosBridge::subscribe_autoaim_target()
{
  std::lock_guard<std::mutex> lock(nav_mutex_);
  if (autoaim_target_time_.time_since_epoch().count() == 0) return {};
  const auto age_ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - autoaim_target_time_)
                        .count();
  if (age_ms > config_.nav_timeout_ms) return {};
  return autoaim_target_ids_;
}

McuRosBridge::Stats McuRosBridge::stats() const
{
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_;
}

}  // namespace io
