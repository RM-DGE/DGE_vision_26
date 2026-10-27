/**
 * @file mcu_ros_delay_test.cpp
 * @brief 串口 <-> ROS2 <-> 导航 的链路延时测试
 *
 * 参考 tests/detector_video_test.cpp 的组织方式：命令行参数 + 主循环 +
 * 统计输出。区别是本测试不需要相机/视频，只测通信链路：
 *
 *   1. 下位机 IMU/状态 -> CBoard 观察者 -> ROS 发布 -> 订阅端回调；
 *   2. 导航信息 -> ROS 发布 -> 订阅端回调（导航 -> 自瞄这一跳）；
 *   3. 自瞄指令 -> /gimbal/cmd -> 桥接层 -> CBoard::send()（自瞄 -> 下位机这一跳）。
 *
 * 默认 loopback 模式使用 FakeBoard 以 1 kHz 产生 IMU、50 Hz 产生状态，
 * 并模拟 MCU 接收云台指令，因此没有硬件也能跑出完整链路延时。
 *
 * 使用方法：
 *   # 先编译 ROS 接口
 *   ./ros2_ws/build.sh
 *   source ~/dge_vision_ros2_ws/install/setup.bash
 *
 *   # 编译并运行（OpenCV 的 CommandLineParser 要求 -c=value 这种写法）
 *   cmake -B build -S . && cmake --build build --target mcu_ros_delay_test -j
 *   ./build/mcu_ros_delay_test -c=configs/sentry.yaml -d=10 -r=100
 *
 *   # 真机串口模式（需要 MCU 在线；没有 MCU 也能测“发布->写串口”这一跳）
 *   ./build/mcu_ros_delay_test -c=configs/sentry.yaml -m=real -d=10
 *
 * 参数：
 *   -c= 配置文件
 *   -d= 测试时长（秒）
 *   -r= /gimbal/cmd 发布频率（Hz）
 *   -m= loopback | real
 */

#include <fmt/core.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/cboard.hpp"
#include "io/ros2/mcu_ros_bridge.hpp"
#include "rclcpp/executors.hpp"
#include "tools/exiter.hpp"

namespace
{
using namespace std::chrono_literals;

double percentile(std::vector<double> values, double p)
{
  if (values.empty()) return std::nan("");
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(
    std::min<double>(values.size() - 1, std::max<double>(0, p * (values.size() - 1))));
  return values[index];
}

void print_stats(const std::string & name, const std::vector<double> & values)
{
  if (values.empty()) {
    fmt::print("[delay] {:20s} no samples\n", name);
    return;
  }
  fmt::print(
    "[delay] {:20s} n={:6d}  p50={:8.3f} ms  p90={:8.3f} ms  p99={:8.3f} ms  max={:8.3f} ms\n",
    name, static_cast<int>(values.size()), percentile(values, 0.50), percentile(values, 0.90),
    percentile(values, 0.99), percentile(values, 1.0));
}

// 无硬件替身：产生 IMU/状态样本，并记录 MCU 收到的云台指令。
class FakeBoard : public io::CBoardBase
{
public:
  FakeBoard()
  {
    imu_thread_ = std::thread([this]() { imu_loop(); });
    status_thread_ = std::thread([this]() { status_loop(); });
  }

  ~FakeBoard() override
  {
    quit_ = true;
    if (imu_thread_.joinable()) imu_thread_.join();
    if (status_thread_.joinable()) status_thread_.join();
  }

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point) override
  {
    std::lock_guard<std::mutex> lock(q_mutex_);
    return latest_q_;
  }

  void send(io::Command) const override {}

  void set_imu_observer(io::ImuObserver observer) override
  {
    std::lock_guard<std::mutex> lock(observer_mutex_);
    imu_observer_ = std::move(observer);
  }

  void set_status_observer(io::StatusObserver observer) override
  {
    std::lock_guard<std::mutex> lock(observer_mutex_);
    status_observer_ = std::move(observer);
  }

  uint64_t sample_count() const { return imu_count_.load(); }

private:
  void imu_loop()
  {
    // 1 kHz，模拟下位机 IMU 帧。
    auto next = std::chrono::steady_clock::now();
    double phase = 0.0;
    while (!quit_) {
      const auto now = std::chrono::steady_clock::now();
      phase += 0.002;
      Eigen::Quaterniond q(Eigen::AngleAxisd(0.5 * std::sin(phase), Eigen::Vector3d::UnitZ()));
      {
        std::lock_guard<std::mutex> lock(q_mutex_);
        latest_q_ = q;
      }
      io::ImuObserver observer;
      {
        std::lock_guard<std::mutex> lock(observer_mutex_);
        observer = imu_observer_;
      }
      if (observer) observer(io::ImuSample{q, now});
      imu_count_++;

      next += 1ms;
      std::this_thread::sleep_until(next);
      if (std::chrono::steady_clock::now() - next > 100ms) next = std::chrono::steady_clock::now();
    }
  }

  void status_loop()
  {
    auto next = std::chrono::steady_clock::now();
    while (!quit_) {
      const auto now = std::chrono::steady_clock::now();
      io::StatusObserver observer;
      {
        std::lock_guard<std::mutex> lock(observer_mutex_);
        observer = status_observer_;
      }
      if (observer) observer(io::StatusSample{25.0, 1, 0, 0.0, now});

      next += 20ms;
      std::this_thread::sleep_until(next);
      if (std::chrono::steady_clock::now() - next > 100ms) next = std::chrono::steady_clock::now();
    }
  }

  std::atomic<bool> quit_{false};
  std::atomic<uint64_t> imu_count_{0};

  mutable std::mutex q_mutex_;
  Eigen::Quaterniond latest_q_ = Eigen::Quaterniond::Identity();

  mutable std::mutex observer_mutex_;
  io::ImuObserver imu_observer_;
  io::StatusObserver status_observer_;

  std::thread imu_thread_;
  std::thread status_thread_;
};

// 导航侧替身：发布 enemy_status/autoaim_target，订阅 MCU 数据和自瞄目标。
class TestNode : public rclcpp::Node
{
public:
  TestNode() : Node("dge_mcu_delay_test")
  {
    auto imu_qos = rclcpp::SensorDataQoS().keep_last(5);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/mcu/imu", imu_qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_delays_ms_.push_back((now() - rclcpp::Time(msg->header.stamp)).seconds() * 1000.0);
      });

    status_sub_ = create_subscription<sp_msgs::msg::McuStatus>(
      "/mcu/status", rclcpp::QoS(1).reliable(), [this](sp_msgs::msg::McuStatus::SharedPtr msg) {
        status_delays_ms_.push_back((now() - rclcpp::Time(msg->header.stamp)).seconds() * 1000.0);
      });

    target_sub_ = create_subscription<sp_msgs::msg::AutoAimTarget>(
      "/auto_aim_target_info", rclcpp::QoS(1).reliable(),
      [this](sp_msgs::msg::AutoAimTarget::SharedPtr msg) {
        target_delays_ms_.push_back((now() - rclcpp::Time(msg->header.stamp)).seconds() * 1000.0);
      });

    enemy_pub_ = create_publisher<sp_msgs::msg::EnemyStatusMsg>(
      "/enemy_status", rclcpp::QoS(1).reliable());
    autoaim_pub_ = create_publisher<sp_msgs::msg::AutoaimTargetMsg>(
      "/autoaim_target", rclcpp::QoS(1).reliable());
    gimbal_pub_ = create_publisher<sp_msgs::msg::GimbalCmd>(
      "/gimbal/cmd", rclcpp::QoS(1).best_effort());

    // 回环订阅：测“导航发布 -> 自瞄订阅”这一跳的 DDS 延时。
    enemy_echo_sub_ = create_subscription<sp_msgs::msg::EnemyStatusMsg>(
      "/enemy_status", rclcpp::QoS(1).reliable(),
      [this](sp_msgs::msg::EnemyStatusMsg::SharedPtr msg) {
        nav_delays_ms_.push_back((now() - rclcpp::Time(msg->timestamp)).seconds() * 1000.0);
      });
  }

  void publish_nav_info()
  {
    sp_msgs::msg::EnemyStatusMsg enemy;
    enemy.timestamp = now();
    enemy.invincible_enemy_ids = {2, 5};
    enemy_pub_->publish(enemy);

    sp_msgs::msg::AutoaimTargetMsg target;
    target.timestamp = now();
    target.target_ids = {1, 3};
    autoaim_pub_->publish(target);
  }

  void publish_gimbal_cmd(uint32_t seq)
  {
    sp_msgs::msg::GimbalCmd msg;
    msg.header.stamp = now();
    msg.header.frame_id = "gimbal";
    msg.seq = seq;
    msg.control = true;
    msg.shoot = false;
    msg.yaw = 0.01f;
    msg.pitch = 0.02f;
    gimbal_pub_->publish(msg);
  }

  std::vector<double> imu_delays_ms_;
  std::vector<double> status_delays_ms_;
  std::vector<double> target_delays_ms_;
  std::vector<double> nav_delays_ms_;

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sp_msgs::msg::McuStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<sp_msgs::msg::AutoAimTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<sp_msgs::msg::EnemyStatusMsg>::SharedPtr enemy_echo_sub_;
  rclcpp::Publisher<sp_msgs::msg::EnemyStatusMsg>::SharedPtr enemy_pub_;
  rclcpp::Publisher<sp_msgs::msg::AutoaimTargetMsg>::SharedPtr autoaim_pub_;
  rclcpp::Publisher<sp_msgs::msg::GimbalCmd>::SharedPtr gimbal_pub_;

};

std::string make_test_config(const std::string & source)
{
  // 复制原配置，只覆盖测试需要的 ros2 字段，写到 /tmp/opencode。
  YAML::Node yaml;
  try {
    yaml = YAML::LoadFile(source);
  } catch (const std::exception &) {
    yaml = YAML::Node(YAML::NodeType::Map);
  }
  auto ros2 = yaml["ros2"];
  if (!ros2) ros2 = YAML::Node(YAML::NodeType::Map);
  ros2["enabled"] = true;
  ros2["enable_remote_gimbal_cmd"] = true;
  ros2["imu_publish_rate"] = 1000.0;
  ros2["status_publish_rate"] = 50.0;
  yaml["ros2"] = ros2;

  std::filesystem::create_directories("/tmp/opencode");
  const std::string path = "/tmp/opencode/dge_mcu_ros_delay_test.yaml";
  std::ofstream out(path);
  out << yaml;
  return path;
}

}  // namespace

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/sentry.yaml | yaml配置文件路径}"
  "{duration d     | 10                  | 测试时长（秒）}"
  "{rate r         | 100                 | /gimbal/cmd 发布频率（Hz）}"
  "{mode m         | loopback            | loopback 或 real}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto config_path = cli.get<std::string>("config-path");
  const auto duration_s = cli.get<double>("duration");
  const auto rate_hz = cli.get<double>("rate");
  const auto mode = cli.get<std::string>("mode");

  fmt::print(
    "[info] config={} duration={} s rate={} Hz mode={}\n", config_path, duration_s, rate_hz, mode);

  tools::Exiter exiter;

  std::unique_ptr<io::CBoardBase> board;
  std::string bridge_config = config_path;
  const bool loopback = (mode != "real");

  if (loopback) {
    board = std::make_unique<FakeBoard>();
    bridge_config = make_test_config(config_path);
    fmt::print("[info] loopback: fake MCU, no serial device needed\n");
  } else {
    board = std::make_unique<io::CBoard>(config_path);
    // 真机模式也强制打开 /gimbal/cmd 回环，才能测“发布->桥接->串口”。
    bridge_config = make_test_config(config_path);
    fmt::print("[info] real serial: {}\n", config_path);
  }

  io::McuRosBridge bridge(*board, bridge_config);

  std::mutex cmd_mutex;
  std::vector<double> cmd_delays_ms;
  std::vector<double> send_call_ms;
  bridge.set_cmd_latency_observer([&](double age_ms, double send_us) {
    std::lock_guard<std::mutex> lock(cmd_mutex);
    cmd_delays_ms.push_back(age_ms);
    send_call_ms.push_back(send_us / 1000.0);  // us -> ms
  });

  auto test_node = std::make_shared<TestNode>();

  rclcpp::executors::SingleThreadedExecutor test_executor;
  test_executor.add_node(test_node);
  std::thread test_spin([&test_executor]() { test_executor.spin(); });

  const auto start = std::chrono::steady_clock::now();
  const auto period = rate_hz > 0.0 ? std::chrono::duration<double>(1.0 / rate_hz)
                                    : std::chrono::duration<double>(10ms);
  auto next = std::chrono::steady_clock::now();
  uint32_t seq = 0;
  uint64_t nav_published = 0;

  const auto nav_every = static_cast<uint32_t>(std::max(1.0, rate_hz / 10.0));

  while (!exiter.exit()) {
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start);
    if (elapsed.count() >= duration_s) break;

    test_node->publish_gimbal_cmd(seq++);

    if (seq % nav_every == 0) {
      test_node->publish_nav_info();
      nav_published++;
    }

    // 每 10 个控制周期回传一次目标信息，测上行目标话题。
    if (seq % 10 == 0) {
      bridge.publish_target(
        Eigen::Vector3d{1.0, 2.0, 0.3}, Eigen::Vector3d{0.1, 0.0, 0.0}, 1, true, "gimbal");
    }

    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
  }

  // 先 shutdown 再 join：Humble 的 SingleThreadedExecutor 在空 wait set 上
  // 等待时，单独 cancel() 不一定能立刻唤醒 spin 线程。
  rclcpp::shutdown();
  if (test_spin.joinable()) test_spin.join();

  const auto stats = bridge.stats();

  print_stats("imu uplink", test_node->imu_delays_ms_);
  print_stats("status uplink", test_node->status_delays_ms_);
  print_stats("target uplink", test_node->target_delays_ms_);
  print_stats("nav -> autoaim", test_node->nav_delays_ms_);
  fmt::print("[info] status 延时包含 status_publish_rate 的固定频率采样保持\n");

  {
    std::lock_guard<std::mutex> lock(cmd_mutex);
    print_stats("cmd publish->send", cmd_delays_ms);
    print_stats("board.send() call", send_call_ms);
  }

  const double drop_rate =
    stats.gimbal_cmd_received > 0
      ? 100.0 * static_cast<double>(stats.gimbal_cmd_dropped) / stats.gimbal_cmd_received
      : 0.0;

  fmt::print(
    "[stats] imu_pub={} status_pub={} cmd_rx={} cmd_sent={} cmd_drop={} drop_rate={:.3f}% "
    "last_cmd_age={:.3f} ms\n",
    stats.imu_published, stats.status_published, stats.gimbal_cmd_received, stats.gimbal_cmd_sent,
    stats.gimbal_cmd_dropped, drop_rate, stats.last_cmd_age_ms);

  fmt::print("[info] nav published: {}  fake IMU samples: {}\n", nav_published,
             loopback ? static_cast<FakeBoard *>(board.get())->sample_count() : 0);
  return 0;
}
