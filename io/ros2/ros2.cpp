#include "ros2.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>

namespace io
{
ROS2::ROS2()
{
  rclcpp::init(0, nullptr);

#ifndef ENABLE_SIM_MODE
  publish2nav_ = std::make_shared<Publish2Nav>();

  subscribe2nav_ = std::make_shared<Subscribe2Nav>();

  publish_spin_thread_ = std::make_unique<std::thread>([this]() { publish2nav_->start(); });

  subscribe_spin_thread_ = std::make_unique<std::thread>([this]() { subscribe2nav_->start(); });
#endif

  ros2_node_ = std::make_shared<rclcpp::Node>("sp25_learn_sim");
  std::cerr << "[ROS2] Node created" << std::endl;
  
  gimbal_cmd_pub_ = ros2_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/rm_gimbal/cmd", rclcpp::SensorDataQoS());
  std::cerr << "[ROS2] Gimbal command publisher created" << std::endl;
  
  init_image_subscription();
  std::cerr << "[ROS2] Image subscription created" << std::endl;
  
  init_gimbal_subscription();
  std::cerr << "[ROS2] Gimbal subscription created" << std::endl;

  std::cerr << "[ROS2] Starting spin thread..." << std::endl;
  std::thread([this]() { 
    std::cerr << "[ROS2] Spin thread started" << std::endl;
    rclcpp::spin(ros2_node_); 
  }).detach();
  std::cerr << "[ROS2] ROS2 constructor done" << std::endl;
}

ROS2::~ROS2()
{
  rclcpp::shutdown();
#ifndef ENABLE_SIM_MODE
  publish_spin_thread_->join();
  subscribe_spin_thread_->join();
#endif
}

#ifndef ENABLE_SIM_MODE
void ROS2::publish(const Eigen::Vector4d & target_pos) { publish2nav_->send_data(target_pos); }

std::vector<int8_t> ROS2::subscribe_enemy_status()
{
  return subscribe2nav_->subscribe_enemy_status();
}

std::vector<int8_t> ROS2::subscribe_autoaim_target()
{
  return subscribe2nav_->subscribe_autoaim_target();
}
#endif

void ROS2::init_image_subscription()
{
  std::cerr << "[ROS2::init_image_subscription] Creating subscription to /image_raw" << std::endl;
  
  image_sub_ = ros2_node_->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
      static int msg_count = 0;
      if (++msg_count == 1 || msg_count % 30 == 0) {
        std::cerr << "[ROS2::image_callback] Received message #" << msg_count 
                  << " size: " << msg->width << "x" << msg->height << std::endl;
      }
      
      std::lock_guard<std::mutex> lock(image_mutex_);
      try {
        auto cv_img = cv_bridge::toCvShare(msg, "bgr8");
        last_image_ = cv_img->image.clone();
        last_image_time_ = ros2_time_to_steady_clock(rclcpp::Time(msg->header.stamp));
        
        if (msg_count == 1) {
          std::cerr << "[ROS2::image_callback] Successfully converted image to OpenCV" << std::endl;
        }
      } catch (const std::exception & e) {
        std::cerr << "[ROS2::image_callback] Error: " << e.what() << std::endl;
      }
    });
  
  std::cerr << "[ROS2::init_image_subscription] Subscription created" << std::endl;
}

void ROS2::init_gimbal_subscription()
{
  gimbal_pose_sub_ = ros2_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/gimbal_pose", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(gimbal_mutex_);
      Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                           msg->pose.orientation.y, msg->pose.orientation.z);
      auto t = ros2_time_to_steady_clock(rclcpp::Time(msg->header.stamp));

      gimbal_queue_.emplace_back(t, q);
      if (gimbal_queue_.size() > 1000) gimbal_queue_.pop_front();
    });
}

std::chrono::steady_clock::time_point ROS2::ros2_time_to_steady_clock(const rclcpp::Time & ros_time)
{
  if (!time_offset_) {
    time_offset_ = std::chrono::steady_clock::now().time_since_epoch() -
                   std::chrono::nanoseconds(ros_time.nanoseconds());
  }
  auto ros_duration = std::chrono::nanoseconds(ros_time.nanoseconds());
  return std::chrono::steady_clock::time_point(ros_duration + *time_offset_);
}

bool ROS2::read_image(cv::Mat & img, std::chrono::steady_clock::time_point & t)
{
  static int call_count = 0;
  if (++call_count % 30 == 0) {
    std::cerr << "[ROS2::read_image] Called " << call_count << " times, image empty: " 
              << last_image_.empty() << " size: " << last_image_.size() << std::endl;
  }
  
  std::lock_guard<std::mutex> lock(image_mutex_);
  if (last_image_.empty()) return false;
  img = last_image_;
  t = last_image_time_;
  return true;
}

Eigen::Quaterniond ROS2::get_gimbal_quaternion(std::chrono::steady_clock::time_point t)
{
  std::lock_guard<std::mutex> lock(gimbal_mutex_);
  
  if (gimbal_queue_.empty()) return Eigen::Quaterniond::Identity();

  auto it = gimbal_queue_.rbegin();
  while (it != gimbal_queue_.rend() && it->first > t) ++it;

  if (it == gimbal_queue_.rend()) return gimbal_queue_.front().second;
  if (it == gimbal_queue_.rbegin()) return gimbal_queue_.back().second;

  auto it_next = std::prev(it.base());
  auto & [t1, q1] = *std::next(it).base();
  auto & [t2, q2] = *it_next;

  double alpha = static_cast<double>((t - t1).count()) / static_cast<double>((t2 - t1).count());
  alpha = std::clamp(alpha, 0.0, 1.0);
  return q1.slerp(alpha, q2);
}

void ROS2::send_gimbal_command(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
                               float pitch, float pitch_vel, float pitch_acc)
{
  static int cmd_count = 0;
  if (++cmd_count % 10 == 0) {
    std::cerr << "[ROS2::send_gimbal_command] #" << cmd_count 
              << " Yaw: " << yaw << " Pitch: " << pitch 
              << " Fire: " << (fire ? 1 : 0) << std::endl;
  }
  
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data = {
    static_cast<double>(control),
    static_cast<double>(fire),
    static_cast<double>(yaw),
    static_cast<double>(yaw_vel),
    static_cast<double>(yaw_acc),
    static_cast<double>(pitch),
    static_cast<double>(pitch_vel),
    static_cast<double>(pitch_acc)
  };
  gimbal_cmd_pub_->publish(msg);
}

}  // namespace io
