#ifndef IO__ROS2_HPP
#define IO__ROS2_HPP

#ifndef ENABLE_SIM_MODE
#include "publish2nav.hpp"
#include "subscribe2nav.hpp"
#endif

#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <deque>
#include <mutex>
#include <memory>
#include <optional>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace io
{
class ROS2
{
public:
  ROS2();

  ~ROS2();

#ifndef ENABLE_SIM_MODE
  void publish(const Eigen::Vector4d & target_pos);

  std::vector<int8_t> subscribe_enemy_status();

  std::vector<int8_t> subscribe_autoaim_target();
#endif

  bool read_image(cv::Mat & img, std::chrono::steady_clock::time_point & t);

  Eigen::Quaterniond get_gimbal_quaternion(std::chrono::steady_clock::time_point t);

  void send_gimbal_command(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
                           float pitch, float pitch_vel, float pitch_acc);

  template <typename T>
  std::shared_ptr<rclcpp::Publisher<T>> create_publisher(
    const std::string & node_name, const std::string & topic_name, size_t queue_size)
  {
    auto node = std::make_shared<rclcpp::Node>(node_name);

    auto publisher = node->create_publisher<T>(topic_name, queue_size);

    std::thread([node]() { rclcpp::spin(node); }).detach();

    return publisher;
  }

private:
#ifndef ENABLE_SIM_MODE
  std::shared_ptr<Publish2Nav> publish2nav_;
  std::shared_ptr<Subscribe2Nav> subscribe2nav_;

  std::unique_ptr<std::thread> publish_spin_thread_;
  std::unique_ptr<std::thread> subscribe_spin_thread_;
#endif

  cv::Mat last_image_;
  std::chrono::steady_clock::time_point last_image_time_;
  std::optional<std::chrono::steady_clock::time_point> delivered_image_time_;
  std::deque<std::pair<std::chrono::steady_clock::time_point, Eigen::Quaterniond>> gimbal_queue_;
  mutable std::mutex image_mutex_;
  mutable std::mutex gimbal_mutex_;

  std::shared_ptr<rclcpp::Node> ros2_node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gimbal_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_cmd_pub_;

  void init_image_subscription();
  void init_gimbal_subscription();

  std::chrono::steady_clock::time_point ros2_time_to_steady_clock(const rclcpp::Time & ros_time);
  std::optional<std::chrono::steady_clock::duration> time_offset_;
};

}  // namespace io
#endif
