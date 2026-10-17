/**
 * @file topic_loop_test.cpp
 * @brief ROS2 收发回环测试：同一节点内发布并订阅 EnemyStatusMsg
 *
 * 测试内容：
 *   1. 创建发布者，向 enemy_status 话题发送无敌敌方 id 和时间戳；
 *   2. 每隔 3 次发布调用一次 subscribe_enemy_status()，检查消息能否回环收到；
 *   3. 打印发送时间戳，用于估算通信延迟；
 *   4. 共进行约 1000 次后退出。
 *
 * 使用方法（需要 ROS2 环境，先在 CMakeLists.txt 中启用 topic_loop_test 目标）：
 *   cmake -B build && cmake --build build -j
 *   ./build/topic_loop_test
 *
 * 操作：Ctrl+C 退出。注意循环内同时有 5 us 和 1 s 两次 sleep，实际频率约 1 Hz。
 */

#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "io/ros2/ros2.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  tools::Exiter exiter;
  io::ROS2 ros2;
  rclcpp::Clock clock;
  // 节点名 temp_node，话题 enemy_status，队列深度 10
  auto string_publisher =
    ros2.create_publisher<sp_msgs::msg::EnemyStatusMsg>("temp_node", "enemy_status", 10);

  int i = 0;
  while (!exiter.exit()) {
    sp_msgs::msg::EnemyStatusMsg msg;
    msg.invincible_enemy_ids = {1, 2, 3};  // 构造固定的无敌 id 列表
    msg.timestamp = clock.now();
    string_publisher->publish(msg);
    RCLCPP_INFO(
      rclcpp::get_logger("msg send timestamp is"), "msg.timestamp: %d.%09u", msg.timestamp.sec,
      msg.timestamp.nanosec);

    i++;
    std::this_thread::sleep_for(std::chrono::microseconds(5));

    // 每 3 次发布检查一次订阅端能否收到
    if (i % 3 == 0) {
      auto x = ros2.subscribe_enemy_status();
      // tools::logger()->info("invincible enemy ids size is{}", x.size());
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (i > 1000) break;
  }

  return 0;
}
