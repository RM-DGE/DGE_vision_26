/**
 * @file publish_test.cpp
 * @brief ROS2 发布测试：周期性发送装甲板观测消息
 *
 * 测试内容：
 *   1. 构造 io::ROS2 节点；
 *   2. 每秒发布一条 Eigen::Vector4d {x, y, z, armor_id}，共 1000 条；
 *   3. 可用 ros2 topic echo 检查消息是否正常发出。
 *
 * 使用方法（需要 ROS2 环境，先在 CMakeLists.txt 中启用 publish_test 目标）：
 *   cmake -B build && cmake --build build -j
 *   ./build/publish_test
 *   另开终端：ros2 topic list / ros2 topic echo <topic>
 *
 * 说明：前三个分量是递增值，仅用于验证通信链路，无实际物理意义。
 */

#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "io/ros2/ros2.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  tools::Exiter exiter;
  io::ROS2 ros2;  // 内部完成 rclcpp 初始化与发布者创建

  double i = 0;
  while (!exiter.exit()) {
    // 第 4 个分量放装甲板编号，用于验证枚举序列化
    Eigen::Vector4d data{i, i + 1, 1, auto_aim::ArmorName::sentry + 1};
    ros2.publish(data);
    i++;

    std::this_thread::sleep_for(std::chrono::seconds(1));  // 1 Hz 发布
    if (i > 1000) break;
  }
  return 0;
}
