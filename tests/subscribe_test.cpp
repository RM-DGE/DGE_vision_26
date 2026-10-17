/**
 * @file subscribe_test.cpp
 * @brief ROS2 订阅测试：读取并打印敌方无敌状态
 *
 * 测试内容：
 *   1. 构造 io::ROS2 节点；
 *   2. 循环调用 subscribe_enemy_status() 获取无敌敌方 id 列表；
 *   3. 打印收到的每个 id，用于验证裁判系统/队友节点的消息能否正常到达。
 *
 * 使用方法（需要 ROS2 环境，先在 CMakeLists.txt 中启用 subscribe_test 目标）：
 *   cmake -B build && cmake --build build -j
 *   ./build/subscribe_test
 *
 * 说明：每 500 us 轮询一次；Ctrl+C 退出。
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

  int i = 0;
  while (!exiter.exit()) {
    auto x = ros2.subscribe_enemy_status();  // 返回当前无敌敌方 id 列表
    // tools::logger()->info("invincible enemy ids size is{}", x.size());
    for (const auto & id : x) {
      tools::logger()->info("id:{}", id);
    }
    // i++;

    std::this_thread::sleep_for(std::chrono::microseconds(500));  // 高频轮询
    // if (i > 1000) break;
  }
  return 0;
}
