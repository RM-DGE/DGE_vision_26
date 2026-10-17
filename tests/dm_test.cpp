/**
 * @file dm_test.cpp
 * @brief 达妙（DM）IMU 读取测试
 *
 * 测试内容：
 *   1. 初始化 io::DM_IMU；
 *   2. 以约 1 kHz 频率读取四元数，转换为 z/y/x 欧拉角（度）并打印；
 *   3. 用于确认 IMU 连接、零漂和姿态方向是否正确。
 *
 * 使用方法（先在 CMakeLists.txt 中取消 dm_test 相关注释并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/dm_test
 *
 * 操作：Ctrl+C 退出。IMU 的串口配置在 io/dm_imu 驱动内部，无需命令行参数。
 */

#include <chrono>
#include <thread>

#include "io/dm_imu/dm_imu.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

int main()
{
  tools::Exiter exiter;
  io::DM_IMU imu;

  while (!exiter.exit()) {
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(1ms);  // 约 1 kHz 读取

    Eigen::Quaterniond q = imu.imu_at(timestamp);

    // 四元数转 z-y-x 欧拉角，并换算成度后打印
    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0) * 57.3;
    tools::logger()->info("z{:.2f} y{:.2f} x{:.2f} degree", eulers[0], eulers[1], eulers[2]);
  }

  return 0;
}