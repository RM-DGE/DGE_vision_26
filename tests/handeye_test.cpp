/**
 * @file handeye_test.cpp
 * @brief 手眼标定验证：把世界坐标系下的网格点投影到相机画面
 *
 * 测试内容：
 *   1. 从 yaml 读取网格高度 height、每边点数 grid_num、点间距 grid_size、IMU 延迟 delay；
 *   2. 按当前云台 IMU 姿态生成世界系网格，并用 Solver 投影到图像上；
 *   3. 若网格点与真实场地标定点对齐，说明手眼标定参数正确；
 *   4. 画面上还会叠加当前 yaw/pitch/roll 角度。
 *
 * 使用方法（先在 CMakeLists.txt 中取消 handeye_test 相关注释并重新编译）：
 *   cmake -B build && cmake --build build -j
 *   ./build/handeye_test -c configs/handeye.yaml         # 只计算并打印日志
 *   ./build/handeye_test -c configs/handeye.yaml -d      # 显示投影结果
 *
 * 操作：显示模式下按 q 或 Ctrl+C 退出。
 */

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/core/eigen.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/handeye.yaml | yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }";

// 世界坐标到像素坐标的转换

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");
  // 读取手眼标定验证所需的网格参数
  auto yaml = YAML::LoadFile(config_path);
  auto height = yaml["height"].as<double>();      // 网格平面相对云台的高度
  auto grid_num = yaml["grid_num"].as<int>();     // 每个方向的点数
  auto grid_size = yaml["grid_size"].as<double>();// 点间距
  auto delay = yaml["delay"].as<int>();           // IMU 相对图像的时间延迟(ms)
  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  // 以场地中心为原点，向两侧生成对称的网格点（z = -height）
  std::vector<cv::Point3f> points;
  for (int x = 0; x < grid_num; x++) {
    for (int y = 0; y < grid_num; y++) {
      points.emplace_back(x * grid_size, y * grid_size - grid_num * grid_size / 2, -height);
      points.emplace_back(-x * grid_size, y * grid_size - grid_num * grid_size / 2, -height);
    }
  }
  while (!exiter.exit()) {
    camera.read(img, t);
    // 用延迟补偿后的 IMU 姿态，保证与图像时刻对齐
    q = cboard.imu_at(t - 1ms * delay);
    solver.set_R_gimbal2world(q);
    cv::Mat result = img.clone();
    // 世界坐标 -> 像素坐标，并只绘制落在图像内的点
    std::vector<cv::Point2f> projectedPoints = solver.world2pixel(points);
    for (const auto & point : projectedPoints) {
      if (point.x >= 0 && point.x < result.cols && point.y >= 0 && point.y < result.rows) {
        cv::circle(result, point, 3, cv::Scalar(255, 255, 255), -1);
      }
    }
    // 在左上角显示当前姿态角，便于检查 IMU 方向
    Eigen::Vector3d euler = solver.R_gimbal2world().eulerAngles(2, 1, 0) * 180.0 / M_PI;
    tools::draw_text(result, fmt::format("yaw   {:.2f}", euler[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(result, fmt::format("pitch {:.2f}", euler[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(result, fmt::format("roll  {:.2f}", euler[2]), {40, 120}, {0, 0, 255});
    if (!display) continue;
    cv::imshow("result", result);
    if (cv::waitKey(1) == 'q') break;
  }
}