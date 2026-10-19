#include <fmt/core.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "chessboard.hpp"
#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }";

void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  // 输出顺序为wxyz
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.close();
}

void capture_loop(
  const std::string & config_path, const std::string & can, const std::string & output_folder)
{
  io::Camera camera(config_path);

  // 棋盘格内角点数从 YAML 读取，避免和 calibrate_* / configs/calibration.yaml 不一致。
  // 注意：pattern_cols/rows 是内角点数，不是方格数。10x7 方格 -> 9x6 内角点。
  auto yaml = YAML::LoadFile(config_path);
  const auto pattern_cols = yaml["pattern_cols"].as<int>();
  const auto pattern_rows = yaml["pattern_rows"].as<int>();
  const auto square_size_mm = yaml["square_size_mm"]
                                ? yaml["square_size_mm"].as<double>()
                                : yaml["center_distance_mm"].as<double>();
  const auto capture_imu = yaml["capture_imu"] ? yaml["capture_imu"].as<bool>() : true;

  const cv::Size pattern_size(pattern_cols, pattern_rows);
  tools::logger()->info(
    "棋盘格内角点: {} x {}，方格边长 {:.2f} mm", pattern_cols, pattern_rows, square_size_mm);

  // 只标内参时可以不接 CBoard；手眼必须采集四元数。
  std::unique_ptr<io::CBoard> cboard;
  if (capture_imu) {
    cboard = std::make_unique<io::CBoard>(config_path);
  } else {
    tools::logger()->warn("capture_imu=false：不读取 IMU，保存的四元数为 identity，只能用于内参");
  }

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  int count = 0;
  while (true) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = cboard ? cboard->imu_at(timestamp) : Eigen::Quaterniond::Identity();

    // 在图像上显示欧拉角，用来判断imuabs系的xyz正方向，同时判断imu是否存在零漂
    auto img_with_ypr = img.clone();
    Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
    tools::draw_text(img_with_ypr, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});

    std::vector<cv::Point2f> corners;
    auto success = calibration::find_chessboard(img, pattern_size, corners);
    cv::drawChessboardCorners(img_with_ypr, pattern_size, corners, success);  // 显示识别结果
    cv::resize(img_with_ypr, img_with_ypr, {}, 0.5, 0.5);  // 显示时缩小图片尺寸

    // 按“s”保存图片和对应四元数，按“q”退出程序
    cv::imshow("Press s to save, q to quit", img_with_ypr);
    auto key = cv::waitKey(1);
    if (key == 'q')
      break;
    else if (key != 's')
      continue;

    // 保存图片和四元数
    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    auto q_path = fmt::format("{}/{}.txt", output_folder, count);
    cv::imwrite(img_path, img);
    write_q(q_path, q);
    tools::logger()->info("[{}] Saved in {}", count, output_folder);
  }

  // 离开该作用域时，camera和cboard会自动关闭
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");

  // 新建输出文件夹
  std::filesystem::create_directory(output_folder);

  tools::logger()->info("默认标定板尺寸为10列7行");
  // 主循环，保存图片和对应四元数
  capture_loop(config_path, "can0", output_folder);

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
