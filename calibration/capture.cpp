#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

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
  // 标定板尺寸从配置读取，必须与calibrate_camera/calibrate_handeye保持一致，
  // 否则此处识别结果与后续标定程序不符，操作者无法确认采集是否可用
  auto yaml = YAML::LoadFile(config_path);
  cv::Size pattern_size(yaml["pattern_cols"].as<int>(), yaml["pattern_rows"].as<int>());

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  int count = 0;
  while (true) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = cboard.imu_at(timestamp);

    // 在图像上显示欧拉角，用来判断imuabs系的xyz正方向，同时判断imu是否存在零漂
    auto img_with_ypr = img.clone();
    Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
    tools::draw_text(img_with_ypr, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});

    std::vector<cv::Point2f> corners_2d;
    auto success = cv::findChessboardCorners(
      img, pattern_size, corners_2d,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    cv::drawChessboardCorners(img_with_ypr, pattern_size, corners_2d, success);  // 显示识别结果

    // 明确提示识别结果，避免保存下无法用于标定的图片
    tools::draw_text(
      img_with_ypr, success ? "DETECTED" : "NOT DETECTED", {40, 160},
      success ? cv::Scalar{0, 255, 0} : cv::Scalar{0, 0, 255});

    cv::resize(img_with_ypr, img_with_ypr, {}, 0.5, 0.5);  // 显示时缩小图片尺寸

    // 按“s”保存图片和对应四元数，按“q”退出程序
    cv::imshow("Press s to save, q to quit", img_with_ypr);
    auto key = cv::waitKey(1);
    if (key == 'q')
      break;
    else if (key != 's')
      continue;

    // 保存图片和四元数。编号必须连续：标定程序遇缺号即停止读取且不报错
    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    auto q_path = fmt::format("{}/{}.txt", output_folder, count);
    cv::imwrite(img_path, img);
    write_q(q_path, q);
    if (success)
      tools::logger()->info("[{}] Saved in {}", count, output_folder);
    else
      tools::logger()->warn("[{}] Saved in {}，但未识别到标定板，该图无法用于标定", count, output_folder);
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

  auto yaml = YAML::LoadFile(config_path);
  tools::logger()->info(
    "标定板尺寸为{}列{}行（棋盘格内角点），读自 {}", yaml["pattern_cols"].as<int>(),
    yaml["pattern_rows"].as<int>(), config_path);

  // 主循环，保存图片和对应四元数
  capture_loop(config_path, "can0", output_folder);

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
