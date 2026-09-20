#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <opencv2/opencv.hpp>

#include "tools/img_tools.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{no-preview     |                          | 不逐张显示，直接标定 }"
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float center_distance)
{
  std::vector<cv::Point3f> centers_3d;

  for (int i = 0; i < pattern_size.height; i++)
    for (int j = 0; j < pattern_size.width; j++)
      centers_3d.push_back({j * center_distance, i * center_distance, 0});

  return centers_3d;
}

void load(
  const std::string & input_folder, const std::string & config_path, cv::Size & img_size,
  std::vector<std::vector<cv::Point3f>> & obj_points,
  std::vector<std::vector<cv::Point2f>> & img_points, bool preview)
{
  // 读取yaml参数
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto center_distance_mm = yaml["center_distance_mm"].as<double>();
  cv::Size pattern_size(pattern_cols, pattern_rows);

  for (int i = 1; true; i++) {
    // 读取图片
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto img = cv::imread(img_path);
    if (img.empty()) break;

    // 设置图片尺寸
    img_size = img.size();

    // 识别标定板（棋盘格）
    std::vector<cv::Point2f> corners_2d;
    auto success = cv::findChessboardCorners(
      img, pattern_size, corners_2d,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    // findChessboardCorners只有整像素精度，标定前必须精化
    if (success) {
      cv::Mat gray;
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
      cv::cornerSubPix(
        gray, corners_2d, {11, 11}, {-1, -1},
        {cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 1e-3});
    }

    // 显示识别结果
    if (preview) {
      auto drawing = img.clone();
      cv::drawChessboardCorners(drawing, pattern_size, corners_2d, success);
      cv::resize(drawing, drawing, {}, 0.5, 0.5);  // 缩小图片尺寸便于显示完全
      cv::imshow("Press any to continue", drawing);
      cv::waitKey(0);
    }

    // 占比过小会导致内参欠约束（重投影误差仍很小但焦距发散）
    if (success) {
      cv::Rect bbox = cv::boundingRect(corners_2d);
      fmt::print(
        "[success] {}  标定板占画面宽 {:.1f}% 高 {:.1f}%\n", img_path,
        100.0 * bbox.width / img.cols, 100.0 * bbox.height / img.rows);
    } else {
      fmt::print("[failure] {}  未识别到{}x{}棋盘格内角点\n", img_path, pattern_size.width,
        pattern_size.height);
      continue;
    }

    // 记录所需的数据
    img_points.emplace_back(corners_2d);
    obj_points.emplace_back(centers_3d(pattern_size, center_distance_mm));
  }
}

// 视角受限时焦距可大幅偏移而重投影误差几乎不变，故改查有物理预期的光心与长宽比
void check_plausibility(const cv::Mat & camera_matrix, const cv::Size & img_size, size_t n_imgs)
{
  auto fx = camera_matrix.at<double>(0, 0);
  auto fy = camera_matrix.at<double>(1, 1);
  auto cx = camera_matrix.at<double>(0, 2);
  auto cy = camera_matrix.at<double>(1, 2);

  auto cx_off = 100.0 * std::abs(cx - img_size.width / 2.0) / img_size.width;
  auto cy_off = 100.0 * std::abs(cy - img_size.height / 2.0) / img_size.height;
  auto aspect = fy / fx;

  fmt::print("\n---- 标定结果自检 ----\n");
  fmt::print("有效图片   {}\n", n_imgs);
  fmt::print("fx fy      {:.1f} {:.1f}\n", fx, fy);
  fmt::print("光心 cx cy {:.1f} {:.1f}   (画面中心 {} {})\n", cx, cy, img_size.width / 2,
    img_size.height / 2);
  fmt::print("光心偏离   横向{:.1f}% 纵向{:.1f}%{}\n", cx_off, cy_off,
    (cx_off > 10 || cy_off > 10) ? "   <== 异常，正常应在10%以内" : "");
  fmt::print("fy/fx      {:.4f}{}\n", aspect,
    (std::abs(aspect - 1.0) > 0.02) ? "   <== 异常，方形像素应接近1.00" : "");

  if (cx_off > 10 || cy_off > 10 || std::abs(aspect - 1.0) > 0.02)
    fmt::print(
      "\n结果不可信。请让标定板占满画面（至少50%宽），"
      "并大幅改变距离和倾角后重新采集。\n");
  else
    fmt::print("自检通过\n");
}

void print_yaml(const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs, double error)
{
  YAML::Emitter result;
  std::vector<double> camera_matrix_data(
    camera_matrix.begin<double>(), camera_matrix.end<double>());
  std::vector<double> distort_coeffs_data(
    distort_coeffs.begin<double>(), distort_coeffs.end<double>());

  result << YAML::BeginMap;
  result << YAML::Comment(fmt::format("重投影误差: {:.4f}px", error));
  result << YAML::Key << "camera_matrix";
  result << YAML::Value << YAML::Flow << camera_matrix_data;
  result << YAML::Key << "distort_coeffs";
  result << YAML::Value << YAML::Flow << distort_coeffs_data;
  result << YAML::Newline;
  result << YAML::EndMap;

  fmt::print("\n{}\n", result.c_str());
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_folder = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto preview = !cli.has("no-preview");

  // 从输入文件夹中加载标定所需的数据
  cv::Size img_size;
  std::vector<std::vector<cv::Point3f>> obj_points;
  std::vector<std::vector<cv::Point2f>> img_points;
  load(input_folder, config_path, img_size, obj_points, img_points, preview);

  if (obj_points.size() < 10) {
    fmt::print("\n仅{}张图可用，样本过少，标定结果不可信\n", obj_points.size());
    if (obj_points.empty()) return 1;
  }

  // 相机标定
  cv::Mat camera_matrix, distort_coeffs;
  std::vector<cv::Mat> rvecs, tvecs;
  auto criteria = cv::TermCriteria(
    cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
    DBL_EPSILON);  // 默认迭代次数(30)有时会导致结果发散，故设为100
  cv::calibrateCamera(
    obj_points, img_points, img_size, camera_matrix, distort_coeffs, rvecs, tvecs, cv::CALIB_FIX_K3,
    criteria);  // 由于视场角较小，不需要考虑k3

  // 重投影误差
  double error_sum = 0;
  size_t total_points = 0;
  for (size_t i = 0; i < obj_points.size(); i++) {
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      obj_points[i], rvecs[i], tvecs[i], camera_matrix, distort_coeffs, reprojected_points);

    total_points += reprojected_points.size();
    for (size_t j = 0; j < reprojected_points.size(); j++)
      error_sum += cv::norm(img_points[i][j] - reprojected_points[j]);
  }
  auto error = error_sum / total_points;

  check_plausibility(camera_matrix, img_size, obj_points.size());

  // 输出yaml
  print_yaml(camera_matrix, distort_coeffs, error);
}
