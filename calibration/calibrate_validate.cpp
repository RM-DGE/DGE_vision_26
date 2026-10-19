/**
 * @file calibrate_validate.cpp
 * @brief 标定外参验收：固定标定板 + 转动云台，检查板在世界系中的重复性。
 *
 * 原理：
 *   solvePnP 得到 board->camera；
 *   用 R_camera2gimbal / t_camera2gimbal 转到 gimbal；
 *   再用 IMU 四元数和 R_gimbal2imubody 转到 world。
 *   如果外参正确且标定板固定不动，那么每张图的板原点/姿态在世界系中应几乎不变。
 *
 * 用法：
 *   ./build/calibrate_validate assets/img_with_q -c configs/calibration.yaml
 *
 * 输出：
 *   每帧板原点、yaw/pitch/roll、重投影误差；
 *   汇总 mean/std/max，并按阈值给出 PASS/FAIL。
 */

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "chessboard.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml 配置文件路径 }"
  "{@input-folder  | assets/img_with_q        | 图片 + 四元数文件夹 }"
  "{max-pos-std    | 0.010                    | 位置标准差阈值 (m) }"
  "{max-angle-std  | 0.5                      | 姿态标准差阈值 (deg) }";

Eigen::Quaterniond read_q(const std::string & q_path)
{
  std::ifstream q_file(q_path);
  double w, x, y, z;
  q_file >> w >> x >> y >> z;
  return {w, x, y, z};
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  const auto input_folder = cli.get<std::string>(0);
  const auto config_path = cli.get<std::string>("config-path");
  const auto max_pos_std = cli.get<double>("max-pos-std");
  const auto max_angle_std = cli.get<double>("max-angle-std");

  auto yaml = YAML::LoadFile(config_path);
  const auto pattern_cols = yaml["pattern_cols"].as<int>();
  const auto pattern_rows = yaml["pattern_rows"].as<int>();
  const auto square_size_mm = yaml["square_size_mm"]
                                ? yaml["square_size_mm"].as<double>()
                                : yaml["center_distance_mm"].as<double>();
  const cv::Size pattern_size(pattern_cols, pattern_rows);

  const auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  const auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  const auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  const auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  const auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();

  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_gimbal2imubody(R_gimbal2imubody_data.data());
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_camera2gimbal(R_camera2gimbal_data.data());
  Eigen::Vector3d t_camera2gimbal(t_camera2gimbal_data.data());
  cv::Matx33d camera_matrix(camera_matrix_data.data());
  cv::Mat distort_coeffs(distort_coeffs_data);

  const auto object_points = calibration::object_points(pattern_size, square_size_mm);

  struct Sample
  {
    Eigen::Vector3d center;
    Eigen::Vector3d ypr_deg;
    double reprojection_error;
  };
  std::vector<Sample> samples;

  for (int index = 1;; ++index) {
    const auto img_path = fmt::format("{}/{}.jpg", input_folder, index);
    const auto q_path = fmt::format("{}/{}.txt", input_folder, index);
    auto img = cv::imread(img_path);
    if (img.empty()) break;

    std::vector<cv::Point2f> corners;
    const auto found = calibration::find_chessboard(img, pattern_size, corners);
    if (!found) {
      fmt::print("[skip] {}: findChessboardCorners failed\n", img_path);
      continue;
    }

    auto q = read_q(q_path).normalized();
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    Eigen::Matrix3d R_gimbal2world =
      R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody;

    cv::Vec3d rvec, tvec;
    const auto ok = cv::solvePnP(
      object_points, corners, camera_matrix, distort_coeffs, rvec, tvec, false,
      cv::SOLVEPNP_IPPE);
    if (!ok) {
      fmt::print("[skip] {}: solvePnP failed\n", img_path);
      continue;
    }

    cv::Mat R_board2camera_cv;
    cv::Rodrigues(rvec, R_board2camera_cv);
    Eigen::Matrix3d R_board2camera;
    cv::cv2eigen(R_board2camera_cv, R_board2camera);
    Eigen::Vector3d t_board2camera(tvec[0] / 1000.0, tvec[1] / 1000.0, tvec[2] / 1000.0);

    const Eigen::Matrix3d R_camera2world = R_gimbal2world * R_camera2gimbal;
    const Eigen::Vector3d t_camera2world = R_gimbal2world * t_camera2gimbal;
    const Eigen::Vector3d center = R_camera2world * t_board2camera + t_camera2world;
    const Eigen::Matrix3d R_board2world = R_camera2world * R_board2camera;
    const Eigen::Vector3d ypr_deg = tools::eulers(R_board2world, 2, 1, 0) * 57.3;

    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, distort_coeffs, reprojected);
    double error_sum = 0.0;
    for (std::size_t i = 0; i < reprojected.size(); ++i)
      error_sum += cv::norm(corners[i] - reprojected[i]);
    const double reprojection_error = error_sum / std::max<std::size_t>(1, reprojected.size());

    samples.push_back({center, ypr_deg, reprojection_error});
    fmt::print(
      "[{:03d}] center=({:+.4f}, {:+.4f}, {:+.4f}) m  ypr=({:+.2f}, {:+.2f}, {:+.2f}) deg  "
      "reproj={:.3f} px\n",
      index, center.x(), center.y(), center.z(), ypr_deg[0], ypr_deg[1], ypr_deg[2],
      reprojection_error);
  }

  if (samples.size() < 5) {
    fmt::print("\n[FAIL] 有效样本不足 ({})，至少需要 5 张\n", samples.size());
    return 1;
  }

  // yaw 在 ±180° 附近会跳变，先以第一帧为参考做 unwrap，再算 mean/std。
  const auto wrap_deg = [](double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
  };
  const auto reference_ypr = samples.front().ypr_deg;
  const auto unwrap_deg = [&](double angle, double reference) {
    return reference + wrap_deg(angle - reference);
  };

  std::vector<Eigen::Vector3d> unwrapped_ypr;
  unwrapped_ypr.reserve(samples.size());
  for (const auto & s : samples) {
    unwrapped_ypr.emplace_back(unwrap_deg(s.ypr_deg.x(), reference_ypr.x()),
      unwrap_deg(s.ypr_deg.y(), reference_ypr.y()), unwrap_deg(s.ypr_deg.z(), reference_ypr.z()));
  }

  Eigen::Vector3d mean_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_ypr = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < samples.size(); ++i) {
    mean_center += samples[i].center;
    mean_ypr += unwrapped_ypr[i];
  }
  mean_center /= static_cast<double>(samples.size());
  mean_ypr /= static_cast<double>(samples.size());

  Eigen::Vector3d var_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d var_ypr = Eigen::Vector3d::Zero();
  double max_center_dev = 0.0;
  double max_angle_dev = 0.0;
  double mean_reproj = 0.0;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const auto dc = samples[i].center - mean_center;
    const auto da = unwrapped_ypr[i] - mean_ypr;
    var_center += dc.cwiseProduct(dc);
    var_ypr += da.cwiseProduct(da);
    max_center_dev = std::max(max_center_dev, dc.norm());
    max_angle_dev = std::max(max_angle_dev, da.norm());
    mean_reproj += samples[i].reprojection_error;
  }
  var_center /= static_cast<double>(samples.size());
  var_ypr /= static_cast<double>(samples.size());
  mean_reproj /= static_cast<double>(samples.size());

  const Eigen::Vector3d std_center = var_center.cwiseSqrt();
  const Eigen::Vector3d std_ypr = var_ypr.cwiseSqrt();
  mean_ypr.x() = wrap_deg(mean_ypr.x());

  fmt::print("\n===== summary ({} samples) =====\n", samples.size());
  fmt::print("center mean = ({:+.4f}, {:+.4f}, {:+.4f}) m\n", mean_center.x(), mean_center.y(),
    mean_center.z());
  fmt::print("center std  = ({:.4f}, {:.4f}, {:.4f}) m   max dev={:.4f} m\n", std_center.x(),
    std_center.y(), std_center.z(), max_center_dev);
  fmt::print("ypr mean    = ({:+.2f}, {:+.2f}, {:+.2f}) deg\n", mean_ypr.x(), mean_ypr.y(),
    mean_ypr.z());
  fmt::print("ypr std     = ({:.3f}, {:.3f}, {:.3f}) deg max dev={:.3f} deg\n", std_ypr.x(),
    std_ypr.y(), std_ypr.z(), max_angle_dev);
  fmt::print("reproj mean = {:.3f} px\n", mean_reproj);

  const auto pos_ok = std_center.norm() <= max_pos_std;
  const auto angle_ok = std_ypr.norm() <= max_angle_std;
  fmt::print("\n[{}] position std {:.4f} m (limit {:.4f})\n", pos_ok ? "PASS" : "FAIL",
    std_center.norm(), max_pos_std);
  fmt::print("[{}] angle std    {:.3f} deg (limit {:.3f})\n", angle_ok ? "PASS" : "FAIL",
    std_ypr.norm(), max_angle_std);

  if (!pos_ok || !angle_ok) {
    fmt::print(
      "\n提示：转动云台时世界系板位置漂移，优先检查：\n"
      "  1) R_gimbal2imubody 是否按机械安装填写；\n"
      "  2) t_camera2gimbal 机械值是否正确；\n"
      "  3) 标定板在采集期间是否真的固定不动；\n"
      "  4) 相机内参/畸变是否与当前分辨率、ROI、翻转一致。\n");
    return 1;
  }
  return 0;
}
