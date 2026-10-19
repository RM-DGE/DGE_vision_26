#ifndef CALIBRATION__CHESSBOARD_HPP
#define CALIBRATION__CHESSBOARD_HPP

#include <opencv2/opencv.hpp>

#include <vector>

namespace calibration
{

// 棋盘格内角点对应的 3D 坐标，单位 mm。
// pattern_size 是“内角点”数量，不是方格数量：
//   10x7 个方格 -> 9x6 个内角点。
inline auto object_points(const cv::Size & pattern_size, double square_size_mm)
  -> std::vector<cv::Point3f>
{
  std::vector<cv::Point3f> points;
  points.reserve(static_cast<std::size_t>(pattern_size.width * pattern_size.height));
  for (int row = 0; row < pattern_size.height; ++row) {
    for (int col = 0; col < pattern_size.width; ++col) {
      points.emplace_back(static_cast<float>(col * square_size_mm),
        static_cast<float>(row * square_size_mm), 0.0F);
    }
  }
  return points;
}

// 棋盘格角点检测：优先用 SB 版本，再做亚像素 refine。
// 标定图片可以慢一点，所以打开 EXHAUSTIVE + ACCURACY。
inline auto find_chessboard(
  const cv::Mat & image, const cv::Size & pattern_size, std::vector<cv::Point2f> & corners) -> bool
{
  if (image.empty()) return false;

  const int flags =
    cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY;
  if (!cv::findChessboardCornersSB(image, pattern_size, corners, flags)) return false;

  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }
  cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
  return true;
}

}  // namespace calibration

#endif  // CALIBRATION__CHESSBOARD_HPP
