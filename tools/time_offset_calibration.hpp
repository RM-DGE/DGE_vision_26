#pragma once

#include <Eigen/Geometry>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "tools/imu_time_buffer.hpp"

namespace tools::time_sync
{
struct BoardObservation
{
  std::uint64_t frame_id = 0;
  TimePoint timestamp{};
  Eigen::Vector3d point_camera_m = Eigen::Vector3d::Zero();
  double reprojection_error_px = 0;
};
struct CalibrationGeometry
{
  Eigen::Matrix3d R_camera2gimbal = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_camera2gimbal = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_gimbal2imubody = Eigen::Matrix3d::Identity();
};
struct OffsetScanOptions
{
  double min_delay_ms = -20, max_delay_ms = 20, step_ms = 0.1;
  double baseline_delay_ms = 1, max_gap_ms = 20;
  std::size_t min_frames = 300;
  double min_duration_s = 20, max_reprojection_px = 1.5;
  double flat_absolute_m = 1e-6, flat_relative = 0.01;
  double segment_tolerance_ms = 1, ambiguity_relative = 0.01;
  std::size_t segment_count = 3;
};
struct OffsetScore
{
  double delay_ms = 0;
  double rms_m = std::numeric_limits<double>::quiet_NaN();
  double p95_m = std::numeric_limits<double>::quiet_NaN();
  std::size_t frame_count = 0;
  bool valid = false;
};
struct OffsetResidual
{
  std::uint64_t frame_id = 0;
  Eigen::Vector3d point_world_m = Eigen::Vector3d::Zero();
  double distance_m = 0, reprojection_error_px = 0;
};
struct OffsetCalibrationResult
{
  std::string status = "invalid_data", reason = "not_evaluated";
  std::vector<OffsetScore> scan;
  std::vector<BoardObservation> common_frames;
  std::vector<OffsetResidual> residuals;
  std::vector<double> segment_best_delay_ms;
  std::vector<double> segment_baseline_rms_m, segment_candidate_rms_m;
  OffsetScore best, baseline;
  std::size_t input_frames = 0, quality_frames = 0;
  double common_coverage = 0, duration_s = 0;
};
OffsetScore score_offset(
  const std::vector<BoardObservation> & frames, const std::vector<TimedImu> & imu,
  const CalibrationGeometry & geometry, double delay_ms, double max_gap_ms,
  std::vector<OffsetResidual> * residuals = nullptr);
OffsetCalibrationResult calibrate_time_offset(
  const std::vector<BoardObservation> & frames, const std::vector<TimedImu> & imu,
  const CalibrationGeometry & geometry, const OffsetScanOptions & options = {});
}  // namespace tools::time_sync
