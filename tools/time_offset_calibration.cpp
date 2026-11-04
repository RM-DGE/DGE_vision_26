#include "tools/time_offset_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
namespace tools::time_sync
{
namespace
{
bool shifted(TimePoint time, double delay, TimePoint & result)
{
  if (!std::isfinite(delay) || std::abs(delay) > 1000) return false;
  const auto delta =
    std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double, std::milli>(delay))
      .count();
  const auto count = time.time_since_epoch().count();
  using Rep = Clock::duration::rep;
  if (
    (delta > 0 && count < std::numeric_limits<Rep>::lowest() + delta) ||
    (delta < 0 && count > std::numeric_limits<Rep>::max() + delta))
    return false;
  result = TimePoint(Clock::duration(count - delta));
  return true;
}
bool rotation(const Eigen::Matrix3d & R)
{
  return R.allFinite() && (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-5 &&
         std::abs(R.determinant() - 1) < 1e-5;
}
bool geometry_valid(const CalibrationGeometry & g)
{
  return rotation(g.R_camera2gimbal) && rotation(g.R_gimbal2imubody) &&
         g.t_camera2gimbal.allFinite();
}
bool options_valid(const OffsetScanOptions & o)
{
  const double values[] = {o.min_delay_ms,         o.max_delay_ms,      o.step_ms,
                           o.baseline_delay_ms,    o.max_gap_ms,        o.min_duration_s,
                           o.max_reprojection_px,  o.flat_absolute_m,   o.flat_relative,
                           o.segment_tolerance_ms, o.ambiguity_relative};
  for (double v : values)
    if (!std::isfinite(v)) return false;
  const double steps = (o.max_delay_ms - o.min_delay_ms) / o.step_ms;
  return o.min_delay_ms < o.max_delay_ms && o.step_ms > 0 && steps >= 2 && steps <= 100000 &&
         std::abs(o.min_delay_ms) <= 1000 && std::abs(o.max_delay_ms) <= 1000 &&
         std::abs(o.baseline_delay_ms) <= 1000 && o.max_gap_ms > 0 && o.min_duration_s >= 0 &&
         o.max_reprojection_px > 0 && o.min_frames >= 3 && o.segment_count >= 2 &&
         o.segment_count <= 100 && o.flat_absolute_m >= 0 && o.flat_relative >= 0 &&
         o.segment_tolerance_ms >= 0 && o.ambiguity_relative >= 0;
}
}  // namespace
OffsetScore score_offset(
  const std::vector<BoardObservation> & frames, const std::vector<TimedImu> & imu,
  const CalibrationGeometry & g, double delay, double max_gap,
  std::vector<OffsetResidual> * residuals)
{
  OffsetScore score;
  score.delay_ms = delay;
  if (residuals) residuals->clear();
  if (
    frames.empty() || !std::isfinite(delay) || std::abs(delay) > 1000 || !std::isfinite(max_gap) ||
    max_gap <= 0 || !geometry_valid(g))
    return score;
  const ImuSampleIndex index(imu);
  if (!index.valid()) return score;
  std::vector<OffsetResidual> points;
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto & frame : frames) {
    TimePoint query_time;
    if (!shifted(frame.timestamp, delay, query_time)) return score;
    const auto query = index.query(query_time, max_gap);
    if (!query.valid() || !frame.point_camera_m.allFinite()) return score;
    const Eigen::Matrix3d R =
      g.R_gimbal2imubody.transpose() * query.q.toRotationMatrix() * g.R_gimbal2imubody;
    const Eigen::Vector3d p = R * (g.R_camera2gimbal * frame.point_camera_m + g.t_camera2gimbal);
    if (!p.allFinite()) return score;
    points.push_back({frame.frame_id, p, 0, frame.reprojection_error_px});
    mean += p;
  }
  mean /= static_cast<double>(points.size());
  double sum = 0;
  std::vector<double> distances;
  for (auto & p : points) {
    p.distance_m = (p.point_world_m - mean).norm();
    sum += p.distance_m * p.distance_m;
    distances.push_back(p.distance_m);
  }
  std::sort(distances.begin(), distances.end());
  score.rms_m = std::sqrt(sum / points.size());
  score.p95_m = distances[static_cast<std::size_t>(std::ceil(.95 * distances.size())) - 1];
  score.frame_count = points.size();
  score.valid = std::isfinite(score.rms_m) && std::isfinite(score.p95_m);
  if (residuals) *residuals = std::move(points);
  return score;
}
OffsetCalibrationResult calibrate_time_offset(
  const std::vector<BoardObservation> & frames, const std::vector<TimedImu> & imu,
  const CalibrationGeometry & g, const OffsetScanOptions & o)
{
  OffsetCalibrationResult result;
  result.input_frames = frames.size();
  const auto invalid = [&](const char * reason) {
    result.status = "invalid_data";
    result.reason = reason;
    return result;
  };
  const auto inconclusive = [&](const char * reason) {
    result.status = "inconclusive";
    result.reason = reason;
    return result;
  };
  if (!options_valid(o)) return invalid("invalid_scan_options");
  if (!geometry_valid(g)) return invalid("invalid_geometry");
  if (imu.empty()) return invalid("insufficient_imu");
  for (std::size_t i = 0; i < imu.size(); ++i) {
    const double norm = imu[i].q.squaredNorm();
    if (
      !imu[i].q.coeffs().allFinite() || !std::isfinite(norm) || std::abs(norm - 1) > .1 ||
      (i && imu[i].timestamp <= imu[i - 1].timestamp))
      return invalid("invalid_imu_stream");
  }
  std::vector<BoardObservation> quality;
  for (std::size_t i = 0; i < frames.size(); ++i) {
    const auto & f = frames[i];
    if (
      (i && (f.timestamp <= frames[i - 1].timestamp || f.frame_id <= frames[i - 1].frame_id)) ||
      !f.point_camera_m.allFinite() || !std::isfinite(f.reprojection_error_px) ||
      f.reprojection_error_px < 0)
      return invalid("invalid_observation_stream");
    TimePoint checked;
    if (
      !shifted(f.timestamp, o.min_delay_ms, checked) ||
      !shifted(f.timestamp, o.max_delay_ms, checked) ||
      !shifted(f.timestamp, o.baseline_delay_ms, checked))
      return invalid("timestamp_overflow");
    if (f.reprojection_error_px <= o.max_reprojection_px) quality.push_back(f);
  }
  result.quality_frames = quality.size();
  const ImuSampleIndex index(imu);
  const auto intervals =
    static_cast<std::size_t>(std::floor((o.max_delay_ms - o.min_delay_ms) / o.step_ms + 1e-9));
  std::vector<double> delays;
  for (std::size_t i = 0; i <= intervals; ++i) delays.push_back(o.min_delay_ms + i * o.step_ms);
  if (delays.back() < o.max_delay_ms - 1e-9) delays.push_back(o.max_delay_ms);
  // Include the exact current value in coverage; comparison never changes the sample set.
  for (const auto & frame : quality) {
    TimePoint time;
    shifted(frame.timestamp, o.baseline_delay_ms, time);
    bool valid = index.query(time, o.max_gap_ms).valid();
    for (double delay : delays) {
      if (!valid) break;
      shifted(frame.timestamp, delay, time);
      valid = index.query(time, o.max_gap_ms).valid();
    }
    if (valid) result.common_frames.push_back(frame);
  }
  result.common_coverage =
    quality.empty() ? 0 : static_cast<double>(result.common_frames.size()) / quality.size();
  if (!result.common_frames.empty())
    result.duration_s =
      std::chrono::duration<double>(
        result.common_frames.back().timestamp - result.common_frames.front().timestamp)
        .count();
  for (double delay : delays)
    result.scan.push_back(score_offset(result.common_frames, imu, g, delay, o.max_gap_ms));
  if (result.common_frames.size() < o.min_frames || result.duration_s < o.min_duration_s)
    return invalid("insufficient_common_frames_or_duration");
  auto best = std::min_element(
    result.scan.begin(), result.scan.end(),
    [](const auto & a, const auto & b) { return a.valid && (!b.valid || a.rms_m < b.rms_m); });
  if (best == result.scan.end() || !best->valid) return invalid("all_candidates_invalid");
  result.best = *best;
  result.baseline = score_offset(result.common_frames, imu, g, o.baseline_delay_ms, o.max_gap_ms);
  score_offset(result.common_frames, imu, g, result.best.delay_ms, o.max_gap_ms, &result.residuals);
  const double max_rms =
    std::max_element(result.scan.begin(), result.scan.end(), [](const auto & a, const auto & b) {
      return a.rms_m < b.rms_m;
    })->rms_m;
  // Use contiguous time blocks rather than randomly separating neighbouring frames.
  for (std::size_t k = 0; k < o.segment_count; ++k) {
    const std::size_t begin = result.common_frames.size() * k / o.segment_count;
    const std::size_t end = result.common_frames.size() * (k + 1) / o.segment_count;
    if (end - begin < 3) return invalid("insufficient_segment_frames");
    std::vector<BoardObservation> segment(
      result.common_frames.begin() + begin, result.common_frames.begin() + end);
    OffsetScore segment_best;
    for (double delay : delays) {
      const auto score = score_offset(segment, imu, g, delay, o.max_gap_ms);
      if (score.valid && (!segment_best.valid || score.rms_m < segment_best.rms_m))
        segment_best = score;
    }
    if (!segment_best.valid) return invalid("invalid_segment");
    result.segment_best_delay_ms.push_back(segment_best.delay_ms);
    result.segment_candidate_rms_m.push_back(
      score_offset(segment, imu, g, result.best.delay_ms, o.max_gap_ms).rms_m);
    result.segment_baseline_rms_m.push_back(
      score_offset(segment, imu, g, o.baseline_delay_ms, o.max_gap_ms).rms_m);
  }
  if (max_rms - result.best.rms_m <= std::max(o.flat_absolute_m, o.flat_relative * max_rms))
    return inconclusive("flat_curve");
  if (best == result.scan.begin() || best + 1 == result.scan.end())
    return inconclusive("boundary_minimum");
  const double ambiguity =
    std::max(o.flat_absolute_m, o.ambiguity_relative * (max_rms - result.best.rms_m));
  for (std::size_t i = 1; i + 1 < result.scan.size(); ++i) {
    const auto & point = result.scan[i];
    if (
      std::abs(point.delay_ms - result.best.delay_ms) >
        std::max(o.segment_tolerance_ms, 2 * o.step_ms) &&
      point.rms_m <= result.scan[i - 1].rms_m && point.rms_m <= result.scan[i + 1].rms_m &&
      point.rms_m <= result.best.rms_m + ambiguity)
      return inconclusive("multimodal_curve");
  }
  for (double delay : result.segment_best_delay_ms)
    if (std::abs(delay - result.best.delay_ms) > std::max(o.segment_tolerance_ms, 2 * o.step_ms))
      return inconclusive("inconsistent_segments");
  result.status = "candidate";
  result.reason = "offline_candidate_online_verification_pending";
  return result;
}
}  // namespace tools::time_sync
