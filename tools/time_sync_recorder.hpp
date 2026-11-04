#ifndef TOOLS_TIME_SYNC_RECORDER_HPP
#define TOOLS_TIME_SYNC_RECORDER_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <opencv2/core.hpp>
#include <optional>
#include <string>

namespace tools::time_sync
{
// Validates the complete spatial snapshot before either sensor is opened.
void validate_calibration_geometry(const YAML::Node & calibration);

struct RecorderOptions
{
  std::filesystem::path output;
  YAML::Node calibration;
  std::chrono::steady_clock::time_point t0;
  std::string camera_name = "hikrobot";
  std::string timestamp_semantics = "host_receive_v1";
  std::string source_head = "unknown";
  double image_save_fps = 30;
  double exposure_us = 3000;
  double duration_s = 30;
  std::size_t image_capacity = 32, imu_capacity = 4096, event_capacity = 1024;
};

struct ImageMetadata
{
  std::optional<std::uint64_t> camera_frame_id, device_timestamp_ticks;
  std::optional<double> exposure_us;
};

struct RecorderStats
{
  std::uint64_t images_received = 0, images_accepted = 0, images_written = 0;
  std::uint64_t images_dropped = 0, images_failed = 0, images_skipped = 0,
                camera_source_dropped = 0;
  std::uint64_t imu_received = 0, imu_accepted = 0, imu_written = 0;
  std::uint64_t imu_dropped = 0, imu_failed = 0, imu_invalid = 0;
  std::uint64_t imu_gap_count = 0;
  double max_imu_receive_gap_ms = 0;
  std::uint64_t events_written = 0, events_dropped = 0;
  bool complete = false;
};

class TimeSyncRecorder
{
public:
  explicit TimeSyncRecorder(const RecorderOptions & options);
  ~TimeSyncRecorder();
  TimeSyncRecorder(const TimeSyncRecorder &) = delete;
  TimeSyncRecorder & operator=(const TimeSyncRecorder &) = delete;
  // Producers never wait for disk or queue space. Images are cloned before return.
  bool record_image(
    const cv::Mat &, std::chrono::steady_clock::time_point, const ImageMetadata & metadata = {});
  bool record_imu(const Eigen::Quaterniond &, std::chrono::steady_clock::time_point);
  void record_event(
    std::chrono::steady_clock::time_point, const std::string & type, std::uint64_t local_id,
    const std::string & reason);
  void mark_incomplete(const std::string & reason);
  void record_source_drops(std::uint64_t count);
  // Stop producers before finish. Idempotent; drains all accepted records.
  RecorderStats finish(bool clean_shutdown = true);
  RecorderStats stats() const;
  bool failed() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace tools::time_sync
#endif
