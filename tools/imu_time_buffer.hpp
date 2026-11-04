#ifndef TOOLS__IMU_TIME_BUFFER_HPP
#define TOOLS__IMU_TIME_BUFFER_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <vector>

namespace tools::time_sync
{
using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

struct TimedImu
{
  TimePoint timestamp;
  Eigen::Quaterniond q;
};

enum class ImuQueryStatus
{
  ok,
  empty,
  before_buffer,
  missing_right,
  gap_too_large,
  invalid_sample
};

struct ImuQuery
{
  ImuQueryStatus status = ImuQueryStatus::empty;
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  TimePoint left_time{}, right_time{};
  double ratio = 0, wait_ms = 0;
  bool valid() const { return status == ImuQueryStatus::ok; }
};

// Offline input must be strictly increasing and have finite, approximately unit
// quaternions (squared norm error <= 0.1, matching the CBoard wire validation).
// Exact sample times need no neighbor; other times require two samples.
ImuQuery query_samples(const std::vector<TimedImu> & samples, TimePoint time, double max_gap_ms);
const char * status_name(ImuQueryStatus status);

// Repeated offline scans validate an owned immutable snapshot once instead of
// rescanning all IMU records for every frame/candidate pair.
class ImuSampleIndex
{
public:
  explicit ImuSampleIndex(std::vector<TimedImu> samples);
  bool valid() const { return valid_; }
  ImuQuery query(TimePoint time, double max_gap_ms) const;

private:
  const std::vector<TimedImu> samples_;
  bool valid_ = true;
};

class ImuTimeBuffer
{
public:
  explicit ImuTimeBuffer(std::size_t capacity = 5000);
  bool push(const TimedImu & sample);
  ImuQuery query(TimePoint time, double max_gap_ms) const;
  ImuQuery wait_query(TimePoint time, double max_gap_ms, double max_wait_ms) const;
  std::vector<TimedImu> snapshot() const;
  void clear();

private:
  const std::size_t capacity_;
  std::deque<TimedImu> samples_;
  mutable std::mutex mutex_;
  mutable std::condition_variable changed_;
};
}  // namespace tools::time_sync
#endif
