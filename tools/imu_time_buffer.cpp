#include "tools/imu_time_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace tools::time_sync
{
namespace
{
bool valid_quaternion(const Eigen::Quaterniond & q)
{
  return q.coeffs().allFinite() && std::abs(q.squaredNorm() - 1.0) <= 0.1;
}
bool valid_gap(double max_gap_ms) { return std::isfinite(max_gap_ms) && max_gap_ms > 0; }
ImuQuery failure(ImuQueryStatus status)
{
  ImuQuery result;
  result.status = status;
  return result;
}

// Both callers establish ordered, valid data first. Online queries therefore
// only do a binary search while holding the buffer mutex.
template <typename Samples>
ImuQuery query_ordered(const Samples & samples, TimePoint time, double max_gap_ms)
{
  if (!valid_gap(max_gap_ms)) return failure(ImuQueryStatus::invalid_sample);
  if (samples.empty()) return failure(ImuQueryStatus::empty);
  if (time < samples.front().timestamp) return failure(ImuQueryStatus::before_buffer);
  if (time > samples.back().timestamp) {
    auto result = failure(ImuQueryStatus::missing_right);
    result.left_time = samples.back().timestamp;
    return result;
  }
  const auto right = std::lower_bound(
    samples.begin(), samples.end(), time,
    [](const TimedImu & sample, TimePoint query_time) { return sample.timestamp < query_time; });
  ImuQuery result;
  result.right_time = right->timestamp;
  if (right->timestamp == time) {
    result.left_time = right->timestamp;
    result.q = right->q.normalized();
    result.status = ImuQueryStatus::ok;
    return result;
  }
  const auto left = std::prev(right);
  result.left_time = left->timestamp;
  // Convert absolute counts before subtraction to avoid signed duration overflow
  // even for adversarial offline timestamps near clock limits.
  const long double width = static_cast<long double>(right->timestamp.time_since_epoch().count()) -
                            static_cast<long double>(left->timestamp.time_since_epoch().count());
  const long double position = static_cast<long double>(time.time_since_epoch().count()) -
                               static_cast<long double>(left->timestamp.time_since_epoch().count());
  const long double gap_ms = width * Clock::period::num * 1000.0L / Clock::period::den;
  result.ratio = static_cast<double>(position / width);
  if (gap_ms > max_gap_ms) {
    result.status = ImuQueryStatus::gap_too_large;
    return result;
  }
  result.q = left->q.normalized().slerp(result.ratio, right->q.normalized()).normalized();
  result.status = ImuQueryStatus::ok;
  return result;
}
}  // namespace

namespace
{
bool valid_samples(const std::vector<TimedImu> & samples)
{
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (!valid_quaternion(samples[i].q) || (i && samples[i].timestamp <= samples[i - 1].timestamp))
      return false;
  }
  return true;
}
}  // namespace

ImuQuery query_samples(const std::vector<TimedImu> & samples, TimePoint time, double max_gap_ms)
{
  if (!valid_samples(samples)) return failure(ImuQueryStatus::invalid_sample);
  return query_ordered(samples, time, max_gap_ms);
}

ImuSampleIndex::ImuSampleIndex(std::vector<TimedImu> samples) : samples_(std::move(samples))
{
  valid_ = valid_samples(samples_);
}

ImuQuery ImuSampleIndex::query(TimePoint time, double max_gap_ms) const
{
  if (!valid_) return failure(ImuQueryStatus::invalid_sample);
  return query_ordered(samples_, time, max_gap_ms);
}

ImuTimeBuffer::ImuTimeBuffer(std::size_t capacity) : capacity_(capacity)
{
  if (!capacity) throw std::invalid_argument("IMU buffer capacity must be positive");
}

bool ImuTimeBuffer::push(const TimedImu & sample)
{
  if (!valid_quaternion(sample.q)) return false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!samples_.empty() && sample.timestamp <= samples_.back().timestamp) return false;
    samples_.push_back({sample.timestamp, sample.q.normalized()});
    if (samples_.size() > capacity_) samples_.pop_front();
  }
  changed_.notify_all();
  return true;
}

ImuQuery ImuTimeBuffer::query(TimePoint time, double max_gap_ms) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return query_ordered(samples_, time, max_gap_ms);
}

ImuQuery ImuTimeBuffer::wait_query(TimePoint time, double max_gap_ms, double max_wait_ms) const
{
  if (
    !valid_gap(max_gap_ms) || !std::isfinite(max_wait_ms) || max_wait_ms < 0 || max_wait_ms > 1000)
    return failure(ImuQueryStatus::invalid_sample);
  const auto start = Clock::now();
  const auto deadline = start + std::chrono::duration_cast<Clock::duration>(
                                  std::chrono::duration<double, std::milli>(max_wait_ms));
  std::unique_lock<std::mutex> lock(mutex_);
  auto result = query_ordered(samples_, time, max_gap_ms);
  while (max_wait_ms > 0 && (result.status == ImuQueryStatus::empty ||
                             result.status == ImuQueryStatus::missing_right)) {
    if (changed_.wait_until(lock, deadline) == std::cv_status::timeout) {
      result = query_ordered(samples_, time, max_gap_ms);
      break;
    }
    result = query_ordered(samples_, time, max_gap_ms);
  }
  result.wait_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
  return result;
}

std::vector<TimedImu> ImuTimeBuffer::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {samples_.begin(), samples_.end()};
}

void ImuTimeBuffer::clear()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
  }
  changed_.notify_all();
}

const char * status_name(ImuQueryStatus status)
{
  switch (status) {
    case ImuQueryStatus::ok:
      return "ok";
    case ImuQueryStatus::empty:
      return "empty";
    case ImuQueryStatus::before_buffer:
      return "before_buffer";
    case ImuQueryStatus::missing_right:
      return "missing_right";
    case ImuQueryStatus::gap_too_large:
      return "gap_too_large";
    case ImuQueryStatus::invalid_sample:
      return "invalid_sample";
  }
  return "invalid_sample";
}
}  // namespace tools::time_sync
