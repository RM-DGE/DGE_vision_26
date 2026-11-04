#include "tools/time_sync_config.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace tools::time_sync
{
namespace
{
void validate(double value, double lower, double upper, bool inclusive_lower, const char * name)
{
  if (!std::isfinite(value) || value > upper || (inclusive_lower ? value < lower : value <= lower))
    throw std::invalid_argument(
      std::string(name) + " is nonfinite or outside its allowed millisecond range");
}
}  // namespace
TimeSyncConfig load_config(const YAML::Node & yaml)
{
  TimeSyncConfig config;
  if (!yaml || yaml.IsNull()) return config;
  if (!yaml.IsMap()) throw std::invalid_argument("time sync configuration must be a YAML mapping");
  if (yaml["imu_delay_ms"]) config.imu_delay_ms = yaml["imu_delay_ms"].as<double>();
  if (yaml["delay"]) {
    const double legacy = yaml["delay"].as<double>();
    validate(legacy, -1000, 1000, true, "delay");
    if (yaml["imu_delay_ms"] && legacy != config.imu_delay_ms)
      throw std::invalid_argument("conflicting delay and imu_delay_ms values");
    if (!yaml["imu_delay_ms"]) {
      config.imu_delay_ms = legacy;
      std::clog << "[time_sync] deprecated config key delay; use imu_delay_ms (milliseconds)\n";
    }
  }
  if (yaml["imu_max_gap_ms"]) config.max_gap_ms = yaml["imu_max_gap_ms"].as<double>();
  if (yaml["imu_wait_ms"]) config.wait_ms = yaml["imu_wait_ms"].as<double>();
  validate(config.imu_delay_ms, -1000, 1000, true, "imu_delay_ms");
  validate(config.max_gap_ms, 0, 1000, false, "imu_max_gap_ms");
  validate(config.wait_ms, 0, 1000, true, "imu_wait_ms");
  return config;
}

TimePoint query_time(TimePoint image_time, double delay_ms)
{
  validate(delay_ms, -1000, 1000, true, "imu_delay_ms");
  using Rep = Clock::duration::rep;
  const long double delay_ticks =
    static_cast<long double>(delay_ms) * Clock::period::den / (Clock::period::num * 1000.0L);
  const auto ticks = static_cast<Rep>(delay_ticks);
  const auto image_ticks = image_time.time_since_epoch().count();
  if (
    (ticks > 0 && image_ticks < std::numeric_limits<Rep>::lowest() + ticks) ||
    (ticks < 0 && image_ticks > std::numeric_limits<Rep>::max() + ticks))
    throw std::out_of_range("image time minus IMU delay exceeds steady_clock range");
  return TimePoint{Clock::duration{image_ticks - ticks}};
}
}  // namespace tools::time_sync
