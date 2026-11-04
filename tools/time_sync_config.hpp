#ifndef TOOLS__TIME_SYNC_CONFIG_HPP
#define TOOLS__TIME_SYNC_CONFIG_HPP
#include <yaml-cpp/yaml.h>

#include "tools/imu_time_buffer.hpp"

namespace tools::time_sync
{
struct TimeSyncConfig
{
  double imu_delay_ms = 1.0, max_gap_ms = 20.0, wait_ms = 12.0;
};
// Delay [-1000,1000] ms, gap (0,1000] ms, wait [0,1000] ms.
// Legacy delay is accepted only when no conflicting imu_delay_ms is present.
TimeSyncConfig load_config(const YAML::Node & yaml);
TimePoint query_time(TimePoint image_time, double delay_ms);
}  // namespace tools::time_sync
#endif
