#pragma once
#include "protocol.hpp"
namespace io::rm_aim_sim {
struct MappedTime { Clock::time_point timestamp; bool epoch_changed; };
class ClockMapper {
 public:
  MappedTime map(const SimContext &,double,Clock::time_point);
  void reset() { context_.reset(); }
 private:
  std::optional<SimContext> context_; double base_sim_=0,last_sim_=0; Clock::time_point base_local_{};
};
}
