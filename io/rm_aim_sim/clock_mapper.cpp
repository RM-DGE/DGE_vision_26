#include "clock_mapper.hpp"
#include <cmath>
#include <stdexcept>
namespace io::rm_aim_sim {
MappedTime ClockMapper::map(const SimContext &c,double t,Clock::time_point received) {
  if(!std::isfinite(t)||t<0)throw std::runtime_error("Invalid simulation time");
  bool changed=!context_||!(*context_==c)||t<last_sim_-1e-9;
  if(changed) { context_=c; base_sim_=t; base_local_=received; }
  last_sim_=t;
  return {base_local_+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(t-base_sim_)),changed};
}
}
