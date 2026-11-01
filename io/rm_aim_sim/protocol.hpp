#pragma once
#include <array>
#include <chrono>
#include <optional>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include "io/command.hpp"
namespace io::rm_aim_sim {
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
struct SimContext {
  std::string publisher_id, session_id;
  bool operator==(const SimContext &o) const { return publisher_id==o.publisher_id && session_id==o.session_id; }
};
struct CameraModel {
  int width=0, height=0;
  std::array<double,9> K{}, R_camera2gimbal{};
  std::array<double,5> distortion{};
  std::array<double,3> t_camera2gimbal{};
  bool operator==(const CameraModel &o) const { return width==o.width && height==o.height && K==o.K && distortion==o.distortion && R_camera2gimbal==o.R_camera2gimbal && t_camera2gimbal==o.t_camera2gimbal; }
};
// Diagnostic only. Upstream v2 has no angular velocity; never invent a zero measurement.
struct SimTruthTarget { std::array<double,3> position{}; double yaw=0; std::optional<double> vyaw; bool is_outpost=false; };
struct SimState {
  SimContext context; uint64_t publication=0; double wall_unix_sec=0, sim_time=0, bullet_speed=0;
  bool paused=false, ipc_enabled=false; int64_t last_seq=-1;
  Eigen::Quaterniond gimbal_to_world=Eigen::Quaterniond::Identity();
  CameraModel camera; Json appearance, stats; std::vector<SimTruthTarget> truth;
};
struct FrameMetadata {
  SimContext context; std::string request_id, frame_file, encoding; size_t image_bytes=0;
  double sim_time=0; Eigen::Quaterniond gimbal_to_world=Eigen::Quaterniond::Identity(); CameraModel camera;
};
SimState parse_state(const Json &);
FrameMetadata parse_frame(const Json &);
Json make_command_json(const SimState &, int64_t, const io::Command &);
}
