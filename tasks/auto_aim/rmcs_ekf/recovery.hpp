#pragma once
#include "rmcs_ekf.hpp"
#include <deque>
#include <optional>
#include <string>

namespace rmcs_ekf {
enum class Health { INIT, HEALTHY, COASTING, DEGRADED, LOST };
struct RecoverySettings {
  double trigger_s=0.04, window_s=0.12, sample_s=0.015, expiry_s=0.15;
  double confirm_s=0.02, coast_s=0.15, lost_s=0.5;
  double max_rms_px=5.0, improvement_ratio=0.7, identity_distance_m=0.8;
  double future_position_variance=0.04, covariance_inflation=2.0;
  double max_fit_ms=20.0;
  int min_frames=3, max_frames=8, max_seeds=4, iterations=8;
};
RecoverySettings load_recovery_settings(const YAML::Node & node);
struct RecoveryStatus {
  Health health=Health::INIT;
  bool reliable=false, candidate_active=false, identity_ambiguous=false;
  std::size_t candidate_id=0, recovery_count=0, candidate_failed_count=0;
  int validation_frames=0;
  double last_detection_age=0, last_accepted_age=0, window_begin=0, window_end=0;
  double prior_time=0, candidate_time=0, fit_cost=0, validation_rms=0;
  std::string event;
};

// Owns image-time lifecycle. Every stored prior precedes that frame's pixel update.
class RecoveryManager {
public:
  explicit RecoveryManager(RecoverySettings settings={}) : settings_(settings) {}
  void reset(double time);
  void interrupt();
  void process(RobotModel & model, const RobotModel & prediction,
    const std::vector<Armor2d> & observations, const CameraFeature & camera,
    double time, bool identity_consistent=true);
  const RecoveryStatus & status() const { return status_; }
private:
  struct Frame { double time; RobotModel prior; CameraFeature camera; std::vector<Armor2d> observations; };
  struct Candidate { RobotModel model; double time=0, created=0, first_good=0; int good=0; };
  bool fit(const RobotModel & model, double time);
  RecoverySettings settings_;
  RecoveryStatus status_;
  std::deque<Frame> window_;
  std::optional<Candidate> candidate_;
  double initialized_=0, last_detection_=0, last_accepted_=0, last_time_=0;
  int rejected_frames_=0, accepted_frames_=0;
};
}
