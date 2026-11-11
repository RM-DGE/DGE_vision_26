#pragma once
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <functional>
#include <cstdlib>
#include <chrono>
#include <sstream>
#include <cstdint>
#ifndef HHH_EKF_SOURCE_HASH
#define HHH_EKF_SOURCE_HASH "unavailable"
#endif
#include "tasks/auto_aim/target.hpp"
#include "tools/logger.hpp"
#include "tasks/omniperception/detection.hpp"

namespace tools {
using EkfJson = nlohmann::json;
inline EkfJson ekf_camera_json(const rmcs_ekf::CameraFeature & c) {
  return {{"matrix",c.camera_matrix},{"distortion",c.distort_coeff},
    {"q_wxyz",{c.orientation.w(),c.orientation.x(),c.orientation.y(),c.orientation.z()}},
    {"translation",{c.translation.x(),c.translation.y(),c.translation.z()}}};
}
inline EkfJson ekf_armors_json(const std::list<auto_aim::Armor> & armors) {
  auto result=EkfJson::array(); std::size_t index=0;
  for(const auto & a:armors) {
    auto corners=EkfJson::array();
    for(const auto & p:a.points) corners.push_back({p.x,p.y});
    result.push_back({{"observation_id",index++},{"color",int(a.color)},
      {"name",int(a.name)},{"large",a.type==auto_aim::ArmorType::big},
      {"priority",int(a.priority)},{"confidence",a.confidence},{"corners",corners}});
  }
  return result;
}
inline EkfJson ekf_target_json(const auto_aim::Target * target) {
  if(!target) return nullptr;
  const auto d=target->diagnostics(); const auto s=target->state();
  const auto r=target->recovery_status();
  const auto & report=target->update_report();
  auto reasons=EkfJson::array();
  for(const auto reason:report.reject_reasons)reasons.push_back(static_cast<int>(reason));
  auto cov=EkfJson::array(); const auto p=target->future_covariance(0.0);
  for(int i=0;i<11;++i) {auto row=EkfJson::array();for(int j=0;j<11;++j)row.push_back(p(i,j));cov.push_back(row);}
  auto residuals=EkfJson::array();
  for(int i=0;i<8;++i) {const auto & v=d.pixel_innovations[i]; residuals.push_back({{"id",i},{"valid",v.valid},{"r",v.valid?EkfJson(v.residual):EkfJson(nullptr)},{"nis",v.valid?EkfJson(v.nis):EkfJson(nullptr)}});}
  return {{"state",{s.x,s.y,s.z,s.vx,s.vy,s.vz,s.rotation_angle,s.rotation_speed,s.radius_forward,s.radius_lateral,s.height_lateral}},
    {"P",cov},{"target_name",int(target->name)},
    {"report",{{"detections",report.detection_count},{"legal",report.legal_armor_count},
      {"associated",report.associated_armor_count},{"ambiguous",report.ambiguous},
      {"nis_valid",report.nis_valid},{"nis_count",report.nis_count},{"quality",report.estimated_quality},
      {"association_cost",report.association_cost},{"second_cost",report.association_second_cost},
      {"prediction_rms",report.prediction_rms},{"reject_reasons",reasons},
      {"trimmed",report.trimmed_count},{"duplicates",report.duplicate_count},
      {"matches",report.matched_observation_by_plate},{"swapped",report.matched_swapped_by_plate},
      {"association_ms",report.association_ms},{"update_ms",report.update_ms}}},{"accepted",d.frame_update_count},{"rejected",d.frame_rejected_count},
    {"accepted_frames",d.update_count},{"residuals",residuals},{"health",int(r.health)},
    {"reliable",r.reliable},{"candidate_active",r.candidate_active},{"candidate_id",r.candidate_id},
    {"last_accepted_age",r.last_accepted_age},{"event",r.event},{"recoveries",r.recovery_count},
    {"candidate_failures",r.candidate_failed_count},{"candidate_time",r.candidate_time},
    {"window_begin",r.window_begin},{"window_end",r.window_end},{"prior_time",r.prior_time},
    {"validation_frames",r.validation_frames},{"validation_rms",r.validation_rms},{"fit_cost",r.fit_cost}};
}
class EkfRecorder {
public:
  explicit EkfRecorder(const std::string & config) {
    try {
      const char * base=std::getenv("HHH_EKF_RECORD_DIR");
      const auto id=std::chrono::system_clock::now().time_since_epoch().count();
      const auto directory=std::filesystem::path(base?base:"logs/ekf")/std::to_string(id);
      std::filesystem::create_directories(directory);
      std::filesystem::copy_file(config,directory/"config.yaml");
      input_.open(directory/"observations.jsonl");trace_.open(directory/"trace.jsonl");
      input_.exceptions(std::ios::badbit|std::ios::failbit);trace_.exceptions(std::ios::badbit|std::ios::failbit);
      std::ifstream config_stream(config,std::ios::binary);
      std::uint64_t hash=14695981039346656037ULL; char byte;
      while(config_stream.get(byte)){hash^=static_cast<unsigned char>(byte);hash*=1099511628211ULL;}
      std::ostringstream hash_text;hash_text<<std::hex<<hash;
      std::ofstream manifest(directory/"manifest.json");
      manifest << EkfJson{{"schema",1},{"algorithm","RobotModel4_v0.60"},{"clock","image_steady_ns"},
        {"units","m,s,rad,px"},{"compiler",__VERSION__},{"source_sha256",HHH_EKF_SOURCE_HASH},{"config_fnv1a64",hash_text.str()},{"opencv",CV_VERSION},
        {"input_scope","raw main-camera and optional omni detections before tracker filtering"}}.dump(2);
      logger()->info("[EKF] recording {}",directory.string());
    } catch(const std::exception & e) {enabled_=false;logger()->error("[EKF] recording unavailable: {}",e.what());}
  }
  void frame(const std::list<auto_aim::Armor> & raw,std::chrono::steady_clock::time_point t,
    const rmcs_ekf::CameraFeature & camera,const auto_aim::Target * target,
    const std::string & state,bool color,double milliseconds,const EkfJson & omni=nullptr) noexcept {
    if(!enabled_)return;
    try {
      auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
      EkfJson observation{{"frame_id",frame_},{"timestamp_ns",ns},{"camera",ekf_camera_json(camera)},
        {"armors",ekf_armors_json(raw)},{"use_enemy_color",color}};
      if(!omni.is_null())observation["omni"]=omni;
      input_<<observation.dump()<<'\n';
      trace_ << EkfJson{{"frame_id",frame_++},{"timestamp_ns",ns},{"tracker_state",state},
        {"target",ekf_target_json(target)},{"estimator_ms",milliseconds}}.dump()<<'\n';
      if(frame_%30==0) {input_.flush();trace_.flush();}
    } catch(const std::exception & e) {enabled_=false;logger()->error("[EKF] recording stopped: {}",e.what());}
  }
private:
  std::ofstream input_,trace_;std::size_t frame_=0;bool enabled_=true;
};
class EkfFrameLog {
public:
  EkfFrameLog(EkfRecorder & recorder,const std::list<auto_aim::Armor> & raw,
    std::chrono::steady_clock::time_point t,const rmcs_ekf::CameraFeature & camera,
    std::function<const auto_aim::Target*()> target,std::function<std::string()> state,bool color=true)
    :recorder_(recorder),raw_(raw),t_(t),camera_(camera),target_(std::move(target)),state_(std::move(state)),color_(color),start_(std::chrono::steady_clock::now()){}
  void omni(const std::vector<omniperception::DetectionResult> & queue) {
    omni_=EkfJson::array();
    for(const auto & v:queue)omni_.push_back({{"timestamp_ns",std::chrono::duration_cast<std::chrono::nanoseconds>(v.timestamp.time_since_epoch()).count()},
      {"armors",ekf_armors_json(v.armors)},{"delta_yaw",v.delta_yaw},{"delta_pitch",v.delta_pitch}});
  }
  ~EkfFrameLog() noexcept { try {recorder_.frame(raw_,t_,camera_,target_(),state_(),color_,
    std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start_).count(),omni_);}catch(...){} }
private:
  EkfJson omni_=nullptr;
  EkfRecorder & recorder_; std::list<auto_aim::Armor> raw_;std::chrono::steady_clock::time_point t_;
  rmcs_ekf::CameraFeature camera_;std::function<const auto_aim::Target*()> target_;
  std::function<std::string()> state_;bool color_;std::chrono::steady_clock::time_point start_;
};
}
