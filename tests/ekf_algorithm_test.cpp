// Fixed-input algorithm checks; no camera, serial port or simulator is required.
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/rmcs_ekf/projection.hpp"
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <set>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace {
void require(bool ok, const char * message) {
  if (!ok) throw std::runtime_error(message);
}
rmcs_ekf::CameraFeature camera() {
  rmcs_ekf::CameraFeature c;
  c.camera_matrix={{{1300,0,720},{0,1300,540},{0,0,1}}};
  c.distort_coeff={0,0,0,0,0};
  c.orientation=Eigen::Quaterniond::Identity();
  c.translation.setZero();
  return c;
}
std::vector<rmcs_ekf::Armor2d> observe(const rmcs_ekf::RobotModel & truth) {
  std::vector<rmcs_ekf::Armor2d> result;
  const auto c=camera();
  for (const auto & plate:truth.full()) {
    const auto ray=(c.translation-plate.translation).normalized();
    if (-(plate.orientation*Eigen::Vector3d::UnitX()).dot(ray)<std::cos(65*M_PI/180)) continue;
    const auto l=rmcs_ekf::project_lightbar(truth.state_vector(),false,c,2*plate.id);
    const auto r=rmcs_ekf::project_lightbar(truth.state_vector(),false,c,2*plate.id+1);
    if (!l || !r) continue;
    rmcs_ekf::Armor2d a;
    a.tl={float(l->value[0]),float(l->value[1])};
    a.bl={float(l->value[2]),float(l->value[3])};
    a.tr={float(r->value[0]),float(r->value[1])};
    a.br={float(r->value[2]),float(r->value[3])};
    result.push_back(a);
  }
  return result;
}
void late_geometry() {
  rmcs_ekf::Config cfg; cfg.noise_observation=100;
  rmcs_ekf::RobotModel model(cfg),truth(cfg);
  rmcs_ekf::State s; s.x=2.5;s.radius_forward=.25;s.radius_lateral=.26;
  s.height_lateral=.08;s.rotation_angle=M_PI;
  truth.initialize_synthetic(s);
  const auto plate=truth.full()[0];const auto obs=observe(truth);
  require(model.initialize_from_pose(camera(),plate.translation,plate.orientation,false,&obs[0]),"pose initialization");
  for(int i=0;i<60;++i) {model.predict(.01);model.correct(observe(truth),{},camera());}
  std::set<int> learned;
  for(int i=0;i<300;++i) {
    s.rotation_angle+=.05;s.rotation_speed=5;truth.initialize_synthetic(s);
    model.predict(.01);const auto report=model.correct(observe(truth),{},camera());
    for(int p=0;p<4;++p)if(report.matched_observation_by_plate[p]>=0)learned.insert(p);
    require(model.covariance().allFinite(),"finite covariance");
    Eigen::LLT<rmcs_ekf::RobotModel::Covariance> factor(model.covariance());
    require(factor.info()==Eigen::Success,"positive covariance");
  }
  require(learned.size()==4,"late second group must associate");
  require(std::abs(std::abs(model.state().height_lateral)-.08)<.02,"late group height must learn with NIS enabled");
  require(model.geometry_ready()[0] && model.geometry_ready()[1],"both groups must finish geometry learning");
}
void aim_switching(const std::string & config) {
  const auto yaml=YAML::LoadFile(config);
  const double decision_speed=yaml["decision_speed"].as<double>();
  for(double w:{1.,-1.,5.,-5.,10.,-10.,12.,-12.}) {
    auto_aim::Aimer aimer(config);
    const auto base=auto_aim::Target::synthetic(2.5,w,.25,.08);
    std::set<int> selected;
    const int steps=static_cast<int>(8/(.01*std::abs(w)));
    for(int i=0;i<steps;++i) {
      const auto t=base.at(.01*i);
      const auto cmd=aimer.aim({t},t.time(),24.5,false);
      if(!cmd.control)continue;
      const auto aim=aimer.debug_aim_point.xyza;
      const double delay=yaml[std::abs(w)>decision_speed ? "high_speed_delay_time" : "low_speed_delay_time"].as<double>();
      const double hit_dt=.005+delay+std::hypot(aim.x(),aim.y())/24.5;
      const auto plates=t.at(hit_dt).full();
      int best=-1;double distance=1e9;
      for(const auto & p:plates)if((p.translation-aim.head<3>()).norm()<distance){distance=(p.translation-aim.head<3>()).norm();best=p.id;}
      require(aim.x()<2.65,"aim must not follow a rear-facing plate");
      require(aimer.debug_aim_point.armor_id==best && distance<.02,"recorded aim plate must match predicted geometry");
      selected.insert(best);
    }
    require(selected.size()==4,"aim must switch across all four plates in both directions");
    aimer.aim({},base.time(),24.5,false);
    require(!aimer.debug_aim_point.valid,"empty target must clear stale aim marker");
  }
}
std::string tracker_config(const std::string & base,int confirmations,int missing,bool nis_case=false) {
  auto yaml=YAML::LoadFile(base);
  yaml["min_detect_count"]=confirmations;
  yaml["max_temp_lost_count"]=missing;
  if(nis_case) {
    yaml["ekf"]["noise_observation"]=1.;
    yaml["ekf"]["association_miss_penalty"]=1e6;
    yaml["ekf"]["association_clutter_penalty"]=1e6;
  }
  const auto dir=std::filesystem::path("logs/tracker_cases");
  std::filesystem::create_directories(dir);
  const auto path=dir/(std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".yaml");
  std::ofstream file(path);file.exceptions(std::ios::badbit|std::ios::failbit);file<<yaml;
  return path.string();
}
auto_aim::Armor tracker_armor(const auto_aim::Solver & solver,const std::string & config) {
  rmcs_ekf::RobotModel truth;
  rmcs_ekf::State state;
  state.x=2.5;state.rotation_angle=M_PI-.2;
  state.radius_forward=.25;state.radius_lateral=.26;state.height_lateral=.08;
  truth.initialize_synthetic(state);
  const auto c=solver.camera_feature();
  const auto left=rmcs_ekf::project_lightbar(truth.state_vector(),false,c,0);
  const auto right=rmcs_ekf::project_lightbar(truth.state_vector(),false,c,1);
  require(left.has_value() && right.has_value(),"lifecycle fixture projects");
  auto_aim::Armor armor(9,1.f,cv::Rect{},
    {{float(left->value[0]),float(left->value[1])},
     {float(right->value[0]),float(right->value[1])},
     {float(right->value[2]),float(right->value[3])},
     {float(left->value[2]),float(left->value[3])}});
  armor.color=YAML::LoadFile(config)["enemy_color"].as<std::string>()=="red" ? auto_aim::red : auto_aim::blue;
  return armor;
}
struct TrackerCase {
  std::string config;
  auto_aim::Solver solver;
  auto_aim::Tracker tracker;
  auto_aim::Armor armor;
  bool omni;
  std::chrono::steady_clock::time_point time{std::chrono::seconds(10)};
  TrackerCase(const std::string & base,int confirmations,int missing,bool use_omni=false,bool nis_case=false)
  :config(tracker_config(base,confirmations,missing,nis_case)),solver(config),tracker(config,solver),
   armor(tracker_armor(solver,config)),omni(use_omni) {}
  std::list<auto_aim::Target> step(std::list<auto_aim::Armor> observations,int ms=10,
    const std::vector<omniperception::DetectionResult> & queue={}) {
    time+=std::chrono::milliseconds(ms);
    if(omni)return std::get<1>(tracker.track(queue,observations,time));
    return tracker.track(observations,time);
  }
  void warm() {
    for(int i=0;i<50;++i)step({armor});
    require(tracker.state()=="tracking","fixture becomes tracked");
  }
  auto_aim::Armor unmatched() const {
    auto shifted=armor;
    for(auto & p:shifted.points)p.x+=500;
    shifted.center.x+=500;
    return shifted;
  }
};
void target_lifecycle(const std::string & config,int missing,int ms,bool omni) {
  TrackerCase f(config,5,missing,omni);f.warm();
  const auto first_id=f.tracker.runtime_stats().track_id;
  for(int i=0;i<missing;++i)
    require(!f.step({},ms).empty(),"must retain through the configured missing-frame allowance");
  require(f.tracker.state()=="temp_lost","missing observations enter temp_lost");
  require(f.step({},ms).empty() && f.tracker.state()=="lost","configured missing frames must expire the Target");
  const auto targets=f.step({f.armor});
  require(!targets.empty() && f.tracker.runtime_stats().track_id==first_id+1,"reacquisition must create a new Target");
  require(f.tracker.runtime_stats().reinitializations==1,"reacquisition counts one reinitialization");
  const auto & next=targets.front();
  require(next.diagnostics().update_count==0 && !next.geometry_ready(0) && !next.geometry_ready(1),
    "new Target starts fresh update and geometry confirmation");
  require(next.state().rotation_speed==0 && next.state().vx==0,
    "new Target must use the existing initialization rather than an old fitted state");
}
void confirmation_and_partial_updates(const std::string & config) {
  TrackerCase f(config,5,15);
  f.step({f.armor});f.step({f.armor});
  const auto id=f.tracker.runtime_stats().track_id;
  f.step({});
  require(f.tracker.state()=="detecting","interrupted confirmation stays unconfirmed");
  for(int i=0;i<4;++i) {
    f.step({f.armor});
    require(f.tracker.state()=="detecting","confirmation requires consecutive successful frames");
  }
  f.step({f.armor});
  require(f.tracker.state()=="tracking" && f.tracker.runtime_stats().track_id==id,"fifth confirmation keeps original Target");
  f.warm();
  for(int i=0;i<70;++i) {
    const auto targets=f.step({f.armor,f.unmatched()});
    require(!targets.empty(),"unmatched extra detection must not discard a valid track");
    const auto & r=targets.front().update_report();
    require(r.legal_armor_count==2 && r.associated_armor_count==1 && r.accepted_lightbar_count>0,
      "fixture contains a genuine partial association with an accepted update");
    require(f.tracker.state()=="tracking" && f.tracker.runtime_stats().track_id==id,
      "partial association must not demote or recreate the accepted target");
  }
  for(int cycle=0;cycle<2;++cycle) {
    for(int i=0;i<15;++i)require(!f.step({}).empty(),"short loss keeps the Target");
    f.step({f.armor});
    require(f.tracker.state()=="tracking" && f.tracker.runtime_stats().track_id==id,"accepted update resets missing count");
  }
}
void rejected_observations_and_timestamps(const std::string & config) {
  TrackerCase f(config,5,3,true);f.warm();
  for(int i=0;i<3;++i) {
    const auto targets=f.step({f.unmatched()});
    require(!targets.empty() && targets.front().update_report().legal_armor_count==1 &&
      targets.front().diagnostics().frame_update_count==0,"detected but unmatched is a failed update");
  }
  require(f.step({f.unmatched()}).empty(),"detections without accepted updates must expire too");
  f.warm();
  const auto id=f.tracker.runtime_stats().track_id;
  const auto before=f.step({f.armor}).front().diagnostics().update_count;
  require(f.step({},0).front().diagnostics().update_count==before && f.tracker.state()=="tracking",
    "duplicate timestamp must not consume a missing frame");
  require(f.step({},-1).front().diagnostics().update_count==before && f.tracker.state()=="tracking",
    "backwards timestamp must not consume a missing frame");
  require(f.step({},200).empty() && f.tracker.state()=="lost","frame gap exits even without consuming the frame allowance");
  require(!f.step({f.armor}).empty() && f.tracker.runtime_stats().track_id==id+1,"frame-gap reacquisition creates a fresh Target");
}
void nis_rejection_and_divergence(const std::string & config) {
  TrackerCase partial(config,5,3,false,true);partial.warm();
  const auto id=partial.tracker.runtime_stats().track_id;
  auto one_bad_edge=partial.armor;
  one_bad_edge.points[1].x+=20;one_bad_edge.points[2].x+=20;
  for(int i=0;i<70;++i) {
    const auto targets=partial.step({one_bad_edge});
    require(!targets.empty(),"one accepted endpoint pair must keep the target");
    const auto & r=targets.front().update_report();
    require(r.accepted_lightbar_count==1 && !r.estimated_quality &&
      std::find(r.reject_reasons.begin(),r.reject_reasons.end(),rmcs_ekf::RejectReason::NisGate)!=r.reject_reasons.end(),
      "fixture accepts one endpoint pair and rejects the other by NIS");
    require(partial.tracker.state()=="tracking" && partial.tracker.runtime_stats().track_id==id,
      "partial update is not subject to a second quality timeout");
  }
  TrackerCase f(config,5,3,false,true);f.warm();
  auto shifted=f.armor;
  for(auto & p:shifted.points)p.x+=20;
  shifted.center.x+=20;
  for(int i=0;i<3;++i) {
    const auto targets=f.step({shifted});
    require(!targets.empty(),"NIS rejection retains the short-lived prediction");
    const auto & r=targets.front().update_report();
    require(r.accepted_lightbar_count==0 &&
      std::find(r.reject_reasons.begin(),r.reject_reasons.end(),rmcs_ekf::RejectReason::NisGate)!=r.reject_reasons.end(),
      "fixture reaches NIS rejection after association");
  }
  require(f.step({shifted}).empty(),"persistent NIS rejection must expire the Target");
  TrackerCase d(config,5,1000);d.warm();
  auto targets=d.step({d.armor});
  const double limit=YAML::LoadFile(d.config)["ekf"]["diverge_cov"].as<double>(150.);
  for(int i=0;i<1000;++i) {
    const auto p=targets.front().future_covariance(.01);
    targets=d.step({});
    if(targets.empty()) {
      require(p(0,0)>limit || p(1,1)>limit,"covariance divergence, not a hidden timeout, must cause this loss");
      require(d.tracker.state()=="lost","divergence enters LOST before the frame allowance");
      return;
    }
  }
  require(false,"divergence fixture must reach its covariance limit");
}
void confirmation_boundaries_and_switching(const std::string & config) {
  TrackerCase immediate(config,1,0);
  require(!immediate.step({immediate.armor}).empty() && immediate.tracker.state()=="tracking",
    "min_detect_count one confirms initialization immediately");
  require(immediate.step({}).empty(),"zero allowance loses on the first missing update");

  TrackerCase detecting(config,5,2);
  detecting.step({detecting.armor});
  for(int i=0;i<2;++i)require(!detecting.step({}).empty() && detecting.tracker.state()=="detecting",
    "unconfirmed target retains only its configured allowance");
  require(detecting.step({}).empty(),"unconfirmed target must expire too");

  TrackerCase main(config,3,4,true);main.armor.priority=auto_aim::ArmorPriority::third;main.warm();
  auto higher=main.armor;higher.name=auto_aim::ArmorName::one;higher.priority=auto_aim::ArmorPriority::first;
  const auto id=main.tracker.runtime_stats().track_id;
  main.step({higher});
  require(main.tracker.runtime_stats().track_id==id+1 && main.tracker.state()=="detecting",
    "main-camera priority switch starts confirmation for a new Target");
  main.step({higher});require(main.tracker.state()=="detecting","second frame does not confirm a three-frame target");
  main.step({higher});require(main.tracker.state()=="tracking","third frame confirms priority switch");

  // Precise synthetic corners (R=1) let this single static plate satisfy the
  // unchanged convergence gate required by the actual omni-switching branch.
  TrackerCase omni(config,3,4,true,true);omni.armor.priority=auto_aim::ArmorPriority::third;omni.warm();omni.warm();
  require(omni.step({omni.armor}).front().converge(),"omni fixture meets the existing convergence prerequisite");
  higher=omni.armor;higher.name=auto_aim::ArmorName::one;higher.priority=auto_aim::ArmorPriority::first;
  const std::vector<omniperception::DetectionResult> queue{{{higher},omni.time,0.,0.}};
  omni.step({},10,queue);require(omni.tracker.state()=="switching","actual omni detection enters switching");
  for(int i=1;i<200;++i)require(!omni.step({}).empty() && omni.tracker.state()=="switching",
    "omni turning retains its separate existing 200-frame allowance");
  require(omni.step({}).empty(),"omni turn timeout must still exit");
  omni.warm();omni.warm();
  const auto omni_id=omni.tracker.runtime_stats().track_id;
  omni.step({},10,queue);
  omni.step({higher});require(omni.tracker.state()=="detecting" && omni.tracker.runtime_stats().track_id==omni_id+1,
    "omni acquisition creates and confirms a new Target");
  omni.step({higher});require(omni.tracker.state()=="detecting","omni acquisition needs consecutive confirmation");
  omni.step({higher});require(omni.tracker.state()=="tracking","omni acquisition finishes confirmation");
}
}
int main(int argc,char **argv) {
  int failures=0;
  const auto run=[&](const char * name,auto fn) {
    try {fn();std::cout<<"PASS "<<name<<'\n';}
    catch(const std::exception & e){++failures;std::cerr<<"FAIL "<<name<<": "<<e.what()<<'\n';}
  };
  run("late geometry",late_geometry);
  run("aim switching",[&]{aim_switching(argc>1?argv[1]:"configs/daedalus.yaml");});
  if(argc>2) run("aim switching with unequal delays",[&]{aim_switching(argv[2]);});
  const std::string config=argc>1?argv[1]:"configs/daedalus.yaml";
  run("frame allowance 2",[&]{target_lifecycle(config,2,10,false);});
  run("frame allowance 15",[&]{target_lifecycle(config,15,10,false);});
  run("frame allowance 15 at 50 FPS / omni",[&]{target_lifecycle(config,15,20,true);});
  run("consecutive confirmation and partial updates",[&]{confirmation_and_partial_updates(config);});
  run("rejected observations and timestamps",[&]{rejected_observations_and_timestamps(config);});
  run("NIS rejection and covariance divergence",[&]{nis_rejection_and_divergence(config);});
  run("confirmation boundaries and priority switching",[&]{confirmation_boundaries_and_switching(config);});
  return failures?1:0;
}
