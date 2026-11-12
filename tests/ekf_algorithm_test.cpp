// Fixed-input algorithm checks; no camera, serial port or simulator is required.
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/rmcs_ekf/projection.hpp"
#include <iostream>
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
  return failures?1:0;
}
