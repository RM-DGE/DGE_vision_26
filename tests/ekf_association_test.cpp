#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <cstring>
#include <new>
#include "tasks/auto_aim/rmcs_ekf/rmcs_ekf.hpp"
#include "tasks/auto_aim/tracker.hpp"

using namespace rmcs_ekf;
CameraFeature camera() {
  CameraFeature c;
  c.camera_matrix = {{{1303.6753,0,720},{0,1303.6753,540},{0,0,1}}};
  return c;
}
std::vector<Armor2d> observe(const State& s, const CameraFeature& c, bool single) {
  RobotSolution solution;
  solution.input.center = {s.x,s.y,s.z};
  solution.input.toward = Eigen::Quaterniond(Eigen::AngleAxisd(s.rotation_angle,Eigen::Vector3d::UnitZ()));
  solution.input.radius_forward=s.radius_forward;
  solution.input.radius_lateral=s.radius_lateral;
  solution.input.height_lateral=s.height_lateral;
  auto plates=solution.solve_armors();
  auto bars=solution.solve_lightbars();
  const auto pixel=[&](const Eigen::Vector3d& p) {
    Eigen::Vector3d v=c.orientation.conjugate()*(p-c.translation);
    return cv::Point2f(720-1303.6753*v.y()/v.x(),540-1303.6753*v.z()/v.x());
  };
  std::vector<Armor2d> result;
  for(int i=0;i<4;++i) {
    const Eigen::Vector3d normal=-(plates[i].orientation*Eigen::Vector3d::UnitX());
    const Eigen::Vector3d ray=(c.translation-plates[i].translation).normalized();
    if(normal.dot(ray)<std::cos(65*kPi/180))continue;
    auto u=pixel(bars[2*i].upper), l=pixel(bars[2*i].lower);
    auto u2=pixel(bars[2*i+1].upper), l2=pixel(bars[2*i+1].lower);
    if(u.x>u2.x){std::swap(u,u2);std::swap(l,l2);}
    Armor2d armor;armor.tl=u;armor.bl=l;armor.tr=u2;armor.br=l2;
    result.push_back(armor);
  }
  if(single && result.size()>1)result.resize(1);
  return result;
}
int run(int argc,char** argv) {
  int failed=0;
  {
    alignas(auto_aim::Armor) unsigned char storage[sizeof(auto_aim::Armor)];
    std::memset(storage,0x55,sizeof(storage));
    auto *armor=new(storage) auto_aim::Armor(0,3,1.f,cv::Rect{},
      std::vector<cv::Point2f>{{10,10},{30,10},{30,20},{10,20}});
    int priority=0;static_assert(sizeof(priority)==sizeof(armor->priority));
    std::memcpy(&priority,&armor->priority,sizeof(priority));
    armor->~Armor();
    if(priority!=int(auto_aim::ArmorPriority::fifth)) {
      std::cerr<<"YOLO armor has no deterministic default priority\n";++failed;
    }
  }
  // IPPE pitch/roll are noisy, while Solver's optimized yaw and the EKF's
  // carousel model assume a horizontal radius. Initialization must preserve
  // the measured plate center even when those unused pose angles disagree.
  for(double pitch:{-.3,.6}) {
    const Eigen::Vector3d plate{3.8,.1,-.4};
    RobotModel model;
    model.initialize_from_pose(camera(),plate,euler_to_quaternion(.25,pitch,.15),false);
    if((model.full()[0].translation-plate).norm()>1e-9) {
      std::cerr<<"pose initialization moved the measured plate center by "
               <<(model.full()[0].translation-plate).norm()<<" m\n";
      ++failed;
    }
  }
  for(bool single:{true,false}) {
    double worst=0; int rejected=0, updates=0;
    for(int degree=0;degree<360;degree+=5) {
      State truth;truth.x=4;truth.z=-.4;truth.rotation_angle=degree*kPi/180;
      truth.radius_forward=.25;truth.radius_lateral=.20;truth.height_lateral=-.10;
      RobotModel model;Config cfg;cfg.auto_reinit_after_no_update=0;model.configure(cfg);
      model.initialize_synthetic(truth);
      auto obs=observe(truth,camera(),single);
      model.correct(obs,{},camera());
      auto d=model.diagnostics();
      worst=std::max(worst,d.residual_norm);
      updates+=d.frame_update_count; rejected+=d.frame_rejected_count;
      if(d.residual_norm>.001 || d.frame_update_count!=obs.size()*2) ++failed;
    }
    std::cout<<"single="<<single<<" max_residual="<<worst<<" updates="<<updates<<" rejected="<<rejected<<'\n';
  }
  for(double rate:{0.,4.,-4.}) {
    RobotModel model;
    Config cfg=argc>1?load_config(YAML::LoadFile(argv[1])["ekf"]):Config{};
    model.configure(cfg);
    State truth;truth.x=4;truth.z=-.4;truth.rotation_angle=3.141592653589793;
    truth.radius_forward=.25;truth.radius_lateral=.25;truth.height_lateral=-.1;
    const auto c=camera();
    if(!model.initialize(c,observe(truth,c,false)))throw std::runtime_error("initialization failed");
    double worst=0;int empty_updates=0;
    for(int i=0;i<400;++i) {
      truth.rotation_angle+=rate*.05;
      model.predict(.05);
      model.correct(observe(truth,c,false),{},c);
      auto d=model.diagnostics();
      worst=std::max(worst,d.residual_norm);
      empty_updates+=d.frame_update_count==0;
    }
    const auto s=model.state();const auto d=model.diagnostics();
    std::cout<<"rate="<<rate<<" estimated="<<s.rotation_speed<<" residual_max="<<worst
             <<" reinit="<<d.reinit_count<<" no_update="<<empty_updates
             <<" center="<<s.x<<","<<s.y<<","<<s.z<<'\n';
    if(d.reinit_count!=0 || empty_updates!=0 || !std::isfinite(s.rotation_speed)
        || std::abs(s.rotation_speed-rate)>.1 || std::abs(s.x-4)>.1
        || std::abs(s.y)>.02) {
      std::cerr<<"ideal motion failed to remain stable\n";++failed;
    }
  }
  if(argc>1) {
    auto_aim::Solver solver(argv[1]);solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());
    auto_aim::Tracker tracker(argv[1],solver);
    State s;s.x=4;s.z=-.4;s.rotation_angle=kPi;s.radius_forward=.2;s.radius_lateral=.2;
    auto input=[&] {
      std::list<auto_aim::Armor> result;
      for(auto a:observe(s,solver.camera_feature(),true))
        result.emplace_back(9,1.f,cv::Rect{},std::vector<cv::Point2f>{a.tl,a.tr,a.br,a.bl});
      return result;
    };
    auto t=std::chrono::steady_clock::now();
    std::list<auto_aim::Target> targets;
    for(int i=0;i<8;++i){t+=std::chrono::milliseconds(50);auto a=input();targets=tracker.track(a,t);}
    if(targets.empty())throw std::runtime_error("static tracker failed to initialize");
    const auto count=targets.front().diagnostics().update_count;
    t+=std::chrono::milliseconds(120);auto a=input();targets=tracker.track(a,t);
    if(targets.empty()||targets.front().diagnostics().update_count<=count)
      throw std::runtime_error("120ms CPU frame gap discarded healthy EKF history");
    t+=std::chrono::milliseconds(50);std::list<auto_aim::Armor> empty;
    targets=tracker.track(empty,t);
    if(targets.empty() || targets.front().diagnostics().frame_update_count!=0)
      throw std::runtime_error("missing observation reused the previous frame update diagnostics");
    t+=std::chrono::milliseconds(400);
    if(!tracker.track(empty,t).empty())throw std::runtime_error("true stream gap must still clear target");
  }
  if(failed) {std::cerr<<failed<<" perfect-observation cases corrupted or skipped\n";return 1;}
  return 0;
}
int main(int argc,char** argv) {
  try {return run(argc,argv);}
  catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
