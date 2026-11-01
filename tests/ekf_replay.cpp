// Record detector output from an isolated simulator channel, then replay the
// same pixels/poses/timestamps through Tracker. No truth enters the estimator.
#include <fstream>
#include <iostream>
#include <thread>
#include <cmath>
#include "io/rm_aim_sim/local_client.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/detector.hpp"
using J = io::rm_aim_sim::Json;
namespace sim = io::rm_aim_sim;
int main(int argc,char** argv) {
 try {
  if(argc<4)throw std::runtime_error("usage: ekf_replay capture config file [seconds] | replay|replay-refined config file [trace.jsonl]");
  const std::string mode=argv[1], config=argv[2];
  if(mode=="capture") {
    auto yaml=YAML::LoadFile(config);
    sim::ClientConfig settings;
    settings.channel=yaml["rm_aim_sim"]["channel"].as<std::string>();
    settings.ipc_root=yaml["rm_aim_sim"]["ipc_root"].as<std::string>("/dev/shm");
    settings.state_timeout=std::chrono::milliseconds(yaml["rm_aim_sim"]["state_timeout_ms"].as<int>(200));
    settings.frame_timeout=std::chrono::milliseconds(yaml["rm_aim_sim"]["frame_timeout_ms"].as<int>(5000));
    if(settings.channel=="default")throw std::runtime_error("capture requires an isolated non-default channel");
    sim::LocalClient client(settings);
    auto_aim::YOLO detector(config,false);
    auto_aim::Detector refiner(config,false);
    std::ofstream out(argv[3]); if(!out)throw std::runtime_error("cannot open capture");
    const auto start=sim::Clock::now(); const double seconds=argc>4?std::stod(argv[4]):20;
    if(!std::isfinite(seconds)||seconds<=0)throw std::runtime_error("invalid capture duration");
    size_t captured=0;
    while(std::chrono::duration<double>(sim::Clock::now()-start).count()<seconds) {
      auto change=client.poll_state();
      if(captured && change==sim::PollStateResult::context_changed)
        throw std::runtime_error("capture context changed; start a new recording");
      if(!client.fresh() || client.state()->paused) {std::this_thread::sleep_for(std::chrono::milliseconds(5));continue;}
      client.send_stop(); client.request_frame(); auto frame=client.try_read_frame();
      if(!frame) {std::this_thread::sleep_for(std::chrono::milliseconds(2));continue;}
      const auto check=[&](const auto &actual,const char *key) {
        const auto expected=yaml[key].as<std::vector<double>>();
        if(expected.size()!=actual.size())throw std::runtime_error("capture calibration size mismatch");
        for(size_t i=0;i<actual.size();++i)if(std::abs(expected[i]-actual[i])>1e-5)
          throw std::runtime_error(std::string("capture calibration mismatch: ")+key);
      };
      check(frame->metadata.camera.K,"camera_matrix");
      check(frame->metadata.camera.distortion,"distort_coeffs");
      check(frame->metadata.camera.R_camera2gimbal,"R_camera2gimbal");
      check(frame->metadata.camera.t_camera2gimbal,"t_camera2gimbal");
      J row={{"t",frame->metadata.sim_time},{"armors",J::array()},{"refined_armors",J::array()}};
      auto q=frame->metadata.gimbal_to_world;row["q"]={q.w(),q.x(),q.y(),q.z()};
      auto encode=[](const auto_aim::Armor &a) {
        J points=J::array();for(auto p:a.points)points.push_back({p.x,p.y});
        return J{{"points",points},{"name",int(a.name)},
          {"type",int(a.type)},{"color",int(a.color)},{"priority",int(a.priority)}};
      };
      for(auto a:detector.detect(frame->bgr)) {
        row["armors"].push_back(encode(a));
        refiner.detect(a,frame->bgr);
        row["refined_armors"].push_back(encode(a));
      }
      out<<row.dump()<<'\n';
      if(!out)throw std::runtime_error("capture write failed");
      ++captured;
    }
    out.flush();
    if(!out || !captured)throw std::runtime_error("capture empty or write failed");
    client.send_stop();return 0;
  }
  if(mode!="replay" && mode!="replay-refined")throw std::runtime_error("unknown mode");
  auto_aim::Solver solver(config); auto_aim::Tracker tracker(config,solver);
  std::ifstream input(argv[3]);if(!input)throw std::runtime_error("cannot open replay");
  std::ofstream trace;
  if(argc>4) { trace.open(argv[4]); if(!trace)throw std::runtime_error("cannot open trace"); }
  std::string line;double t0=-1,last_t=0;int frames=0,targets=0,no_update=0,reinits=0,reacquire=0,large_dt=0;
  int previous_updates=-1,previous_reinit=0;double max_anchor=0,max_v=0,max_nis=0;
  const auto epoch=std::chrono::steady_clock::now();
  while(std::getline(input,line)) {
    J row=J::parse(line);double t=row["t"];
    if(!std::isfinite(t) || (frames && t<=last_t))throw std::runtime_error("non-monotonic replay time");
    if(!frames)t0=t;
    if(frames && t-last_t>.1)++large_dt;last_t=t;++frames;
    const auto q=row["q"].get<std::vector<double>>();
    if(q.size()!=4)throw std::runtime_error("invalid replay quaternion");
    Eigen::Quaterniond orientation(q[0],q[1],q[2],q[3]);
    if(!orientation.coeffs().allFinite() || std::abs(orientation.norm()-1)>.001)
      throw std::runtime_error("invalid replay quaternion");
    solver.set_R_gimbal2world(orientation);
    std::list<auto_aim::Armor> armors;
    for(const auto& item:row.at(mode=="replay-refined"?"refined_armors":"armors")) {
      std::vector<cv::Point2f> points;
      for(const auto& p:item["points"])points.emplace_back(float(p[0]),float(p[1]));
      if(points.size()!=4)throw std::runtime_error("replay armor must have four corners");
      for(auto p:points)if(!std::isfinite(p.x)||!std::isfinite(p.y))
        throw std::runtime_error("non-finite replay corner");
      auto_aim::Armor a(9,1.0f,cv::Rect{},points);
      a.name=static_cast<auto_aim::ArmorName>(int(item["name"]));
      a.type=static_cast<auto_aim::ArmorType>(int(item["type"]));
      a.color=static_cast<auto_aim::Color>(int(item["color"]));
      a.priority=static_cast<auto_aim::ArmorPriority>(int(item["priority"]));
      armors.push_back(a);
    }
    auto timestamp=epoch+std::chrono::duration_cast<sim::Clock::duration>(std::chrono::duration<double>(t-t0));
    auto result=tracker.track(armors,timestamp);
    if(result.empty()){
      if(trace)trace<<J{{"frame",frames},{"t",t},{"tracked",false}}.dump()<<'\n';
      previous_updates=-1;previous_reinit=0;continue;
    }
    ++targets;const auto d=result.front().diagnostics();const auto s=result.front().state();
    no_update+=d.frame_update_count==0;
    if(previous_updates<0 || (int(d.update_count)<previous_updates && int(d.reinit_count)<=previous_reinit))++reacquire;
    if(int(d.reinit_count)>previous_reinit)reinits+=int(d.reinit_count)-previous_reinit;
    previous_reinit=d.reinit_count;previous_updates=d.update_count;
    max_anchor=std::max(max_anchor,d.anchor_error);max_nis=std::max(max_nis,d.nis);
    max_v=std::max(max_v,std::hypot(s.vx,s.vy));
    if(trace)trace<<J{{"frame",frames},{"t",t},{"tracked",true},
      {"x",s.x},{"y",s.y},{"z",s.z},{"vx",s.vx},{"vy",s.vy},
      {"yaw",s.rotation_angle},{"w",s.rotation_speed},
      {"r1",s.radius_forward},{"r2",s.radius_lateral},{"h",s.height_lateral},
      {"anchor",d.anchor_error},{"nis",d.nis},{"updates",d.frame_update_count},
      {"total_updates",d.update_count},{"no_update",d.no_update_frames},
      {"reinit",d.reinit_count},{"id",d.last_observed_id}}.dump()<<'\n';
  }
  if(!frames)throw std::runtime_error("empty replay");
  if(trace.is_open()){trace.flush();if(!trace)throw std::runtime_error("trace write failed");}
  std::cout<<J{{"frames",frames},{"tracked_frames",targets},{"zero_update_frames",no_update},
    {"auto_reinits",reinits},{"acquisitions",reacquire},{"gaps_over_100ms",large_dt},
    {"max_anchor_px",max_anchor},{"max_nis",max_nis},{"max_speed",max_v}}.dump()<<'\n';
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
