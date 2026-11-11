// Fixed-input replay of the real Tracker. No detector, SDK, simulator or truth input.
#include "tasks/auto_aim/tracker.hpp"
#include "tools/ekf_record.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <cmath>
using Json=nlohmann::json;
namespace {
std::list<auto_aim::Armor> read_armors(const Json & array) {
  std::list<auto_aim::Armor> result;
  for(const auto & a:array) {
    std::vector<cv::Point2f> points;
    for(const auto & p:a.at("corners")) {
      const auto number=[](const Json & v){return v.is_number()?v.get<float>():std::numeric_limits<float>::quiet_NaN();};
      points.emplace_back(number(p.at(0)),number(p.at(1)));
    }
    const int color=a.at("color"),name=a.at("name");
    if(color<0||color>3||name<0||name>8)throw std::runtime_error("Invalid armor identity");
    // Construct a safe value before retaining malformed input for Tracker validation.
    auto_aim::Armor armor(0,1.0f,cv::Rect{},{{0,0},{1,0},{1,1},{0,1}});
    armor.points=points;armor.center={0,0};for(const auto & p:points)armor.center+=p;
    if(!points.empty())armor.center*=1.f/points.size();
    armor.color=static_cast<auto_aim::Color>(color);armor.name=static_cast<auto_aim::ArmorName>(name);
    armor.type=a.at("large").get<bool>()?auto_aim::ArmorType::big:auto_aim::ArmorType::small;
    const int priority=a.value("priority",5);if(priority<1||priority>5)throw std::runtime_error("Invalid priority");
    armor.priority=static_cast<auto_aim::ArmorPriority>(priority);
    armor.confidence=a.at("confidence").is_number()?a.at("confidence").get<double>():0;
    result.push_back(armor);
  }
  return result;
}
Eigen::Matrix3d matrix(const YAML::Node & n) {
  auto v=n.as<std::vector<double>>();if(v.size()!=9)throw std::runtime_error("Expected 3x3 extrinsics");
  return Eigen::Matrix<double,3,3,Eigen::RowMajor>(v.data());
}
}
int main(int argc,char **argv) {
  if(argc!=3 || std::string(argv[1])=="--help") {
    std::cout<<"Usage: ekf_replay RUN_DIRECTORY OUTPUT.jsonl\nRun directory must contain config.yaml and observations.jsonl.\n";
    return argc==2?0:1;
  }
  try {
    const auto run=std::filesystem::path(argv[1]);const auto output=std::filesystem::path(argv[2]);
    if(std::filesystem::exists(output))throw std::runtime_error("Output exists; choose a new filename");
    const auto config=run/"config.yaml";const auto yaml=YAML::LoadFile(config.string());
    const auto rgi=matrix(yaml["R_gimbal2imubody"]),rcg=matrix(yaml["R_camera2gimbal"]);
    Eigen::Matrix3d optical_to_ros;optical_to_ros<<0,0,1,-1,0,0,0,-1,0;
    auto_aim::Solver solver(config.string());auto_aim::Tracker tracker(config.string(),solver);
    std::ifstream input(run/"observations.jsonl");if(!input)throw std::runtime_error("Cannot open observations");
    std::ofstream trace(output);trace.exceptions(std::ios::badbit|std::ios::failbit);
    std::string line;std::size_t frames=0,available=0;
    while(std::getline(input,line)) {
      if(line.empty())continue;
      const auto j=Json::parse(line);const auto & c=j.at("camera");const auto & q=c.at("q_wxyz");
      Eigen::Quaterniond camera_q(q.at(0).get<double>(),q.at(1).get<double>(),q.at(2).get<double>(),q.at(3).get<double>());
      if(!camera_q.coeffs().allFinite()||std::abs(camera_q.norm()-1)>1e-5)throw std::runtime_error("Invalid recorded pose");
      const Eigen::Matrix3d gimbal=camera_q.toRotationMatrix()*optical_to_ros*rcg.transpose();
      solver.set_R_gimbal2world(Eigen::Quaterniond(rgi*gimbal*rgi.transpose()));
      const auto restored=tools::ekf_camera_json(solver.camera_feature());
      for(const char *key:{"matrix","distortion","translation"}) {
        const auto a=restored.at(key).flatten(),b=c.at(key).flatten();
        if(a.size()!=b.size())throw std::runtime_error("Recorded camera differs from config");
        for(auto it=a.begin();it!=a.end();++it)
          if(std::abs(it.value().get<double>()-b.at(it.key()).get<double>())>1e-6)
            throw std::runtime_error("Recorded camera differs from config");
      }
      auto armors=read_armors(j.at("armors"));
      const auto t=std::chrono::steady_clock::time_point(std::chrono::nanoseconds(j.at("timestamp_ns").get<int64_t>()));
      std::list<auto_aim::Target> targets;
      if(j.contains("omni")) {
        std::vector<omniperception::DetectionResult> queue;
        for(const auto & v:j.at("omni"))queue.push_back({read_armors(v.at("armors")),
          std::chrono::steady_clock::time_point(std::chrono::nanoseconds(v.at("timestamp_ns").get<int64_t>())),
          v.at("delta_yaw").get<double>(),v.at("delta_pitch").get<double>()});
        targets=std::get<1>(tracker.track(queue,armors,t,j.value("use_enemy_color",true)));
      } else targets=tracker.track(armors,t,j.value("use_enemy_color",true));
      trace<<Json{{"frame_id",j.at("frame_id")},{"timestamp_ns",j.at("timestamp_ns")},
        {"tracker_state",tracker.state()},{"target",tools::ekf_target_json(targets.empty()?nullptr:&targets.front())}}.dump()<<'\n';
      ++frames;if(!targets.empty())++available;
    }
    std::cout<<"frames="<<frames<<" available="<<available<<" trace="<<output<<'\n';
    return frames?0:2;
  }catch(const std::exception & e){std::cerr<<"ekf_replay: "<<e.what()<<'\n';return 1;}
}
