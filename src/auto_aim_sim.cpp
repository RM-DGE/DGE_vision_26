#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <yaml-cpp/yaml.h>
#include "io/rm_aim_sim/local_client.hpp"
#include "io/rm_aim_sim/clock_mapper.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tools/exiter.hpp"
#include "tools/math_tools.hpp"
#include "tools/Foxglove/foxglove.hpp"

using namespace std::chrono_literals;
namespace sim = io::rm_aim_sim;
struct CalibrationError : std::runtime_error {using std::runtime_error::runtime_error;};

static void validate_camera(const sim::CameraModel & camera, const YAML::Node & yaml)
{
  auto check = [&](const auto & actual, const char * name) {
    const auto expected = yaml[name].as<std::vector<double>>();
    if (expected.size() != actual.size()) throw CalibrationError(std::string("Calibration length: ") + name);
    for (size_t i=0;i<actual.size();++i)
      if (std::abs(expected[i]-actual[i]) > 1e-5) throw CalibrationError(std::string("Calibration mismatch: ") + name);
  };
  check(camera.K,"camera_matrix"); check(camera.distortion,"distort_coeffs");
  check(camera.R_camera2gimbal,"R_camera2gimbal"); check(camera.t_camera2gimbal,"t_camera2gimbal");
}

int main(int argc,char **argv)
{
  try {
    const std::string path=argc>1?argv[1]:"configs/rm_aim_sim.yaml";
    const auto yaml=YAML::LoadFile(path);
    sim::ClientConfig config;
    auto settings=yaml["rm_aim_sim"];
    config.channel=settings["channel"].as<std::string>("default");
    config.ipc_root=settings["ipc_root"].as<std::string>("/dev/shm");
    config.state_timeout=std::chrono::milliseconds(settings["state_timeout_ms"].as<int>(200));
    config.frame_timeout=std::chrono::milliseconds(settings["frame_timeout_ms"].as<int>(5000));
    const bool truth=settings["publish_truth"].as<bool>(true);
    const double duration=argc>2?std::stod(argv[2]):0;
    sim::LocalClient client(config);
    sim::ClockMapper clock;
    tools::Exiter exiter;
    tools::FoxgloveServer foxglove;
    foxglove.configure_camera(path);
    auto_aim::YOLO detector(path,false);
    auto_aim::Solver solver(path);
    std::unique_ptr<auto_aim::Tracker> tracker;
    std::unique_ptr<auto_aim::Aimer> aimer;
    std::unique_ptr<auto_aim::Shooter> shooter;
    auto reset=[&] {
      tracker=std::make_unique<auto_aim::Tracker>(path,solver);
      aimer=std::make_unique<auto_aim::Aimer>(path);
      shooter=std::make_unique<auto_aim::Shooter>(path);
      clock.reset();
      foxglove.reset_sim_diagnostics();
    };
    reset();
    const auto begin=sim::Clock::now();
    auto report=begin;
    auto last_frame_received=begin;
    size_t frames=0;
    std::cerr<<"rm_aim_sim pixel closed loop; channel="<<config.channel<<std::endl;
    while(!exiter.exit() && (duration<=0 || std::chrono::duration<double>(sim::Clock::now()-begin).count()<duration)) {
      try {
        auto update=client.poll_state();
        if(client.fresh()) validate_camera(client.state()->camera,yaml);
        if(update==sim::PollStateResult::context_changed) {
          client.send_stop(); reset(); validate_camera(client.state()->camera,yaml);
        }
        if(!client.fresh() || client.state()->paused || !client.state()->ipc_enabled) {
          foxglove.publish_sim_status(client.state() ? &*client.state() : nullptr,client.fresh(),false);
          client.send_stop(); client.cancel_frame(); reset(); std::this_thread::sleep_for(10ms); continue;
        }
        client.request_frame();
        auto frame=client.try_read_frame();
        if(!frame) {
          if(sim::Clock::now()-last_frame_received>200ms)
            foxglove.publish_sim_status(&*client.state(),client.fresh(),false);
          std::this_thread::sleep_for(2ms);continue;
        }
        auto mapped=clock.map(frame->metadata.context,frame->metadata.sim_time,frame->received_at);
        if(mapped.epoch_changed) {
          tracker=std::make_unique<auto_aim::Tracker>(path,solver);
          aimer=std::make_unique<auto_aim::Aimer>(path);
          shooter=std::make_unique<auto_aim::Shooter>(path);
        }
        solver.set_R_gimbal2world(frame->metadata.gimbal_to_world);
        auto armors=detector.detect(frame->bgr);
        auto targets=tracker->track(armors,mapped.timestamp);
        // Refresh state after inference: only measured simulation elapsed time enters prediction.
        auto post=client.poll_state();
        if(post==sim::PollStateResult::context_changed || !client.fresh() ||
           !(client.state()->context==frame->metadata.context)) {client.send_stop();reset();continue;}
        const double age=std::max(0.0,client.state()->sim_time-frame->metadata.sim_time);
        if(age>.2) {client.send_stop();reset();continue;}
        auto command=aimer->aim(targets,mapped.timestamp,client.state()->bullet_speed,false,age);
        auto ypr=tools::eulers(solver.R_gimbal2world(),2,1,0);
        command.shoot=shooter->shoot(command,*aimer,targets,ypr);
        client.send(command);
        const double frame_dt=std::chrono::duration<double>(frame->received_at-last_frame_received).count();
        const double fps=frame_dt>0?1.0/frame_dt:0;
        last_frame_received=frame->received_at;
        const auto aim_xyza=aimer->debug_aim_point.valid?aimer->debug_aim_point.xyza:Eigen::Vector4d::Zero().eval();
        foxglove.publish_image(frame->bgr); foxglove.publish_annotations(armors);
        foxglove.publish_number_image(armors);
        foxglove.publish_rmvision(armors,targets,aim_xyza);
        foxglove.publish_detector_markers(armors);
        foxglove.publish_tracker_markers(targets);
        // State truth is newer than the image, and its yaw is not an observed armor yaw.
        // Do not feed it to legacy auto-aligned error curves.
        foxglove.publish_debug(fps,command,targets,armors,aimer->debug_aim_point.valid,aim_xyza,ypr);
        foxglove.publish_sim_status(&*client.state(),true,true,frame->metadata.sim_time,age);
        foxglove.publish_tf(frame->metadata.gimbal_to_world);
        foxglove.publish_scene(armors,targets,aimer->debug_aim_point.valid,aimer->debug_aim_point.xyza,frame->metadata.gimbal_to_world);
        foxglove.publish_prediction(targets);
        if(truth)foxglove.publish_ground_truth(client.state()->truth);
        ++frames;
        if(sim::Clock::now()-report>1s) {
          std::cout<<sim::Json{{"observation_age_s",age},{"sim_time",frame->metadata.sim_time},{"frames",frames},{"armors",armors.size()},{"targets",targets.size()},{"control",command.control},{"fire",command.shoot},{"stats",client.state()->stats}}.dump()<<std::endl;
          report=sim::Clock::now();
        }
      } catch(const CalibrationError &) {throw;
      } catch(const std::exception &e) {
        foxglove.publish_sim_status(client.state() ? &*client.state() : nullptr,client.fresh(),false);
        try{client.send_stop();}catch(...){}
        client.cancel_frame();reset();std::cerr<<e.what()<<std::endl;
        std::this_thread::sleep_for(50ms);
      }
    }
    client.send_stop();
    return 0;
  } catch(const std::exception &e) {std::cerr<<e.what()<<std::endl;return 1;}
}
