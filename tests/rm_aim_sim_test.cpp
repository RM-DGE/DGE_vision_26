#include "io/rm_aim_sim/local_client.hpp"
#include "io/rm_aim_sim/clock_mapper.hpp"
#include <fstream>
#include <filesystem>
#include <iostream>
#include <thread>
#include <sys/stat.h>
#include <unistd.h>
using namespace io::rm_aim_sim;
void check_at(bool value,int line){if(!value)throw std::runtime_error("test assertion failed at line "+std::to_string(line));}
#define check(value) check_at((value),__LINE__)
template<class F> void rejects(F f){bool caught=false;try{f();}catch(...){caught=true;}check(caught);}
int main(){try{
  Json camera={{"width",160},{"height",120},{"K",{100,0,80,0,100,60,0,0,1}},
    {"distortion",{0,0,0,0,0}},{"R_camera2gimbal",{0,0,1,-1,0,0,0,-1,0}},{"t_camera2gimbal",{.08,0,.04}}};
  const double wall=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  Json state={{"protocol","rm_aim_sim/2"},{"type","state"},{"publisher_id","server"},{"session_id","session"},
    {"publication",1},{"wall_unix_sec",wall},{"sim_time",1.0},{"paused",false},
    {"ipc",{{"enabled",true},{"last_seq",4}}},{"weapon",{{"speed",24.5}}},
    {"gimbal",{{"quaternion_wxyz",{1,0,0,0}},{"origin",{0,0,1}}}},{"camera",camera},
    {"appearance",{{"robot","infantry_3"}}},{"stats",Json::object()}};
  auto parsed=parse_state(state);check(parsed.last_seq==4);
  auto godot=state;godot["camera"]["width"]=160.0;godot["camera"]["height"]=120.0;check(parse_state(godot).camera.width==160);
  godot["camera"]["width"]=160.5;rejects([&]{parse_state(godot);});
  auto invalid=state;invalid["protocol"]="bad";rejects([&]{parse_state(invalid);});
  invalid=state;invalid["camera"]["K"]={1};rejects([&]{parse_state(invalid);});
  invalid=state;invalid["paused"]="false";rejects([&]{parse_state(invalid);});
  rejects([&]{make_command_json(parsed,-1,{true,true,0,0});});
  rejects([&]{make_command_json(parsed,9007199254740992LL,{true,true,0,0});});
  auto cmd=make_command_json(parsed,5,{true,true,.1,-.2});check(cmd["pitch"]==-.2&&cmd["fire"]==true);
  check(make_command_json(parsed,6,{false,true,0,0})["fire"]==false);
  ClockMapper mapper;auto now=Clock::now();auto a=mapper.map(parsed.context,1,now);auto b=mapper.map(parsed.context,1.1,now);
  check(std::abs(std::chrono::duration<double>(b.timestamp-a.timestamp).count()-.1)<1e-6);
  check(mapper.map({"server","new-session"},0,now).epoch_changed);
  char root[]="/tmp/dge-sim-test-XXXXXX";check(mkdtemp(root)!=nullptr);
  auto dir=std::filesystem::path(root)/"rm_aim_sim_test";std::filesystem::create_directory(dir);chmod(dir.c_str(),0700);
  auto write=[&](const char*name,const Json&j){std::ofstream(dir/name)<<j.dump();};
  write("state.json",state);
  ClientConfig config;config.channel="test";config.ipc_root=root;config.state_timeout=std::chrono::milliseconds(100);
  {
    LocalClient client(config);check(client.poll_state()==PollStateResult::context_changed);
    rejects([&]{LocalClient second(config);});
    client.request_frame();Json request;std::ifstream(dir/"request.json")>>request;
    auto frame=state;frame["type"]="frame";frame["request_id"]=request["request_id"];
    std::string raw="frame-"+request["request_id"].get<std::string>()+".raw";
    frame["frame_file"]=raw;frame["encoding"]="rgb8";frame["image_bytes"]=160*120*3;
    std::vector<unsigned char> pixels(160*120*3,0);pixels[0]=255;
    {std::ofstream out(dir/raw,std::ios::binary);out.write(reinterpret_cast<char*>(pixels.data()),pixels.size());}
    write("frame.json",frame);auto image=client.try_read_frame();check(image.has_value());check(image->bgr.at<cv::Vec3b>(0,0)==cv::Vec3b(0,0,255));
    client.request_frame();std::ifstream(dir/"request.json")>>request;
    raw="frame-"+request["request_id"].get<std::string>()+".raw";
    frame["request_id"]=request["request_id"];frame["frame_file"]=raw;frame["encoding"]="rgba8";frame["image_bytes"]=160*120*4;
    pixels.resize(160*120*4);pixels[0]=0;pixels[1]=255;pixels[2]=0;pixels[3]=255;
    {std::ofstream out(dir/raw,std::ios::binary);out.write(reinterpret_cast<char*>(pixels.data()),pixels.size()-1);}
    write("frame.json",frame);rejects([&]{client.try_read_frame();});
    {std::ofstream out(dir/raw,std::ios::binary);out.write(reinterpret_cast<char*>(pixels.data()),pixels.size());}
    image=client.try_read_frame();check(image&&image->bgr.at<cv::Vec3b>(0,0)==cv::Vec3b(0,255,0));
    client.send({true,true,.1,-.2});Json sent;std::ifstream(dir/"command.json")>>sent;check(sent["seq"]==5&&sent["fire"]==true);
    std::this_thread::sleep_for(std::chrono::milliseconds(120));check(client.poll_state()==PollStateResult::unchanged);check(!client.fresh());
    client.send({true,true,0,0});std::ifstream(dir/"command.json")>>sent;check(sent["control"]==false&&sent["fire"]==false);
  }
  std::cout<<"protocol, clock, lock, handshake, RGB copy, sequence and watchdog PASS; fixture "<<root<<std::endl;
  return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}}
