#include "protocol.hpp"
#include <cmath>
#include <regex>
#include <stdexcept>
namespace io::rm_aim_sim {
namespace {
void require(bool ok,const char *message) { if(!ok) throw std::runtime_error(message); }
double number(const Json &j) { require(j.is_number(),"Expected number"); double v=j.get<double>(); require(std::isfinite(v),"Nonfinite number"); return v; }
int64_t integer(const Json &j,int64_t low,int64_t high) { const auto v=number(j); require(std::floor(v)==v && v>=low && v<=high,"Integer out of range"); return static_cast<int64_t>(v); }
template<size_t N> std::array<double,N> array(const Json &j) { require(j.is_array()&&j.size()==N,"Invalid calibration array"); std::array<double,N> out{}; for(size_t i=0;i<N;++i)out[i]=number(j[i]); return out; }
SimContext context(const Json &j,const char *type) { require(j.at("protocol")=="rm_aim_sim/2" && j.at("type")==type,"Incompatible protocol/type"); SimContext c{j.at("publisher_id").get<std::string>(),j.at("session_id").get<std::string>()}; require(!c.publisher_id.empty()&&!c.session_id.empty(),"Empty context"); return c; }
CameraModel camera(const Json &j) { CameraModel c; c.width=integer(j.at("width"),160,3840); c.height=integer(j.at("height"),120,2160); c.K=array<9>(j.at("K")); c.distortion=array<5>(j.at("distortion")); c.R_camera2gimbal=array<9>(j.at("R_camera2gimbal")); c.t_camera2gimbal=array<3>(j.at("t_camera2gimbal")); require(c.K[0]>0&&c.K[4]>0&&c.K[8]==1,"Invalid focal length"); return c; }
Eigen::Quaterniond quaternion(const Json &j) { auto a=array<4>(j.at("gimbal").at("quaternion_wxyz")); Eigen::Quaterniond q(a[0],a[1],a[2],a[3]); require(std::abs(q.norm()-1)<1e-3,"Invalid quaternion"); return q.normalized(); }
double time(const Json &j) { double t=number(j.at("sim_time")); require(t>=0,"Negative simulation time"); return t; }
}
SimState parse_state(const Json &j) {
  SimState s; s.context=context(j,"state"); s.publication=integer(j.at("publication"),0,9007199254740991LL); s.wall_unix_sec=number(j.at("wall_unix_sec")); s.sim_time=time(j); s.paused=j.at("paused").get<bool>(); s.ipc_enabled=j.at("ipc").at("enabled").get<bool>(); s.last_seq=integer(j.at("ipc").at("last_seq"),-1,9007199254740991LL); s.bullet_speed=number(j.at("weapon").at("speed")); require(s.bullet_speed>0,"Invalid bullet speed"); s.gimbal_to_world=quaternion(j); s.camera=camera(j.at("camera")); s.appearance=j.at("appearance"); s.stats=j.at("stats");
  if(j.contains("truth")) { SimTruthTarget t; t.position=array<3>(j.at("truth").at("center")); auto origin=array<3>(j.at("gimbal").at("origin")); for(int i=0;i<3;++i)t.position[i]-=origin[i]; t.yaw=number(j.at("truth").at("angle")); s.truth.push_back(t); }
  return s;
}
FrameMetadata parse_frame(const Json &j) {
  FrameMetadata f; f.context=context(j,"frame"); f.request_id=j.at("request_id").get<std::string>(); require(std::regex_match(f.request_id,std::regex("[A-Za-z0-9_-]{1,80}")),"Invalid request id"); f.frame_file=j.at("frame_file").get<std::string>(); require(f.frame_file=="frame-"+f.request_id+".raw","Invalid raw filename"); f.encoding=j.at("encoding").get<std::string>(); require(f.encoding=="rgb8"||f.encoding=="rgba8","Invalid encoding"); f.camera=camera(j.at("camera")); f.image_bytes=integer(j.at("image_bytes"),1,3840LL*2160*4); require(f.image_bytes==size_t(f.camera.width)*f.camera.height*(f.encoding=="rgb8"?3:4),"Invalid image size"); f.sim_time=time(j); f.gimbal_to_world=quaternion(j); return f;
}
Json make_command_json(const SimState &s,int64_t seq,const io::Command &c) {
  require(seq>=0&&seq<=9007199254740991LL,"Sequence exhausted"); require(std::isfinite(c.yaw)&&std::isfinite(c.pitch)&&std::abs(c.yaw)<=1000&&std::abs(c.pitch)<=1000,"Invalid command angle");
  return {{"protocol","rm_aim_sim/2"},{"type","command"},{"publisher_id",s.context.publisher_id},{"session_id",s.context.session_id},{"seq",seq},{"control",c.control},{"fire",c.control&&c.shoot},{"yaw",c.yaw},{"pitch",c.pitch},{"yaw_vel",0.0},{"pitch_vel",0.0},{"yaw_acc",0.0},{"pitch_acc",0.0}};
}
}
