#include "local_client.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <regex>
#include <stdexcept>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
namespace io::rm_aim_sim {
namespace {
struct Fd { int value; ~Fd(){if(value>=0)::close(value);} };
void read_all(int fd,void *out,size_t n) { auto p=static_cast<char *>(out); while(n) { auto k=::read(fd,p,n); if(k<0&&errno==EINTR)continue; if(k<=0)throw std::runtime_error("Incomplete IPC read"); p+=k;n-=k; } }
}
LocalClient::LocalClient(ClientConfig c):config_(std::move(c)) {
  if(!std::regex_match(config_.channel,std::regex("[A-Za-z0-9_-]{1,80}"))||config_.state_timeout.count()<=0||config_.frame_timeout.count()<=0)throw std::runtime_error("Invalid client configuration");
  const auto path=config_.ipc_root+"/rm_aim_sim_"+config_.channel;
  Fd dir{::open(path.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC|O_NOFOLLOW)};
  struct stat info{};
  if(dir.value<0||::fstat(dir.value,&info)||info.st_uid!=::getuid()||(info.st_mode&0777)!=0700)throw std::runtime_error("Start simulator first: IPC directory must be owned by current UID with mode 0700: "+path);
  Fd lock{::openat(dir.value,"client.lock",O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600)};
  if(lock.value<0||::fstat(lock.value,&info)||!S_ISREG(info.st_mode)||::flock(lock.value,LOCK_EX|LOCK_NB))throw std::runtime_error("Cannot acquire client.lock; another client may own channel");
  directory_=dir.value;dir.value=-1;lock_=lock.value;lock.value=-1;
  token_=std::to_string(::getpid())+"-"+std::to_string(Clock::now().time_since_epoch().count());
}
LocalClient::~LocalClient() { try{send_stop();}catch(...){} if(lock_>=0)::close(lock_);if(directory_>=0)::close(directory_); }
Json LocalClient::read_json(const char *name) const {
  Fd fd{::openat(directory_,name,O_RDONLY|O_CLOEXEC|O_NOFOLLOW)};
  if(fd.value<0) { if(errno==ENOENT)return Json(); throw std::runtime_error("Cannot open IPC JSON"); }
  struct stat s{}; if(::fstat(fd.value,&s)||!S_ISREG(s.st_mode)||s.st_size<1||s.st_size>65536)throw std::runtime_error("Invalid IPC JSON size/type");
  std::string data(s.st_size,'\0'); read_all(fd.value,data.data(),data.size()); return Json::parse(data);
}
void LocalClient::write_json(const char *name,const Json &j) const {
  const auto temporary=std::string(name)+"."+token_+".tmp";
  Fd fd{::openat(directory_,temporary.c_str(),O_CREAT|O_EXCL|O_WRONLY|O_CLOEXEC|O_NOFOLLOW,0600)};
  if(fd.value<0)throw std::runtime_error("Cannot create IPC temporary");
  try { const auto data=j.dump();size_t n=0;while(n<data.size()){auto k=::write(fd.value,data.data()+n,data.size()-n);if(k<0&&errno==EINTR)continue;if(k<=0)throw std::runtime_error("Incomplete IPC write");n+=k;} if(::close(fd.value)){fd.value=-1;throw std::runtime_error("IPC close failed");}fd.value=-1;if(::renameat(directory_,temporary.c_str(),directory_,name))throw std::runtime_error("IPC atomic publish failed"); }
  catch(...) {::unlinkat(directory_,temporary.c_str(),0);throw;}
}
bool LocalClient::fresh() const { return state_&&Clock::now()-received_<=config_.state_timeout; }
PollStateResult LocalClient::poll_state() {
  auto j=read_json("state.json"); if(j.is_null())return PollStateResult::missing;
  auto s=parse_state(j);
  const bool changed=!state_||!(s.context==state_->context)||!(s.camera==state_->camera)||s.appearance!=state_->appearance||s.paused!=state_->paused||s.ipc_enabled!=state_->ipc_enabled||s.sim_time<state_->sim_time;
  if(!state_||s.context.publisher_id!=state_->context.publisher_id) { const double wall=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); const auto age=wall-s.wall_unix_sec; if(age<-.1||age>.25)throw std::runtime_error("Stale initial simulator state"); }
  if(state_&&s.context.publisher_id==state_->context.publisher_id) { if(s.publication<state_->publication)throw std::runtime_error("State publication regressed");if(s.publication==state_->publication)return PollStateResult::unchanged; }
  if(changed) {request_.clear();sequence_=0;}
  state_=std::move(s);received_=Clock::now();return changed?PollStateResult::context_changed:PollStateResult::updated;
}
void LocalClient::request_frame() {
  if(!request_.empty()||!fresh()||state_->paused||!state_->ipc_enabled)return;
  auto request=token_+"-"+std::to_string(requests_++);
  write_json("request.json",{{"publisher_id",state_->context.publisher_id},{"request_id",request}});request_=request;requested_=Clock::now();
}
std::optional<ReceivedFrame> LocalClient::try_read_frame() {
  if(request_.empty())return {};
  if(!fresh()){request_.clear();return {};}
  if(Clock::now()-requested_>config_.frame_timeout){request_.clear();throw std::runtime_error("Frame timeout: use graphical mode, not --headless");}
  auto j=read_json("frame.json");if(j.is_null()||j.value("request_id","")!=request_)return {};
  auto f=parse_frame(j);if(!(f.context==state_->context)){request_.clear();return {};}
  auto age=state_->sim_time-f.sim_time;if(age>.2||age<-.1){request_.clear();throw std::runtime_error("Stale frame");}
  if(!(f.camera==state_->camera)){request_.clear();throw std::runtime_error("Frame calibration differs from state");}
  Fd fd{::openat(directory_,f.frame_file.c_str(),O_RDONLY|O_CLOEXEC|O_NOFOLLOW)};struct stat s{};
  if(fd.value<0||::fstat(fd.value,&s)||!S_ISREG(s.st_mode)||s.st_size!=static_cast<off_t>(f.image_bytes))throw std::runtime_error("Raw image size/type mismatch");
  cv::Mat raw(f.camera.height,f.camera.width,f.encoding=="rgb8"?CV_8UC3:CV_8UC4);read_all(fd.value,raw.data,f.image_bytes);
  ReceivedFrame out;cv::cvtColor(raw,out.bgr,f.encoding=="rgb8"?cv::COLOR_RGB2BGR:cv::COLOR_RGBA2BGR);out.metadata=std::move(f);out.received_at=Clock::now();request_.clear();return out;
}
void LocalClient::send(io::Command c) {
  if(!state_)return;
  sequence_=std::max(sequence_,state_->last_seq+1);
  if(!fresh()||state_->paused||!state_->ipc_enabled)c.control=false;
  c.shoot=c.control&&c.shoot;write_json("command.json",make_command_json(*state_,sequence_,c));++sequence_;
}
void LocalClient::send_stop(){send({false,false,0,0});}
}
