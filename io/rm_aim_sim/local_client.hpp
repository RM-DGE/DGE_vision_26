#pragma once
#include "protocol.hpp"
#include <opencv2/core.hpp>
namespace io::rm_aim_sim {
struct ClientConfig { std::string channel="default",ipc_root="/dev/shm"; std::chrono::milliseconds state_timeout{200},frame_timeout{5000}; };
enum class PollStateResult { missing,unchanged,updated,context_changed };
struct ReceivedFrame { cv::Mat bgr; FrameMetadata metadata; Clock::time_point received_at; };
class LocalClient {
 public:
  explicit LocalClient(ClientConfig);
  ~LocalClient();
  LocalClient(const LocalClient &)=delete; LocalClient &operator=(const LocalClient &)=delete;
  PollStateResult poll_state(); bool fresh() const;
  const std::optional<SimState> &state() const { return state_; }
  void request_frame(); std::optional<ReceivedFrame> try_read_frame();
  void cancel_frame() { request_.clear(); }
  void send(io::Command); void send_stop();
 private:
  ClientConfig config_; int directory_=-1,lock_=-1; std::string token_,request_; uint64_t requests_=0; int64_t sequence_=0;
  std::optional<SimState> state_; Clock::time_point received_{},requested_{};
  Json read_json(const char *) const; void write_json(const char *,const Json &) const;
};
}
