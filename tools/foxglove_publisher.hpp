#ifndef TOOLS__FOXGLOVE_PUBLISHER_HPP
#define TOOLS__FOXGLOVE_PUBLISHER_HPP

#ifdef ENABLE_FOXGLOVE

#include <cstdint>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace io
{
class Talos;
struct Command;
}

namespace tools
{

class FoxglovePublisher
{
public:
  FoxglovePublisher(
    std::string host = "127.0.0.1", uint16_t port = 8765, std::string config_path = "");
  ~FoxglovePublisher();

  FoxglovePublisher(const FoxglovePublisher &) = delete;
  FoxglovePublisher & operator=(const FoxglovePublisher &) = delete;

  bool publishFrame(
    const cv::Mat & img, const io::Command & command, double target_distance, double fps,
    const io::Talos & talos, bool aim_valid, const Eigen::Vector4d & aim_xyza);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tools

#endif  // ENABLE_FOXGLOVE
#endif  // TOOLS__FOXGLOVE_PUBLISHER_HPP
