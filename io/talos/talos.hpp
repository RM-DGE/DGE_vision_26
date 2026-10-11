#ifndef IO__TALOS_HPP
#define IO__TALOS_HPP

#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>

#include <chrono>
#include <cstdint>
#include <memory>

namespace io
{
class Talos
{
public:
  Talos();
  ~Talos();

  Talos(const Talos &) = delete;
  Talos & operator=(const Talos &) = delete;

  bool read_image(cv::Mat & image, std::chrono::steady_clock::time_point & timestamp);
  Eigen::Quaterniond get_gimbal_quaternion(
    std::chrono::steady_clock::time_point timestamp) const;
  void send_gimbal_command(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
    float pitch_vel, float pitch_acc, float distance_m);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}

#endif
