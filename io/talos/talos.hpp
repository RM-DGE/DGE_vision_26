#ifndef IO__TALOS_HPP
#define IO__TALOS_HPP

#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

namespace io
{

struct Pose
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // ROS/FLU, m
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

// Field names match PoseIndex order (Gimbal=0, Odom=1, Muzzle=2, Camera=3).
struct Poses
{
  Pose gimbal;
  Pose odom;
  Pose muzzle;
  Pose camera;
};

struct GroundTruthTarget
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // ROS/FLU, m
  double yaw = 0.0;
  double vyaw = 0.0;
  int team = 0;  // 0=Red, 1=Blue
  int armor_label = 0;
};

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

  // Read-only snapshots. They return owned copies and never expose pointers or
  // references into the shared memory. Ground truth is not part of the image/pose
  // handshake, so read_ground_truth() uses a best-effort consistency check.
  bool read_poses(Poses & out) const;
  std::vector<GroundTruthTarget> read_ground_truth() const;

  // Unix nanosecond timestamp of the most recent image (0 if no frame yet).
  std::uint64_t last_image_timestamp_ns() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}

#endif
