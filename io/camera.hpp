#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <array>
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace io
{
// 仿真器敌方真值（camera_name: daedalus 时有效；真机相机返回空向量）
// 注意：position 是仿真器世界系（机器人出生点为原点），与 DGE 的 odom 系
// （云台为原点）差一个固定平移；yaw 零向也可能不同。vyaw 无坐标歧义，
// 是 EKF 对比的首选指标。
struct SimTargetTruth
{
  std::array<double, 3> position;  // 仿真器世界系 xyz (m)
  double yaw;                      // 当前朝向 (rad)
  double vyaw;                     // 角速度 (rad/s)
  bool is_outpost;
};

class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
  virtual std::vector<SimTargetTruth> ground_truth() { return {}; }
};

class Camera
{
public:
  Camera(const std::string & config_path);
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
  std::vector<SimTargetTruth> ground_truth();

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP