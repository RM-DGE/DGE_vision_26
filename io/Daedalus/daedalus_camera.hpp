#ifndef IO__DAEDALUS_CAMERA_HPP
#define IO__DAEDALUS_CAMERA_HPP

#include "io/camera.hpp"

namespace io
{

// 从 Daedalus 仿真器共享内存读取图像的 CameraBase 实现。
// 每次 read() 消费一帧 RGB 并转成 BGR（OpenCV 惯例），同时把该帧对应的
// 云台位姿四元数（经坐标系变换）压入共享 IMU 队列，供 DaedalusCBoard::imu_at 对齐。
class DaedalusCamera : public CameraBase
{
public:
  explicit DaedalusCamera(const std::string & config_path);

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

  // 敌方真值（read() 时随帧刷新）；真机/未连接仿真器时返回空
  std::vector<SimTargetTruth> ground_truth() override;
};

}  // namespace io

#endif  // IO__DAEDALUS_CAMERA_HPP
