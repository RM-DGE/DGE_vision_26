#include "io/Daedalus/daedalus_camera.hpp"

#include "io/Daedalus/shared.hpp"
#include "tools/logger.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

namespace io
{

DaedalusCamera::DaedalusCamera(const std::string & config_path)
{
  auto & shared = DaedalusShared::instance(config_path);
  const auto & ci = shared.client->camera_info();
  tools::logger()->info(
    "[DaedalusCamera] {}x{} fx={:.2f} fy={:.2f} cx={:.2f} cy={:.2f}", ci.width, ci.height, ci.fx,
    ci.fy, ci.cx, ci.cy);
}

void DaedalusCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{ read_for(img, timestamp, std::chrono::seconds(5)); }

bool DaedalusCamera::read_for(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp,
                             std::chrono::milliseconds timeout)
{
  auto & shared = DaedalusShared::instance("");
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (true) {
    // 消费全部位姿通道：仿真器的图像发布握手中，gimbal/odom/muzzle/camera 四个
    // 位姿通道的 FLAG_NEW 都必须被清掉才会发布下一帧图像（见 talos-ipc
    // publisher.rs 的 synchronized_frame_consumed）。不消费会导致死锁。
    for (int i = 0; i <= 4; ++i) {
      auto index = static_cast<ipc::PoseIndex>(i);
      if (auto pose = shared.client->recv_pose(index)) {
        if (index == ipc::POSE_GIMBAL) {
          shared.stage_gimbal_pose(
            Eigen::Quaterniond(pose->qw, pose->qx, pose->qy, pose->qz), pose->frame_seq);
        }
      }
    }

    if (auto frame = shared.client->recv_image()) {
      // 仿真器发布 RGB8，转 BGR 供 OpenCV/识别器使用
      cv::cvtColor(frame->image, img, cv::COLOR_RGB2BGR);
      timestamp = std::chrono::steady_clock::now();

      // The producer may publish between the pose poll and image poll. Fetch
      // that pose once more, but never replace an already matched pose with a
      // subsequent frame after releasing the image handshake.
      if (!shared.pending_gimbal_frame || *shared.pending_gimbal_frame < frame->seq) {
        if (auto pose = shared.client->recv_pose(ipc::POSE_GIMBAL))
          shared.stage_gimbal_pose(
            Eigen::Quaterniond(pose->qw, pose->qx, pose->qy, pose->qz), pose->frame_seq);
      }
      shared.publish_frame_imu(frame->seq, timestamp);
      last_metadata_.frame_id = frame->seq;
      // Simulator UNIX timestamps have no declared tick mapping here. Leave the
      // device field empty rather than mislabeling a different clock as ticks.
      // 随帧刷新敌方真值，供 EKF 对比（见 ground_truth()）
      shared.last_ground_truth = shared.client->ground_truth();
      return true;
    }

    if (!shared.client->is_producer_alive()) {
      tools::logger()->warn("[DaedalusCamera] 仿真器心跳丢失，等待恢复...");
    }
    if (std::chrono::steady_clock::now() > deadline) {
      // 不抛异常：置空图像让上层跳过本帧（standard.cpp 会 continue）
      img.release();
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

std::vector<SimTargetTruth> DaedalusCamera::ground_truth()
{
  auto & shared = DaedalusShared::instance("");
  std::vector<SimTargetTruth> out;
  const auto & batch = shared.last_ground_truth;
  for (uint32_t i = 0;
       i < batch.target_count && i < ipc::GROUND_TRUTH_MAX_TARGETS; ++i) {
    const auto & t = batch.targets[i];
    out.push_back(SimTargetTruth{
      {t.position[0], t.position[1], t.position[2]}, t.yaw, t.vyaw, t.is_outpost != 0});
  }
  return out;
}

}  // namespace io
