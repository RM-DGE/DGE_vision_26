#ifndef TOOLS__FOXGLOVE_HPP
#define TOOLS__FOXGLOVE_HPP

#include <list>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "io/command.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/target.hpp"

namespace tools
{
// 把图像、识别框、调试曲线、TF 与 3D 场景推给 Foxglove Studio 的工具。
// 连接方式：Foxglove Studio -> Foxglove WebSocket -> ws://<本机IP>:8765。
// 底层是 io/foxglove 下的官方 C++ SDK，SDK 细节不暴露给调用方。
class FoxgloveServer
{
public:
  explicit FoxgloveServer(int port = 8765, std::string name = "DGE_vision_26");
  ~FoxgloveServer();

  bool has_clients() const;  // 无客户端时可跳过图像编码

  void publish_image(const cv::Mat & bgr, int quality = 70);            // /camera/image + /detector/result_img/compressed
  void publish_annotations(const std::list<auto_aim::Armor> & armors);  // /camera/annotations
  // 数字分类图案拼接图（/detector/number_img，原布局第二个 ImageViewPanel）
  void publish_number_image(const std::list<auto_aim::Armor> & armors);

  // rm_auto_aim-compatible topics used by the supplied Foxglove layouts.
  void publish_rmvision(
    const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets,
    const Eigen::Vector4d & aim_xyza);

  // 3D 面板 Marker 话题（CDR 编码 visualization_msgs/msg/MarkerArray）
  void publish_detector_markers(const std::list<auto_aim::Armor> & armors);   // /detector/marker
  void publish_tracker_markers(const std::list<auto_aim::Target> & targets);  // /tracker/marker
  void publish_debug(                                                   // /dge/debug
    double fps, const io::Command & command, const std::list<auto_aim::Target> & targets,
    const std::list<auto_aim::Armor> & armors, bool aim_valid, const Eigen::Vector4d & aim_xyza,
    const Eigen::Vector3d & gimbal_ypr = Eigen::Vector3d::Zero());

  // 一次性：读取配置里的相机内参/外参，并发布 /camera_info
  void configure_camera(const std::string & config_path);

  // 每个图像帧：发布 /tf（odom -> gimbal_link -> camera_optical_frame）
  void publish_tf(const Eigen::Quaterniond & gimbal_world_q);

  // 每个图像帧：发布 /dge/scene（检测装甲板 / 跟踪目标 / 自车云台 / 瞄准点的 3D 实体）
  void publish_scene(
    const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets,
    bool aim_valid, const Eigen::Vector4d & aim_xyza,
    const Eigen::Quaterniond & gimbal_world_q = Eigen::Quaterniond::Identity());

  // 每个图像帧：发布 /dge/prediction（跟踪目标未来一段时间的预测轨迹）
  void publish_prediction(
    const std::list<auto_aim::Target> & targets, double horizon_s = 0.6, double step_s = 0.02);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tools

#endif  // TOOLS__FOXGLOVE_HPP
