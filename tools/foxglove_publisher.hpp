#ifndef TOOLS__FOXGLOVE_PUBLISHER_HPP
#define TOOLS__FOXGLOVE_PUBLISHER_HPP

#ifdef ENABLE_FOXGLOVE

#include <list>
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/target.hpp"

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
  FoxglovePublisher(int port, std::string name, std::string config_path);
  ~FoxglovePublisher();

  FoxglovePublisher(const FoxglovePublisher &) = delete;
  FoxglovePublisher & operator=(const FoxglovePublisher &) = delete;

  bool has_clients() const;

  void observe(const io::Talos & talos);
  double total_hit_rate() const;
  double armor_hit_rate(const std::string & name) const;

  void publish(
    const cv::Mat & bgr, const io::Command & command, double fps,
    const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets,
    bool aim_valid, const Eigen::Vector4d & aim_xyza, const Eigen::Quaterniond & gimbal_world_q,
    const Eigen::Vector3d & gimbal_ypr);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  void configure_camera(const std::string & config_path);
  void publish_tf(const Eigen::Quaterniond & gimbal_world_q);
  void publish_scene(
    const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets,
    bool aim_valid, const Eigen::Vector4d & aim_xyza, const Eigen::Quaterniond & gimbal_world_q);
  void publish_prediction(
    const std::list<auto_aim::Target> & targets, double horizon_s, double step_s);
  void publish_image(const cv::Mat & bgr, int quality);
  void publish_number_image(const std::list<auto_aim::Armor> & armors);
  void publish_annotations(const std::list<auto_aim::Armor> & armors);
  void publish_rmvision(
    const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets,
    const Eigen::Vector4d & aim_xyza);
  void publish_detector_markers(const std::list<auto_aim::Armor> & armors);
  void publish_tracker_markers(const std::list<auto_aim::Target> & targets);
  void publish_debug(
    double fps, const io::Command & command, const std::list<auto_aim::Target> & targets,
    const std::list<auto_aim::Armor> & armors, bool aim_valid, const Eigen::Vector4d & aim_xyza,
    const Eigen::Vector3d & gimbal_ypr);
};

}  // namespace tools

#endif  // ENABLE_FOXGLOVE
#endif  // TOOLS__FOXGLOVE_PUBLISHER_HPP
