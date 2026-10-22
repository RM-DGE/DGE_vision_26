/**
 * @file decider.cpp
 * @brief 多相机决策器的实现：轮询相机、过滤目标、换算云台角度
 *
 * 核心思路：
 *   三路相机（左、右、后）固定安装在机器人上，各自有固定朝向。
 *   每次 decide() 只读取其中一路，检测到装甲板后，用
 *   “画面归一化坐标 -> 角度”的线性映射得到目标相对该相机的偏角，
 *   再加上相机安装偏角和当前云台 yaw，得到云台需要转到的绝对角度。
 *
 * 注意：
 *   - 前向工业相机为 0°，左/右 USB 相机默认 +120° / -120°，
 *     可通过 YAML 的 left_camera_yaw_deg / right_camera_yaw_deg 调整；
 *   - 本模块只输出瞄准指令（shoot=false），不负责开火和 EKF 跟踪。
 */

#include "decider.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace omniperception
{
Decider::Decider(const std::string & config_path) : detector_(config_path), count_(0)
{
  auto yaml = YAML::LoadFile(config_path);
  img_width_ = yaml["image_width"].as<double>();    // 原始图像尺寸
  img_height_ = yaml["image_height"].as<double>();
  fov_h_ = yaml["fov_h"].as<double>();              // 相机原始水平视场角
  fov_v_ = yaml["fov_v"].as<double>();              // 相机原始垂直视场角
  new_fov_h_ = yaml["new_fov_h"] ? yaml["new_fov_h"].as<double>()
                                 : fov_h_;           // 裁剪/缩放后实际使用的视场角
  new_fov_v_ = yaml["new_fov_v"] ? yaml["new_fov_v"].as<double>()
                                 : fov_v_;
  tools::logger()->info(
    "[Decider] FOV native h/v={:.1f}/{:.1f}, effective h/v={:.1f}/{:.1f}", fov_h_, fov_v_,
    new_fov_h_, new_fov_v_);
  left_camera_yaw_deg_ =
    yaml["left_camera_yaw_deg"] ? yaml["left_camera_yaw_deg"].as<double>() : 120.0;
  right_camera_yaw_deg_ =
    yaml["right_camera_yaw_deg"] ? yaml["right_camera_yaw_deg"].as<double>() : -120.0;
  enemy_color_ =                                    // 只打配置指定的敌方颜色
    (yaml["enemy_color"].as<std::string>() == "red") ? auto_aim::Color::red : auto_aim::Color::blue;
  mode_ = yaml["mode"].as<double>();                // 1/2 选择不同优先级表
}

// 旧三相机接口：只用于“左 USB / 右 USB / 主相机”轮询；
// 当前 sentry_DGE 使用 Perceptron 队列 + decide(queue)，不走这条路径。
io::Command Decider::decide(
  auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::USBCamera & usbcam1,
  io::USBCamera & usbcam2, io::Camera & back_camera)
{
  Eigen::Vector2d delta_angle;
  // count_ 轮询三路：0 -> 左 USB，1 -> 右 USB，2 -> 主相机（旧接口）
  io::USBCamera * cams[] = {&usbcam1, &usbcam2};

  cv::Mat usb_img;
  std::chrono::steady_clock::time_point timestamp;
  if (count_ < 0 || count_ > 2) {
    throw std::runtime_error("count_ out of valid range [0,2]");
  }
  // 每次只取一路图像，三路轮流处理，避免同时推理拖慢帧率
  if (count_ == 2) {
    back_camera.read(usb_img, timestamp);
  } else {
    cams[count_]->read(usb_img, timestamp);
  }
  auto armors = yolo.detect(usb_img);
  auto empty = armor_filter(armors);  // true 表示过滤后没有可打目标

  if (!empty) {
    // 根据当前相机名选择对应的安装偏角映射
    if (count_ == 2) {
      delta_angle = this->delta_angle(armors, "back");
    } else {
      delta_angle = this->delta_angle(armors, cams[count_]->device_name);
    }

    tools::logger()->debug(
      "[{} camera] delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}",
      (count_ == 2 ? "back" : cams[count_]->device_name), delta_angle[0], delta_angle[1],
      armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

    count_ = (count_ + 1) % 3;  // 无论是否命中都切到下一路

    // yaw 为绝对角度：当前 yaw + 目标相对相机的偏角；pitch 直接使用偏角
    return io::Command{
      true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
      tools::limit_rad(delta_angle[1] / 57.3)};
  }

  count_ = (count_ + 1) % 3;
  // 如果没有找到目标，返回默认命令
  return io::Command{false, false, 0, 0};
}

// 只用后置相机的版本：用于只装一个后置相机的简化配置
io::Command Decider::decide(
  auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::Camera & back_cammera)
{
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  back_cammera.read(img, timestamp);
  auto armors = yolo.detect(img);
  auto empty = armor_filter(armors);

  if (!empty) {
    auto delta_angle = this->delta_angle(armors, "back");
    tools::logger()->debug(
      "[back camera] delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}",
      delta_angle[0], delta_angle[1], armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

    // 与三路版本相同：yaw 为云台绝对角，pitch 为偏角
    return io::Command{
      true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
      tools::limit_rad(delta_angle[1] / 57.3)};
  }

  return io::Command{false, false, 0, 0};
}

// 使用 Perceptron 预先算好的结果：delta_yaw/delta_pitch 是相对相机光轴的角度（弧度）。
// 注意：本函数只返回相对角，调用方需要再叠加当前云台 yaw。
// 旧版 sentry_multithread.cpp 的用法是：
//   command = decider.decide(detection_queue);
//   command.yaw = tools::limit_rad(command.yaw + gimbal_pos[0]);
io::Command Decider::decide(const std::vector<DetectionResult> & detection_queue)
{
  if (detection_queue.empty()) {
    return io::Command{false, false, 0, 0};
  }

  DetectionResult dr = detection_queue.front();
  if (dr.armors.empty()) return io::Command{false, false, 0, 0};
  tools::logger()->info(
    "omniperceptron find {},delta yaw is {:.4f}", auto_aim::ARMOR_NAMES[dr.armors.front().name],
    dr.delta_yaw * 57.3);

  return io::Command{true, false, dr.delta_yaw, dr.delta_pitch};
};

// 把检测框中心的归一化坐标（0~1）映射成相对相机光轴的角度。
// 前向工业相机为 0°；左/右 USB 默认 +120°/-120°，由 YAML 配置。
Eigen::Vector2d Decider::delta_angle(
  const std::list<auto_aim::Armor> & armors, const std::string & camera)
{
  Eigen::Vector2d delta_angle;
  if (camera == "left") {
    // 左相机：+120° 安装角；x 越大目标越靠右，因此 yaw 偏角减小
    delta_angle[0] = left_camera_yaw_deg_ + (new_fov_h_ / 2) -
                     armors.front().center_norm.x * new_fov_h_;
    // center_norm.y 向下为正，pitch 偏角相应向下
    delta_angle[1] = armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2;
    return delta_angle;
  }

  else if (camera == "right") {
    // 右相机：-120° 安装角，映射方式与左相机对称
    delta_angle[0] = right_camera_yaw_deg_ + (new_fov_h_ / 2) -
                     armors.front().center_norm.x * new_fov_h_;
    delta_angle[1] = armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2;
    return delta_angle;
  }

  else {
    // 旧三相机接口的“后置”分支；当前前向工业相机 + 左右 USB 布局不使用。
    // 保留仅为兼容旧调用，安装角 170° 不再代表现车布局。
    delta_angle[0] = 170 + (54.2 / 2) - armors.front().center_norm.x * 54.2;
    delta_angle[1] = armors.front().center_norm.y * 44.5 - 44.5 / 2;
    return delta_angle;
  }
}

// 目标过滤：返回 true 表示过滤后没有可打目标。
// 过滤条件依次为：非敌方颜色、5 号装甲板、前哨站、当前无敌目标。
bool Decider::armor_filter(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return true;
  // 过滤非敌方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 25赛季没有5号装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::five; });
  // 不打工程
  // armors.remove_if([&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::two; });
  // 不打前哨站
  armors.remove_if(
    [&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::outpost; });

  // 过滤掉刚复活无敌的装甲板
  armors.remove_if([&](const auto_aim::Armor & a) {
    return std::find(invincible_armor_.begin(), invincible_armor_.end(), a.name) !=
           invincible_armor_.end();
  });

  return armors.empty();
}

// 按配置的 mode 给每个装甲板写入优先级，供后续排序使用
void Decider::set_priority(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return;

  const PriorityMap & priority_map = (mode_ == MODE_ONE) ? mode1 : mode2;

  if (!armors.empty()) {
    for (auto & armor : armors) {
      armor.priority = priority_map.at(armor.name);
    }
  }
}

// 对多路相机的检测结果统一做过滤、优先级排序：
// 先按目标优先级对每帧内的装甲板排序，再按帧首装甲板的优先级对帧排序。
void Decider::sort(std::vector<DetectionResult> & detection_queue)
{
  if (detection_queue.empty()) return;

  // 对每个 DetectionResult 调用 armor_filter 和 set_priority
  for (auto & dr : detection_queue) {
    armor_filter(dr.armors);
    set_priority(dr.armors);

    // 对每个 DetectionResult 中的 armors 进行排序
    dr.armors.sort(
      [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });
  }

  // 过滤后可能整帧没有可打目标，先移除空结果，
  // 否则下面的排序比较器会访问空 list 的 front()，导致段错误。
  detection_queue.erase(
    std::remove_if(
      detection_queue.begin(), detection_queue.end(),
      [](const DetectionResult & dr) { return dr.armors.empty(); }),
    detection_queue.end());

  if (detection_queue.empty()) return;

  // 根据优先级对 DetectionResult 进行排序
  std::sort(
    detection_queue.begin(), detection_queue.end(),
    [](const DetectionResult & a, const DetectionResult & b) {
      return a.armors.front().priority < b.armors.front().priority;
    });
}

// 取跟踪目标对应装甲板在云台系下的位置，末尾附带“装甲板编号+1”。
// 编号 +1 是为了和通信协议中的 1-based 编码保持一致。
Eigen::Vector4d Decider::get_target_info(
  const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets)
{
  if (armors.empty() || targets.empty()) return Eigen::Vector4d::Zero();

  auto target = targets.front();

  for (const auto & armor : armors) {
    if (armor.name == target.name) {
      return Eigen::Vector4d{
        armor.xyz_in_gimbal[0], armor.xyz_in_gimbal[1], 1,
        static_cast<double>(armor.name) + 1};  //避免歧义+1(详见通信协议)
    }
  }

  return Eigen::Vector4d::Zero();
}

// 裁判系统/导航发来的 id 是 1-based，转成 ArmorName 时需要减 1
void Decider::get_invincible_armor(const std::vector<int8_t> & invincible_enemy_ids)
{
  invincible_armor_.clear();

  if (invincible_enemy_ids.empty()) return;

  for (const auto & id : invincible_enemy_ids) {
    tools::logger()->info("invincible armor id: {}", id);
    invincible_armor_.push_back(auto_aim::ArmorName(id - 1));
  }
}

// 只保留导航指定要打的目标编号；输入同样使用 1-based 编码
void Decider::get_auto_aim_target(
  std::list<auto_aim::Armor> & armors, const std::vector<int8_t> & auto_aim_target)
{
  if (auto_aim_target.empty()) return;

  std::vector<auto_aim::ArmorName> auto_aim_targets;

  for (const auto & target : auto_aim_target) {
    if (target <= 0 || static_cast<size_t>(target) > auto_aim::ARMOR_NAMES.size()) {
      tools::logger()->warn("Received invalid auto_aim target value: {}", int(target));
      continue;
    }
    auto_aim_targets.push_back(static_cast<auto_aim::ArmorName>(target - 1));
    tools::logger()->info("nav send auto_aim target is {}", auto_aim::ARMOR_NAMES[target - 1]);
  }

  if (auto_aim_targets.empty()) return;

  armors.remove_if([&](const auto_aim::Armor & a) {
    return std::find(auto_aim_targets.begin(), auto_aim_targets.end(), a.name) ==
           auto_aim_targets.end();
  });
}

}  // namespace omniperception