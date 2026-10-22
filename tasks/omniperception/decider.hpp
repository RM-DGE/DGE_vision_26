/**
 * @file decider.hpp
 * @brief 多相机目标决策：把单路相机的检测结果换算成云台 yaw/pitch 指令
 *
 * 模块定位：
 *   Perceptron（多相机并行检测）或单路 YOLO 得到装甲板列表后，由 Decider：
 *   1. 按敌方颜色、装甲板编号、无敌状态过滤目标；
 *   2. 根据相机安装位置（左/右/后）和画面归一化坐标计算角度偏差；
 *   3. 生成 io::Command，交给 CBoard / Gimbal 执行。
 *
 * 角度约定：
 *   - 前向工业相机为 0°；左/右 USB 相机安装角由 YAML 配置，
 *     默认左 +120°、右 -120°（三相机间隔 120°）；
 *   - delta_angle 为“目标相对该相机光轴”的角度（度）；
 *   - 最终云台 yaw = 当前云台 yaw + delta_angle[0]，pitch = delta_angle[1]；
 *   - 类内部计算使用弧度，日志中乘 57.3 显示为度。
 *
 * 注意：
 *   当前分支没有可执行程序链接 omniperception，本模块属于预留逻辑；
 *   接入时需要同时提供 config 中的 mode、fov_h/fov_v、new_fov_h/new_fov_v。
 */

#ifndef OMNIPERCEPTION__DECIDER_HPP
#define OMNIPERCEPTION__DECIDER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <iostream>
#include <list>
#include <unordered_map>

#include "detection.hpp"
#include "io/camera.hpp"
#include "io/command.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace omniperception
{
// 多相机决策器：维护一路“当前轮询相机”，每次 decide 只处理一路画面。
class Decider
{
public:
  explicit Decider(const std::string & config_path);

  /**
   * @brief 旧三相机轮询决策（左 USB / 右 USB / 主相机）
   *        当前前向工业相机 + 左右 USB 布局请使用 decide(detection_queue)。
   * @param yolo          用于检测当前帧的 YOLO 实例
   * @param gimbal_pos    当前云台 yaw/pitch/roll（弧度）
   * @param usbcam1       左相机
   * @param usbcam2       右相机
   * @param back_cammera  主相机（旧接口参数名保留）
   * @return control=false 表示本帧无目标或未命中；否则 yaw/pitch 为绝对角度
   */
  io::Command decide(
    auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::USBCamera & usbcam1,
    io::USBCamera & usbcam2, io::Camera & back_cammera);

  /// 只用主相机的简化版本（旧接口）
  io::Command decide(
    auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::Camera & back_cammera);

  /// 直接使用 Perceptron 线程已经算好的 DetectionResult（角度已是弧度）
  io::Command decide(const std::vector<DetectionResult> & detection_queue);

  /**
   * @brief 由检测框中心归一化坐标计算相对相机光轴的角度偏差
   * @param armors  当前帧装甲板，使用 armors.front()
   * @param camera  相机名："left" / "right" / 其他（旧接口按后置相机处理）
   * @return {yaw 偏差, pitch 偏差}，单位：度
   */
  Eigen::Vector2d delta_angle(
    const std::list<auto_aim::Armor> & armors, const std::string & camera);

  /// 过滤非敌方、5 号、前哨站以及当前无敌的目标；返回 true 表示过滤后为空
  bool armor_filter(std::list<auto_aim::Armor> & armors);

  /// 根据 mode_ 给装甲板写入优先级
  void set_priority(std::list<auto_aim::Armor> & armors);

  /// 对多个检测结果依次过滤、写优先级并按优先级排序
  void sort(std::vector<DetectionResult> & detection_queue);

  /// 取跟踪目标对应装甲板在云台系下的位置，末尾附带装甲板编号+1
  Eigen::Vector4d get_target_info(
    const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets);

  /// 把裁判系统/导航发来的无敌敌方 id 转成 ArmorName 列表
  void get_invincible_armor(const std::vector<int8_t> & invincible_enemy_ids);

  /// 只保留导航指定要打的目标编号
  void get_auto_aim_target(
    std::list<auto_aim::Armor> & armors, const std::vector<int8_t> & auto_aim_target);

private:
  int img_width_;    // 图像宽（像素）
  int img_height_;   // 图像高（像素）
  double fov_h_, new_fov_h_;  // 原始/裁剪或缩放后的水平视场角（度）
  double fov_v_, new_fov_v_;  // 原始/裁剪或缩放后的垂直视场角（度）
  double left_camera_yaw_deg_;   // 左 USB 相机安装角（度，向前为 0，向左为正）
  double right_camera_yaw_deg_;  // 右 USB 相机安装角（度，向前为 0，向右为负）
  int mode_;   // 优先级模式：1/2 对应 mode1/mode2
  int count_;  // 轮询计数：0=左 USB，1=右 USB，2=主相机（旧三相机接口）

  auto_aim::Color enemy_color_;       // 需要打击的敌方颜色
  auto_aim::YOLO detector_;           // 预留的单路检测器
  std::vector<auto_aim::ArmorName> invincible_armor_;  // 当前无敌的敌方编号

  // 定义ArmorName到ArmorPriority的映射类型
  using PriorityMap = std::unordered_map<auto_aim::ArmorName, auto_aim::ArmorPriority>;

  // 两种目标优先级配置：数值越小越优先
  const PriorityMap mode1 = {
    {auto_aim::ArmorName::one, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::two, auto_aim::ArmorPriority::forth},
    {auto_aim::ArmorName::three, auto_aim::ArmorPriority::first},
    {auto_aim::ArmorName::four, auto_aim::ArmorPriority::first},
    {auto_aim::ArmorName::five, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::sentry, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::outpost, auto_aim::ArmorPriority::fifth},
    {auto_aim::ArmorName::base, auto_aim::ArmorPriority::fifth},
    {auto_aim::ArmorName::not_armor, auto_aim::ArmorPriority::fifth}};

  const PriorityMap mode2 = {
    {auto_aim::ArmorName::two, auto_aim::ArmorPriority::first},
    {auto_aim::ArmorName::one, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::three, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::four, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::five, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::sentry, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::outpost, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::base, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::not_armor, auto_aim::ArmorPriority::third}};
};

// 优先级模式，对应 config 中的 mode
enum PriorityMode
{
  MODE_ONE = 1,
  MODE_TWO
};

}  // namespace omniperception

#endif