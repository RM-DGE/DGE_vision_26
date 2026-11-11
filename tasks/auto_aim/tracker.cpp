#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <tuple>
#include <cmath>
#include <stdexcept>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/ekf_record.hpp"

namespace auto_aim
{

namespace
{

/// DGE now only tracks 4-armor robots with the rmcs pixel EKF.
/// 2/3-armor targets (balance / outpost / base) are no longer supported.
auto is_supported_target(const Armor & armor) -> bool
{
  if (armor.name < ArmorName::one || armor.name >= ArmorName::not_armor ||
      (armor.type != ArmorType::small && armor.type != ArmorType::big) ||
      armor.color < Color::red || armor.color > Color::purple) return false;
  if (armor.name == ArmorName::outpost || armor.name == ArmorName::base) return false;

  const auto is_balance = armor.type == ArmorType::big &&
                          (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                           armor.name == ArmorName::five);
  return !is_balance;
}

bool valid_corners(const Armor & armor)
{
  if (armor.points.size()!=4) return false;
  for (const auto & p:armor.points) if(!std::isfinite(p.x) || !std::isfinite(p.y)) return false;
  double sign=0;
  for(int i=0;i<4;++i) {
    const auto d=armor.points[(i+1)%4]-armor.points[i];
    const auto e=armor.points[(i+2)%4]-armor.points[(i+1)%4];
    const double cross=double(d.x)*e.y-double(d.y)*e.x;
    if(cv::norm(d)<1 || std::abs(cross)<1e-3) return false;
    if(i==0) sign=cross; else if(cross*sign<=0) return false;
  }
  return sign>0 && armor.points[0].y<armor.points[3].y && armor.points[1].y<armor.points[2].y;
}

}  // namespace

Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth}
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  max_frame_gap_s_ = yaml["max_frame_gap_s"].as<double>(0.1);
  if (!std::isfinite(max_frame_gap_s_) || max_frame_gap_s_ <= 0)
    throw std::invalid_argument("max_frame_gap_s must be finite and positive");
  ekf_config_ = rmcs_ekf::load_config(yaml["ekf"]);
  recovery_settings_=rmcs_ekf::load_recovery_settings(yaml["recovery"]);
  recorder_=std::make_shared<tools::EkfRecorder>(config_path);
}

bool Tracker::accept_timestamp(std::chrono::steady_clock::time_point t)
{
  if(timestamp_initialized_) {
    const double dt=tools::delta_time(t,last_timestamp_);
    if(!std::isfinite(dt) || dt<=0) return false;
    if(dt>max_frame_gap_s_) {
      target_.interrupt();state_="lost";pre_state_="lost";
      detect_count_=temp_lost_count_=0;
    }
  }
  timestamp_initialized_=true;last_timestamp_=t;return true;
}

std::string Tracker::state() const { return state_; }

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  tools::EkfFrameLog log(*recorder_,armors,t,solver_.camera_feature(),
    [this]()->const Target* {return state_=="lost"?nullptr:&target_;},[this]{return state_;},use_enemy_color);
  if(!accept_timestamp(t)) return state_=="lost" ? std::list<Target>{} : std::list<Target>{target_};
  // 只保留己方颜色且受支持的 4 板目标
  armors.remove_if(
    [&](const auto_aim::Armor & a) { return (use_enemy_color && a.color != enemy_color_) || !is_supported_target(a) || !valid_corners(a); });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug(
      "[Tracker] Target diverged! name={}",
      ARMOR_NAMES.at(static_cast<size_t>(target_.name)));
    state_ = "lost";
    return {};
  }

  if (state_ == "lost") return {};

  std::list<Target> targets = {target_};
  return targets;
}

std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty() && !detection_queue.front().armors.empty()) {
    temp_target = detection_queue.front();
  }

  tools::EkfFrameLog log(*recorder_,armors,t,solver_.camera_feature(),
    [this]()->const Target* {return state_=="lost"?nullptr:&target_;},[this]{return state_;},use_enemy_color);
  log.omni(detection_queue);
  if(!accept_timestamp(t)) return {switch_target,state_=="lost" ? std::list<Target>{} : std::list<Target>{target_}};

  // 全向感知结果同样只保留 4 板目标，避免切到不支持的前哨站/平衡/基地
  armors.remove_if([&](const auto_aim::Armor & a) {
    return !is_supported_target(a) || !valid_corners(a) || (use_enemy_color && a.color!=enemy_color_);
  });
  temp_target.armors.remove_if(
    [&](const auto_aim::Armor & a) { return !is_supported_target(a) || !valid_corners(a) || (use_enemy_color && a.color!=enemy_color_); });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (
    state_ == "tracking" && !temp_target.armors.empty() &&
    temp_target.armors.front().priority < target_.priority && target_.converge()) {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
      temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") {
    found = !armors.empty() && armors.front().priority == omni_target_priority_ && set_target(armors,t);
  }

  else {
    found = update_target(armors, t);
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug(
      "[Tracker] Target diverged! name={}",
      ARMOR_NAMES.at(static_cast<size_t>(target_.name)));
    state_ = "lost";
    return {switch_target, {}};  // 返回switch_target和空的targets
  }

  if (state_ == "lost") return {switch_target, {}};  // 返回switch_target和空的targets

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

void Tracker::state_machine(bool found)
{
  // RecoveryManager owns estimator retention and loss; business switching remains separate.
  if(state_!="lost" && state_!="switching") {
    if(target_.recovery_status().health==rmcs_ekf::Health::LOST) {state_="lost";return;}
    if(found) {
      ++detect_count_;
      state_=(target_.reliable() && detect_count_>=min_detect_count_) ? "tracking" : "detecting";
    } else state_="temp_lost";
    return;
  }
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;

  auto & armor = armors.front();
  if (!is_supported_target(armor) || !valid_corners(armor)) return false;

  if (!solver_.solve(armor)) return false;

  Target next(armor, t, ekf_config_, solver_.camera_feature(), recovery_settings_);
  if (!next.checkinit()) {
    tools::logger()->warn(
      "[Tracker] failed to initialize 4-armor EKF for {}", ARMOR_NAMES[armor.name]);
    return false;
  }
  recovery_count_+=target_.recovery_status().recovery_count;
  candidate_failed_count_+=target_.recovery_status().candidate_failed_count;
  if(target_.checkinit()) ++reinit_count_;
  target_=std::move(next); ++track_id_;
  detect_count_=temp_lost_count_=0;

  return true;
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);

  int found_count = 0;
  for (const auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type || armor.color!=target_.color()) continue;
    found_count++;
  }

  if (found_count == 0) {
    // Clear per-frame diagnostics without counting a missed detection as a
    // rejected measurement. Recovery remains controlled by the lost state.
    target_.update({}, solver_.camera_feature());
    return false;
  }

  std::list<Armor> matched_armors;
  for (auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type || armor.color!=target_.color()) continue;

    // v0.32: 4 板像素 EKF 只用角点，不用 PnP 结果。skip_tracking_pnp=true 时跳过
    // 逐板解算（省掉每块板 140 次 yaw 重投影搜索），但 Foxglove 的 detections
    // 方块会丢失世界系位姿，默认关闭。
    if (!ekf_config_.skip_tracking_pnp) (void)solver_.solve(armor);
    else armor.pose_valid=false;
    matched_armors.push_back(armor);
  }

  target_.update(matched_armors, solver_.camera_feature());
  return target_.diagnostics().frame_update_count>0;
}

}  // namespace auto_aim
