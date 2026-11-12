#include "rmcs_ekf.hpp"
#include "projection.hpp"
#include "association.hpp"
#include <stdexcept>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

#include <opencv2/core/eigen.hpp>

#include "tools/logger.hpp"

namespace rmcs_ekf
{

namespace
{

constexpr double kEpsilon = 1e-9;

template <typename T>
auto read_or(const YAML::Node & node, const char * key, T fallback) -> T
{
  if (node && node[key]) return node[key].as<T>();
  return fallback;
}

auto armor_shape_points(bool large) -> std::array<cv::Point3f, 4>
{
  const auto width = large ? kLargeArmorWidth : kSmallArmorWidth;
  const auto half_w = static_cast<float>(0.5 * width);
  const auto half_h = static_cast<float>(0.5 * kLightBarHeight);
  return { {
    cv::Point3f{ -half_w, -half_h, 0.0F }, // tl
    cv::Point3f{ -half_w, +half_h, 0.0F }, // bl
    cv::Point3f{ +half_w, +half_h, 0.0F }, // br
    cv::Point3f{ +half_w, -half_h, 0.0F }, // tr
  } };
}

struct ArmorPose
{
  Eigen::Vector3d translation{ Eigen::Vector3d::Zero() };
  Eigen::Quaterniond orientation{ Eigen::Quaterniond::Identity() };
};

auto compute_armor_poses(const RobotSolution::Input & input) -> std::array<ArmorPose, 4>
{
  const auto q = input.toward;
  const auto c = input.center;

  const auto px_dir = Eigen::Vector3d{ q * Eigen::Vector3d::UnitX() };
  const auto py_dir = Eigen::Vector3d{ q * Eigen::Vector3d::UnitY() };

  const auto pitch = kPredictedOtherArmorPitch;
  const auto make_orientation = [&q, pitch](double delta) {
    return Eigen::Quaterniond{ Eigen::AngleAxisd{ delta, Eigen::Vector3d::UnitZ() } * q
      * Eigen::AngleAxisd{ pitch, Eigen::Vector3d::UnitY() } };
  };

  const auto fr = input.radius_forward;
  const auto lr = input.radius_lateral;
  const auto lh = input.height_lateral;
  const auto vz = Eigen::Vector3d::UnitZ();

  return { {
    ArmorPose{ c + fr * px_dir, make_orientation(kPi) },
    ArmorPose{ c + lr * py_dir + vz * lh, make_orientation(-0.5 * kPi) },
    ArmorPose{ c - fr * px_dir, make_orientation(0.0) },
    ArmorPose{ c - lr * py_dir + vz * lh, make_orientation(0.5 * kPi) },
  } };
}

} // namespace

// ---------------------------------------------------------------------------
// CameraFeature
// ---------------------------------------------------------------------------

auto CameraFeature::intrinsic() const -> cv::Mat
{
  cv::Mat matrix(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      matrix.at<double>(row, col) = camera_matrix[row][col];
    }
  }
  return matrix;
}

auto CameraFeature::distortion() const -> cv::Mat
{
  cv::Mat matrix(1, 5, CV_64F);
  for (int col = 0; col < 5; ++col) {
    matrix.at<double>(0, col) = distort_coeff[col];
  }
  return matrix;
}

// ---------------------------------------------------------------------------
// RobotSolution
// ---------------------------------------------------------------------------

auto RobotSolution::solve_armors() const -> std::array<Armor3d, 4>
{
  const auto poses = compute_armor_poses(input);

  return { {
    Armor3d{ 0, poses[0].translation, poses[0].orientation },
    Armor3d{ 1, poses[1].translation, poses[1].orientation },
    Armor3d{ 2, poses[2].translation, poses[2].orientation },
    Armor3d{ 3, poses[3].translation, poses[3].orientation },
  } };
}

auto RobotSolution::solve_lightbars() const -> std::array<Lightbar3d, 8>
{
  const auto poses = compute_armor_poses(input);

  const auto half_w = (input.large ? kLargeArmorWidth : kSmallArmorWidth) * 0.5;
  const auto half_h = kLightBarHeight * 0.5;

  std::array<Lightbar3d, 8> result{};
  for (int i = 0; i < 4; ++i) {
    const auto y_axis = Eigen::Vector3d{ poses[i].orientation * Eigen::Vector3d::UnitY() };
    const auto z_axis = Eigen::Vector3d{ poses[i].orientation * Eigen::Vector3d::UnitZ() };

    const auto center = poses[i].translation;
    const auto even_point = center + y_axis * half_w;
    const auto odd_point = center - y_axis * half_w;

    result[i * 2 + 0] = Lightbar3d{ even_point + z_axis * half_h, even_point - z_axis * half_h };
    result[i * 2 + 1] = Lightbar3d{ odd_point + z_axis * half_h, odd_point - z_axis * half_h };
  }
  return result;
}

// ---------------------------------------------------------------------------
// RobustPnP
// ---------------------------------------------------------------------------

auto RobustPnp::solve(Armor3d & output, std::string * reason, double * rms) const -> bool
{
  const auto fail = [&](const char * why) {
    if (reason) *reason = why;
    if (rms) *rms = std::numeric_limits<double>::quiet_NaN();
    return false;
  };
  if (!valid_camera(camera) || !std::isfinite(noise_observation) || noise_observation <= 0 ||
      !std::isfinite(max_rms_px) || max_rms_px <= 0) return fail("invalid_camera_or_noise");
  const std::array<cv::Point2f,4> corners{armor.tl,armor.tr,armor.br,armor.bl};
  double sign=0;
  for(int i=0;i<4;++i) {
    const auto d=corners[(i+1)%4]-corners[i], e=corners[(i+2)%4]-corners[(i+1)%4];
    const double cross=double(d.x)*e.y-double(d.y)*e.x;
    if(!std::isfinite(corners[i].x)||!std::isfinite(corners[i].y)||
       cv::norm(d)<1 || !std::isfinite(cross)||std::abs(cross)<1e-3) return fail("invalid_corners");
    if(i==0) sign=cross; else if(sign*cross<=0) return fail("invalid_corners");
  }
  if(sign<=0 || armor.tl.y>=armor.bl.y || armor.tr.y>=armor.br.y) return fail("invalid_corner_order");

  using X=RobotModel::StateVector;
  using Residual=Eigen::Matrix<double,8,1>;
  using Jacobian=Eigen::Matrix<double,8,4>;
  const Residual z=armor_measurement(armor,false);
  const auto evaluate=[&](const X & x,Residual & r,Jacobian & H) {
    for(int b=0;b<2;++b) {
      const auto p=project_lightbar(x,armor.large,camera,b);
      if(!p) return false;
      r.segment<4>(4*b)=p->value-z.segment<4>(4*b);
      H.block<4,3>(4*b,0)=p->jacobian.leftCols<3>();
      H.block<4,1>(4*b,3)=p->jacobian.col(6);
    }
    return r.allFinite() && H.allFinite();
  };
  struct Candidate { Armor3d pose; double cost; };
  std::vector<Candidate> candidates;
  try {
    const auto shape=armor_shape_points(armor.large);
    std::vector<cv::Point3f> object(shape.begin(),shape.end());
    std::vector<cv::Point2f> image{armor.tl,armor.bl,armor.br,armor.tr};
    std::vector<cv::Vec3d> rotations,translations;
    if(!cv::solvePnPGeneric(object,image,camera.intrinsic(),camera.distortion(),
       rotations,translations,false,cv::SOLVEPNP_IPPE) || rotations.size()!=translations.size())
      return fail("pnp_failed");
    for(std::size_t i=0;i<rotations.size();++i) {
      cv::Mat cv_rotation; cv::Rodrigues(rotations[i],cv_rotation);
      Eigen::Matrix3d rotation; cv::cv2eigen(cv_rotation,rotation);
      const Eigen::Vector3d t(translations[i][0],translations[i][1],translations[i][2]);
      if(!t.allFinite() || !rotation.allFinite() || t.z()<=0.1) continue;
      const Eigen::Quaterniond q= camera.orientation * Eigen::Quaterniond(opencv2ros_rotation(rotation));
      const Eigen::Vector3d p=camera.orientation*opencv2ros_position(t)+camera.translation;
      Eigen::Vector3d inward=q*Eigen::Vector3d::UnitX(); inward.z()=0;
      if(!inward.allFinite() || inward.norm()<1e-9) continue;
      inward.normalize();
      X x=X::Zero(); x.head<3>()=p+0.2*inward;
      x[6]=normalize_angle(std::atan2(-inward.y(),-inward.x())); x[8]=x[9]=0.2;
      Residual r; Jacobian H;
      if(!evaluate(x,r,H)) continue;
      double cost=r.squaredNorm();
      // Fit only center and phase; radii/velocities remain initialization priors.
      for(int iteration=0;iteration<20;++iteration) {
        const Eigen::Matrix4d normal=H.transpose()*H;
        const Eigen::Vector4d scale=normal.diagonal().cwiseMax(1e-9).cwiseSqrt().cwiseInverse();
        Eigen::Matrix4d system=scale.asDiagonal()*normal*scale.asDiagonal();
        system.diagonal().array()+=1e-6;
        const Eigen::Vector4d step=-(scale.asDiagonal()*system.ldlt().solve(scale.asDiagonal()*H.transpose()*r));
        if(!step.allFinite() || step.norm()<1e-9) break;
        bool accepted=false;
        for(int line=0;line<8;++line) {
          const Eigen::Vector4d delta=std::ldexp(1.0,-line)*step;
          X next=x; next.head<3>()+=delta.head<3>(); next[6]=normalize_angle(next[6]+delta[3]);
          Residual nr; Jacobian nh;
          if(evaluate(next,nr,nh) && nr.squaredNorm()<cost) {
            x=next;r=nr;H=nh;cost=r.squaredNorm();accepted=true;break;
          }
        }
        if(!accepted) break;
      }
      const Eigen::Vector3d radial(std::cos(x[6]),std::sin(x[6]),0);
      const Eigen::Vector3d plate=x.head<3>()+0.2*radial;
      const Eigen::Quaterniond orientation=euler_to_quaternion(normalize_angle(x[6]+kPi),kPredictedOtherArmorPitch,0);
      if(!plate.allFinite() || -(orientation*Eigen::Vector3d::UnitX()).dot(camera.translation-plate)<=0 ||
         std::sqrt(cost/4)>max_rms_px) continue;
      candidates.push_back({Armor3d{0,plate,orientation},cost});
    }
  } catch(const cv::Exception &) { return fail("pnp_exception"); }
  if(candidates.empty()) return fail("no_valid_pose");
  std::sort(candidates.begin(),candidates.end(),[](const auto & a,const auto & b){return a.cost<b.cost;});
  const auto & best=candidates.front();
  for(std::size_t i=1;i<candidates.size();++i) {
    const auto & other=candidates[i];
    const bool same=(best.pose.translation-other.pose.translation).norm()<0.02 &&
      best.pose.orientation.angularDistance(other.pose.orientation)<2*kPi/180;
    // Compare distinct physical modes using the same pixel likelihood.
    if(!same && (other.cost-best.cost)/noise_observation<2.0) return fail("ambiguous_pose");
  }
  output=best.pose;
  if(reason) *reason="valid";
  if(rms) *rms=std::sqrt(best.cost/4);
  return true;
}

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

auto load_config(const YAML::Node & node) -> Config
{
  Config config;

  config.noise_radius_forward = read_or(node, "noise_radius_forward", config.noise_radius_forward);
  config.noise_radius_lateral = read_or(node, "noise_radius_lateral", config.noise_radius_lateral);
  config.noise_height_lateral = read_or(node, "noise_height_lateral", config.noise_height_lateral);

  config.init_max_rms_px = read_or(node, "init_max_rms_px", config.init_max_rms_px);
  config.noise_observation = read_or(node, "noise_observation", config.noise_observation);

  config.q_accel_xy = read_or(node, "q_accel_xy", config.q_accel_xy);
  config.q_accel_z = read_or(node, "q_accel_z", config.q_accel_z);
  config.q_alpha = read_or(node, "q_alpha", config.q_alpha);

  config.radius_forward_min = read_or(node, "radius_forward_min", config.radius_forward_min);
  config.radius_forward_max = read_or(node, "radius_forward_max", config.radius_forward_max);
  config.radius_lateral_min = read_or(node, "radius_lateral_min", config.radius_lateral_min);
  config.radius_lateral_max = read_or(node, "radius_lateral_max", config.radius_lateral_max);
  config.height_lateral_min = read_or(node, "height_lateral_min", config.height_lateral_min);
  config.height_lateral_max = read_or(node, "height_lateral_max", config.height_lateral_max);

  config.velocity_min = read_or(node, "velocity_min", config.velocity_min);
  config.velocity_max = read_or(node, "velocity_max", config.velocity_max);
  config.rotation_speed_limit =
    read_or(node, "rotation_speed_limit", config.rotation_speed_limit);

  config.yaw_full_max = read_or(node, "yaw_full_max_deg", 70.0) * kPi / 180.0;
  config.yaw_part_max = read_or(node, "yaw_part_max_deg", 90.0) * kPi / 180.0;


  config.converge_min_update = read_or(node, "converge_min_update", config.converge_min_update);
  config.converge_max_cov_xy = read_or(node, "converge_max_cov_xy", config.converge_max_cov_xy);
  config.converge_max_cov_v = read_or(node, "converge_max_cov_v", config.converge_max_cov_v);
  config.converge_max_cov_w = read_or(node, "converge_max_cov_w", config.converge_max_cov_w);
  config.converge_max_no_update_frames = read_or(
    node, "converge_max_no_update_frames", config.converge_max_no_update_frames);
  config.converge_delay_s = read_or(node, "converge_delay_s", config.converge_delay_s);

  config.diverge_cov = read_or(node, "diverge_cov", config.diverge_cov);
  config.diverge_xy = read_or(node, "diverge_xy", config.diverge_xy);
  config.diverge_z = read_or(node, "diverge_z", config.diverge_z);
  config.diverge_w = read_or(node, "diverge_w", config.diverge_w);
  config.diverge_v = read_or(node, "diverge_v", config.diverge_v);
  config.diverge_vz = read_or(node, "diverge_vz", config.diverge_vz);

  config.initial_cov_position = read_or(node, "initial_cov_position", config.initial_cov_position);
  config.initial_cov_z = read_or(node, "initial_cov_z", config.initial_cov_z);
  config.initial_cov_velocity = read_or(node, "initial_cov_velocity", config.initial_cov_velocity);
  config.initial_cov_vz = read_or(node, "initial_cov_vz", config.initial_cov_vz);
  config.initial_cov_angle = read_or(node, "initial_cov_angle", config.initial_cov_angle);
  config.initial_cov_rotation_speed =
    read_or(node, "initial_cov_rotation_speed", config.initial_cov_rotation_speed);
  config.initial_cov_radius = read_or(node, "initial_cov_radius", config.initial_cov_radius);
  config.initial_cov_height = read_or(node, "initial_cov_height", config.initial_cov_height);

  config.anchor_max_corner_error =
    read_or(node, "anchor_max_corner_error", config.anchor_max_corner_error);
  config.anchor_max_relative_error =
    read_or(node, "anchor_max_relative_error", config.anchor_max_relative_error);
  config.association_early_updates=read_or(node,"association_early_updates",config.association_early_updates);
  config.association_early_max_corner_error=read_or(node,"association_early_max_corner_error",config.association_early_max_corner_error);
  config.geometry_confirm_s=read_or(node,"geometry_confirm_s",config.geometry_confirm_s);
  config.geometry_radius_std=read_or(node,"geometry_radius_std",config.geometry_radius_std);
  config.geometry_height_std=read_or(node,"geometry_height_std",config.geometry_height_std);

  config.nis_threshold = read_or(node, "nis_threshold", config.nis_threshold);
  config.nis_window = read_or(node, "nis_window", config.nis_window);

  config.nis_gate_enabled = read_or(node, "nis_gate_enabled", config.nis_gate_enabled);
  config.nis_gate_threshold = read_or(node, "nis_gate_threshold", config.nis_gate_threshold);
  config.nis_gate_warmup_updates =
    read_or(node, "nis_gate_warmup_updates", config.nis_gate_warmup_updates);

  config.skip_tracking_pnp = read_or(node, "skip_tracking_pnp", config.skip_tracking_pnp);

  config.association_miss_penalty=read_or(node,"association_miss_penalty",config.association_miss_penalty);
  config.association_clutter_penalty=read_or(node,"association_clutter_penalty",config.association_clutter_penalty);
  config.association_ambiguity_gap=read_or(node,"association_ambiguity_gap",config.association_ambiguity_gap);
  config.nis_gate_warmup_s=read_or(node,"nis_gate_warmup_s",config.nis_gate_warmup_s);
  return config;
}

// ---------------------------------------------------------------------------
// RobotModel
// ---------------------------------------------------------------------------

RobotModel::RobotModel(const Config & config)
{
  configure(config);
  reset_covariance();
}

auto RobotModel::configure(const Config & config) -> void
{
  for(double q:{config.q_accel_xy,config.q_accel_z,config.q_alpha,
    config.noise_radius_forward,config.noise_radius_lateral,config.noise_height_lateral})
    if(!std::isfinite(q) || q<0) throw std::invalid_argument("invalid process noise intensity");
  if(!std::isfinite(config.noise_observation) || config.noise_observation<=0)
    throw std::invalid_argument("pixel variance must be positive");
  for(double v:{config.radius_forward_min,config.radius_forward_max,
    config.radius_lateral_min,config.radius_lateral_max,config.height_lateral_min,
    config.height_lateral_max,config.velocity_min,config.velocity_max,config.rotation_speed_limit})
    if(!std::isfinite(v)) throw std::invalid_argument("state bounds must be finite");
  for(double v:{config.association_miss_penalty,config.association_clutter_penalty,
    config.association_ambiguity_gap,config.nis_gate_warmup_s,config.converge_delay_s})
    if(!std::isfinite(v) || v<0) throw std::invalid_argument("invalid association or time threshold");
  if(!std::isfinite(config.yaw_full_max) || !std::isfinite(config.yaw_part_max)
    || config.yaw_full_max<=0 || config.yaw_full_max>config.yaw_part_max
    || config.yaw_part_max>kPi/2)
    throw std::invalid_argument("invalid visibility angles");
  if(config.radius_forward_min<=0 || config.radius_forward_min>=config.radius_forward_max
    || config.radius_lateral_min<=0 || config.radius_lateral_min>=config.radius_lateral_max
    || config.height_lateral_min>=config.height_lateral_max || config.velocity_min>=config.velocity_max)
    throw std::invalid_argument("invalid state bounds");
  if (!std::isfinite(config.init_max_rms_px) || config.init_max_rms_px<=0)
    throw std::invalid_argument("init_max_rms_px must be positive finite");
  if(config.association_early_updates<0) throw std::invalid_argument("negative geometry update count");
  for(double v:{config.association_early_max_corner_error,config.geometry_confirm_s,
                config.geometry_radius_std,config.geometry_height_std})
    if(!std::isfinite(v) || v<=0) throw std::invalid_argument("invalid geometry learning threshold");
  config_ = config;
  observable_.yaw_full_max = config_.yaw_full_max;
  observable_.yaw_part_max = config_.yaw_part_max;
  set_observation_noise();
}

auto RobotModel::set_observation_noise() -> void
{
  observation_noise_.setZero();
  observation_noise_.diagonal() << config_.noise_observation, config_.noise_observation,
    config_.noise_observation, config_.noise_observation;
}

auto RobotModel::reset_covariance() -> void
{
  StateVector diagonal;
  diagonal << config_.initial_cov_position, config_.initial_cov_position, config_.initial_cov_z,
    config_.initial_cov_velocity, config_.initial_cov_velocity, config_.initial_cov_vz,
    config_.initial_cov_angle, config_.initial_cov_rotation_speed,
    config_.initial_cov_radius, config_.initial_cov_radius, config_.initial_cov_height;
  // Never allow a zero variance: it would make the covariance singular.
  for (Eigen::Index index = 0; index < diagonal.size(); ++index) {
    if(!std::isfinite(diagonal[index]) || diagonal[index]<=0)
      throw std::invalid_argument("initial covariance must be positive finite");
  }
  if(config_.association_early_updates>0) {
    // Geometry is unknown at startup. Its bounded prior must also reach NIS/K,
    // not just the association gate. Apply ONCE, never inflate after a rejection.
    const auto prior_variance=[](double lo,double hi,double initial) {
      const double bias=(lo+hi)/2-initial;
      return (hi-lo)*(hi-lo)/12+bias*bias;
    };
    diagonal[8]=std::max(diagonal[8],prior_variance(config_.radius_forward_min,config_.radius_forward_max,state_[8]));
    diagonal[9]=std::max(diagonal[9],prior_variance(config_.radius_lateral_min,config_.radius_lateral_max,state_[9]));
    diagonal[10]=std::max(diagonal[10],prior_variance(config_.height_lateral_min,config_.height_lateral_max,state_[10]));
  }
  covariance_ = diagonal.asDiagonal();
}

auto RobotModel::initialize(const CameraFeature & camera, const std::vector<Armor2d> & armors)
  -> bool
{
  if (armors.empty() || !valid_camera(camera)) return false;

  const Armor2d * best_armor = nullptr;
  auto best_width = 0.0;
  for (const auto & armor : armors) {
    if (!armor_measurement(armor,false).allFinite()) continue;
    const auto width = std::abs(armor.tr.x - armor.tl.x);
    if (width <= best_width) continue;
    best_width = width;
    best_armor = &armor;
  }
  if (!best_armor) return false;


  RobustPnp pnp;
  pnp.camera = camera;
  pnp.armor = *best_armor;
  pnp.noise_observation = config_.noise_observation;
  pnp.max_rms_px = config_.init_max_rms_px;

  Armor3d armor3d;
  if (!pnp.solve(armor3d)) return false;

  return initialize_from_pose(camera, armor3d.translation, armor3d.orientation, best_armor->large, best_armor);
}

auto RobotModel::initialize_from_pose(const CameraFeature & camera,
  const Eigen::Vector3d & translation, const Eigen::Quaterniond & orientation, bool large,
  const Armor2d * observation) -> bool
{
  if (!valid_camera(camera) || !translation.allFinite() || !orientation.coeffs().allFinite()
      || orientation.norm() < kEpsilon) return false;
  // The constrained pose and RobotSolution share the same horizontal radius model.
  Eigen::Vector3d armor_to_center = orientation.normalized() * Eigen::Vector3d::UnitX();
  armor_to_center.z() = 0.0;
  if (armor_to_center.norm() < kEpsilon) return false;
  armor_to_center.normalize();
  const auto yaw = normalize_angle(std::atan2(-armor_to_center.y(), -armor_to_center.x()));
  const auto center = Eigen::Vector3d{ translation + 0.2 * armor_to_center };


  StateVector candidate=StateVector::Zero();
  candidate.head<3>()=center;candidate[6]=yaw;candidate[8]=candidate[9]=0.2;
  const auto left=project_lightbar(candidate,large,camera,0);
  const auto right=project_lightbar(candidate,large,camera,1);
  if(!valid_state(candidate) || !left || !right) return false;
  const Eigen::Vector3d normal=euler_to_quaternion(yaw+kPi,kPredictedOtherArmorPitch,0)*Eigen::Vector3d::UnitX();
  if(normal.dot(camera.translation-translation)>=0) return false;
  if(observation) {
    Eigen::Matrix<double,8,1> pixels; pixels << left->value,right->value;
    const double rms=(pixels-armor_measurement(*observation,false)).norm()/2;
    if(!std::isfinite(rms) || rms>config_.init_max_rms_px) return false;
  }
  return finish_initialize(camera, center, yaw, large);
}

auto RobotModel::initialize_synthetic(const State & state) -> void
{
  state_.setZero();
  state_[kStateX] = state.x;
  state_[kStateY] = state.y;
  state_[kStateZ] = state.z;
  state_[kStateVx] = state.vx;
  state_[kStateVy] = state.vy;
  state_[kStateVz] = state.vz;
  state_[kStateA] = normalize_angle(state.rotation_angle);
  state_[kStateW] = state.rotation_speed;
  state_[kStateRF] = state.radius_forward;
  state_[kStateRL] = state.radius_lateral;
  state_[kStateHL] = state.height_lateral;

  large_ = false;
  reset_covariance();
  observable_dirty_ = true;

  elapsed_s_ = 0.0;
  report_ = {};
  jumped_ = false;
  last_observed_id_ = -1;
  // A synthetic state supplies both radii and height explicitly.
  geometry_ready_={true,true};geometry_good_updates_={};
  geometry_first_good_={-1,-1};geometry_last_good_={-1,-1};
  pixel_innovations_ = {};
  update_count_ = 0;
  no_update_frames_ = 0;
}

auto RobotModel::finish_initialize(
    const CameraFeature & camera, const Eigen::Vector3d & center, double yaw, bool large) -> bool
{
  large_ = large;
  state_.setZero();
  state_[kStateX] = center.x();
  state_[kStateY] = center.y();
  state_[kStateZ] = center.z();
  state_[kStateRF] = 0.2;
  state_[kStateRL] = 0.2;
  state_[kStateHL] = 0.0;

  observable_.camera = camera;
  observable_.large = large;

  state_[kStateA] = yaw;
  observable_.update(state_);

  reset_covariance();
  observable_dirty_ = false;

  elapsed_s_ = 0.0;
  report_ = {};
  jumped_ = false;
  last_observed_id_ = 0;  // the validated initialization plate
  geometry_ready_={false,false};geometry_good_updates_={};
  geometry_first_good_={-1,-1};geometry_last_good_={-1,-1};
  pixel_innovations_ = {};
  update_count_ = 0;
  last_frame_update_count_ = 0;
  last_frame_rejected_count_ = 0;
  frame_update_count_ = 0;
  frame_rejected_count_ = 0;
  nis_rejected_count_ = 0;
  consecutive_nis_rejects_ = 0;
  recent_nis_failures_.clear();

  last_frame_nis_ = 0.0;
  last_frame_nis_mean_ = 0.0;
  last_frame_residual_norm_ = 0.0;
  frame_nis_max_ = 0.0;
  frame_nis_sum_ = 0.0;
  frame_nis_count_ = 0;
  last_anchor_error_ = 0.0;
  no_update_frames_ = 0;
  return true;
}

auto RobotModel::extrapolate_state(const StateVector & state, double dt) const -> StateVector
{
  auto next = state;
  next[kStateX] += next[kStateVx] * dt;
  next[kStateY] += next[kStateVy] * dt;
  next[kStateZ] += next[kStateVz] * dt;
  next[kStateA] = normalize_angle(next[kStateA] + next[kStateW] * dt);
  return next;
}

auto RobotModel::transition(double dt) -> Covariance {
  Covariance F=Covariance::Identity();
  F(0,3)=F(1,4)=F(2,5)=F(6,7)=dt;
  return F;
}
auto RobotModel::process_covariance(double dt) const -> ProcessNoise {
  if(!std::isfinite(dt) || dt<0) throw std::invalid_argument("CWNA requires nonnegative finite dt");
  ProcessNoise Q=ProcessNoise::Zero();
  const auto axis=[&](int p,int v,double q) {
    Q(p,p)=q*dt*dt*dt/3; Q(p,v)=Q(v,p)=q*dt*dt/2; Q(v,v)=q*dt;
  };
  axis(0,3,config_.q_accel_xy);axis(1,4,config_.q_accel_xy);
  axis(2,5,config_.q_accel_z);axis(6,7,config_.q_alpha);
  Q(8,8)=config_.noise_radius_forward*dt;
  Q(9,9)=config_.noise_radius_lateral*dt;
  Q(10,10)=config_.noise_height_lateral*dt;
  return Q;
}
auto RobotModel::future_covariance(double dt) const -> Covariance {
  const auto F=transition(dt);
  Covariance P=F*covariance_*F.transpose()+process_covariance(dt);
  return (0.5*(P+P.transpose())).eval();
}
auto RobotModel::propagate_covariance(double dt) -> void { covariance_=future_covariance(dt); }

bool RobotModel::valid_state(const StateVector & x) const {
  if(!x.allFinite()) return false;
  if(x[8]<config_.radius_forward_min || x[8]>config_.radius_forward_max
    || x[9]<config_.radius_lateral_min || x[9]>config_.radius_lateral_max
    || x[10]<config_.height_lateral_min || x[10]>config_.height_lateral_max) return false;
  for(int i=3;i<6;++i) if(x[i]<config_.velocity_min || x[i]>config_.velocity_max) return false;
  return config_.rotation_speed_limit<=0 || std::abs(x[7])<=config_.rotation_speed_limit;
}
bool RobotModel::import_snapshot(const StateVector & x,const Covariance & P,bool large,double elapsed) {
  if(!valid_state(x) || !P.allFinite() || !std::isfinite(elapsed) || elapsed<0
    || !P.isApprox(P.transpose(),1e-10)) return false;
  Eigen::LDLT<Covariance> factor(P);
  if(factor.info()!=Eigen::Success || (factor.vectorD().array()<=0).any()) return false;
  state_=x;state_[6]=normalize_angle(state_[6]);covariance_=P;large_=large;
  elapsed_s_=elapsed;observable_.large=large;observable_dirty_=true;
  // A fitted snapshot may change the phase/plate labeling; reconfirm its geometry.
  geometry_ready_={false,false};geometry_good_updates_={};
  geometry_first_good_={-1,-1};geometry_last_good_={-1,-1};
  last_observed_id_=-1;jumped_=false;
  return true;
}

auto RobotModel::refresh_observable() const -> void
{
  if (!observable_dirty_) return;
  observable_.update(state_);
  observable_dirty_ = false;
}

auto RobotModel::predict(double dt) -> void
{
  // Filter time update only. State-only lookahead must use extrapolate().
  if (!std::isfinite(dt) || dt <= 0.0) {
    last_dt_ = 0.0;
    return;
  }

  last_dt_ = dt;
  elapsed_s_ += dt;
  state_ = extrapolate_state(state_, dt);
  propagate_covariance(dt);

  // Do not project here. correct()/diagnostics() refresh on demand; full()
  // only needs RobotSolution::solve_armors() and stays projection-free.
  observable_dirty_ = true;
}

auto RobotModel::extrapolate(double dt) const -> State
{
  if (!std::isfinite(dt)) return state();
  return state_from_vector(extrapolate_state(state_, dt));
}

auto RobotModel::extrapolated(double dt) const -> RobotModel
{
  RobotModel copy = *this;
  if (std::isfinite(dt)) {
    copy.state_ = copy.extrapolate_state(copy.state_, dt);
  }
  copy.observable_dirty_ = true;
  return copy;
}


auto RobotModel::correct(const std::vector<Armor2d> & armors,
  const std::vector<Lightbar2d> & lightbars, const CameraFeature & camera) -> UpdateReport
{
  report_={}; report_.detection_count=armors.size();
  observable_.camera=camera;observable_.large=large_;observable_dirty_=true;
  pixel_innovations_={};frame_nis_sum_=frame_nis_max_=frame_residual_sum_=0;
  frame_nis_count_=frame_update_count_=frame_rejected_count_=0;
  last_update_accepted_=false;last_anchor_error_=0;
  std::array<bool,2> complete_updates{};
  if(!lightbars.empty()) report_.reject_reasons.push_back(RejectReason::UnsupportedLightbar);
  if(!valid_camera(camera)) report_.reject_reasons.push_back(RejectReason::UnsupportedCamera);
  else {
    const auto association_start=std::chrono::steady_clock::now();
    const auto association=associate_armors(state_,covariance_,large_,camera,armors,config_,&geometry_ready_);
    report_.association_gate_px=association.gate_px;
    report_.association_ms=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-association_start).count();
    report_.legal_armor_count=association.legal_observations.size();
    report_.associated_armor_count=association.matches.size();
    report_.ambiguous=association.ambiguous;report_.association_cost=association.cost;
    report_.association_second_cost=association.second_cost;report_.prediction_rms=association.rms;
    report_.trimmed_count=association.trimmed_count;report_.duplicate_count=association.duplicate_count;
    if(association.invalid_count) report_.reject_reasons.push_back(RejectReason::InvalidObservation);
    if(association.duplicate_count) report_.reject_reasons.push_back(RejectReason::DuplicateObservation);
    if(association.trimmed_count) report_.reject_reasons.push_back(RejectReason::CandidateLimit);
    if(association.matches.empty() && !armors.empty()) report_.reject_reasons.push_back(RejectReason::NoAssociation);
    if(association.ambiguous) report_.reject_reasons.push_back(RejectReason::AmbiguousAssociation);
    refresh_observable();
    const auto update_start=std::chrono::steady_clock::now();
    // Fixed one-to-one association, deterministic model plate/bar order.
    for(const auto & match:association.matches) {
      // Do not turn an ambiguous startup identity into a confident Gaussian.
      if(association.ambiguous) continue;
      report_.matched_observation_by_plate[match.armor_id]=static_cast<int>(match.observation_index);
      report_.matched_swapped_by_plate[match.armor_id]=match.swapped;
      const auto accepted_before=frame_update_count_;
      const auto z=armor_measurement(armors[match.observation_index],match.swapped);
      update_one(z.segment<2>(0),z.segment<2>(2),2*match.armor_id);
      update_one(z.segment<2>(4),z.segment<2>(6),2*match.armor_id+1);
      if(frame_update_count_>accepted_before) {
        if(last_observed_id_>=0 && last_observed_id_!=match.armor_id) jumped_=true;
        last_observed_id_=match.armor_id;
      }
      if(frame_update_count_==accepted_before+2) {
        complete_updates[match.armor_id%2]=true;
        if(!geometry_ready_[match.armor_id%2]) report_.geometry_learning_progress=true;
      }
    }
    report_.update_ms=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-update_start).count();
  }
  if(frame_update_count_) {++update_count_;no_update_frames_=0;}
  else ++no_update_frames_;
  last_frame_update_count_=frame_update_count_;last_frame_rejected_count_=frame_rejected_count_;
  const double missing=std::numeric_limits<double>::quiet_NaN();
  last_frame_nis_=frame_nis_count_?frame_nis_max_:missing;
  last_frame_nis_mean_=frame_nis_count_?frame_nis_sum_/frame_nis_count_:missing;
  last_frame_residual_norm_=frame_nis_count_?frame_residual_sum_/frame_nis_count_:missing;
  report_.nis_valid=frame_nis_count_>0;report_.nis_count=frame_nis_count_;
  report_.accepted_lightbar_count=frame_update_count_;report_.rejected_lightbar_count=frame_rejected_count_;
  report_.accepted_frame_count=update_count_;
  last_update_accepted_=frame_update_count_>0;
  report_.estimated_quality=frame_update_count_>=2 && !report_.ambiguous && !diverged()
    && std::isfinite(report_.prediction_rms) && report_.prediction_rms<=std::sqrt(config_.noise_observation)*2;
  update_geometry_learning(complete_updates);
  return report_;
}

void RobotModel::update_geometry_learning(const std::array<bool,2> & complete_updates) {
  for(int g=0;g<2;++g) {
    if(geometry_ready_[g]) continue;
    // A long gap cannot count as confirmation time without new observations.
    if(geometry_last_good_[g]>=0 && elapsed_s_-geometry_last_good_[g]>0.5) {
      geometry_good_updates_[g]=0;
      geometry_first_good_[g]=geometry_last_good_[g]=-1;
    }
    const bool matched=report_.matched_observation_by_plate[g]>=0 || report_.matched_observation_by_plate[g+2]>=0;
    const bool good=complete_updates[g] && report_.estimated_quality
      && report_.associated_armor_count==report_.legal_armor_count && !report_.trimmed_count;
    if(good) {
      if(geometry_good_updates_[g]==0) geometry_first_good_[g]=elapsed_s_;
      ++geometry_good_updates_[g];geometry_last_good_[g]=elapsed_s_;
    } else if(matched || report_.ambiguous) {
      geometry_good_updates_[g]=0;geometry_first_good_[g]=-1;
    }
  }
  // Both height levels must constrain the shared center/height gauge.
  const bool both_seen=geometry_last_good_[0]>=0 && geometry_last_good_[1]>=0;
  for(int g=0;g<2;++g) {
    if(both_seen && geometry_good_updates_[g]>=std::max(1,config_.association_early_updates)
       && geometry_last_good_[g]-geometry_first_good_[g]>=config_.geometry_confirm_s
       && covariance_(8+g,8+g)<=config_.geometry_radius_std*config_.geometry_radius_std
       && (g==0 || covariance_(10,10)<=config_.geometry_height_std*config_.geometry_height_std))
      geometry_ready_[g]=true;
  }
  report_.geometry_ready=geometry_ready_;
  report_.geometry_good_updates=geometry_good_updates_;
}

auto RobotModel::update_one(const Eigen::Vector2d & upper, const Eigen::Vector2d & lower,
  int lightbar_id) -> void
{
  if (lightbar_id < 0 || lightbar_id >= 8) return;
  pixel_innovations_[lightbar_id] = {};
  if (!observable_.visible[lightbar_id]) {
    last_update_accepted_ = false;
    ++frame_rejected_count_;
    report_.reject_reasons.push_back(RejectReason::Projection);
    return;
  }

  // Skip measurements whose predicted endpoints are behind the camera or
  // failed to project. Feeding (0, 0) into the EKF would corrupt the state.
  if (!observable_.upper_valid[lightbar_id] || !observable_.lower_valid[lightbar_id]) {
    last_update_accepted_ = false;
    ++frame_rejected_count_;
    report_.reject_reasons.push_back(RejectReason::Projection);
    return;
  }

  const auto & predicted_upper = observable_.upper2d[lightbar_id];
  const auto & predicted_lower = observable_.lower2d[lightbar_id];

  Eigen::Matrix<double, 4, 1> innovation;
  innovation << upper.x() - predicted_upper.x(), upper.y() - predicted_upper.y(),
    lower.x() - predicted_lower.x(), lower.y() - predicted_lower.y();

  const auto jacobian = make_observation_jacobian(lightbar_id);
  if (jacobian.hasNaN() || !jacobian.allFinite()) {
    last_update_accepted_ = false;
    ++frame_rejected_count_;
    report_.reject_reasons.push_back(RejectReason::Projection);
    return;
  }

  // v0.32: 显式类型，避免 Eigen 惰性表达式在每次使用时重复求值（含求逆）。
  const Eigen::Matrix4d innovation_covariance =
    jacobian * covariance_ * jacobian.transpose() + observation_noise_;
  Eigen::LDLT<Eigen::Matrix4d> factor(innovation_covariance);
  if(!innovation_covariance.allFinite() || factor.info()!=Eigen::Success
    || (factor.vectorD().array()<=0).any()) {
    last_update_accepted_=false;++frame_rejected_count_;
    report_.reject_reasons.push_back(RejectReason::InnovationSystem);return;
  }
  const Eigen::Vector4d solved=factor.solve(innovation);
  const double nis=innovation.dot(solved);
  if(!solved.allFinite() || !std::isfinite(nis) || nis<0) {
    ++frame_rejected_count_;report_.reject_reasons.push_back(RejectReason::InnovationSystem);return;
  }
  last_nis_ = nis;
  last_residual_norm_ = innovation.norm();
  // Telemetry only: preserve signed innovation before gating or state correction.
  // If association assigns an ID twice, its slot represents the last attempt.
  auto & pixel = pixel_innovations_[lightbar_id];
  pixel.valid = innovation.allFinite() && std::isfinite(nis);
  pixel.residual = {innovation[0], innovation[1], innovation[2], innovation[3]};
  pixel.nis = nis;

  // Diagnostics count every valid measurement attempt, including rejected ones.
  frame_nis_sum_ += nis;
  frame_nis_max_ = std::max(frame_nis_max_, nis);
  frame_residual_sum_ += last_residual_norm_;
  ++frame_nis_count_;

  const auto gate_active =
    config_.nis_gate_enabled
    && update_count_ >= static_cast<std::size_t>(std::max(0, config_.nis_gate_warmup_updates))
    && elapsed_s_ >= config_.nis_gate_warmup_s;
  const auto gate_threshold =
    config_.nis_gate_threshold > 0.0 ? config_.nis_gate_threshold : config_.nis_threshold;

  if (gate_active && nis > gate_threshold) {
    // Reject without changing the prior; recovery handles persistent mismatch.
    last_update_accepted_ = false;
    ++frame_rejected_count_;
    ++nis_rejected_count_;
    ++consecutive_nis_rejects_;

    report_.reject_reasons.push_back(RejectReason::NisGate);

    recent_nis_failures_.push_back(1);
    const auto window = static_cast<std::size_t>(std::max(1, config_.nis_window));
    while (recent_nis_failures_.size() > window) recent_nis_failures_.pop_front();
    return;
  }

  consecutive_nis_rejects_ = 0;

  const KalmanGain kalman_gain = factor.solve((jacobian * covariance_).eval()).transpose();

  auto posterior_state = StateVector{ state_ + kalman_gain * innovation };
  if (posterior_state.hasNaN() || !posterior_state.allFinite()) {
    last_update_accepted_ = false;
    ++frame_rejected_count_;
    report_.reject_reasons.push_back(RejectReason::StateBounds);
    return;
  }

  posterior_state[kStateA] = normalize_angle(posterior_state[kStateA]);

  if(!valid_state(posterior_state)) {
    last_update_accepted_=false;++frame_rejected_count_;
    report_.reject_reasons.push_back(RejectReason::StateBounds);return;
  }

  const Covariance complement = Covariance::Identity() - kalman_gain * jacobian;
  Covariance posterior_covariance =
    complement * covariance_ * complement.transpose();
  posterior_covariance += kalman_gain * observation_noise_ * kalman_gain.transpose();
  posterior_covariance =
    (0.5 * (posterior_covariance + posterior_covariance.transpose())).eval();
  Eigen::LDLT<Covariance> posterior_factor(posterior_covariance);
  if(!posterior_covariance.allFinite() || posterior_factor.info()!=Eigen::Success
    || (posterior_factor.vectorD().array()<=0).any()) {
    last_update_accepted_=false;++frame_rejected_count_;
    report_.reject_reasons.push_back(RejectReason::Covariance);return;
  }

  state_ = posterior_state;
  covariance_ = posterior_covariance;

  observable_.update(state_);
  observable_dirty_ = false;

  last_update_accepted_ = true;
  ++frame_update_count_;

  recent_nis_failures_.push_back(last_nis_ > config_.nis_threshold ? 1 : 0);
  const auto window = static_cast<std::size_t>(std::max(1, config_.nis_window));
  while (recent_nis_failures_.size() > window) recent_nis_failures_.pop_front();
}

auto RobotModel::diagnostics() const -> Diagnostics
{
  refresh_observable();

  Diagnostics result;
  result.pixel_innovations = pixel_innovations_;
  result.report = report_; result.nis_valid=report_.nis_valid; result.nis_count=report_.nis_count;
  result.state = state();
  result.variance = covariance_.diagonal();
  result.covariance_xy = covariance_.topLeftCorner<2, 2>();

  result.nis = last_frame_nis_;
  result.nis_mean = last_frame_nis_mean_;
  result.anchor_error = last_anchor_error_;
  result.nis_threshold = config_.nis_threshold;
  result.nis_gate_enabled = config_.nis_gate_enabled;
  result.nis_gate_threshold =
    config_.nis_gate_threshold > 0.0 ? config_.nis_gate_threshold : config_.nis_threshold;
  result.residual_norm = last_frame_residual_norm_;
  result.dt = last_dt_;

  result.update_count = update_count_;
  result.frame_update_count = last_frame_update_count_;
  result.frame_rejected_count = last_frame_rejected_count_;
  result.nis_rejected_count = nis_rejected_count_;
  result.nis_consecutive_rejects = consecutive_nis_rejects_;
  result.no_update_frames = no_update_frames_;
  result.reinit_count = reinit_count_;
  result.last_observed_id = last_observed_id_;
  result.converged = converge();
  result.diverged = diverged();
  result.last_update_accepted = last_update_accepted_;

  if (!recent_nis_failures_.empty()) {
    const auto failures =
      std::count(recent_nis_failures_.begin(), recent_nis_failures_.end(), 1);
    result.nis_fail_rate =
      static_cast<double>(failures) / static_cast<double>(recent_nis_failures_.size());
  }
  return result;
}

auto RobotModel::make_observation_jacobian(int lightbar_id) const -> Eigen::Matrix<double,4,11> {
  const auto projected=project_lightbar(state_,large_,observable_.camera,lightbar_id);
  if(projected) return projected->jacobian;
  return Eigen::Matrix<double,4,11>::Constant(std::numeric_limits<double>::quiet_NaN());
}

auto RobotModel::state_from_vector(const StateVector & state) const -> State
{
  State result;
  result.x = state[kStateX];
  result.y = state[kStateY];
  result.z = state[kStateZ];
  result.vx = state[kStateVx];
  result.vy = state[kStateVy];
  result.vz = state[kStateVz];
  result.rotation_angle = state[kStateA];
  result.rotation_speed = state[kStateW];
  result.radius_forward = state[kStateRF];
  result.radius_lateral = state[kStateRL];
  result.height_lateral = state[kStateHL];
  return result;
}

auto RobotModel::state() const -> State { return state_from_vector(state_); }

auto RobotModel::full() const -> std::array<Armor3d, 4>
{
  RobotSolution solution;
  solution.input.center = Eigen::Vector3d{ state_[kStateX], state_[kStateY], state_[kStateZ] };
  solution.input.toward =
    Eigen::Quaterniond{ Eigen::AngleAxisd{ state_[kStateA], Eigen::Vector3d::UnitZ() } };
  solution.input.radius_forward = state_[kStateRF];
  solution.input.radius_lateral = state_[kStateRL];
  solution.input.height_lateral = state_[kStateHL];
  solution.input.large = large_;

  return solution.solve_armors();
}

auto RobotModel::converge() const -> bool
{
  if (diverged()) return false;
  if (update_count_ < static_cast<std::size_t>(std::max(0, config_.converge_min_update)))
    return false;

  // A filter that has not accepted any measurement for a few frames is not
  // converged, no matter how small its covariance looks.
  if (no_update_frames_ > std::max(0, config_.converge_max_no_update_frames)) return false;

  if (covariance_(kStateX, kStateX) > config_.converge_max_cov_xy) return false;
  if (covariance_(kStateY, kStateY) > config_.converge_max_cov_xy) return false;
  if (covariance_(kStateVx, kStateVx) > config_.converge_max_cov_v) return false;
  if (covariance_(kStateVy, kStateVy) > config_.converge_max_cov_v) return false;
  if (covariance_(kStateW, kStateW) > config_.converge_max_cov_w) return false;

  if (elapsed_s_ < config_.converge_delay_s || report_.ambiguous) return false;

  return true;
}

auto RobotModel::diverged() const -> bool
{
  if (state_.hasNaN() || !state_.allFinite() || covariance_.hasNaN()
    || !covariance_.allFinite())
    return true;

  // A deadlock where detections keep arriving but nothing is accepted is just
  // as bad as a state blow-up; let the tracker drop and re-acquire the target.


  // Let the filter settle for a few frames after initialization; the first
  // sequential updates can legitimately produce large velocity corrections.
  if (!valid_state(state_)) return true;

  return covariance_(kStateX, kStateX) > config_.diverge_cov
    || covariance_(kStateY, kStateY) > config_.diverge_cov
    || std::abs(state_[kStateX]) > config_.diverge_xy
    || std::abs(state_[kStateY]) > config_.diverge_xy
    || std::abs(state_[kStateZ]) > config_.diverge_z
    || std::abs(state_[kStateW]) > config_.diverge_w
    || std::abs(state_[kStateVx]) > config_.diverge_v
    || std::abs(state_[kStateVy]) > config_.diverge_v
    || std::abs(state_[kStateVz]) > config_.diverge_vz;
}

auto RobotModel::Observable::update(const StateVector & state) -> void
{
  const auto q_cam_from_odom = camera.orientation.conjugate();
  const auto camera_position = camera.translation;

  std::fill(upper2d.begin(), upper2d.end(), Eigen::Vector2d::Zero());
  std::fill(lower2d.begin(), lower2d.end(), Eigen::Vector2d::Zero());
  std::fill(upper3d.begin(), upper3d.end(), Eigen::Vector3d::Zero());
  std::fill(lower3d.begin(), lower3d.end(), Eigen::Vector3d::Zero());
  std::fill(visible.begin(), visible.end(), false);
  std::fill(upper_valid.begin(), upper_valid.end(), false);
  std::fill(lower_valid.begin(), lower_valid.end(), false);

  RobotSolution solution;
  solution.input.center =
    Eigen::Vector3d{ state[RobotModel::kStateX], state[RobotModel::kStateY],
      state[RobotModel::kStateZ] };
  solution.input.toward = Eigen::Quaterniond{
    Eigen::AngleAxisd{ state[RobotModel::kStateA], Eigen::Vector3d::UnitZ() } };
  solution.input.radius_forward = state[RobotModel::kStateRF];
  solution.input.radius_lateral = state[RobotModel::kStateRL];
  solution.input.height_lateral = state[RobotModel::kStateHL];
  solution.input.large = large;

  const auto lightbars = solution.solve_lightbars();
  const auto armors = solution.solve_armors();

  const auto intrinsic = camera.intrinsic();
  const auto distortion = camera.distortion();

  for (int index = 0; index < 8; ++index) {
    const auto upper_odom = lightbars[index].upper;
    const auto lower_odom = lightbars[index].lower;

    const auto upper_camera = q_cam_from_odom * (upper_odom - camera_position);
    const auto lower_camera = q_cam_from_odom * (lower_odom - camera_position);

    upper3d[index] = upper_camera;
    lower3d[index] = lower_camera;

    if(const auto projected=project_lightbar(state,large,camera,index)) {
      upper2d[index]=projected->value.head<2>();lower2d[index]=projected->value.tail<2>();
      upper_valid[index]=lower_valid[index]=true;
    }
  }

  for (int index = 0; index < 4; ++index) {
    const auto orientation = armors[index].orientation;
    const auto position = armors[index].translation;

    const auto armor_to_center = orientation * Eigen::Vector3d::UnitX();
    const auto armor_face = -armor_to_center;
    const auto armor_to_camera = (camera_position - position).normalized();
    const auto yaw =
      std::acos(std::clamp(armor_face.dot(armor_to_camera), -1.0, 1.0));

    if (yaw >= yaw_part_max) continue;

    const auto left_id = index * 2;
    const auto right_id = index * 2 + 1;

    if (yaw < yaw_full_max) {
      visible[left_id] = true;
      visible[right_id] = true;
    } else {
      const auto y_axis = Eigen::Vector3d{ orientation * Eigen::Vector3d::UnitY() };
      const auto on_plus_y = armor_to_camera.dot(y_axis) > 0.0;
      visible[on_plus_y ? right_id : left_id] = true;
    }
  }
}

auto RobotModel::Observable::visible_armors() const -> std::size_t
{
  std::size_t count = 0;
  for (int index = 0; index < 4; ++index) {
    if (visible[index * 2] && visible[index * 2 + 1]) ++count;
  }
  return count;
}

} // namespace rmcs_ekf
