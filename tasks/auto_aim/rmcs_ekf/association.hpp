#pragma once
#include "projection.hpp"
#include <limits>
namespace rmcs_ekf {
struct ArmorMatch { std::size_t observation_index=0; int armor_id=-1; bool swapped=false; };
struct AssociationResult {
  std::vector<ArmorMatch> matches;
  std::vector<std::size_t> legal_observations;
  double cost=std::numeric_limits<double>::infinity();
  double second_cost=std::numeric_limits<double>::infinity();
  double rms=std::numeric_limits<double>::quiet_NaN();
  bool ambiguous=false;
  std::size_t invalid_count=0, duplicate_count=0, trimmed_count=0;
  std::size_t missed_count=0, clutter_count=0;
};
Eigen::Matrix<double,8,1> armor_measurement(const Armor2d & armor,bool swapped);
AssociationResult associate_armors(const RobotModel::StateVector & state,
  const RobotModel::Covariance & covariance,bool large,const CameraFeature & camera,
  const std::vector<Armor2d> & armors,const Config & config);
}
