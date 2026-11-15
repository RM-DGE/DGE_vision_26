#ifndef AUTO_AIM__RMCS_EKF__PROJECTION_HPP
#define AUTO_AIM__RMCS_EKF__PROJECTION_HPP

#include "rmcs_ekf.hpp"

namespace rmcs_ekf
{
struct PixelProjection
{
  Eigen::Vector4d value;
  Eigen::Matrix<double, 4, 11> jacobian;
};
// Zero-skew, five-coefficient Brown-Conrady camera only.
bool valid_camera(const CameraFeature & camera);
std::optional<PixelProjection> project_lightbar(
  const RobotModel::StateVector & state, bool large, const CameraFeature & camera, int lightbar_id);
}  // namespace rmcs_ekf

#endif  // AUTO_AIM__RMCS_EKF__PROJECTION_HPP
