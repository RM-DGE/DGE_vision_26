#include "extended_kalman_filter.hpp"

#include <numeric>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nis_threshold"] = nis_threshold;
  data["nis_fail"] = 0.0;
  data["residual_norm"] = 0.0;
  // NEES 需要真值，实车不可计算；保留字段仅为兼容旧日志。
  data["nees"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  const Eigen::VectorXd x_prior = x;
  const Eigen::MatrixXd P_prior = P;

  // 标准 EKF：
  //   r = z - h(x_prior)
  //   S = H P_prior H^T + R
  //   K = P_prior H^T S^-1
  // NIS 必须在状态/协方差更新之前用先验计算，否则不是新息一致性指标。
  const Eigen::MatrixXd S = H * P_prior * H.transpose() + R;
  const Eigen::VectorXd residual = z_subtract(z, h(x_prior));

  const Eigen::LDLT<Eigen::MatrixXd> S_ldlt(S);
  if (S_ldlt.info() != Eigen::Success) {
    last_update_accepted = false;
    return x;
  }

  const Eigen::MatrixXd K =
    P_prior * H.transpose() * S_ldlt.solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));

  // Stable computation of the posterior covariance (Joseph form).
  P = (I - K * H) * P_prior * (I - K * H).transpose() + K * R * K.transpose();
  x = x_add(x_prior, K * residual);

  const double nis = residual.transpose() * S_ldlt.solve(residual);
  last_nis = nis;
  last_residual_norm = residual.norm();
  last_update_accepted = nis <= nis_threshold;

  if (!last_update_accepted) nis_count_++;
  total_count_++;

  recent_nis_failures.push_back(last_update_accepted ? 0 : 1);
  while (recent_nis_failures.size() > window_size) recent_nis_failures.pop_front();

  const int recent_failures =
    std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  const double recent_rate = recent_nis_failures.empty()
                               ? 0.0
                               : static_cast<double>(recent_failures) /
                                   static_cast<double>(recent_nis_failures.size());

  data["residual_yaw"] = residual[0];
  data["residual_pitch"] = residual[1];
  data["residual_distance"] = residual[2];
  data["residual_angle"] = residual[3];
  data["nis"] = nis;
  data["nis_threshold"] = nis_threshold;
  data["nis_fail"] = last_update_accepted ? 0.0 : 1.0;
  data["residual_norm"] = last_residual_norm;
  data["recent_nis_failures"] = recent_rate;
  // NEES 需要真值；实车/离线无真值时不要把它当作滤波器指标。
  data["nees"] = 0.0;
  data["nees_fail"] = 0.0;

  return x;
}

}  // namespace tools