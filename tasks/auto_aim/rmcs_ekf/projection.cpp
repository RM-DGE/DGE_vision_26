#include "projection.hpp"
#include <cmath>

namespace rmcs_ekf {
bool valid_camera(const CameraFeature & c) {
  for (const auto & row : c.camera_matrix) for (double v : row)
    if (!std::isfinite(v)) return false;
  for (double v : c.distort_coeff) if (!std::isfinite(v)) return false;
  return c.camera_matrix[0][0] > 0 && c.camera_matrix[1][1] > 0
    && std::abs(c.camera_matrix[0][1]) < 1e-12
    && std::abs(c.camera_matrix[1][0]) < 1e-12
    && std::abs(c.camera_matrix[2][0]) < 1e-12
    && std::abs(c.camera_matrix[2][1]) < 1e-12
    && std::abs(c.camera_matrix[2][2] - 1) < 1e-12
    && c.translation.allFinite() && c.orientation.coeffs().allFinite()
    && std::abs(c.orientation.norm() - 1) < 1e-6;
}

std::optional<PixelProjection> project_lightbar(const RobotModel::StateVector & x,
  bool large, const CameraFeature & c, int id) {
  if (id < 0 || id >= 8 || !x.allFinite() || !valid_camera(c)) return std::nullopt;
  RobotSolution solution;
  solution.input.center = x.head<3>();
  solution.input.toward = Eigen::Quaterniond(Eigen::AngleAxisd(x[6], Eigen::Vector3d::UnitZ()));
  solution.input.radius_forward = x[8]; solution.input.radius_lateral = x[9];
  solution.input.height_lateral = x[10]; solution.input.large = large;
  const auto bars = solution.solve_lightbars();
  Eigen::Matrix3d C;
  C << 0,-1,0, 0,0,-1, 1,0,0;
  const Eigen::Matrix3d T = C * c.orientation.conjugate().toRotationMatrix();
  const Eigen::Vector3d u(std::cos(x[6]),std::sin(x[6]),0);
  const Eigen::Vector3d v(-std::sin(x[6]),std::cos(x[6]),0);
  PixelProjection out;
  for (int e=0;e<2;++e) {
    const Eigen::Vector3d b = e == 0 ? bars[id].upper : bars[id].lower;
    const Eigen::Vector3d q = T * (b - c.translation);
    if (!q.allFinite() || q.z() <= 0.1) return std::nullopt;
    const double nx=q.x()/q.z(), ny=q.y()/q.z(), s=nx*nx+ny*ny;
    const auto & k=c.distort_coeff;
    const double L=1+k[0]*s+k[1]*s*s+k[4]*s*s*s;
    const double Lx=2*nx*(k[0]+2*k[1]*s+3*k[4]*s*s);
    const double Ly=2*ny*(k[0]+2*k[1]*s+3*k[4]*s*s);
    out.value[2*e]=c.camera_matrix[0][0]*(nx*L+2*k[2]*nx*ny+k[3]*(s+2*nx*nx))+c.camera_matrix[0][2];
    out.value[2*e+1]=c.camera_matrix[1][1]*(ny*L+k[2]*(s+2*ny*ny)+2*k[3]*nx*ny)+c.camera_matrix[1][2];
    Eigen::Matrix2d D;
    D << L+nx*Lx+2*k[2]*ny+6*k[3]*nx, nx*Ly+2*k[2]*nx+2*k[3]*ny,
      ny*Lx+2*k[2]*nx+2*k[3]*ny, L+ny*Ly+6*k[2]*ny+2*k[3]*nx;
    D.row(0)*=c.camera_matrix[0][0]; D.row(1)*=c.camera_matrix[1][1];
    Eigen::Matrix<double,2,3> N;
    N << 1/q.z(),0,-nx/q.z(), 0,1/q.z(),-ny/q.z();
    Eigen::Matrix<double,3,11> B=Eigen::Matrix<double,3,11>::Zero();
    B.leftCols<3>().setIdentity();
    B.col(6)=Eigen::Vector3d::UnitZ().cross(b-x.head<3>());
    const int a=id/2;
    if(a==0) B.col(8)=u; if(a==2) B.col(8)=-u;
    if(a==1) B.col(9)=v; if(a==3) B.col(9)=-v;
    if(a%2) B(2,10)=1;
    out.jacobian.middleRows<2>(2*e)=D*N*T*B;
  }
  if(!out.value.allFinite() || !out.jacobian.allFinite()) return std::nullopt;
  return out;
}
}
