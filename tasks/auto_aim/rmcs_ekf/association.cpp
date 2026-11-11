#include "association.hpp"
#include <functional>
#include <algorithm>
#include <cmath>
namespace rmcs_ekf {
Eigen::Matrix<double,8,1> armor_measurement(const Armor2d & a,bool swap) {
  Eigen::Matrix<double,8,1> z;
  const auto l=swap?a.tr:a.tl, lb=swap?a.br:a.bl;
  const auto r=swap?a.tl:a.tr, rb=swap?a.bl:a.br;
  z << l.x,l.y,lb.x,lb.y,r.x,r.y,rb.x,rb.y;
  return z;
}
static bool legal(const Armor2d & a) {
  if(!armor_measurement(a,false).allFinite()) return false;
  const std::array<cv::Point2f,4> p{a.tl,a.tr,a.br,a.bl};
  double sign=0;
  for(int i=0;i<4;++i) {
    const auto d=p[(i+1)%4]-p[i], e=p[(i+2)%4]-p[(i+1)%4];
    double cross=d.x*e.y-d.y*e.x;
    if(cv::norm(d)<1 || std::abs(cross)<1e-3) return false;
    if(i==0) sign=cross; else if(cross*sign<=0) return false;
  }
  return a.tl.y<a.bl.y && a.tr.y<a.br.y && sign>0;
}
AssociationResult associate_armors(const RobotModel::StateVector & x,
  const RobotModel::Covariance & P,bool large,const CameraFeature & camera,
  const std::vector<Armor2d> & observations,const Config & config) {
  AssociationResult result;
  if(!x.allFinite() || !P.allFinite() || !P.isApprox(P.transpose(),1e-10)
    || !valid_camera(camera) || !std::isfinite(config.noise_observation)
    || config.noise_observation<=0) return result;
  Eigen::LDLT<RobotModel::Covariance> prior_factor(P);
  if(prior_factor.info()!=Eigen::Success || (prior_factor.vectorD().array()<=0).any())
    return result;
  for(std::size_t i=0;i<observations.size();++i) {
    if(observations[i].large!=large || !legal(observations[i])) {++result.invalid_count;continue;}
    bool duplicate=false;
    const auto z=armor_measurement(observations[i],false);
    for(auto j:result.legal_observations) {
      const auto other=armor_measurement(observations[j],false);
      for(int a=0;a<4;++a) for(int b=0;b<4;++b)
        if((z.segment<2>(2*a)-other.segment<2>(2*b)).norm()<0.5) duplicate=true;
    }
    if(duplicate) {++result.duplicate_count;continue;}
    if(result.legal_observations.size()>=6) {++result.trimmed_count;continue;}
    result.legal_observations.push_back(i);
  }
  struct Edge { ArmorMatch match; Eigen::Matrix<double,8,1> r; Eigen::Matrix<double,8,11> H; };
  std::array<std::vector<Edge>,4> edges;
  int visible=0;
  RobotSolution geometry;
  geometry.input.center=x.head<3>(); geometry.input.toward=Eigen::Quaterniond(Eigen::AngleAxisd(x[6],Eigen::Vector3d::UnitZ()));
  geometry.input.radius_forward=x[8]; geometry.input.radius_lateral=x[9]; geometry.input.height_lateral=x[10]; geometry.input.large=large;
  const auto plates=geometry.solve_armors();
  for(int a=0;a<4;++a) {
    const auto p=project_lightbar(x,large,camera,2*a), q=project_lightbar(x,large,camera,2*a+1);
    const Eigen::Vector3d face=-(plates[a].orientation*Eigen::Vector3d::UnitX());
    const Eigen::Vector3d ray=(camera.translation-plates[a].translation).normalized();
    if(!p || !q || !ray.allFinite() || face.dot(ray)<=std::cos(config.yaw_full_max)) continue;
    ++visible;
    for(auto i:result.legal_observations) for(bool swap:{false,true}) {
      Edge e; e.match={i,a,swap}; e.r.head<4>()=armor_measurement(observations[i],swap).head<4>()-p->value;
      e.r.tail<4>()=armor_measurement(observations[i],swap).tail<4>()-q->value;
      e.H.topRows<4>()=p->jacobian; e.H.bottomRows<4>()=q->jacobian;
      const double width=(p->value.head<2>()-q->value.head<2>()).norm();
      const double gate=std::max(config.anchor_max_corner_error,config.anchor_max_relative_error*width);
      if(gate>0 && std::sqrt(e.r.squaredNorm()/4)>gate) continue;
      edges[a].push_back(e);
    }
  }
  std::vector<const Edge*> chosen;
  std::function<void(int)> visit=[&](int a) {
    if(a<4) {
      visit(a+1);
      for(const auto & e:edges[a]) {
        bool used=false;for(auto c:chosen) if(c->match.observation_index==e.match.observation_index) used=true;
        if(used) continue;
        chosen.push_back(&e);visit(a+1);chosen.pop_back();
      }
      return;
    }
    const int n=static_cast<int>(chosen.size()), m=8*n;
    double cost=config.association_miss_penalty*(visible-n)
      +config.association_clutter_penalty*(result.legal_observations.size()-n);
    double sq=0;
    if(m) {
      Eigen::MatrixXd H(m,11); Eigen::VectorXd r(m);
      for(int i=0;i<n;++i) {H.middleRows(8*i,8)=chosen[i]->H;r.segment(8*i,8)=chosen[i]->r;}
      const Eigen::MatrixXd S=H*P*H.transpose()+config.noise_observation*Eigen::MatrixXd::Identity(m,m);
      Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
      if(!S.allFinite() || ldlt.info()!=Eigen::Success || (ldlt.vectorD().array()<=0).any()) return;
      const Eigen::VectorXd solved=ldlt.solve(r);
      const double nis=r.dot(solved);
      if(!solved.allFinite() || !std::isfinite(nis) || nis<0) return;
      cost+=0.5*(nis+ldlt.vectorD().array().log().sum()+m*std::log(2*kPi)); sq=r.squaredNorm();
    }
    if(!std::isfinite(cost)) return;
    if(cost<result.cost) {
      result.second_cost=result.cost;result.cost=cost;result.matches.clear();
      for(auto e:chosen) result.matches.push_back(e->match);
      result.rms=n?std::sqrt(sq/(4*n)):std::numeric_limits<double>::quiet_NaN();
      result.missed_count=visible-n;result.clutter_count=result.legal_observations.size()-n;
    } else if(cost<result.second_cost) result.second_cost=cost;
  };
  visit(0);
  result.ambiguous=std::isfinite(result.second_cost) && result.second_cost-result.cost<config.association_ambiguity_gap;
  return result;
}
}
