#include "recovery.hpp"
#include "association.hpp"
#include "projection.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <chrono>
#include <stdexcept>

namespace rmcs_ekf {
RecoverySettings load_recovery_settings(const YAML::Node & node) {
  RecoverySettings s;
  const auto positive=[&](const char * key,double & value,bool zero=false) {
    if(node && node[key]) value=node[key].as<double>();
    if(!std::isfinite(value) || (zero ? value<0 : value<=0))
      throw std::invalid_argument(std::string("invalid recovery.")+key);
  };
  positive("trigger_s",s.trigger_s);positive("window_s",s.window_s);positive("sample_s",s.sample_s);
  positive("expiry_s",s.expiry_s);positive("confirm_s",s.confirm_s);positive("coast_s",s.coast_s);
  positive("lost_s",s.lost_s);positive("max_rms_px",s.max_rms_px);positive("improvement_ratio",s.improvement_ratio);
  positive("identity_distance_m",s.identity_distance_m);positive("future_position_variance",s.future_position_variance);
  positive("covariance_inflation",s.covariance_inflation);positive("max_fit_ms",s.max_fit_ms,true);
  const auto integer=[&](const char * key,int & value,int low,int high) {
    if(node && node[key]) value=node[key].as<int>();
    if(value<low || value>high) throw std::invalid_argument(std::string("invalid recovery.")+key);
  };
  integer("min_frames",s.min_frames,3,8);integer("max_frames",s.max_frames,s.min_frames,8);
  integer("max_seeds",s.max_seeds,1,4);integer("iterations",s.iterations,1,8);
  if(s.window_s>0.15 || s.window_s<s.trigger_s || s.sample_s>s.window_s ||
     s.expiry_s<s.confirm_s || s.confirm_s<0.02 || s.coast_s>s.lost_s ||
     s.improvement_ratio>=1 || s.covariance_inflation<1)
    throw std::invalid_argument("inconsistent recovery settings");
  return s;
}
namespace {
using V = RobotModel::StateVector;
using P = RobotModel::Covariance;
V unpack(const Eigen::VectorXd & y, int j, int n) {
  V x; x.head<8>()=y.segment<8>(8*j); x.tail<3>()=y.tail<3>(); return x;
}
Eigen::MatrixXd scatter(int j,int n) {
  Eigen::MatrixXd a=Eigen::MatrixXd::Zero(11,8*n+3);
  a.block<8,8>(0,8*j).setIdentity(); a.block<3,3>(8,8*n).setIdentity(); return a;
}
bool spd(const Eigen::MatrixXd & a) {
  Eigen::LDLT<Eigen::MatrixXd> d(a);
  return a.allFinite() && d.info()==Eigen::Success && (d.vectorD().array()>1e-12).all();
}
}
void RecoveryManager::reset(double time) {
  const auto recoveries=status_.recovery_count, failures=status_.candidate_failed_count;
  status_={}; status_.recovery_count=recoveries; status_.candidate_failed_count=failures;
  window_.clear(); candidate_.reset(); initialized_=last_detection_=last_accepted_=last_time_=time;
  accepted_frames_=rejected_frames_=0;
}
void RecoveryManager::interrupt() {
  window_.clear(); candidate_.reset(); status_.candidate_active=false;
  status_.health=Health::LOST; status_.reliable=false; status_.event="input_interruption";
}

bool RecoveryManager::fit(const RobotModel & model,double time) {
  const auto started=std::chrono::steady_clock::now();
  const auto over_budget=[&] {return settings_.max_fit_ms>0 &&
    std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count()>settings_.max_fit_ms;};
  const int n=static_cast<int>(window_.size());
  if(n<settings_.min_frames || window_.back().time-window_.front().time<settings_.trigger_s) return false;
  const auto & first=window_.front();
  // This snapshot is at the first exposure but BEFORE its measurement. It contains no window pixels.
  const V mu=first.prior.state_vector();
  P prior=first.prior.covariance();
  Eigen::Matrix<double,11,11> relax=P::Identity(); relax.diagonal().head<8>().array()=2.0;
  prior=(relax*prior*relax).eval();
  if(!spd(prior)) return false;
  const int dim=8*n+3;
  const auto config=model.config();
  if(config.q_accel_xy<=0 || config.q_accel_z<=0 || config.q_alpha<=0) return false;
  std::vector<V> seeds{mu};
  RobotModel pnp=model;
  if(pnp.initialize(first.camera,first.observations)) {
    V seed=pnp.state_vector();
    // PnP provides a nonlinear starting point, never an extra information factor.
    seed.segment<3>(3)=mu.segment<3>(3); seed(7)=mu(7); seed.tail<3>()=mu.tail<3>();
    if((seed.head<3>()-mu.head<3>()).norm()<=settings_.identity_distance_m) {
      seeds.push_back(seed);
      V phase=seed; phase(6)=normalize_angle(seed(6)+M_PI/2); seeds.push_back(phase);
      phase(6)=normalize_angle(seed(6)-M_PI/2); seeds.push_back(phase);
    }
  }
  double best=std::numeric_limits<double>::infinity();
  std::optional<RobotModel> solution;
  std::vector<std::pair<double,RobotModel>> alternatives;
  auto evaluate=[&](const Eigen::VectorXd & y,Eigen::MatrixXd & normal,Eigen::VectorXd & gradient,
                    std::set<int> & observed)->double {
    if(over_budget()) return std::numeric_limits<double>::infinity();
    normal=Eigen::MatrixXd::Zero(dim,dim); gradient=Eigen::VectorXd::Zero(dim);
    double cost=0;
    auto add=[&](const Eigen::VectorXd & r,const Eigen::MatrixXd & j,const Eigen::MatrixXd & cov) {
      Eigen::LDLT<Eigen::MatrixXd> ldlt(cov);
      if(ldlt.info()!=Eigen::Success || (ldlt.vectorD().array()<=0).any()) return false;
      const Eigen::VectorXd wr=ldlt.solve(r);
      normal.noalias()+=j.transpose()*ldlt.solve(j); gradient.noalias()+=j.transpose()*wr;
      cost+=r.dot(wr); return true;
    };
    V r0=unpack(y,0,n)-mu; r0(6)=normalize_angle(r0(6));
    if(!add(r0,scatter(0,n),prior)) return std::numeric_limits<double>::infinity();
    for(int k=0;k<n;++k) {
      const V x=unpack(y,k,n);
      if(!model.valid_state(x)) return std::numeric_limits<double>::infinity();
      if(k) {
        const double dt=window_[k].time-window_[k-1].time;
        const Eigen::Matrix<double,8,8> f=RobotModel::transition(dt).topLeftCorner<8,8>();
        Eigen::Matrix<double,8,1> r=x.head<8>()-f*unpack(y,k-1,n).head<8>();
        r(6)=normalize_angle(r(6));
        Eigen::MatrixXd j=Eigen::MatrixXd::Zero(8,dim);
        j.block<8,8>(0,8*k).setIdentity(); j.block<8,8>(0,8*(k-1))=-f;
        if(!add(r,j,model.process_covariance(dt).topLeftCorner<8,8>())) return std::numeric_limits<double>::infinity();
      }
      const auto & frame=window_[k];
      const auto a=associate_armors(x,frame.prior.covariance(),model.is_large(),frame.camera,frame.observations,config);
      // Unexplained detections can be another same-class vehicle. Do not inherit identity in that case.
      if(a.ambiguous || a.matches.empty() || a.matches.size()!=a.legal_observations.size())
        return std::numeric_limits<double>::infinity();
      for(const auto & match:a.matches) {
        observed.insert(match.armor_id);
        const auto z=armor_measurement(frame.observations[match.observation_index],match.swapped);
        for(int b=0;b<2;++b) {
          const auto h=project_lightbar(x,model.is_large(),frame.camera,2*match.armor_id+b);
          if(!h) return std::numeric_limits<double>::infinity();
          const Eigen::Vector4d r=h->value-z.segment<4>(4*b);
          // Huber IRLS: covariance is a local robust Hessian approximation, inflated below.
          const double norm=r.norm()/std::sqrt(config.noise_observation);
          const double weight=norm>3.0 ? 3.0/norm : 1.0;
          if(!add(r,h->jacobian*scatter(k,n),Eigen::Matrix4d::Identity()*(config.noise_observation/weight)))
            return std::numeric_limits<double>::infinity();
        }
      }
    }
    return cost;
  };
  for(int s=0;s<std::min<int>(settings_.max_seeds,seeds.size());++s) {
    if(over_budget()) break;
    Eigen::VectorXd y(dim); y.tail<3>()=seeds[s].tail<3>();
    for(int k=0;k<n;++k) y.segment<8>(8*k)=(RobotModel::transition(window_[k].time-first.time)*seeds[s]).head<8>();
    Eigen::MatrixXd h; Eigen::VectorXd g; std::set<int> observed;
    double cost=evaluate(y,h,g,observed);
    for(int iteration=0;iteration<settings_.iterations && std::isfinite(cost);++iteration) {
      Eigen::VectorXd scale=h.diagonal().cwiseMax(1e-12).cwiseSqrt().cwiseInverse();
      Eigen::MatrixXd scaled=scale.asDiagonal()*h*scale.asDiagonal();
      scaled.diagonal().array()+=1e-6;
      Eigen::LDLT<Eigen::MatrixXd> ldlt(scaled);
      if(ldlt.info()!=Eigen::Success) break;
    Eigen::VectorXd step=-(scale.asDiagonal()*ldlt.solve(scale.asDiagonal()*g));
      if(!step.allFinite()) break;
      bool accepted=false;
      for(int line=0;line<5;++line) {
        Eigen::VectorXd next=y+std::ldexp(1.0,-line)*step;
        for(int k=0;k<n;++k) next(8*k+6)=normalize_angle(next(8*k+6));
        Eigen::MatrixXd nh; Eigen::VectorXd ng; std::set<int> no;
        const double nc=evaluate(next,nh,ng,no);
        if(nc<cost) { y=next;h=nh;g=ng;cost=nc;observed=no;accepted=true;break; }
      }
      if(!accepted || step.norm()<1e-6) break;
    }
    if(!std::isfinite(cost) || observed.size()<2) continue;
    // Normalize information before conditioning checks; do not hide rank failure with damping.
    Eigen::VectorXd scale=h.diagonal().cwiseMax(1e-12).cwiseSqrt().cwiseInverse();
    Eigen::MatrixXd normalized=scale.asDiagonal()*h*scale.asDiagonal();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(normalized);
    if(spectrum.info()!=Eigen::Success || spectrum.eigenvalues().minCoeff()<1e-9) continue;
    Eigen::LDLT<Eigen::MatrixXd> factor(normalized);
    const Eigen::MatrixXd selector=scatter(n-1,n);
    // Solve the FULL normal matrix against selector columns, then select rows. This is marginal P.
    P covariance=selector*scale.asDiagonal()*factor.solve(scale.asDiagonal()*selector.transpose());
    covariance*=settings_.covariance_inflation;
    covariance=(0.5*(covariance+covariance.transpose())).eval();
    RobotModel fitted=model;
    if(!fitted.import_snapshot(unpack(y,n-1,n),covariance,model.is_large(),window_.back().time-initialized_)) continue;
    // Sampling may omit the current frame. Advance from the actual last fitted exposure.
    if(time>window_.back().time) fitted.predict(time-window_.back().time);
    if((fitted.state_vector().head<3>()-model.state_vector().head<3>()).norm()>settings_.identity_distance_m) continue;
    alternatives.emplace_back(cost,fitted);
    if(cost<best) {best=cost; solution=fitted;}
  }
  if(over_budget()) {++status_.candidate_failed_count;status_.event="fit_budget_exceeded";return false;}
  if(!solution) { ++status_.candidate_failed_count;status_.event="fit_unobservable_or_invalid";return false; }
  // Keep ambiguity explicit: similar-cost physically different modes must not collapse into one P.
  for(const auto & alternative:alternatives) {
    if(alternative.first>best+2.0) continue;
    const auto selected=solution->full(), other=alternative.second.full();
    double geometry_error=0;
    for(const auto & plate:selected) {
      double nearest=std::numeric_limits<double>::infinity();
      for(const auto & b:other) nearest=std::min(nearest,(plate.translation-b.translation).norm());
      geometry_error=std::max(geometry_error,nearest);
    }
    if(geometry_error>0.05 ||
       (solution->state_vector().segment<3>(3)-alternative.second.state_vector().segment<3>(3)).norm()>0.5 ||
       std::abs(solution->state_vector()(7)-alternative.second.state_vector()(7))>1.0) {
      ++status_.candidate_failed_count;status_.identity_ambiguous=true;
      status_.event="fit_ambiguous";return false;
    }
  }
  candidate_=Candidate{*solution,time,time,0,0}; ++status_.candidate_id;
  status_.fit_cost=best;status_.window_begin=first.time;status_.window_end=window_.back().time;
  status_.prior_time=first.time;status_.candidate_time=time;status_.event="candidate_created";
  return true;
}

void RecoveryManager::process(RobotModel & model,const RobotModel & prediction,
 const std::vector<Armor2d> & observations,const CameraFeature & camera,double time,bool identity_consistent) {
  if(!std::isfinite(time) || time<=last_time_) return;
  status_.event.clear(); last_time_=time;
  const auto main_score=associate_armors(prediction.state_vector(),prediction.covariance(),prediction.is_large(),camera,observations,prediction.config());
  const bool detected=!main_score.legal_observations.empty();
  const bool accepted=model.last_report().estimated_quality;
  const bool clear=identity_consistent && !main_score.ambiguous && !main_score.matches.empty() &&
    main_score.matches.size()==main_score.legal_observations.size() && main_score.trimmed_count==0;
  status_.identity_ambiguous=!clear;
  if(detected) last_detection_=time;
  if(accepted && clear) { last_accepted_=time; ++accepted_frames_;rejected_frames_=0; }
  else if(detected) ++rejected_frames_;
  status_.last_detection_age=time-last_detection_;status_.last_accepted_age=time-last_accepted_;
  status_.health=accepted_frames_<2 ? Health::INIT : Health::HEALTHY;
  if(!detected) status_.health=Health::COASTING;
  else if(!accepted || !clear) status_.health=Health::DEGRADED;
  if(time-last_accepted_>settings_.lost_s || model.diverged()) status_.health=Health::LOST;
  if(candidate_) {
    auto & c=*candidate_;
    if(time-c.created>settings_.expiry_s || !identity_consistent) {
      candidate_.reset();++status_.candidate_failed_count;status_.event="candidate_expired_or_identity";
    } else {
      c.model.predict(time-c.time);c.time=time;
      const auto score=associate_armors(c.model.state_vector(),c.model.covariance(),c.model.is_large(),camera,observations,c.model.config());
      const double main_error=main_score.matches.size()==main_score.legal_observations.size() &&
        std::isfinite(main_score.rms) ? main_score.rms : std::numeric_limits<double>::infinity();
      const bool good=detected && !score.ambiguous && !score.matches.empty() && score.trimmed_count==0 &&
        score.matches.size()==main_score.legal_observations.size() &&
        (c.model.state_vector().head<3>()-prediction.state_vector().head<3>()).norm()<=settings_.identity_distance_m &&
        score.rms<settings_.max_rms_px && score.rms<settings_.improvement_ratio*main_error &&
        c.model.future_covariance(0.1).topLeftCorner<3,3>().diagonal().maxCoeff()<settings_.future_position_variance;
      status_.validation_rms=score.rms;
      if(good) { if(!c.good)c.first_good=time; ++c.good; } else c.good=0;
      // Score strictly before this frame's one and only update.
      c.model.correct(observations,{},camera); status_.validation_frames=c.good;status_.candidate_time=time;
      if(good && c.good>=2 && time-c.first_good>=settings_.confirm_s && c.model.diagnostics().frame_update_count>=2 && !c.model.diverged()) {
        model=c.model;candidate_.reset();window_.clear();last_accepted_=time;accepted_frames_=2;rejected_frames_=0;
        ++status_.recovery_count;status_.health=Health::HEALTHY;status_.event="candidate_takeover";
        status_.identity_ambiguous=false;status_.last_accepted_age=0;
      } else if(accepted && clear && main_score.rms<settings_.max_rms_px) {
        candidate_.reset();status_.event="main_recovered";
      }
    }
  }
  while(!window_.empty() && time-window_.front().time>settings_.window_s) window_.pop_front();
  if(detected && identity_consistent && (window_.empty() || time-window_.back().time>=settings_.sample_s)) {
    window_.push_back(Frame{time,prediction,camera,observations});
    while(static_cast<int>(window_.size())>settings_.max_frames)window_.pop_front();
  }
  if(!candidate_ && detected && identity_consistent && rejected_frames_>=settings_.min_frames && time-last_accepted_>=settings_.trigger_s && status_.health!=Health::LOST)
    fit(model,time);
  status_.candidate_active=candidate_.has_value();
  status_.reliable=status_.health==Health::HEALTHY && (clear || status_.event=="candidate_takeover") && time-last_accepted_<=settings_.coast_s &&
    model.future_covariance(0.1).topLeftCorner<3,3>().diagonal().maxCoeff()<settings_.future_position_variance;
}
}
