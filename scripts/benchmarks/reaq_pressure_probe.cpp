// Dependency integration check; no dataset, reference coordinates or accuracy claim.
#include <iostream>
#include <memory>
#include "rovio/RovioFilter.hpp"
#include "reaq_pressure_events.hpp"

int main() {
  using Filter = rovio::RovioFilter<rovio::FilterState<25,4,8,2,0>>;
  auto f = std::make_unique<Filter>();
  // Explicit fixture settings, not an accuracy-tuned dataset configuration.
  // The upstream default BaroUpdate noise is zero and its fixed offsets are
  // not initialized by its constructor; never rely on allocator contents.
  f->init_.cov_.setIdentity();
  auto& baro = std::get<3>(f->mUpdates_);
  baro.IrIW_.setZero();
  baro.MrMV_.setZero();
  baro.defaultUpdnoiP_.setIdentity();
  baro.defaultUpdnoiP_ *= 0.01;
  f->resetWithAccelerometer(V3D(0,0,9.81),0.0);
  using Prediction = Filter::mtPrediction::mtMeas;
  Prediction imu;
  imu.template get<Prediction::_acc>() = V3D(0,0,9.81);
  imu.template get<Prediction::_gyr>() = V3D::Zero();
  // Missing-image wait is 0.1 s upstream. Feed beyond that wait so this
  // pressure-only fixture actually processes updates rather than just queues them.
  for (int i=1;i<=60;++i) f->addPredictionMeas(imu, i*0.005);

  aqua_benchmark::ReaqPressureEvents pressure(2660,241);
  int submitted=0;
  auto sink=[&](double z,int64_t stamp) {
    rovio::BaroUpdateMeas measurement;
    measurement.pos() = V3D(0,0,z);
    f->template addUpdateMeas<3>(measurement,stamp*1e-9);
    ++submitted;
  };
  pressure.push(0,20000000,2660,true,0,100000000,0,sink);
  pressure.push(1,40000000,2684.1,true,0,100000000,0,sink);
  pressure.push(2,60000000,2708.2,true,0,100000000,0,sink);
  f->updateSafe();
  std::cout << "safe_time=" << f->safe_.t_ << " updates=" << f->logCountRegUpd_
            << " finite_covariance=" << f->safe_.cov_.allFinite()
            << " z=" << f->safe_.state_.WrWM()(2) << '\n';
  if (submitted!=2 || pressure.native_count()!=3 ||
      pressure.submitted_updates()!=2 || f->safe_.t_<0.06-1e-10 ||
      !f->safe_.cov_.allFinite() || f->logCountRegUpd_ != 2 ||
      !(f->safe_.state_.WrWM()(2)<-1e-9)) return 2;
  std::cout << "PASS: all 3 pressure events retained, anchor plus 2 SDK BaroUpdate submissions; finite safe covariance at "
            << f->safe_.t_ << ", z=" << f->safe_.state_.WrWM()(2)
            << ". Contract integration only, not accuracy.\n";
}
