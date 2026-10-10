// SDK upper bound is open: processing horizon changes, native stamp does not.
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include "rovio/RovioFilter.hpp"
int main() {
  using Filter=rovio::RovioFilter<rovio::FilterState<1,4,8,1,0>>;
  auto f=std::make_unique<Filter>();
  auto& b=std::get<3>(f->mUpdates_); b.IrIW_.setZero(); b.MrMV_.setZero();
  b.defaultUpdnoiP_.setIdentity(); b.defaultUpdnoiP_*=.01;
  f->resetWithAccelerometer(V3D(0,0,9.81),0);
  using M=Filter::mtPrediction::mtMeas; M imu; imu.setIdentity();
  imu.template get<M::_acc>()=V3D(0,0,9.81);
  for(int i=1;i<=50;i++)f->addPredictionMeas(imu,i*.005);
  const double native_time=.04; rovio::BaroUpdateMeas observation; observation.pos()=V3D(0,0,-.1);
  f->template addUpdateMeas<3>(observation,native_time);
  f->updateSafe(&native_time);
  if(f->safe_.t_!=0 || std::get<3>(f->updateTimelineTuple_).measMap_.size()!=1)return 2;
  const double upper=std::nextafter(native_time,std::numeric_limits<double>::infinity());
  f->updateSafe(&upper);
  if(f->safe_.t_!=native_time || (std::get<3>(f->updateTimelineTuple_).measMap_.upper_bound(f->safe_.t_)!=std::get<3>(f->updateTimelineTuple_).measMap_.end())
      || f->logCountRegUpd_!=1 || !f->safe_.cov_.allFinite())return 3;
  rovio::BaroUpdateMeas tail; tail.pos()=V3D(0,0,-.2);
  const double tail_stamp=.24;
  f->template addUpdateMeas<3>(tail,tail_stamp);
  const double tail_upper=std::nextafter(tail_stamp,std::numeric_limits<double>::infinity());
  f->updateSafe(&tail_upper);
  if(f->safe_.t_>=tail_stamp)return 4; // Native IMU ends at .25: wait has insufficient future support.
  std::get<0>(f->updateTimelineTuple_).maxWaitTime_=0;
  std::get<1>(f->updateTimelineTuple_).maxWaitTime_=0;
  std::get<2>(f->updateTimelineTuple_).maxWaitTime_=0;
  std::get<3>(f->updateTimelineTuple_).maxWaitTime_=0;
  const double native_imu_end=.25;
  const double final_upper=std::nextafter(native_imu_end,std::numeric_limits<double>::infinity());
  f->updateSafe(&final_upper);
  if(f->safe_.t_!=tail_stamp || !f->safe_.cov_.allFinite() || f->logCountRegUpd_!=1)return 5;
  std::cout<<"PASS: native .24 tail processed after .25 IMU end-of-stream, no extrapolation or timestamp change\n";
}
