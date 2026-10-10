// SDK regression: a weighted mean must lie between its two scalar samples.
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include "rovio/ImuPrediction.hpp"
using State=rovio::FilterState<1,4,8,1,0>;
struct Prediction:rovio::ImuPrediction<State> {
  double captured=0;
  void preProcess(State&,const mtMeas& m,double) override {
    captured=m.template get<mtMeas::_gyr>()[0];
  }
};
int main() {
  auto s=std::make_unique<State>(); s->t_=0;
  s->state_.CfP(0).camID_=-1;
  Prediction p; Prediction::mtMeas a,b;
  a.setIdentity(); b.setIdentity();
  a.template get<Prediction::mtMeas::_acc>()=V3D(0,0,9.81);
  b.template get<Prediction::mtMeas::_acc>()=V3D(0,0,9.81);
  b.template get<Prediction::mtMeas::_gyr>()=V3D(1,0,0);
  std::map<double,Prediction::mtMeas> measurements{{.005,a},{.010,b}};
  p.predictMergedEKF(*s,.010,measurements);
  std::cout<<"merged gyro x="<<p.captured<<", independent weighted mean=0.5\n";
  if(std::abs(p.captured-.5)>1e-12)return 2;
  // Partial last interval: 5ms at 0 plus 2.5ms at 1 gives 1/3.
  s->t_=0; p.predictMergedEKF(*s,.0075,measurements);
  std::cout<<"partial interval merged="<<p.captured<<", expected="<<1./3<<'\n';
  return std::abs(p.captured-1./3)>1e-12 ? 2 : 0;
}
