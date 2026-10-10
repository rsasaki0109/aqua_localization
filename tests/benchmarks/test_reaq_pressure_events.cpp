#include "reaq_pressure_events.hpp"
#include <limits>
#include <iostream>
#include <vector>

using aqua_benchmark::ReaqPressureEvents;
void check(bool ok) { if (!ok) throw std::runtime_error("pressure contract failed"); }
template<class F> void rejects(F f) {
  bool caught=false; try { f(); } catch (const std::invalid_argument&) { caught=true; }
  check(caught);
}
int main() {
  std::vector<std::pair<double,int64_t>> observed;
  auto sink=[&](double z,int64_t ns){observed.emplace_back(z,ns);};
  ReaqPressureEvents p(2660,241);
  check(p.push(0,0,2660,true,1,100,4,sink).disposition=="outside_available_imu_support");
  check(p.push(1,1,2660,false,1,100,4,sink).disposition=="estimator_not_initialized");
  check(p.push(2,2,std::numeric_limits<double>::quiet_NaN(),true,1,100,4,sink).disposition=="nonfinite_raw_pressure");
  check(p.push(3,3,2660,true,1,100,4,sink).disposition=="initial_pressure_anchor_no_update");
  check(p.push(4,4,2901,true,1,100,999,sink).disposition=="submitted_pressure_update");
  check(observed.size()==1 && observed[0].first==3 && observed[0].second==4);
  check(p.push(5,100,2419,true,1,100,-999,sink).disposition=="submitted_pressure_update");
  check(observed[1].first==5 && observed[1].second==100);
  check(p.native_count()==6 && p.submitted_updates()==2);
  rejects([&]{p.push(7,101,2660,true,1,100,4,sink);});
  rejects([&]{p.push(6,100,2660,true,1,100,4,sink);});
  check(p.native_count()==6); // hard failures do not impersonate accepted packets
  rejects([]{ReaqPressureEvents bad(2660,0);});
  rejects([]{ReaqPressureEvents bad(2660,std::numeric_limits<double>::infinity());});
  ReaqPressureEvents overflow(0,1e-300);
  check(overflow.push(0,1,1e300,true,0,2,0,sink).disposition=="nonfinite_converted_depth");
  ReaqPressureEvents invalid_anchor(2660,241);
  rejects([&]{invalid_anchor.push(0,1,2660,true,0,2,std::numeric_limits<double>::quiet_NaN(),sink);});
  std::cout << "PASS pressure support/initialization, sign/units, fixed anchor, exact epoch, all counts, ordinal/duplicate-clock rejection and finite gates\n";
}
