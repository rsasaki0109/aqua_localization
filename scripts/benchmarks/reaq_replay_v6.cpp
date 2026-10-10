// Sensor-only, explicitly configured ReAqROVIO mono/depth ablation adapter.
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <limits>
#include <cmath>
#include <vector>
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <boost/property_tree/info_parser.hpp>
#include "rovio/RovioFilter.hpp"
#include "reaq_pressure_events.hpp"

using Filter = rovio::RovioFilter<rovio::FilterState<25,4,8,1,0>>;
using Tree = boost::property_tree::ptree;
template<class T> T read(std::istream& in) {
  T value{}; if (!in.read(reinterpret_cast<char*>(&value),sizeof(value)))
    throw std::runtime_error("truncated wire input");
  return value;
}
void merge(Tree& dst,const Tree& src,const std::string& prefix="") {
  for (const auto& entry:src) {
    auto path=prefix.empty()?entry.first:prefix+"."+entry.first;
    if (entry.second.empty()) dst.put(path,entry.second.data());
    else merge(dst,entry.second,path);
  }
}
int main(int argc,char** argv) try {
  if (argc!=7) throw std::runtime_error("wire config cam0 output depth(0/1) resolved-info");
  const bool depth=std::string(argv[5])=="1";
  if (!depth && std::string(argv[5])!="0") throw std::runtime_error("invalid ablation arm");
  cv::setNumThreads(1);
  auto f=std::make_unique<Filter>();
  auto& baro=std::get<3>(f->mUpdates_);
  baro.IrIW_.setZero(); baro.MrMV_.setZero();
  baro.defaultUpdnoiP_=Eigen::Matrix3d::Identity()*0.01;
  Tree config,publisher; f->buildPropertyTree(config);
  boost::property_tree::read_info(argv[2],publisher); merge(config,publisher);
  config.put("Camera0.CalibrationFile",argv[3]);
  config.put("ImgUpdate.doFrameVisualisation",false);
  config.put("ImgUpdate.visualizePatches",false);
  f->readPropertyTree(config); f->refreshProperties();
  for (auto& entry:f->subHandlers_) entry.second->refreshProperties();
  f->setRefractiveIndex(1.33);
  // The complete resolved profile is persisted before any estimator trial.
  boost::property_tree::write_info(argv[6],config);
  if (std::string(argv[1])=="--profile-only") return 0;
  std::ifstream in(argv[1],std::ios::binary);
  char magic[8]; if(!in.read(magic,8) || std::string(magic,8)!="AQMONO01")
    throw std::runtime_error("invalid wire magic");
  const auto origin=read<int64_t>(in), last_imu=read<int64_t>(in);
  std::ofstream poses(std::string(argv[4])+".tum"), events(std::string(argv[4])+".events.tsv");
  if(!poses || !events) throw std::runtime_error("output open failed");
  poses<<std::setprecision(17);
  events<<"type\tordinal\tnative_effective_ns\tdisposition\tsafe_ns\tfeatures\n";
  using Prediction=Filter::mtPrediction::mtMeas;
  using Image=std::tuple_element<0,Filter::mtUpdates>::type::mtMeas;
  auto clahe=cv::createCLAHE(3.2,cv::Size(8,8));
  aqua_benchmark::ReaqPressureEvents pressure(2660,241);
  std::map<double,int64_t> clock;
  bool initialized=false; int64_t previous_pose=-1;
  uint64_t images=0, tracked=0, queued_pressure=0, processed_images=0, processed_pressure=0;
  while(in.peek()!=EOF) {
    const auto kind=read<char>(in); const auto stamp=read<int64_t>(in);
    const auto ordinal=read<uint64_t>(in);
    const double t=(stamp-origin)*1e-9; clock[t]=stamp;
    std::string disposition;
    if(kind=='I') {
      Prediction m;
      V3D acc,gyr;
      for(int k=0;k<3;k++)acc[k]=read<double>(in);
      for(int k=0;k<3;k++)gyr[k]=read<double>(in);
      if(!acc.allFinite() || !gyr.allFinite())throw std::runtime_error("nonfinite IMU");
      m.template get<Prediction::_acc>()=acc; m.template get<Prediction::_gyr>()=gyr;
      if(!initialized) { f->resetWithAccelerometer(acc,t); initialized=true; }
      else f->addPredictionMeas(m,t);
      continue;
    } else if(kind=='C') {
      const auto w=read<uint32_t>(in),h=read<uint32_t>(in),n=read<uint32_t>(in);
      if(w!=720 || h!=540 || n!=w*h)throw std::runtime_error("image contract changed");
      cv::Mat image(h,w,CV_8UC1);
      if(!in.read(reinterpret_cast<char*>(image.data),n))throw std::runtime_error("truncated image");
      if(!initialized || stamp<origin || stamp>last_imu)throw std::runtime_error("unsupported image in wire");
      if(std::get<0>(f->mUpdates_).histogramEqualize_)clahe->apply(image,image);
      Image m; m.template get<Image::_aux>().reset(t);
      m.template get<Image::_aux>().pyr_[0].computeFromImage(image,true);
      m.template get<Image::_aux>().isValidPyr_[0]=true;
      f->template addUpdateMeas<0>(m,t); ++images;
      disposition="queued_native_cam0";
    } else if(kind=='P') {
      const double raw=read<double>(in);
      auto sink=[&](double z,int64_t ns) {
        if(depth) { rovio::BaroUpdateMeas m; m.pos()=V3D(0,0,z);
          f->template addUpdateMeas<3>(m,(ns-origin)*1e-9); ++queued_pressure; }
      };
      auto event=pressure.push(ordinal,stamp,raw,initialized,origin,last_imu,
                              f->safe_.state_.WrWM()(2),sink);
      disposition=event.disposition;
      if(!depth && disposition=="submitted_pressure_update")disposition="depth_ablation_disabled";
    } else throw std::runtime_error("unknown wire event");
    std::vector<double> pending_images,pending_pressure;
    const double old_safe=f->safe_.t_;
    // clean() intentionally retains one already-processed timeline sentinel.
    // Count only events strictly newer than the previous safe state.
    for(const auto& e:std::get<0>(f->updateTimelineTuple_).measMap_)
      if(e.first>old_safe)pending_images.push_back(e.first);
    for(const auto& e:std::get<3>(f->updateTimelineTuple_).measMap_)
      if(e.first>old_safe)pending_pressure.push_back(e.first);
    // SDK update selection uses a strict upper bound. Include the event at t
    // without changing ANY measurement time; no additional observation exists
    // between t and the next representable relative double.
    const double processing_horizon=std::nextafter(t,std::numeric_limits<double>::infinity());
    f->updateSafe(&processing_horizon);
    processed_images+=std::count_if(pending_images.begin(),pending_images.end(),[&](double epoch){return epoch<=f->safe_.t_;});
    processed_pressure+=std::count_if(pending_pressure.begin(),pending_pressure.end(),[&](double epoch){return epoch<=f->safe_.t_;});
    if(!f->safe_.cov_.allFinite() || !f->safe_.state_.WrWM().allFinite())
      throw std::runtime_error("nonfinite estimator state");
    auto exact=clock.find(f->safe_.t_);
    if(exact==clock.end())throw std::runtime_error("safe timestamp absent from native clock ledger");
    const auto ns=exact->second; const int features=f->safe_.fsm_.getValidCount();
    if(kind=='C' && features>0)++tracked;
    events<<kind<<'\t'<<ordinal<<'\t'<<stamp<<'\t'<<disposition<<'\t'<<ns<<'\t'<<features<<'\n';
    if(ns!=previous_pose) {
      const auto& p=f->safe_.state_.WrWM();
      const auto q=f->safe_.state_.qWM().toImplementation();
      poses<<ns/1000000000<<'.'<<std::setfill('0')<<std::setw(9)<<ns%1000000000<<std::setfill(' ')
           <<' '<<p[0]<<' '<<p[1]<<' '<<p[2]<<' '<<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w()<<'\n';
      previous_pose=ns;
    }
  }
  poses.flush(); events.flush();
  if(processed_images!=images || processed_pressure!=queued_pressure)
    throw std::runtime_error("queued native observation remains unprocessed at prefix end");
  if(!poses || !events || images==0 || tracked==0)throw std::runtime_error("no tracked images or output failure");
  std::cout<<"images="<<images<<" images_with_live_features="<<tracked
           <<" pressure_packets="<<pressure.native_count()<<" queued_pressure="<<queued_pressure
           <<" processed_images="<<processed_images<<" processed_pressure="<<processed_pressure<<" safe_ns="<<previous_pose<<'\n';
  return 0;
} catch(const std::exception& e) { std::cerr<<"FAILED: "<<e.what()<<'\n'; return 2; }
