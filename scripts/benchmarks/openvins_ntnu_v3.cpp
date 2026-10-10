// Standalone pinned OpenVINS baseline on the independently verified AQMONO01 wire.
#include "core/VioManager.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "cam/CamEqui.h"
#include "utils/quat_ops.h"
#include <opencv2/core.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <cstdint>
#include <stdexcept>

template<class T> T take(std::istream& in) {
  T x{}; if(!in.read(reinterpret_cast<char*>(&x),sizeof(x)))throw std::runtime_error("truncated input");
  return x;
}
ov_msckf::VioManagerOptions options(const char* path) {
  cv::FileStorage config(path,cv::FileStorage::READ|cv::FileStorage::FORMAT_JSON);
  if(!config.isOpened())throw std::runtime_error("configuration missing");
  ov_msckf::VioManagerOptions o;
  o.use_aruco=false; o.use_stereo=false; o.num_opencv_threads=1;
  o.use_multi_threading_pubs=false; o.use_multi_threading_subs=false;
  o.init_options.init_dyn_use=true; o.init_options.init_dyn_min_deg=10;
  o.init_options.init_dyn_mle_max_threads=1; o.init_options.init_dyn_mle_max_time=5; o.init_options.init_dyn_mle_max_iter=1000;
  o.gravity_mag=double(config["gravity"]); o.init_options.gravity_mag=o.gravity_mag;
  o.calib_camimu_dt=double(config["additional_camera_offset_s"]);
  o.vec_dw<<1,0,0,1,0,1; o.vec_da=o.vec_dw; o.vec_tg.setZero();
  o.q_ACCtoIMU<<0,0,0,1; o.q_GYROtoIMU=o.q_ACCtoIMU;
  o.imu_noises.sigma_w=double(config["gyro_noise"]);
  o.imu_noises.sigma_a=double(config["acc_noise"]);
  o.imu_noises.sigma_wb=double(config["gyro_bias_noise"]);
  o.imu_noises.sigma_ab=double(config["acc_bias_noise"]);
  o.imu_noises.sigma_w_2=std::pow(o.imu_noises.sigma_w,2);
  o.imu_noises.sigma_a_2=std::pow(o.imu_noises.sigma_a,2);
  o.imu_noises.sigma_wb_2=std::pow(o.imu_noises.sigma_wb,2);
  o.imu_noises.sigma_ab_2=std::pow(o.imu_noises.sigma_ab,2);
  o.init_options.sigma_w=o.imu_noises.sigma_w; o.init_options.sigma_a=o.imu_noises.sigma_a;
  o.init_options.sigma_wb=o.imu_noises.sigma_wb; o.init_options.sigma_ab=o.imu_noises.sigma_ab;
  if(config["intrinsics"].size()!=8 || config["R_imu_to_camera"].size()!=9 || config["p_imu_in_camera"].size()!=3)
    throw std::runtime_error("calibration dimensions changed");
  Eigen::VectorXd k(8); for(int i=0;i<8;++i)k(i)=double(config["intrinsics"][i]);
  auto camera=std::make_shared<ov_core::CamEqui>(int(config["width"]),int(config["height"]));
  camera->set_value(k); o.camera_intrinsics[0]=camera;
  Eigen::Matrix3d R; for(int i=0;i<9;++i)R(i/3,i%3)=double(config["R_imu_to_camera"][i]);
  if(!R.allFinite() || (R*R.transpose()-Eigen::Matrix3d::Identity()).norm()>1e-8 || std::abs(R.determinant()-1)>1e-8)
    throw std::runtime_error("invalid calibrated rotation");
  Eigen::VectorXd ext(7); ext.head<4>()=ov_core::rot_2_quat(R);
  for(int i=0;i<3;++i)ext(i+4)=double(config["p_imu_in_camera"][i]);
  o.camera_extrinsics[0]=ext;
  o.init_options.camera_intrinsics=o.camera_intrinsics; o.init_options.camera_extrinsics=o.camera_extrinsics;
  return o;
}
int main(int argc,char** argv) try {
  if(argc==3 && std::string(argv[1])=="--describe") {
    auto o=options(argv[2]); ov_core::Printer::setPrintLevel("DEBUG"); o.print_and_load();
    const Eigen::Vector4d q=o.camera_extrinsics.at(0).head<4>();
    const Eigen::Matrix3d R=ov_core::quat_2_Rot(q);
    auto k=o.camera_intrinsics.at(0)->get_value();
    std::cout<<std::setprecision(17)<<"CALIBRATION_JSON={\"intrinsics\":[";
    for(int i=0;i<8;++i)std::cout<<(i?",":"")<<k(i);
    std::cout<<"],\"R_imu_to_camera\":[";
    for(int i=0;i<9;++i)std::cout<<(i?",":"")<<R(i/3,i%3);
    std::cout<<"],\"p_imu_in_camera\":[";
    for(int i=0;i<3;++i)std::cout<<(i?",":"")<<o.camera_extrinsics.at(0)(i+4);
    std::cout<<"],\"noise\":["<<o.imu_noises.sigma_w<<','<<o.imu_noises.sigma_a<<','<<o.imu_noises.sigma_wb<<','<<o.imu_noises.sigma_ab
      <<"],\"camera_offset\":"<<o.calib_camimu_dt<<"}\n";
    return 0;
  }
  if(argc!=4)throw std::runtime_error("input.bin fixed.json output-prefix");
  auto o=options(argv[2]); ov_core::Printer::setPrintLevel("DEBUG"); o.print_and_load();
  ov_core::Printer::setPrintLevel("INFO");
  cv::setNumThreads(1); cv::setRNGSeed(0); std::srand(0);
  ov_msckf::VioManager manager(o);
  std::ifstream in(argv[1],std::ios::binary);char magic[8];
  if(!in.read(magic,8)||std::string(magic,8)!="AQMONO01")throw std::runtime_error("wire identity");
  const auto origin=take<int64_t>(in),last=take<int64_t>(in);
  std::ofstream poses(std::string(argv[3])+".tum"),events(std::string(argv[3])+".events.tsv");
  if(!poses||!events)throw std::runtime_error("outputs missing");
  events<<"type\tordinal\tnative_effective_ns\tdisposition\n"; poses<<std::setprecision(17);
  uint64_t ni=0,nc=0,np=0,nposes=0;int64_t previous_imu=origin-1,previous_obs=origin-1;
  while(in.peek()!=EOF) {
    const char kind=take<char>(in);const auto ns=take<int64_t>(in);const auto ordinal=take<uint64_t>(in);
    if(kind=='I') {
      ov_core::ImuData m;m.timestamp=(ns-origin)*1e-9;
      for(int i=0;i<3;++i)m.am(i)=take<double>(in);
      for(int i=0;i<3;++i)m.wm(i)=take<double>(in);
      if(ordinal!=ni++||ns<=previous_imu||ns<origin||ns>last||!m.am.allFinite()||!m.wm.allFinite())
        throw std::runtime_error("IMU accounting/value/support changed");
      previous_imu=ns;manager.feed_measurement_imu(m);continue;
    }
    if(ns<previous_obs)throw std::runtime_error("native observation order changed");previous_obs=ns;
    if(kind=='P') {
      const double raw=take<double>(in);
      if(ordinal!=np++||!std::isfinite(raw))throw std::runtime_error("pressure accounting/value changed");
      events<<kind<<'\t'<<ordinal<<'\t'<<ns<<'\t'
        <<(ns<origin||ns>last?"outside_available_imu_support":"native_pressure_retained_no_depth_update")<<'\n';
      continue; // Standard OpenVINS: pressure is not an estimator input.
    }
    if(kind!='C')throw std::runtime_error("unknown wire event");
    const auto w=take<uint32_t>(in),h=take<uint32_t>(in),bytes=take<uint32_t>(in);
    if(ordinal!=nc++||w!=720||h!=540||bytes!=w*h||ns<origin||ns>last||previous_imu<ns)
      throw std::runtime_error("native camera/support accounting changed");
    cv::Mat image(h,w,CV_8UC1);if(!in.read(reinterpret_cast<char*>(image.data),bytes))throw std::runtime_error("truncated image");
    ov_core::CameraData m;m.timestamp=(ns-origin)*1e-9;m.sensor_ids={0};m.images={image};m.masks={cv::Mat::zeros(h,w,CV_8UC1)};
    manager.feed_measurement_camera(m);auto s=manager.get_state();
    if(!manager.initialized()||std::abs(s->_timestamp-m.timestamp)>1e-8) {
      events<<kind<<'\t'<<ordinal<<'\t'<<ns<<"\tinitialization_or_no_state_at_camera\n";continue;
    }
    const Eigen::Vector3d p=s->_imu->pos();const Eigen::Quaterniond q(s->_imu->Rot().transpose());
    if(!p.allFinite()||!q.coeffs().allFinite()||!ov_msckf::StateHelper::get_full_covariance(s).allFinite())
      throw std::runtime_error("nonfinite state/covariance");
    poses<<ns/1000000000<<'.'<<std::setfill('0')<<std::setw(9)<<ns%1000000000<<std::setfill(' ')
      <<' '<<p.x()<<' '<<p.y()<<' '<<p.z()<<' '<<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w()<<'\n';++nposes;
    events<<kind<<'\t'<<ordinal<<'\t'<<ns<<"\testimated_imu_pose\n";
  }
  poses.flush();events.flush();
  if(ni!=63698||nc!=6368||np!=15926||previous_imu!=last||!nposes||!poses||!events)
    throw std::runtime_error("incomplete full native replay or no initialized trajectory");
  std::cout<<"COMPLETE imu="<<ni<<" camera="<<nc<<" pressure="<<np<<" poses="<<nposes<<'\n';
  return 0;
} catch(const std::exception& e) {std::cerr<<"FAILED: "<<e.what()<<'\n';return 2;}
