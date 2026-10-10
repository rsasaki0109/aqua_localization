// Separately registered reconstruction, not the unavailable legacy runner.
#include "core/VioManager.h"
#include "state/State.h"
#include "state/Propagator.h"
#include "state/StateHelper.h"
#include "cam/CamEqui.h"
#include "utils/sensor_data.h"
#include "utils/quat_ops.h"
#include "openvins_pressure.hpp"
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdint>

template<class T> T read() {
  T value; std::cin.read(reinterpret_cast<char*>(&value), sizeof(value));
  if (!std::cin) throw std::runtime_error("truncated registered sensor stream");
  return value;
}
ov_msckf::VioManagerOptions options() {
  ov_msckf::VioManagerOptions o;
  o.gravity_mag = HydrostaticPressure::gravity;
  o.use_aruco = false; o.num_opencv_threads = 1;
  o.use_multi_threading_pubs = false; o.use_multi_threading_subs = false;
  o.init_options.init_dyn_use = true; o.init_options.init_dyn_min_deg = 10;
  o.init_options.init_dyn_mle_max_threads = 1;
  o.init_options.init_dyn_mle_max_time = 5;
  o.calib_camimu_dt = 0;
  o.vec_dw << 1, 0, 0, 1, 0, 1; o.vec_da = o.vec_dw; o.vec_tg.setZero();
  o.q_ACCtoIMU << 0, 0, 0, 1; o.q_GYROtoIMU = o.q_ACCtoIMU;
  o.imu_noises.sigma_w = .001; o.imu_noises.sigma_a = .02;
  o.imu_noises.sigma_wb = 5e-5; o.imu_noises.sigma_ab = .001;
  o.imu_noises.sigma_w_2 = 1e-6; o.imu_noises.sigma_a_2 = .0004;
  o.imu_noises.sigma_wb_2 = 2.5e-9; o.imu_noises.sigma_ab_2 = 1e-6;
  o.init_options.sigma_w = .001; o.init_options.sigma_a = .02;
  o.init_options.sigma_wb = 5e-5; o.init_options.sigma_ab = .001;
  o.init_options.gravity_mag = HydrostaticPressure::gravity;
  Eigen::VectorXd intrinsics(8);
  intrinsics << 413.32595366566017, 413.70198739483686,
      305.9507483284928, 259.4439948946375, -.06125568297136998,
      -.003796743395135256, .027326634771204592, -.030296403142887066;
  auto camera = std::make_shared<ov_core::CamEqui>(640, 512);
  camera->set_value(intrinsics); o.camera_intrinsics[0] = camera;
  Eigen::Matrix3d R;
  R << -.99978035,.01210101,.01711187,
       .0169654,-.01210461,.9997828,
       .01230552,.99985351,.01189665;
  Eigen::Vector3d p(-.01866921,.02125636,-.1489863);
  Eigen::VectorXd extrinsics(7);
  // Official Kalibr T_cam_imu maps IMU into camera, exactly the SDK convention.
  extrinsics.head<4>() = ov_core::rot_2_quat(R);
  extrinsics.tail<3>() = p;
  o.camera_extrinsics[0] = extrinsics;
  o.init_options.camera_intrinsics = o.camera_intrinsics;
  o.init_options.camera_extrinsics = o.camera_extrinsics;
  return o;
}
int main(int argc, char** argv) {
  try {
    if (argc==2 && std::string(argv[1])=="--describe-calibration") {
      auto o=options(); Eigen::Vector4d q=o.camera_extrinsics.at(0).head<4>();
      Eigen::Matrix3d R=ov_core::quat_2_Rot(q);
      auto k=o.camera_intrinsics.at(0)->get_value();
      std::cout<<std::setprecision(17)<<"{\"intrinsics\":[";
      for (int i=0;i<8;++i) std::cout<<(i?",":"")<<k(i);
      std::cout<<"],\"R_imu_to_camera\":[";
      for (int i=0;i<9;++i) std::cout<<(i?",":"")<<R(i/3,i%3);
      std::cout<<"],\"p_imu_in_camera\":[";
      for (int i=0;i<3;++i) std::cout<<(i?",":"")<<o.camera_extrinsics.at(0)(i+4);
      std::cout<<"],\"imu_noise\":["<<o.imu_noises.sigma_w<<','<<o.imu_noises.sigma_wb<<','
               <<o.imu_noises.sigma_a<<','<<o.imu_noises.sigma_ab
               <<"],\"additional_camera_offset_s\":"<<o.calib_camimu_dt<<"}\n";
      return 0;
    }
    if (argc==2 && std::string(argv[1])=="--self-test") {
      ov_msckf::StateOptions state_options;
      auto s=std::make_shared<ov_msckf::State>(state_options);
      ov_msckf::StateHelper::set_initial_covariance(s,Eigen::MatrixXd::Identity(15,15),{s->_imu});
      s->_timestamp=0;
      ov_msckf::StateHelper::augment_clone(s,Eigen::Vector3d::Zero());
      HydrostaticPressure p; p.update(s,101325);
      const int z=s->_imu->p()->id()+2;
      auto anchored=ov_msckf::StateHelper::get_full_covariance(s);
      if (std::abs(anchored(p.offset->id(),p.offset->id())-1.04)>1e-10 || std::abs(anchored(z,p.offset->id())+1)>1e-10)
        throw std::runtime_error("anchor covariance/cross-covariance incorrect");
      ov_msckf::StateHelper::marginalize(s,s->_clones_IMU.at(0)); s->_clones_IMU.erase(0);
      const int b=p.offset->id();
      if (b!=15 || ov_msckf::StateHelper::get_full_covariance(s).rows()!=16)
        throw std::runtime_error("temporary pressure clone lifecycle corrupted state ordering");
      Eigen::MatrixXd Q=Eigen::MatrixXd::Zero(3,3); Q(2,2)=1;
      ov_msckf::StateHelper::EKFPropagation(s,{s->_imu->p()},{s->_imu->p()},Eigen::MatrixXd::Identity(3,3),Q);
      auto before=ov_msckf::StateHelper::get_full_covariance(s);
      Eigen::VectorXd h=Eigen::VectorXd::Zero(before.rows()); h(z)=h(b)=1;
      Eigen::VectorXd gain=before*h/(h.dot(before*h)+.04);
      Eigen::MatrixXd expected=before-gain*(h.transpose()*before);
      p.update(s,101325+HydrostaticPressure::density*HydrostaticPressure::gravity);
      if (std::abs(s->_imu->pos()(2)+gain(z))>1e-10 ||
          (ov_msckf::StateHelper::get_full_covariance(s)-expected).norm()>1e-10)
        throw std::runtime_error("pressure sign or full-state Gaussian update incorrect");
      bool rejected=false; try { p.update(s,1013.25); } catch(const std::runtime_error&) { rejected=true; }
      if (!rejected) throw std::runtime_error("unregistered pressure units accepted");
      std::cout<<"PASS anchor uncertainty, cross-covariance, pressure sign, full covariance, Pa gate\n";
      return 0;
    }
    if (argc != 4) throw std::runtime_error("usage: runner vio|pressure trajectory.csv events.csv");
    bool pressure = std::string(argv[1]) == "pressure";
    if (!pressure && std::string(argv[1]) != "vio") throw std::runtime_error("unknown arm");
    char magic[8]; std::cin.read(magic, 8);
    if (!std::cin || std::string(magic, 8) != "AQUAVIO3") throw std::runtime_error("wrong sensor stream version");
    auto origin = read<int64_t>(); auto count = read<uint32_t>();
    std::vector<int64_t> stamps; std::vector<ov_core::ImuData> imus;
    for (uint32_t i=0; i<count; ++i) {
      auto ns=read<int64_t>(); ov_core::ImuData m;
      m.timestamp=double(ns-origin)*1e-9;
      for (int j=0;j<3;++j) m.wm(j)=read<double>();
      for (int j=0;j<3;++j) m.am(j)=read<double>();
      if (!m.wm.allFinite() || !m.am.allFinite() || (i && ns<=stamps.back()))
        throw std::runtime_error("invalid native IMU sequence");
      stamps.push_back(ns); imus.push_back(m);
    }
    if (count<2 || stamps.front()!=origin) throw std::runtime_error("invalid IMU support");
    cv::setRNGSeed(0); std::srand(0);
    auto configuration=options();
    ov_msckf::VioManager manager(configuration); HydrostaticPressure model;
    std::ofstream trajectory(argv[2]), events(argv[3]);
    if (!trajectory || !events) throw std::runtime_error("cannot open trial outputs");
    trajectory << "timestamp_ns,native_camera_ordinal,x,y,z,qx,qy,qz,qw\n" << std::setprecision(17);
    events << "sensor,native_ordinal,timestamp_ns,status\n";
    size_t nextimu=0, output_count=0; int64_t previous=origin;
    while (true) {
      int type=std::cin.get();
      if (type==EOF) { if (!std::cin.eof()) throw std::runtime_error("stream read failed"); break; }
      auto ns=read<int64_t>(); auto ordinal=read<uint32_t>();
      if (ns<previous || ns<stamps.front() || ns>stamps.back())
        throw std::runtime_error("observation outside registered native order/support");
      previous=ns;
      // Feed through the first IMU at or after the observation, never extrapolate.
      while (nextimu<imus.size() && (nextimu==0 || stamps[nextimu-1]<ns))
        manager.feed_measurement_imu(imus[nextimu++]);
      const double t=double(ns-origin)*1e-9;
      if (type=='C') {
        auto width=read<uint32_t>(), height=read<uint32_t>(), bytes=read<uint32_t>();
        if (width!=640 || height!=512 || bytes!=width*height)
          throw std::runtime_error("registered mono8 camera geometry violated");
        cv::Mat image(height,width,CV_8UC1); std::cin.read((char*)image.data, bytes);
        if (!std::cin) throw std::runtime_error("truncated image");
        ov_core::CameraData camera; camera.timestamp=t; camera.sensor_ids={0};
        camera.images={image}; camera.masks={cv::Mat::zeros(height,width,CV_8UC1)};
        manager.feed_measurement_camera(camera);
        auto state=manager.get_state();
        if (!manager.initialized() || std::abs(state->_timestamp-t)>1e-8) {
          events << "camera,"<<ordinal<<','<<ns<<",initialization_or_no_state_at_camera\n"; continue;
        }
        auto extrinsic=state->_calib_IMUtoCAM.at(0);
        Eigen::Vector3d pc=-extrinsic->Rot().transpose()*extrinsic->pos();
        Eigen::Vector3d world=state->_imu->pos()+state->_imu->Rot().transpose()*pc;
        Eigen::Matrix3d Rcg=state->_calib_IMUtoCAM.at(0)->Rot()*state->_imu->Rot();
        Eigen::Quaterniond q(Rcg.transpose());
        if (!world.allFinite() || !q.coeffs().allFinite()) throw std::runtime_error("nonfinite estimate");
        trajectory<<ns<<','<<ordinal<<','<<world(0)<<','<<world(1)<<','<<world(2)<<','
          <<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<'\n'; ++output_count;
        events<<"camera,"<<ordinal<<','<<ns<<",estimated\n";
      } else if (type=='P') {
        const double pa=read<double>();
        const bool valid_pressure=HydrostaticPressure::valid(pa);
        if (!manager.initialized()) { events<<"pressure,"<<ordinal<<','<<ns<<",pre_initialization\n"; continue; }
        auto state=manager.get_state(); bool temporary=t>state->_timestamp;
        if (t<state->_timestamp) throw std::runtime_error("pressure precedes current estimator state");
        // Both arms share this propagation schedule; baseline ignores the value.
        if (temporary) manager.get_propagator()->propagate_and_clone(state,t);
        if (pressure && valid_pressure) model.update(state,pa);
        if (temporary) {
          auto clone=state->_clones_IMU.at(t);
          ov_msckf::StateHelper::marginalize(state,clone); state->_clones_IMU.erase(t);
        }
        events<<"pressure,"<<ordinal<<','<<ns<<','
          <<(!valid_pressure?"invalid_pressure_measurement_excluded":pressure?"updated":"value_ignored")<<'\n';
      } else throw std::runtime_error("unknown sensor record");
    }
    trajectory.flush(); events.flush();
    if (!trajectory || !events) throw std::runtime_error("trial output write failed");
    if (!output_count) throw std::runtime_error("no initialized camera trajectory; trial failed");
    std::cerr<<"AQUA_OUTPUT_COUNT="<<output_count<<" INIT_TIME="<<manager.initialized_time()
             <<" IMU_FED="<<nextimu<<" NATIVE_IMU_COUNT="<<count<<'\n';
    return 0;
  } catch(const std::exception& e) { std::cerr<<"AQUA_FAILURE: "<<e.what()<<'\n'; return 2; }
}
