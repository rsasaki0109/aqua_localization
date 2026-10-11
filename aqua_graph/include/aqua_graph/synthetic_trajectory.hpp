// SPDX-License-Identifier: Apache-2.0
// Analytic underwater survey trajectory with exact IMU / DVL / depth
// measurements, used by unit tests and by the synthetic replay bag generator.
#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

#include "aqua_graph/estimator.hpp"

namespace aqua_graph
{

struct TruthSample
{
  double stamp_s{0.0};
  gtsam::Pose3 pose{};
  gtsam::Vector3 velocity_world{gtsam::Vector3::Zero()};
  gtsam::Vector3 accel_world{gtsam::Vector3::Zero()};
  gtsam::Vector3 omega_body{gtsam::Vector3::Zero()};
};

struct SyntheticImuSample
{
  double stamp_s{0.0};
  gtsam::Vector3 accel_body{gtsam::Vector3::Zero()};
  gtsam::Vector3 gyro_body{gtsam::Vector3::Zero()};
};

struct SyntheticScalarSample
{
  double stamp_s{0.0};
  double value{0.0};
};

struct SyntheticVectorSample
{
  double stamp_s{0.0};
  gtsam::Vector3 value{gtsam::Vector3::Zero()};
};

struct SyntheticSensorConfig
{
  double duration_s{60.0};
  double imu_rate_hz{200.0};
  double dvl_rate_hz{5.0};
  double depth_rate_hz{10.0};
  double gravity_mps2{9.80665};
  double accel_noise_density{0.02};
  double gyro_noise_density{0.002};
  gtsam::imuBias::ConstantBias imu_bias{
    gtsam::Vector3(0.05, -0.03, 0.02), gtsam::Vector3(0.002, -0.001, 0.0015)};
  gtsam::Rot3 body_R_dvl{gtsam::Rot3::Ypr(0.7853981633974483, 0.0, 3.141592653589793)};
  gtsam::Point3 dvl_lever_arm_body{0.2, 0.0, -0.1};
  double dvl_sigma_mps{0.02};
  gtsam::Point3 pressure_lever_arm_body{0.0, 0.0, 0.0};
  double depth_sigma_m{0.02};
  // Downward-looking stereo camera over a seabed point field.
  double camera_rate_hz{10.0};
  VisualConfig camera;  // intrinsics, baseline, body_P_camera
  int image_width{612};
  int image_height{512};
  double seabed_z{-4.0};
  int seabed_points{4000};
  double pixel_sigma{0.5};
  uint32_t seed{42};
};

struct SyntheticStereoFrame
{
  double stamp_s{0.0};
  std::vector<StereoObservation> observations;
};

// Camera looking straight down (optical axis -z_world when the body is
// level), image x along body +y; used by the synthetic tests and replay bags.
gtsam::Pose3 downward_camera_mount();

// Smooth figure-eight survey at ~2 m depth with gentle roll/pitch/yaw motion.
TruthSample synthetic_truth(double stamp_s, double gravity_mps2 = 9.80665);

struct SyntheticDataset
{
  std::vector<TruthSample> truth;  // at IMU rate
  std::vector<SyntheticImuSample> imu;
  std::vector<SyntheticVectorSample> dvl;
  std::vector<SyntheticScalarSample> depth;
  std::vector<SyntheticStereoFrame> stereo;
};

SyntheticDataset make_synthetic_dataset(const SyntheticSensorConfig & config);

}  // namespace aqua_graph
