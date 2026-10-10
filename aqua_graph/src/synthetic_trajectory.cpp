// SPDX-License-Identifier: Apache-2.0
#include "aqua_graph/synthetic_trajectory.hpp"

#include <cmath>

namespace aqua_graph
{

namespace
{

gtsam::Point3 position(double t)
{
  return gtsam::Point3(
    6.0 * std::sin(0.10 * t) + 0.15 * t,
    4.0 * std::sin(0.20 * t),
    -2.0 - 0.5 * std::sin(0.07 * t));
}

gtsam::Vector3 velocity(double t)
{
  return gtsam::Vector3(
    0.6 * std::cos(0.10 * t) + 0.15,
    0.8 * std::cos(0.20 * t),
    -0.035 * std::cos(0.07 * t));
}

gtsam::Vector3 acceleration(double t)
{
  return gtsam::Vector3(
    -0.06 * std::sin(0.10 * t),
    -0.16 * std::sin(0.20 * t),
    0.00245 * std::sin(0.07 * t));
}

gtsam::Rot3 attitude(double t)
{
  const double yaw = 0.9 * std::sin(0.05 * t) + 0.3 * std::sin(0.21 * t);
  const double pitch = 0.06 * std::sin(0.31 * t);
  const double roll = 0.05 * std::sin(0.23 * t + 0.4);
  return gtsam::Rot3::Ypr(yaw, pitch, roll);
}

}  // namespace

TruthSample synthetic_truth(double stamp_s, double gravity_mps2)
{
  (void)gravity_mps2;
  TruthSample s;
  s.stamp_s = stamp_s;
  const gtsam::Rot3 R = attitude(stamp_s);
  s.pose = gtsam::Pose3(R, position(stamp_s));
  s.velocity_world = velocity(stamp_s);
  s.accel_world = acceleration(stamp_s);
  // Body rate from a central difference of the analytic attitude.
  constexpr double h = 1.0e-5;
  const gtsam::Rot3 delta = attitude(stamp_s - h).between(attitude(stamp_s + h));
  const gtsam::Vector3 omega_mid = gtsam::Rot3::Logmap(delta) / (2.0 * h);
  // Logmap(R(t-h)^T R(t+h)) is expressed in the R(t-h) body frame; rotate it
  // into the R(t) body frame (second-order accurate either way).
  s.omega_body = attitude(stamp_s).unrotate(attitude(stamp_s - h).rotate(omega_mid));
  return s;
}

SyntheticDataset make_synthetic_dataset(const SyntheticSensorConfig & config)
{
  SyntheticDataset data;
  std::mt19937 rng(config.seed);
  std::normal_distribution<double> normal(0.0, 1.0);
  auto noise3 = [&](double sigma) {
      return gtsam::Vector3(sigma * normal(rng), sigma * normal(rng), sigma * normal(rng));
    };
  const gtsam::Vector3 gravity_world(0.0, 0.0, -config.gravity_mps2);

  const auto imu_count =
    static_cast<int64_t>(std::floor(config.duration_s * config.imu_rate_hz)) + 1;
  const double imu_dt = 1.0 / config.imu_rate_hz;
  const double accel_sigma = config.accel_noise_density * std::sqrt(config.imu_rate_hz);
  const double gyro_sigma = config.gyro_noise_density * std::sqrt(config.imu_rate_hz);
  for (int64_t i = 0; i < imu_count; ++i) {
    const double t = static_cast<double>(i) * imu_dt;
    const TruthSample truth = synthetic_truth(t, config.gravity_mps2);
    data.truth.push_back(truth);
    SyntheticImuSample imu;
    imu.stamp_s = t;
    imu.accel_body = truth.pose.rotation().unrotate(truth.accel_world - gravity_world) +
      config.imu_bias.accelerometer() + noise3(accel_sigma);
    imu.gyro_body = truth.omega_body + config.imu_bias.gyroscope() + noise3(gyro_sigma);
    data.imu.push_back(imu);
  }

  const auto dvl_count = static_cast<int64_t>(std::floor(config.duration_s * config.dvl_rate_hz));
  for (int64_t i = 1; i <= dvl_count; ++i) {
    const double t = static_cast<double>(i) / config.dvl_rate_hz;
    const TruthSample truth = synthetic_truth(t, config.gravity_mps2);
    const gtsam::Vector3 v_body = truth.pose.rotation().unrotate(truth.velocity_world) +
      truth.omega_body.cross(config.dvl_lever_arm_body);
    SyntheticVectorSample dvl;
    dvl.stamp_s = t;
    dvl.value = config.body_R_dvl.unrotate(v_body) + noise3(config.dvl_sigma_mps);
    data.dvl.push_back(dvl);
  }

  const auto depth_count =
    static_cast<int64_t>(std::floor(config.duration_s * config.depth_rate_hz));
  for (int64_t i = 1; i <= depth_count; ++i) {
    const double t = static_cast<double>(i) / config.depth_rate_hz;
    const TruthSample truth = synthetic_truth(t, config.gravity_mps2);
    SyntheticScalarSample depth;
    depth.stamp_s = t;
    depth.value = -truth.pose.transformFrom(config.pressure_lever_arm_body).z() +
      config.depth_sigma_m * normal(rng);
    data.depth.push_back(depth);
  }
  return data;
}

}  // namespace aqua_graph
