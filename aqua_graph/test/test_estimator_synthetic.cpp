// SPDX-License-Identifier: Apache-2.0
// End-to-end checks of the tightly-coupled estimator on an analytic
// underwater survey with noisy, biased IMU and noisy DVL / depth.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "aqua_graph/estimator.hpp"
#include "aqua_graph/factors.hpp"
#include "aqua_graph/synthetic_trajectory.hpp"

namespace
{

struct RunResult
{
  std::vector<aqua_graph::EstimatorOutput> outputs;
  double position_rmse_m{0.0};
  double max_depth_error_m{0.0};
  aqua_graph::EstimatorCounters counters;
  gtsam::imuBias::ConstantBias final_bias;
};

aqua_graph::EstimatorConfig config_for(const aqua_graph::SyntheticSensorConfig & sensors)
{
  aqua_graph::EstimatorConfig config;
  config.imu.gravity_mps2 = sensors.gravity_mps2;
  config.imu.accel_noise_density = sensors.accel_noise_density;
  config.imu.gyro_noise_density = sensors.gyro_noise_density;
  config.dvl.body_R_dvl = sensors.body_R_dvl;
  config.dvl.lever_arm_body = sensors.dvl_lever_arm_body;
  config.dvl.velocity_sigma_mps = sensors.dvl_sigma_mps;
  config.depth.lever_arm_body = sensors.pressure_lever_arm_body;
  config.depth.depth_sigma_m = sensors.depth_sigma_m;
  config.visual = sensors.camera;
  config.visual.pixel_sigma = std::max(sensors.pixel_sigma, 0.5);
  return config;
}

aqua_graph::SyntheticSensorConfig sensors_with_camera()
{
  aqua_graph::SyntheticSensorConfig sensors;
  sensors.camera.body_P_camera = aqua_graph::downward_camera_mount();
  return sensors;
}

// Merge all sensor streams in stamp order (IMU first on ties, as the replay
// harness does for equal stamps) and run the estimator.
RunResult run(
  const aqua_graph::SyntheticDataset & data, const aqua_graph::EstimatorConfig & config,
  bool use_dvl, bool use_depth, bool use_stereo = false)
{
  aqua_graph::TightlyCoupledEstimator estimator(config);
  const auto & first = data.truth.front();
  estimator.set_initial_state(
    first.stamp_s, first.pose, first.velocity_world, gtsam::imuBias::ConstantBias());

  RunResult result;
  size_t i_dvl = 0;
  size_t i_depth = 0;
  size_t i_stereo = 0;
  for (const auto & imu : data.imu) {
    for (auto & out : estimator.add_imu(imu.stamp_s, imu.accel_body, imu.gyro_body)) {
      result.outputs.push_back(out);
    }
    while (i_dvl < data.dvl.size() && data.dvl[i_dvl].stamp_s <= imu.stamp_s) {
      if (use_dvl) {
        estimator.add_dvl(data.dvl[i_dvl].stamp_s, data.dvl[i_dvl].value);
      }
      ++i_dvl;
    }
    while (i_depth < data.depth.size() && data.depth[i_depth].stamp_s <= imu.stamp_s) {
      if (use_depth) {
        estimator.add_depth(data.depth[i_depth].stamp_s, data.depth[i_depth].value);
      }
      ++i_depth;
    }
    while (i_stereo < data.stereo.size() && data.stereo[i_stereo].stamp_s <= imu.stamp_s) {
      if (use_stereo) {
        estimator.add_stereo_frame(
          data.stereo[i_stereo].stamp_s, data.stereo[i_stereo].observations);
      }
      ++i_stereo;
    }
  }

  double sq_sum = 0.0;
  for (const auto & out : result.outputs) {
    const auto truth = aqua_graph::synthetic_truth(out.stamp_s);
    const gtsam::Vector3 err = out.pose.translation() - truth.pose.translation();
    sq_sum += err.squaredNorm();
    result.max_depth_error_m = std::max(result.max_depth_error_m, std::abs(err.z()));
  }
  result.position_rmse_m = std::sqrt(sq_sum / static_cast<double>(result.outputs.size()));
  result.counters = estimator.counters();
  result.final_bias = estimator.latest_state()->bias;
  return result;
}

TEST(TightlyCoupledEstimator, DvlDepthImuTracksSurvey)
{
  aqua_graph::SyntheticSensorConfig sensors;
  const auto data = aqua_graph::make_synthetic_dataset(sensors);
  const RunResult result = run(data, config_for(sensors), true, true);

  // 60 s at a 0.1 s grid.
  EXPECT_NEAR(static_cast<double>(result.outputs.size()), 600.0, 2.0);
  // The measurements stamped exactly at the end attach to the final grid
  // state only once another state is created, so at most one stays pending.
  EXPECT_GE(result.counters.dvl_applied + 1, data.dvl.size());
  EXPECT_GE(result.counters.depth_applied + 1, data.depth.size());
  EXPECT_EQ(result.counters.measurements_too_old, 0u);
  RecordProperty("position_rmse_m", std::to_string(result.position_rmse_m));
  std::printf("[aqua_graph] DVL+depth+IMU causal position RMSE: %.4f m (max |dz| %.4f m)\n",
    result.position_rmse_m, result.max_depth_error_m);
  EXPECT_LT(result.position_rmse_m, 0.25);
  EXPECT_LT(result.max_depth_error_m, 0.10);
  // Gyro bias x/y become observable through gravity + DVL.
  const gtsam::Vector3 gyro_err =
    result.final_bias.gyroscope() - sensors.imu_bias.gyroscope();
  EXPECT_LT(std::abs(gyro_err.x()), 1.0e-3);
  EXPECT_LT(std::abs(gyro_err.y()), 1.0e-3);
}

TEST(TightlyCoupledEstimator, WithoutDvlDriftsMuchMore)
{
  aqua_graph::SyntheticSensorConfig sensors;
  sensors.duration_s = 30.0;
  const auto data = aqua_graph::make_synthetic_dataset(sensors);
  const auto config = config_for(sensors);
  const RunResult aided = run(data, config, true, true);
  const RunResult unaided = run(data, config, false, true);
  std::printf("[aqua_graph] 30 s RMSE aided %.4f m vs IMU+depth only %.4f m\n",
    aided.position_rmse_m, unaided.position_rmse_m);
  EXPECT_GT(unaided.position_rmse_m, 5.0 * aided.position_rmse_m);
  // Pressure keeps the aided vertical channel tight. (Without DVL the
  // horizontal velocity error reaches >10 m/s, so the unaided depth error is
  // not a meaningful bound.)
  EXPECT_LT(aided.max_depth_error_m, 0.05);
}

TEST(TightlyCoupledEstimator, StereoLandmarksReduceDrift)
{
  auto sensors = sensors_with_camera();
  sensors.duration_s = 40.0;
  const auto data = aqua_graph::make_synthetic_dataset(sensors);
  const auto config = config_for(sensors);
  const RunResult dvl_only = run(data, config, true, true, false);
  const RunResult with_stereo = run(data, config, true, true, true);
  std::printf("[aqua_graph] 40 s RMSE DVL+depth %.4f m vs +stereo %.4f m (%lu landmarks)\n",
    dvl_only.position_rmse_m, with_stereo.position_rmse_m,
    static_cast<unsigned long>(with_stereo.counters.landmarks_created));
  EXPECT_GT(with_stereo.counters.landmarks_created, 100u);
  EXPECT_GT(with_stereo.counters.stereo_observations, 1000u);
  EXPECT_LT(with_stereo.position_rmse_m, 0.7 * dvl_only.position_rmse_m);
}

TEST(TightlyCoupledEstimator, RepeatedRunsAreBitIdentical)
{
  auto sensors = sensors_with_camera();
  sensors.duration_s = 20.0;
  const auto data = aqua_graph::make_synthetic_dataset(sensors);
  const auto config = config_for(sensors);
  const RunResult a = run(data, config, true, true, true);
  const RunResult b = run(data, config, true, true, true);
  ASSERT_EQ(a.outputs.size(), b.outputs.size());
  for (size_t i = 0; i < a.outputs.size(); ++i) {
    const gtsam::Matrix4 ma = a.outputs[i].pose.matrix();
    const gtsam::Matrix4 mb = b.outputs[i].pose.matrix();
    ASSERT_EQ(0, std::memcmp(ma.data(), mb.data(), sizeof(double) * 16)) << "state " << i;
  }
}

TEST(TightlyCoupledEstimator, AccelerometerInitializationLevelsAttitude)
{
  aqua_graph::EstimatorConfig config;
  config.init.static_window_s = 0.5;
  aqua_graph::TightlyCoupledEstimator estimator(config);
  // Vehicle at rest, pitched up 0.1 rad and rolled -0.05 rad.
  const gtsam::Rot3 R = gtsam::Rot3::Ypr(0.4, 0.1, -0.05);
  const gtsam::Vector3 f = R.unrotate(gtsam::Vector3(0.0, 0.0, config.imu.gravity_mps2));
  const gtsam::Vector3 gyro_bias(0.003, -0.002, 0.001);
  for (int i = 0; i <= 300; ++i) {
    estimator.add_imu(i * 0.005, f, gyro_bias);
  }
  ASSERT_TRUE(estimator.initialized());
  const auto state = estimator.latest_state();
  ASSERT_TRUE(state.has_value());
  const gtsam::Vector3 ypr = state->pose.rotation().ypr();
  EXPECT_NEAR(ypr.x(), 0.0, 1.0e-9);  // yaw is not observable at rest: zeroed
  EXPECT_NEAR(ypr.y(), 0.1, 1.0e-3);
  EXPECT_NEAR(ypr.z(), -0.05, 1.0e-3);
  // Static: stays put.
  EXPECT_LT(state->pose.translation().norm(), 1.0e-3);
}

TEST(TightlyCoupledEstimator, AccelerometerInitializationWaitsForHeading)
{
  aqua_graph::EstimatorConfig config;
  config.init.static_window_s = 0.5;
  config.heading.enabled = true;
  aqua_graph::TightlyCoupledEstimator estimator(config);
  const gtsam::Rot3 R = gtsam::Rot3::Ypr(0.4, 0.1, -0.05);
  const gtsam::Vector3 f = R.unrotate(gtsam::Vector3(0.0, 0.0, config.imu.gravity_mps2));
  for (int i = 0; i <= 200; ++i) {
    estimator.add_imu(i * 0.005, f, gtsam::Vector3::Zero());
  }
  // Static window complete, but no heading yet.
  EXPECT_FALSE(estimator.initialized());
  estimator.add_heading(1.0, 1.2);
  for (int i = 201; i <= 300; ++i) {
    estimator.add_imu(i * 0.005, f, gtsam::Vector3::Zero());
  }
  ASSERT_TRUE(estimator.initialized());
  const auto state = estimator.latest_state();
  ASSERT_TRUE(state.has_value());
  EXPECT_NEAR(
    aqua_graph::heading_of_axis(state->pose.rotation(), gtsam::Vector3::UnitX()), 1.2, 1.0e-3);
  EXPECT_NEAR(state->pose.rotation().pitch(), 0.1, 1.0e-3);
}

TEST(TightlyCoupledEstimator, HeadingFactorCorrectsInitialYawError)
{
  aqua_graph::SyntheticSensorConfig sensors;
  const auto data = aqua_graph::make_synthetic_dataset(sensors);
  auto final_yaw_error = [&](bool use_heading) {
      auto config = config_for(sensors);
      config.init.yaw_sigma_rad = 0.2;
      config.heading.enabled = use_heading;
      config.heading.align_initial_yaw = false;
      config.heading.sigma_rad = 0.005;
      aqua_graph::TightlyCoupledEstimator estimator(config);
      const auto & first = data.truth.front();
      // Start 0.1 rad off in yaw.
      const gtsam::Pose3 start(
        gtsam::Rot3::Rz(0.1) * first.pose.rotation(), first.pose.translation());
      estimator.set_initial_state(
        first.stamp_s, start, first.velocity_world, gtsam::imuBias::ConstantBias());
      size_t i_dvl = 0;
      double next_heading_s = first.stamp_s;
      for (const auto & imu : data.imu) {
        estimator.add_imu(imu.stamp_s, imu.accel_body, imu.gyro_body);
        while (i_dvl < data.dvl.size() && data.dvl[i_dvl].stamp_s <= imu.stamp_s) {
          estimator.add_dvl(data.dvl[i_dvl].stamp_s, data.dvl[i_dvl].value);
          ++i_dvl;
        }
        if (use_heading && imu.stamp_s >= next_heading_s) {
          const auto truth = aqua_graph::synthetic_truth(imu.stamp_s);
          estimator.add_heading(
            imu.stamp_s,
            aqua_graph::heading_of_axis(truth.pose.rotation(), gtsam::Vector3::UnitX()));
          next_heading_s += 0.2;
        }
      }
      const auto state = estimator.latest_state();
      const auto truth = aqua_graph::synthetic_truth(state->stamp_s);
      if (use_heading) {
        EXPECT_GT(estimator.counters().heading_applied, 250u);
      }
      return std::abs(
        gtsam::Rot3::Logmap(truth.pose.rotation().between(state->pose.rotation())).z());
    };
  const double without = final_yaw_error(false);
  const double with = final_yaw_error(true);
  std::printf("[aqua_graph] final yaw error: %.4f rad without heading, %.4f with\n",
    without, with);
  EXPECT_GT(without, 0.05);
  EXPECT_LT(with, 0.01);
}

}  // namespace
