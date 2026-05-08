#include <gtest/gtest.h>

#include "aqua_imu_loc/static_bias_initializer.hpp"

namespace
{

aqua_imu_loc::ImuSample make_sample(
  const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel)
{
  aqua_imu_loc::ImuSample sample;
  sample.angular_velocity = gyro;
  sample.linear_acceleration = accel;
  return sample;
}

aqua_imu_loc::StaticBiasInitializerConfig make_config()
{
  aqua_imu_loc::StaticBiasInitializerConfig config;
  config.window_seconds = 1.0;
  config.gyro_motion_threshold_radps = 0.10;
  config.accel_motion_threshold_mps2 = 0.50;
  config.gravity_mps2 = 9.80665;
  config.minimum_samples = 10;
  return config;
}

}  // namespace

TEST(StaticBiasInitializer, AveragesGyroBiasOverWindow)
{
  aqua_imu_loc::StaticBiasInitializer init;
  init.configure(make_config());

  const Eigen::Vector3d gyro_bias_truth(0.01, -0.02, 0.005);
  const Eigen::Vector3d gravity_in_body(0.0, 0.0, 9.80665);

  aqua_imu_loc::StaticBiasInitializerStatus last_status =
    aqua_imu_loc::StaticBiasInitializerStatus::kAccumulating;
  for (int i = 0; i <= 100; ++i) {
    const double t = static_cast<double>(i) * 0.02;  // 50 Hz
    last_status = init.add_sample(t, make_sample(gyro_bias_truth, gravity_in_body));
    if (last_status == aqua_imu_loc::StaticBiasInitializerStatus::kReady) {
      break;
    }
  }
  EXPECT_EQ(last_status, aqua_imu_loc::StaticBiasInitializerStatus::kReady);
  EXPECT_NEAR(init.gyro_bias().x(), gyro_bias_truth.x(), 1.0e-9);
  EXPECT_NEAR(init.gyro_bias().y(), gyro_bias_truth.y(), 1.0e-9);
  EXPECT_NEAR(init.gyro_bias().z(), gyro_bias_truth.z(), 1.0e-9);
  EXPECT_GE(init.sample_count(), 10u);
}

TEST(StaticBiasInitializer, AbortsWhenGyroExceedsThreshold)
{
  aqua_imu_loc::StaticBiasInitializer init;
  init.configure(make_config());

  EXPECT_EQ(
    init.add_sample(0.0, make_sample(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 9.80665))),
    aqua_imu_loc::StaticBiasInitializerStatus::kAccumulating);

  // Spike beyond gyro_motion_threshold_radps.
  EXPECT_EQ(
    init.add_sample(0.02, make_sample(Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 9.80665))),
    aqua_imu_loc::StaticBiasInitializerStatus::kAborted);

  // Subsequent samples must remain aborted regardless of input.
  EXPECT_EQ(
    init.add_sample(0.04, make_sample(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.80665))),
    aqua_imu_loc::StaticBiasInitializerStatus::kAborted);
}

TEST(StaticBiasInitializer, AbortsWhenAccelMagnitudeFarFromGravity)
{
  aqua_imu_loc::StaticBiasInitializer init;
  init.configure(make_config());

  // |accel| = sqrt(0+0+25) = 5 m/s^2, residual ~4.8 from g = 9.80665, exceeds threshold.
  EXPECT_EQ(
    init.add_sample(0.0, make_sample(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 5.0))),
    aqua_imu_loc::StaticBiasInitializerStatus::kAborted);
}

TEST(StaticBiasInitializer, NeedsMinimumSamplesEvenAfterWindow)
{
  auto config = make_config();
  config.window_seconds = 0.05;
  config.minimum_samples = 200;
  aqua_imu_loc::StaticBiasInitializer init;
  init.configure(config);

  for (int i = 0; i < 30; ++i) {
    const double t = static_cast<double>(i) * 0.02;
    const auto status = init.add_sample(
      t, make_sample(Eigen::Vector3d(0.001, 0.001, 0.001), Eigen::Vector3d(0.0, 0.0, 9.80665)));
    EXPECT_EQ(status, aqua_imu_loc::StaticBiasInitializerStatus::kAccumulating);
  }
}

TEST(StaticBiasInitializer, ResetClearsState)
{
  aqua_imu_loc::StaticBiasInitializer init;
  init.configure(make_config());

  init.add_sample(0.0, make_sample(Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 9.80665)));
  EXPECT_EQ(init.status(), aqua_imu_loc::StaticBiasInitializerStatus::kAborted);

  init.reset();
  EXPECT_EQ(init.status(), aqua_imu_loc::StaticBiasInitializerStatus::kAccumulating);
  EXPECT_EQ(init.sample_count(), 0u);
}

TEST(StaticBiasInitializer, DisableSkipsAccumulation)
{
  aqua_imu_loc::StaticBiasInitializer init;
  init.configure(make_config());
  init.enable(false);

  EXPECT_EQ(init.status(), aqua_imu_loc::StaticBiasInitializerStatus::kDisabled);
  const auto status = init.add_sample(
    0.0, make_sample(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.80665)));
  EXPECT_EQ(status, aqua_imu_loc::StaticBiasInitializerStatus::kDisabled);
  EXPECT_EQ(init.sample_count(), 0u);
}
