#include <vector>

#include <gtest/gtest.h>

#include "aqua_imu_loc/additive_ukf.hpp"

namespace
{

std::vector<double> diagonal(double value)
{
  return std::vector<double>(static_cast<size_t>(aqua_imu_loc::kStateDim), value);
}

TEST(AdditiveUkf, NormalizesAnglesWhenStateIsSet)
{
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);

  aqua_imu_loc::StateVector state = aqua_imu_loc::StateVector::Zero();
  state(8) = 4.0 * aqua_imu_loc::kPi;
  ukf.set_state(state);

  EXPECT_NEAR(ukf.state()(8), 0.0, 1.0e-12);
}

TEST(AdditiveUkf, StationaryImuPredictionStaysNearOrigin)
{
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(1.0e-6));
  ukf.set_process_noise(diagonal(0.0));

  aqua_imu_loc::ImuSample imu;
  aqua_imu_loc::DynamicsParams dynamics;
  dynamics.enable_linear_drag = false;
  imu.linear_acceleration.z() = dynamics.gravity_mps2;

  ukf.predict(0.01, imu, dynamics);

  EXPECT_NEAR(ukf.state().segment<6>(0).norm(), 0.0, 1.0e-6);
}

TEST(AdditiveUkf, DepthUpdateMovesZNegativeForPositiveDepth)
{
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(0.1));
  ukf.set_process_noise(diagonal(0.0));

  ukf.update_depth(2.0, 0.01);

  EXPECT_LT(ukf.state().z(), -1.0);
  EXPECT_NEAR(-ukf.state().z(), 2.0, 0.25);
}

TEST(AdditiveUkf, LinearDragReducesForwardVelocity)
{
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(1.0e-6));
  ukf.set_process_noise(diagonal(0.0));

  aqua_imu_loc::StateVector state = aqua_imu_loc::StateVector::Zero();
  state(3) = 1.0;
  ukf.set_state(state);

  aqua_imu_loc::ImuSample imu;
  aqua_imu_loc::DynamicsParams dynamics;
  dynamics.enable_linear_drag = true;
  dynamics.linear_drag_coeff = 0.2;
  imu.linear_acceleration.z() = dynamics.gravity_mps2;

  ukf.predict(1.0, imu, dynamics);

  EXPECT_LT(ukf.state()(3), 1.0);
  EXPECT_GT(ukf.state()(3), 0.0);
}

TEST(AdditiveUkf, WaterCurrentVelocityAffectsLinearDrag)
{
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(1.0e-6));
  ukf.set_process_noise(diagonal(0.0));

  aqua_imu_loc::ImuSample imu;
  aqua_imu_loc::DynamicsParams dynamics;
  dynamics.enable_linear_drag = true;
  dynamics.linear_drag_coeff = 0.2;
  dynamics.current_velocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  imu.linear_acceleration.z() = dynamics.gravity_mps2;

  ukf.predict(1.0, imu, dynamics);

  EXPECT_GT(ukf.state()(3), 0.0);
  EXPECT_NEAR(ukf.state()(3), 0.2, 0.03);
}

TEST(AdditiveUkf, YawUpdateMovesYawTowardMeasurement)
{
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(0.1));
  ukf.set_process_noise(diagonal(0.0));

  aqua_imu_loc::StateVector state = aqua_imu_loc::StateVector::Zero();
  state(8) = 0.0;
  ukf.set_state(state);

  ukf.update_yaw(1.0, 1.0e-4);

  EXPECT_GT(ukf.state()(8), 0.5);
  EXPECT_NEAR(ukf.state()(8), 1.0, 0.05);
}

TEST(AdditiveUkf, YawUpdateHandlesAngleWrapAcrossPi)
{
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(0.05));
  ukf.set_process_noise(diagonal(0.0));

  aqua_imu_loc::StateVector state = aqua_imu_loc::StateVector::Zero();
  state(8) = aqua_imu_loc::kPi - 0.05;
  ukf.set_state(state);

  // Measurement just past +pi wraps to near -pi; the update should pull yaw the
  // short way (~+0.1 rad) rather than the long way (~-2*pi + 0.1 rad).
  ukf.update_yaw(-aqua_imu_loc::kPi + 0.05, 1.0e-4);

  EXPECT_GT(std::abs(ukf.state()(8)), aqua_imu_loc::kPi - 0.1);
  // Either close to +pi or wrapped to -pi; what matters is short-arc adjustment.
  const double normalized = aqua_imu_loc::normalize_angle(ukf.state()(8));
  // Short arc from (pi - 0.05) to (pi + 0.05) is +0.1 rad; the normalized result
  // should be close to either of those endpoints, not several radians away.
  EXPECT_LT(
    std::abs(aqua_imu_loc::normalize_angle(normalized - (aqua_imu_loc::kPi - 0.05))) +
    std::abs(aqua_imu_loc::normalize_angle(normalized - (-aqua_imu_loc::kPi + 0.05))), 0.3);
}

TEST(AdditiveUkf, GyroBiasZUpdateMovesStateTowardObservation)
{
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(0.05));
  ukf.set_process_noise(diagonal(0.0));

  // Apply the same bias observation many times; the state should converge to it.
  for (int i = 0; i < 50; ++i) {
    ukf.update_gyro_bias_z(0.02, 1.0e-3);
  }
  EXPECT_NEAR(ukf.state()(14), 0.02, 1.0e-3);
}

TEST(AdditiveUkf, GyroBiasXyzUpdateMovesAllThreeAxesTowardObservation)
{
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(0.05));
  ukf.set_process_noise(diagonal(0.0));

  const Eigen::Vector3d truth(0.012, -0.018, 0.025);
  const Eigen::Vector3d variance(1.0e-3, 1.0e-3, 1.0e-3);
  for (int i = 0; i < 60; ++i) {
    ukf.update_gyro_bias_xyz(truth, variance);
  }
  EXPECT_NEAR(ukf.state()(12), truth.x(), 1.0e-3);
  EXPECT_NEAR(ukf.state()(13), truth.y(), 1.0e-3);
  EXPECT_NEAR(ukf.state()(14), truth.z(), 1.0e-3);
}

TEST(AdditiveUkf, GyroBiasXyzUpdateRejectsNonPositiveVariance)
{
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(0.1));
  ukf.set_process_noise(diagonal(0.0));

  aqua_imu_loc::StateVector before = ukf.state();
  ukf.update_gyro_bias_xyz(Eigen::Vector3d(0.01, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0e-3, 1.0e-3));
  ukf.update_gyro_bias_xyz(Eigen::Vector3d(0.01, 0.0, 0.0), Eigen::Vector3d(1.0e-3, -1.0, 1.0e-3));
  EXPECT_NEAR((ukf.state() - before).norm(), 0.0, 1.0e-12);
}

TEST(AdditiveUkf, GyroBiasZUpdateRejectsNonPositiveVariance)
{
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(0.1));
  ukf.set_process_noise(diagonal(0.0));

  aqua_imu_loc::StateVector before = ukf.state();
  ukf.update_gyro_bias_z(0.02, 0.0);
  ukf.update_gyro_bias_z(0.02, -1.0);
  EXPECT_NEAR((ukf.state() - before).norm(), 0.0, 1.0e-12);
}

TEST(AdditiveUkf, YawUpdateRejectsNonPositiveVariance)
{
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(0.1));
  ukf.set_process_noise(diagonal(0.0));

  aqua_imu_loc::StateVector before = ukf.state();
  ukf.update_yaw(1.0, 0.0);
  ukf.update_yaw(1.0, -1.0);
  EXPECT_NEAR((ukf.state() - before).norm(), 0.0, 1.0e-12);
}

}  // namespace
