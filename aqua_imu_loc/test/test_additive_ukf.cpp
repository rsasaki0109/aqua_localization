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

TEST(AdditiveUkf, PositionUpdateMovesStateTowardMeasurement)
{
  // A 3D position observation with a tight covariance must pull the UKF
  // position toward the measurement and shrink the diagonal of the position
  // covariance block.
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(1.0));
  ukf.set_process_noise(diagonal(0.0));

  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * 1.0e-4;
  const Eigen::Vector3d measurement(2.0, -1.5, 0.7);

  ukf.update_position(measurement, covariance);

  EXPECT_NEAR(ukf.state()(0), measurement.x(), 0.05);
  EXPECT_NEAR(ukf.state()(1), measurement.y(), 0.05);
  EXPECT_NEAR(ukf.state()(2), measurement.z(), 0.05);
  // Position covariance must shrink.
  EXPECT_LT(ukf.covariance()(0, 0), 0.5);
  EXPECT_LT(ukf.covariance()(1, 1), 0.5);
  EXPECT_LT(ukf.covariance()(2, 2), 0.5);
}

TEST(AdditiveUkf, PositionUpdatePropagatesIntoBiasStatesAfterMotion)
{
  // The whole point of tightly-coupled fusion: a sonar position observation
  // should correct not just position but also the cross-correlated bias
  // states. Run a non-zero IMU prediction first so cross-covariance between
  // position and the accel-bias states builds up, then observe a position
  // that disagrees with the integrated motion. The accel-bias state should
  // shift in response.
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(0.1));
  ukf.set_process_noise(diagonal(1.0e-4));
  aqua_imu_loc::DynamicsParams dynamics;
  dynamics.gravity_mps2 = 9.80665;
  dynamics.enable_linear_drag = false;

  // Accelerate forward at 1 m/s^2 along body-x for 1 s. The IMU sample
  // includes the constant +g on body-z (REP-145 stationary-up convention).
  aqua_imu_loc::ImuSample sample;
  sample.linear_acceleration = Eigen::Vector3d(1.0, 0.0, 9.80665);
  sample.angular_velocity = Eigen::Vector3d::Zero();
  for (int i = 0; i < 100; ++i) {
    ukf.predict(0.01, sample, dynamics);
  }

  const aqua_imu_loc::StateVector before = ukf.state();
  EXPECT_GT(before(0), 0.3);  // sanity: position grew along x

  // Observe a position that says the boat is at the origin: this is a strong
  // contradiction with the integrated motion, so the bias states should
  // adjust to absorb part of the residual.
  const Eigen::Vector3d measurement(0.0, 0.0, 0.0);
  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * 1.0e-3;
  ukf.update_position(measurement, covariance);

  // accel_bias_x is state[9]; it should have moved (no requirement on sign,
  // only that the cross-covariance pulled it away from zero).
  EXPECT_GT(std::abs(ukf.state()(9)), 1.0e-4);
}

TEST(AdditiveUkf, BodyVelocityUpdateMovesVelocityTowardMeasurementAtZeroRotation)
{
  // With identity rotation, the body-frame velocity equals the world-frame
  // velocity. A tight body-velocity observation should pull state[3..5]
  // toward the measurement.
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(1.0));
  ukf.set_process_noise(diagonal(0.0));

  const Eigen::Vector3d measurement(0.7, -0.3, 0.05);
  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * 1.0e-4;

  ukf.update_body_velocity(measurement, covariance);

  EXPECT_NEAR(ukf.state()(3), measurement.x(), 0.05);
  EXPECT_NEAR(ukf.state()(4), measurement.y(), 0.05);
  EXPECT_NEAR(ukf.state()(5), measurement.z(), 0.05);
}

TEST(AdditiveUkf, BodyVelocityUpdateRespectsYawRotation)
{
  // Boat is rotated 90° yaw (state[8] = pi/2). A body-frame velocity of
  // (1, 0, 0) means "forward in body" which in world coordinates is
  // (0, 1, 0). The UKF should pull world-frame velocity toward (0, 1, 0).
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(1.0));
  ukf.set_process_noise(diagonal(0.0));

  aqua_imu_loc::StateVector state = aqua_imu_loc::StateVector::Zero();
  state(8) = aqua_imu_loc::kPi / 2.0;
  ukf.set_state(state);

  const Eigen::Vector3d measurement_body(1.0, 0.0, 0.0);
  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * 1.0e-3;
  ukf.update_body_velocity(measurement_body, covariance);

  EXPECT_NEAR(ukf.state()(3), 0.0, 0.10);
  EXPECT_NEAR(ukf.state()(4), 1.0, 0.10);
  EXPECT_NEAR(ukf.state()(5), 0.0, 0.10);
}

TEST(AdditiveUkf, BodyVelocityUpdateRejectsNonFiniteInputs)
{
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(0.1));
  ukf.set_process_noise(diagonal(0.0));

  const aqua_imu_loc::StateVector before = ukf.state();
  ukf.update_body_velocity(
    Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
    Eigen::Matrix3d::Identity() * 1.0e-3);
  ukf.update_body_velocity(
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_NEAR((ukf.state() - before).norm(), 0.0, 1.0e-12);
}

TEST(AdditiveUkf, PositionUpdateRejectsNonFiniteInputs)
{
  aqua_imu_loc::AdditiveUkf ukf;
  ukf.configure(0.2, 2.0, 0.0);
  ukf.set_initial_covariance(diagonal(0.1));
  ukf.set_process_noise(diagonal(0.0));

  const aqua_imu_loc::StateVector before = ukf.state();
  ukf.update_position(
    Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
    Eigen::Matrix3d::Identity() * 1.0e-3);
  ukf.update_position(
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_NEAR((ukf.state() - before).norm(), 0.0, 1.0e-12);
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
