// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>

#include "aqua_graph/loop_bias.hpp"

namespace
{

using aqua_graph::LoopBiasConfig;
using aqua_graph::LoopBiasEstimator;

double true_yaw(double t) {return 0.3 * std::sin(t / 50.0) + 0.001 * t;}

// Dead reckoning with a vertical gyro bias error: the estimator subtracts
// `used_bias` while the gyro carries `true_bias`.
void feed(
  LoopBiasEstimator & estimator, double true_bias, double used_bias, double t_end,
  const gtsam::Rot3 & tilt = gtsam::Rot3())
{
  for (double t = 0.0; t <= t_end + 1e-9; t += 0.1) {
    const gtsam::Rot3 attitude = gtsam::Rot3::Rz(true_yaw(t) + (true_bias - used_bias) * t) * tilt;
    const gtsam::Vector3 bias =
      LoopBiasEstimator::with_vertical_bias(attitude, gtsam::Vector3(1e-4, -2e-4, 0.0), used_bias);
    estimator.add_state(t, attitude, bias);
  }
}

gtsam::Rot3 measured(double ta, double tb)
{
  return gtsam::Rot3::Rz(true_yaw(tb) - true_yaw(ta));
}

}  // namespace

TEST(LoopBiasTest, LoopObservesMeanVerticalBias)
{
  LoopBiasEstimator estimator;
  const double true_bias = 3e-4;  // ~62 deg/h
  feed(estimator, true_bias, 0.0, 800.0);
  estimator.add_keyframe(1, 100.0);
  estimator.add_keyframe(2, 700.0);
  const auto update = estimator.add_loop(1, 2, measured(100.0, 700.0));
  ASSERT_TRUE(update.has_value());
  EXPECT_NEAR(update->span_s, 600.0, 1e-9);
  EXPECT_NEAR(update->yaw_residual_rad, true_bias * 600.0, 1e-4);
  EXPECT_NEAR(update->observed_bias, true_bias, 1e-6);
  EXPECT_NEAR(update->observation_sigma, 0.05 / 600.0, 1e-12);
  // Kalman update from the prior (bias 0, sigma 1e-3): gain ~0.993.
  const double prior = 1e-3 * 1e-3;
  const double r = update->observation_sigma * update->observation_sigma;
  EXPECT_NEAR(update->bias, prior / (prior + r) * update->observed_bias, 1e-12);
  EXPECT_LT(update->sigma, update->observation_sigma);
}

TEST(LoopBiasTest, AccountsForTheBiasAlreadyApplied)
{
  // The estimator already subtracts part of the bias; the loop must recover
  // the total, not just the remaining error.
  LoopBiasEstimator estimator;
  feed(estimator, 3e-4, 1e-4, 800.0, gtsam::Rot3::Rx(0.05) * gtsam::Rot3::Ry(-0.03));
  estimator.add_keyframe(1, 100.0);
  estimator.add_keyframe(2, 700.0);
  const auto update = estimator.add_loop(1, 2, measured(100.0, 700.0));
  ASSERT_TRUE(update.has_value());
  EXPECT_NEAR(update->observed_bias, 3e-4, 5e-6);
}

TEST(LoopBiasTest, LoopDirectionDoesNotMatter)
{
  LoopBiasEstimator estimator;
  feed(estimator, -2e-4, 0.0, 800.0);
  estimator.add_keyframe(1, 100.0);
  estimator.add_keyframe(2, 700.0);
  const auto update = estimator.add_loop(2, 1, measured(700.0, 100.0));
  ASSERT_TRUE(update.has_value());
  EXPECT_NEAR(update->observed_bias, -2e-4, 1e-6);
}

TEST(LoopBiasTest, RejectsShortSpansAndUnknownKeyframes)
{
  LoopBiasConfig config;
  config.min_span_s = 120.0;
  LoopBiasEstimator estimator(config);
  feed(estimator, 3e-4, 0.0, 800.0);
  estimator.add_keyframe(1, 100.0);
  estimator.add_keyframe(2, 160.0);
  EXPECT_FALSE(estimator.add_loop(1, 2, measured(100.0, 160.0)).has_value());
  EXPECT_FALSE(estimator.add_loop(1, 3, measured(100.0, 160.0)).has_value());
}

TEST(LoopBiasTest, FusesLoopsAsRandomWalk)
{
  LoopBiasConfig config;
  config.yaw_sigma_rad = 0.05;
  LoopBiasEstimator estimator(config);
  feed(estimator, 3e-4, 0.0, 1500.0);
  estimator.add_keyframe(1, 100.0);
  estimator.add_keyframe(2, 700.0);
  estimator.add_keyframe(3, 800.0);
  estimator.add_keyframe(4, 1400.0);
  // A loop whose yaw is off by 0.1 rad pulls the estimate, a correct one
  // brings it back with a smaller sigma.
  const auto first = estimator.add_loop(1, 2, measured(100.0, 700.0) * gtsam::Rot3::Rz(0.1));
  const auto second = estimator.add_loop(3, 4, measured(800.0, 1400.0));
  ASSERT_TRUE(first && second);
  EXPECT_GT(std::abs(first->bias - 3e-4), 1e-4);
  EXPECT_LT(std::abs(second->bias - 3e-4), std::abs(first->bias - 3e-4));
  EXPECT_LT(second->sigma, first->sigma);
}

TEST(LoopBiasTest, VerticalBiasFollowsTheUpAxis)
{
  const gtsam::Rot3 attitude = gtsam::Rot3::Rz(1.0) * gtsam::Rot3::Rx(0.3);
  const gtsam::Vector3 bias(1e-3, 2e-3, 3e-3);
  const gtsam::Vector3 out = LoopBiasEstimator::with_vertical_bias(attitude, bias, 5e-4);
  const gtsam::Vector3 up = attitude.unrotate(gtsam::Vector3::UnitZ());
  EXPECT_NEAR(up.dot(out), 5e-4, 1e-12);
  // Horizontal components are untouched.
  EXPECT_NEAR(((out - bias) - up * up.dot(out - bias)).norm(), 0.0, 1e-12);
}
