// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "aqua_graph/factors.hpp"

namespace
{

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

gtsam::Values make_values()
{
  gtsam::Values values;
  values.insert(
    X(0), gtsam::Pose3(gtsam::Rot3::Ypr(0.7, -0.2, 0.3), gtsam::Point3(1.0, -2.0, -3.0)));
  values.insert(V(0), gtsam::Vector3(0.4, -0.3, 0.1));
  values.insert(
    B(0), gtsam::imuBias::ConstantBias(
      gtsam::Vector3(0.01, 0.02, -0.03), gtsam::Vector3(0.004, -0.002, 0.003)));
  return values;
}

TEST(DvlVelocityFactor, ZeroErrorAtTruth)
{
  const gtsam::Values values = make_values();
  const auto pose = values.at<gtsam::Pose3>(X(0));
  const auto v = values.at<gtsam::Vector3>(V(0));
  const auto bias = values.at<gtsam::imuBias::ConstantBias>(B(0));
  const gtsam::Rot3 body_R_dvl = gtsam::Rot3::Ypr(0.78, 0.0, 3.14159);
  const gtsam::Point3 lever(0.3, -0.1, 0.2);
  const gtsam::Vector3 omega_meas(0.1, -0.05, 0.2);
  const gtsam::Vector3 omega = omega_meas - bias.gyroscope();
  const gtsam::Vector3 expected =
    body_R_dvl.unrotate(pose.rotation().unrotate(v) + omega.cross(lever));

  aqua_graph::DvlVelocityFactor factor(
    X(0), V(0), B(0), expected, omega_meas, body_R_dvl, lever,
    gtsam::noiseModel::Isotropic::Sigma(3, 0.02));
  EXPECT_LT(factor.unwhitenedError(values).norm(), 1.0e-12);
}

TEST(DvlVelocityFactor, JacobiansMatchNumerical)
{
  const gtsam::Values values = make_values();
  aqua_graph::DvlVelocityFactor factor(
    X(0), V(0), B(0), gtsam::Vector3(0.2, 0.1, -0.05), gtsam::Vector3(0.1, -0.05, 0.2),
    gtsam::Rot3::Ypr(0.78, 0.1, 3.0), gtsam::Point3(0.3, -0.1, 0.2),
    gtsam::noiseModel::Isotropic::Sigma(3, 0.02));
  EXPECT_TRUE(gtsam::internal::testFactorJacobians("dvl", factor, values, 1.0e-6, 1.0e-6));
}

TEST(PressureDepthFactor, ZeroErrorAtTruthWithLeverArm)
{
  const gtsam::Values values = make_values();
  const auto pose = values.at<gtsam::Pose3>(X(0));
  const gtsam::Point3 lever(0.5, 0.1, -0.2);
  const double depth = -pose.transformFrom(lever).z();
  aqua_graph::PressureDepthFactor factor(
    X(0), depth, lever, gtsam::noiseModel::Isotropic::Sigma(1, 0.05));
  EXPECT_LT(factor.unwhitenedError(values).norm(), 1.0e-12);
  EXPECT_NEAR(depth, 3.0 - (pose.rotation() * lever).z(), 1.0e-12);
}

TEST(PressureDepthFactor, JacobiansMatchNumerical)
{
  const gtsam::Values values = make_values();
  aqua_graph::PressureDepthFactor factor(
    X(0), 2.5, gtsam::Point3(0.5, 0.1, -0.2), gtsam::noiseModel::Isotropic::Sigma(1, 0.05));
  EXPECT_TRUE(gtsam::internal::testFactorJacobians("depth", factor, values, 1.0e-6, 1.0e-6));
}

TEST(HeadingFactor, ZeroErrorAtTruthAndWraps)
{
  const gtsam::Values values = make_values();
  const auto pose = values.at<gtsam::Pose3>(X(0));
  const gtsam::Vector3 axis(0.0, 1.0, 0.0);
  const double heading = aqua_graph::heading_of_axis(pose.rotation(), axis);
  aqua_graph::HeadingFactor exact(X(0), heading, axis, gtsam::noiseModel::Isotropic::Sigma(1, 0.01));
  EXPECT_LT(exact.unwhitenedError(values).norm(), 1.0e-12);
  // A measurement 2*pi away is the same heading.
  aqua_graph::HeadingFactor wrapped(
    X(0), heading + 2.0 * M_PI, axis, gtsam::noiseModel::Isotropic::Sigma(1, 0.01));
  EXPECT_LT(wrapped.unwhitenedError(values).norm(), 1.0e-9);
}

TEST(HeadingFactor, JacobiansMatchNumerical)
{
  const gtsam::Values values = make_values();
  aqua_graph::HeadingFactor factor(
    X(0), 0.3, gtsam::Vector3(1.0, 0.2, 0.1), gtsam::noiseModel::Isotropic::Sigma(1, 0.01));
  EXPECT_TRUE(gtsam::internal::testFactorJacobians("heading", factor, values, 1.0e-6, 1.0e-6));
}

}  // namespace
