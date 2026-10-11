// SPDX-License-Identifier: Apache-2.0
#include "aqua_graph/factors.hpp"

#include <cmath>

namespace aqua_graph
{

DvlVelocityFactor::DvlVelocityFactor(
  gtsam::Key pose_key, gtsam::Key velocity_key, gtsam::Key bias_key,
  const gtsam::Vector3 & measured_velocity_dvl, const gtsam::Vector3 & omega_measured,
  const gtsam::Rot3 & body_R_dvl, const gtsam::Point3 & lever_arm_body,
  const gtsam::SharedNoiseModel & noise_model)
: Base(noise_model, pose_key, velocity_key, bias_key),
  measured_(measured_velocity_dvl),
  omega_measured_(omega_measured),
  body_R_dvl_(body_R_dvl),
  lever_arm_body_(lever_arm_body)
{
}

gtsam::Vector DvlVelocityFactor::evaluateError(
  const gtsam::Pose3 & pose, const gtsam::Vector3 & velocity_world,
  const gtsam::imuBias::ConstantBias & bias,
  boost::optional<gtsam::Matrix &> H_pose,
  boost::optional<gtsam::Matrix &> H_velocity,
  boost::optional<gtsam::Matrix &> H_bias) const
{
  const gtsam::Rot3 & R_wb = pose.rotation();
  gtsam::Matrix3 H_unrotate_R;
  gtsam::Matrix3 H_unrotate_v;
  const gtsam::Vector3 velocity_body = R_wb.unrotate(
    velocity_world, H_pose ? &H_unrotate_R : nullptr, H_velocity ? &H_unrotate_v : nullptr);

  const gtsam::Vector3 omega = omega_measured_ - bias.gyroscope();
  const gtsam::Vector3 lever_velocity = omega.cross(lever_arm_body_);
  const gtsam::Matrix3 dvl_R_body = body_R_dvl_.transpose();
  const gtsam::Vector3 predicted = dvl_R_body * (velocity_body + lever_velocity);

  if (H_pose) {
    // Pose3 tangent is [rotation, translation]; only the rotation enters.
    H_pose->setZero(3, 6);
    H_pose->leftCols<3>() = dvl_R_body * H_unrotate_R;
  }
  if (H_velocity) {
    *H_velocity = dvl_R_body * H_unrotate_v;
  }
  if (H_bias) {
    // d(omega x p)/d(b_g) = [p]x because omega = omega_meas - b_g.
    // ConstantBias tangent is [accelerometer, gyroscope].
    H_bias->setZero(3, 6);
    H_bias->rightCols<3>() = dvl_R_body * gtsam::skewSymmetric(lever_arm_body_);
  }
  return predicted - measured_;
}

PressureDepthFactor::PressureDepthFactor(
  gtsam::Key pose_key, double measured_depth_m, const gtsam::Point3 & lever_arm_body,
  const gtsam::SharedNoiseModel & noise_model)
: Base(noise_model, pose_key),
  measured_(measured_depth_m),
  lever_arm_body_(lever_arm_body)
{
}

gtsam::Vector PressureDepthFactor::evaluateError(
  const gtsam::Pose3 & pose, boost::optional<gtsam::Matrix &> H_pose) const
{
  gtsam::Matrix36 H_transform;
  const gtsam::Point3 sensor_world =
    pose.transformFrom(lever_arm_body_, H_pose ? &H_transform : nullptr);
  if (H_pose) {
    *H_pose = -H_transform.row(2);
  }
  gtsam::Vector1 error;
  error << -sensor_world.z() - measured_;
  return error;
}

HeadingFactor::HeadingFactor(
  gtsam::Key pose_key, double measured_heading_rad, const gtsam::Vector3 & axis_body,
  const gtsam::SharedNoiseModel & noise_model)
: Base(noise_model, pose_key),
  measured_(measured_heading_rad),
  axis_body_(axis_body)
{
}

gtsam::Vector HeadingFactor::evaluateError(
  const gtsam::Pose3 & pose, boost::optional<gtsam::Matrix &> H_pose) const
{
  const gtsam::Rot3 & R = pose.rotation();
  const gtsam::Vector3 d = R * axis_body_;
  const double n2 = d.x() * d.x() + d.y() * d.y();
  const double heading = std::atan2(d.y(), d.x());
  if (H_pose) {
    // Right perturbation R*Exp(w): d' = d - R [a]x w.
    gtsam::Matrix13 dh_dd;
    dh_dd << -d.y() / n2, d.x() / n2, 0.0;
    H_pose->setZero(1, 6);
    H_pose->leftCols<3>() = -dh_dd * R.matrix() * gtsam::skewSymmetric(axis_body_);
  }
  gtsam::Vector1 error;
  error << std::remainder(heading - measured_, 2.0 * M_PI);
  return error;
}

double heading_of_axis(const gtsam::Rot3 & world_R_body, const gtsam::Vector3 & axis_body)
{
  const gtsam::Vector3 d = world_R_body * axis_body;
  return std::atan2(d.y(), d.x());
}

}  // namespace aqua_graph
