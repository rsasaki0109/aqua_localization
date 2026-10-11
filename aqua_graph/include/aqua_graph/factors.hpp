// SPDX-License-Identifier: Apache-2.0
// Underwater measurement factors for the aqua_graph factor-graph backend.
//
// Frame conventions match aqua_imu_loc: the world frame is z-up with gravity
// along -z, depth is positive downward (depth = -z), and Pose3 maps body
// (base_link) coordinates into the world frame.
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace aqua_graph
{

// DVL bottom-track velocity measured in the DVL frame.
//
//   h = R_bd^T * (R_wb^T * v_w + (omega_meas - b_g) x p_bd)
//
// R_bd rotates DVL-frame vectors into the body frame (same meaning as
// aqua_imu_loc's imu.dvl.mount.rotation_rpy_rad) and p_bd is the DVL lever
// arm in the body frame. omega_meas is the raw gyro sample closest to the DVL
// stamp, so the lever-arm term couples the gyro bias into the residual.
class DvlVelocityFactor
  : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>
{
public:
  using Base =
    gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>;

  DvlVelocityFactor(
    gtsam::Key pose_key, gtsam::Key velocity_key, gtsam::Key bias_key,
    const gtsam::Vector3 & measured_velocity_dvl, const gtsam::Vector3 & omega_measured,
    const gtsam::Rot3 & body_R_dvl, const gtsam::Point3 & lever_arm_body,
    const gtsam::SharedNoiseModel & noise_model);

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose, const gtsam::Vector3 & velocity_world,
    const gtsam::imuBias::ConstantBias & bias,
    boost::optional<gtsam::Matrix &> H_pose = boost::none,
    boost::optional<gtsam::Matrix &> H_velocity = boost::none,
    boost::optional<gtsam::Matrix &> H_bias = boost::none) const override;

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new DvlVelocityFactor(*this)));
  }

  const gtsam::Vector3 & measured() const {return measured_;}

private:
  gtsam::Vector3 measured_;
  gtsam::Vector3 omega_measured_;
  gtsam::Rot3 body_R_dvl_;
  gtsam::Point3 lever_arm_body_;
};

// Pressure-derived depth of the pressure sensor:
//
//   h = -(t_wb + R_wb * p_bp).z
//
// where p_bp is the pressure-sensor lever arm in the body frame.
class PressureDepthFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3>
{
public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

  PressureDepthFactor(
    gtsam::Key pose_key, double measured_depth_m, const gtsam::Point3 & lever_arm_body,
    const gtsam::SharedNoiseModel & noise_model);

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose,
    boost::optional<gtsam::Matrix &> H_pose = boost::none) const override;

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new PressureDepthFactor(*this)));
  }

  double measured() const {return measured_;}

private:
  double measured_;
  gtsam::Point3 lever_arm_body_;
};

// Absolute heading of a body-fixed axis (e.g. a dual-antenna GNSS baseline or
// a magnetometer-referenced forward axis), measured counter-clockwise from
// world +x in the world xy plane:
//
//   h = atan2(d.y, d.x),  d = R_wb * axis_body
//
// The residual is wrapped to (-pi, pi]. The axis must not be near vertical.
class HeadingFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3>
{
public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

  HeadingFactor(
    gtsam::Key pose_key, double measured_heading_rad, const gtsam::Vector3 & axis_body,
    const gtsam::SharedNoiseModel & noise_model);

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose,
    boost::optional<gtsam::Matrix &> H_pose = boost::none) const override;

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new HeadingFactor(*this)));
  }

  double measured() const {return measured_;}

private:
  double measured_;
  gtsam::Vector3 axis_body_;
};

// Heading of a body axis under a rotation (same definition as HeadingFactor).
double heading_of_axis(const gtsam::Rot3 & world_R_body, const gtsam::Vector3 & axis_body);

}  // namespace aqua_graph
