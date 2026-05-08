#include "aqua_imu_loc/additive_ukf.hpp"

#include <cmath>
#include <limits>

namespace aqua_imu_loc
{

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

Eigen::Vector3d normalize_angles(const Eigen::Vector3d & angles)
{
  return {normalize_angle(angles.x()), normalize_angle(angles.y()), normalize_angle(angles.z())};
}

Eigen::Matrix3d rotation_from_rpy(const Eigen::Vector3d & rpy)
{
  const double cr = std::cos(rpy.x());
  const double sr = std::sin(rpy.x());
  const double cp = std::cos(rpy.y());
  const double sp = std::sin(rpy.y());
  const double cy = std::cos(rpy.z());
  const double sy = std::sin(rpy.z());

  Eigen::Matrix3d rotation;
  rotation << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr;
  return rotation;
}

Eigen::Vector3d euler_rates_from_body_rates(
  const Eigen::Vector3d & rpy, const Eigen::Vector3d & body_rates)
{
  const double roll = rpy.x();
  double pitch = rpy.y();
  const double cos_pitch = std::cos(pitch);

  if (std::abs(cos_pitch) < 1.0e-3) {
    pitch = std::copysign((kPi / 2.0) - 1.0e-3, pitch);
  }

  const double sr = std::sin(roll);
  const double cr = std::cos(roll);
  const double tp = std::tan(pitch);
  const double cp = std::cos(pitch);

  Eigen::Matrix3d transform;
  transform << 1.0, sr * tp, cr * tp,
    0.0, cr, -sr,
    0.0, sr / cp, cr / cp;
  return transform * body_rates;
}

void AdditiveUkf::configure(double alpha, double beta, double kappa)
{
  alpha_ = alpha;
  beta_ = beta;
  kappa_ = kappa;
  lambda_ = alpha_ * alpha_ * (kStateDim + kappa_) - kStateDim;

  const double sigma_count = 2.0 * kStateDim + 1.0;
  mean_weights_.assign(static_cast<size_t>(sigma_count), 0.5 / (kStateDim + lambda_));
  covariance_weights_ = mean_weights_;
  mean_weights_[0] = lambda_ / (kStateDim + lambda_);
  covariance_weights_[0] = mean_weights_[0] + (1.0 - alpha_ * alpha_ + beta_);
}

void AdditiveUkf::set_state(const StateVector & state)
{
  state_ = state;
  state_.segment<3>(6) = normalize_angles(state_.segment<3>(6));
}

void AdditiveUkf::set_initial_covariance(const std::vector<double> & diagonal)
{
  covariance_.setZero();
  for (int i = 0; i < kStateDim; ++i) {
    covariance_(i, i) = diagonal.at(static_cast<size_t>(i));
  }
}

void AdditiveUkf::set_process_noise(const std::vector<double> & diagonal)
{
  process_noise_.setZero();
  for (int i = 0; i < kStateDim; ++i) {
    process_noise_(i, i) = diagonal.at(static_cast<size_t>(i));
  }
}

void AdditiveUkf::predict(double dt, const ImuSample & imu, const DynamicsParams & dynamics)
{
  const auto sigma_points = make_sigma_points();
  std::vector<StateVector> propagated;
  propagated.reserve(sigma_points.size());

  for (const auto & sigma : sigma_points) {
    propagated.push_back(process_model(sigma, dt, imu, dynamics));
  }

  state_ = weighted_mean(propagated);
  covariance_.setZero();
  for (size_t i = 0; i < propagated.size(); ++i) {
    StateVector delta = propagated[i] - state_;
    delta.segment<3>(6) = normalize_angles(delta.segment<3>(6));
    covariance_ += covariance_weights_[i] * delta * delta.transpose();
  }
  covariance_ += process_noise_ * dt;
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

void AdditiveUkf::update_gyro_bias_xyz(
  const Eigen::Vector3d & observed_bias_rad_s,
  const Eigen::Vector3d & variance_diagonal)
{
  if (!observed_bias_rad_s.allFinite() || !variance_diagonal.allFinite()) {
    return;
  }
  if ((variance_diagonal.array() <= 0.0).any()) {
    return;
  }

  const auto sigma_points = make_sigma_points();

  Eigen::Vector3d predicted = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < sigma_points.size(); ++i) {
    predicted += mean_weights_[i] * sigma_points[i].segment<3>(12);
  }

  Eigen::Matrix3d innovation_covariance = variance_diagonal.asDiagonal();
  Eigen::Matrix<double, kStateDim, 3> cross_covariance =
    Eigen::Matrix<double, kStateDim, 3>::Zero();
  for (size_t i = 0; i < sigma_points.size(); ++i) {
    Eigen::Vector3d measurement_delta = sigma_points[i].segment<3>(12) - predicted;
    StateVector state_delta = sigma_points[i] - state_;
    state_delta.segment<3>(6) = normalize_angles(state_delta.segment<3>(6));

    innovation_covariance += covariance_weights_[i] * measurement_delta * measurement_delta.transpose();
    cross_covariance += covariance_weights_[i] * state_delta * measurement_delta.transpose();
  }

  Eigen::Matrix3d innovation_inv;
  bool invertible = false;
  innovation_covariance.computeInverseWithCheck(innovation_inv, invertible);
  if (!invertible) {
    return;
  }

  const Eigen::Matrix<double, kStateDim, 3> kalman_gain = cross_covariance * innovation_inv;
  const Eigen::Vector3d innovation = observed_bias_rad_s - predicted;
  state_ += kalman_gain * innovation;
  state_.segment<3>(6) = normalize_angles(state_.segment<3>(6));
  covariance_ -= kalman_gain * innovation_covariance * kalman_gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

void AdditiveUkf::update_gyro_bias_z(double observed_bias_rad_s, double variance)
{
  if (!std::isfinite(observed_bias_rad_s) || !std::isfinite(variance) || variance <= 0.0) {
    return;
  }

  const auto sigma_points = make_sigma_points();

  double predicted = 0.0;
  for (size_t i = 0; i < sigma_points.size(); ++i) {
    predicted += mean_weights_[i] * sigma_points[i](14);
  }

  double innovation_covariance = variance;
  StateVector cross_covariance = StateVector::Zero();
  for (size_t i = 0; i < sigma_points.size(); ++i) {
    const double measurement_delta = sigma_points[i](14) - predicted;
    StateVector state_delta = sigma_points[i] - state_;
    state_delta.segment<3>(6) = normalize_angles(state_delta.segment<3>(6));

    innovation_covariance += covariance_weights_[i] * measurement_delta * measurement_delta;
    cross_covariance += covariance_weights_[i] * state_delta * measurement_delta;
  }

  if (innovation_covariance <= std::numeric_limits<double>::epsilon() ||
    !std::isfinite(innovation_covariance))
  {
    return;
  }

  const StateVector kalman_gain = cross_covariance / innovation_covariance;
  const double innovation = observed_bias_rad_s - predicted;
  state_ += kalman_gain * innovation;
  state_.segment<3>(6) = normalize_angles(state_.segment<3>(6));
  covariance_ -= kalman_gain * innovation_covariance * kalman_gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

void AdditiveUkf::update_yaw(double yaw_rad, double variance)
{
  if (!std::isfinite(yaw_rad) || !std::isfinite(variance) || variance <= 0.0) {
    return;
  }

  const auto sigma_points = make_sigma_points();

  double predicted_sin = 0.0;
  double predicted_cos = 0.0;
  for (size_t i = 0; i < sigma_points.size(); ++i) {
    predicted_sin += mean_weights_[i] * std::sin(sigma_points[i](8));
    predicted_cos += mean_weights_[i] * std::cos(sigma_points[i](8));
  }
  const double predicted_yaw = std::atan2(predicted_sin, predicted_cos);

  double innovation_covariance = variance;
  StateVector cross_covariance = StateVector::Zero();
  for (size_t i = 0; i < sigma_points.size(); ++i) {
    const double measurement_delta = normalize_angle(sigma_points[i](8) - predicted_yaw);
    StateVector state_delta = sigma_points[i] - state_;
    state_delta.segment<3>(6) = normalize_angles(state_delta.segment<3>(6));

    innovation_covariance += covariance_weights_[i] * measurement_delta * measurement_delta;
    cross_covariance += covariance_weights_[i] * state_delta * measurement_delta;
  }

  if (innovation_covariance <= std::numeric_limits<double>::epsilon() ||
    !std::isfinite(innovation_covariance))
  {
    return;
  }

  const StateVector kalman_gain = cross_covariance / innovation_covariance;
  const double innovation = normalize_angle(yaw_rad - predicted_yaw);
  state_ += kalman_gain * innovation;
  state_.segment<3>(6) = normalize_angles(state_.segment<3>(6));
  covariance_ -= kalman_gain * innovation_covariance * kalman_gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

void AdditiveUkf::update_position(
  const Eigen::Vector3d & position, const Eigen::Matrix3d & covariance)
{
  if (!position.allFinite() || !covariance.allFinite()) {
    return;
  }
  const auto sigma_points = make_sigma_points();

  Eigen::Vector3d predicted = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < sigma_points.size(); ++i) {
    predicted += mean_weights_[i] * sigma_points[i].segment<3>(0);
  }

  Eigen::Matrix3d innovation_covariance = covariance;
  Eigen::Matrix<double, kStateDim, 3> cross_covariance =
    Eigen::Matrix<double, kStateDim, 3>::Zero();
  for (size_t i = 0; i < sigma_points.size(); ++i) {
    const Eigen::Vector3d measurement_delta = sigma_points[i].segment<3>(0) - predicted;
    StateVector state_delta = sigma_points[i] - state_;
    state_delta.segment<3>(6) = normalize_angles(state_delta.segment<3>(6));

    innovation_covariance +=
      covariance_weights_[i] * measurement_delta * measurement_delta.transpose();
    cross_covariance +=
      covariance_weights_[i] * state_delta * measurement_delta.transpose();
  }

  // Reject pathological innovation covariance (e.g. floored to zero).
  Eigen::LLT<Eigen::Matrix3d> llt(innovation_covariance);
  if (llt.info() != Eigen::Success) {
    return;
  }

  const Eigen::Matrix<double, kStateDim, 3> kalman_gain =
    cross_covariance * innovation_covariance.inverse();
  const Eigen::Vector3d innovation = position - predicted;
  state_ += kalman_gain * innovation;
  state_.segment<3>(6) = normalize_angles(state_.segment<3>(6));
  covariance_ -= kalman_gain * innovation_covariance * kalman_gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

void AdditiveUkf::update_body_velocity(
  const Eigen::Vector3d & velocity_body, const Eigen::Matrix3d & covariance)
{
  if (!velocity_body.allFinite() || !covariance.allFinite()) {
    return;
  }
  const auto sigma_points = make_sigma_points();

  // Predicted body velocity per sigma point. v_body = R(rpy)^T * v_world.
  std::vector<Eigen::Vector3d> predicted_per_sigma;
  predicted_per_sigma.reserve(sigma_points.size());
  Eigen::Vector3d predicted = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < sigma_points.size(); ++i) {
    const Eigen::Vector3d v_world = sigma_points[i].segment<3>(3);
    const Eigen::Vector3d rpy = sigma_points[i].segment<3>(6);
    const Eigen::Matrix3d rotation = rotation_from_rpy(rpy);
    const Eigen::Vector3d v_body = rotation.transpose() * v_world;
    predicted_per_sigma.push_back(v_body);
    predicted += mean_weights_[i] * v_body;
  }

  Eigen::Matrix3d innovation_covariance = covariance;
  Eigen::Matrix<double, kStateDim, 3> cross_covariance =
    Eigen::Matrix<double, kStateDim, 3>::Zero();
  for (size_t i = 0; i < sigma_points.size(); ++i) {
    const Eigen::Vector3d measurement_delta = predicted_per_sigma[i] - predicted;
    StateVector state_delta = sigma_points[i] - state_;
    state_delta.segment<3>(6) = normalize_angles(state_delta.segment<3>(6));

    innovation_covariance +=
      covariance_weights_[i] * measurement_delta * measurement_delta.transpose();
    cross_covariance +=
      covariance_weights_[i] * state_delta * measurement_delta.transpose();
  }

  Eigen::LLT<Eigen::Matrix3d> llt(innovation_covariance);
  if (llt.info() != Eigen::Success) {
    return;
  }

  const Eigen::Matrix<double, kStateDim, 3> kalman_gain =
    cross_covariance * innovation_covariance.inverse();
  const Eigen::Vector3d innovation = velocity_body - predicted;
  state_ += kalman_gain * innovation;
  state_.segment<3>(6) = normalize_angles(state_.segment<3>(6));
  covariance_ -= kalman_gain * innovation_covariance * kalman_gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

void AdditiveUkf::update_depth(double depth_m, double variance)
{
  const auto sigma_points = make_sigma_points();

  double predicted_depth = 0.0;
  for (size_t i = 0; i < sigma_points.size(); ++i) {
    predicted_depth += mean_weights_[i] * depth_measurement_model(sigma_points[i]);
  }

  double innovation_covariance = variance;
  StateVector cross_covariance = StateVector::Zero();
  for (size_t i = 0; i < sigma_points.size(); ++i) {
    const double measurement_delta = depth_measurement_model(sigma_points[i]) - predicted_depth;
    StateVector state_delta = sigma_points[i] - state_;
    state_delta.segment<3>(6) = normalize_angles(state_delta.segment<3>(6));

    innovation_covariance += covariance_weights_[i] * measurement_delta * measurement_delta;
    cross_covariance += covariance_weights_[i] * state_delta * measurement_delta;
  }

  if (innovation_covariance <= std::numeric_limits<double>::epsilon() ||
    !std::isfinite(innovation_covariance))
  {
    return;
  }

  const StateVector kalman_gain = cross_covariance / innovation_covariance;
  const double innovation = depth_m - predicted_depth;
  state_ += kalman_gain * innovation;
  state_.segment<3>(6) = normalize_angles(state_.segment<3>(6));
  covariance_ -= kalman_gain * innovation_covariance * kalman_gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

const StateVector & AdditiveUkf::state() const
{
  return state_;
}

const StateMatrix & AdditiveUkf::covariance() const
{
  return covariance_;
}

std::vector<StateVector> AdditiveUkf::make_sigma_points() const
{
  StateMatrix square_root;
  bool decomposed = false;

  for (double jitter : {0.0, 1.0e-9, 1.0e-7, 1.0e-5}) {
    Eigen::LLT<StateMatrix> llt(covariance_ + jitter * StateMatrix::Identity());
    if (llt.info() == Eigen::Success) {
      square_root = llt.matrixL();
      decomposed = true;
      break;
    }
  }

  if (!decomposed) {
    square_root = StateMatrix::Identity() * 1.0e-3;
  }

  const double scale = std::sqrt(kStateDim + lambda_);
  std::vector<StateVector> sigma_points;
  sigma_points.reserve(2 * kStateDim + 1);
  sigma_points.push_back(state_);

  for (int i = 0; i < kStateDim; ++i) {
    StateVector positive = state_ + scale * square_root.col(i);
    StateVector negative = state_ - scale * square_root.col(i);
    positive.segment<3>(6) = normalize_angles(positive.segment<3>(6));
    negative.segment<3>(6) = normalize_angles(negative.segment<3>(6));
    sigma_points.push_back(positive);
    sigma_points.push_back(negative);
  }
  return sigma_points;
}

StateVector AdditiveUkf::process_model(
  const StateVector & state, double dt, const ImuSample & imu,
  const DynamicsParams & dynamics) const
{
  StateVector next = state;

  const Eigen::Vector3d position = state.segment<3>(0);
  const Eigen::Vector3d velocity = state.segment<3>(3);
  const Eigen::Vector3d rpy = state.segment<3>(6);
  const Eigen::Vector3d accel_bias = state.segment<3>(9);
  const Eigen::Vector3d gyro_bias = state.segment<3>(12);

  const Eigen::Vector3d angular_velocity = imu.angular_velocity - gyro_bias;
  const Eigen::Vector3d linear_acceleration_body = imu.linear_acceleration - accel_bias;

  Eigen::Vector3d next_rpy = rpy + euler_rates_from_body_rates(rpy, angular_velocity) * dt;
  next_rpy = normalize_angles(next_rpy);

  const Eigen::Matrix3d rotation = rotation_from_rpy(next_rpy);
  Eigen::Vector3d acceleration_world =
    rotation * linear_acceleration_body + Eigen::Vector3d(0.0, 0.0, -dynamics.gravity_mps2);

  if (dynamics.enable_linear_drag) {
    const Eigen::Vector3d relative_velocity = velocity - dynamics.current_velocity;
    acceleration_world += -dynamics.linear_drag_coeff * relative_velocity;
  }
  if (dynamics.enable_buoyancy) {
    acceleration_world.z() += dynamics.buoyancy_accel_z_mps2;
  }

  next.segment<3>(0) = position + velocity * dt + 0.5 * acceleration_world * dt * dt;
  next.segment<3>(3) = velocity + acceleration_world * dt;
  next.segment<3>(6) = next_rpy;
  return next;
}

StateVector AdditiveUkf::weighted_mean(const std::vector<StateVector> & sigma_points) const
{
  StateVector mean = StateVector::Zero();
  for (size_t i = 0; i < sigma_points.size(); ++i) {
    mean += mean_weights_[i] * sigma_points[i];
  }

  for (int angle_index = 6; angle_index < 9; ++angle_index) {
    double sin_sum = 0.0;
    double cos_sum = 0.0;
    for (size_t i = 0; i < sigma_points.size(); ++i) {
      sin_sum += mean_weights_[i] * std::sin(sigma_points[i](angle_index));
      cos_sum += mean_weights_[i] * std::cos(sigma_points[i](angle_index));
    }
    mean(angle_index) = std::atan2(sin_sum, cos_sum);
  }
  return mean;
}

double AdditiveUkf::depth_measurement_model(const StateVector & state) const
{
  return -state.z();
}

}  // namespace aqua_imu_loc
