// SPDX-License-Identifier: Apache-2.0
#include "aqua_graph/estimator.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "aqua_graph/factors.hpp"

namespace aqua_graph
{

namespace
{

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

gtsam::Rot3 strip_yaw(const gtsam::Rot3 & rotation)
{
  const gtsam::Vector3 ypr = rotation.ypr();
  return gtsam::Rot3::Ypr(0.0, ypr.y(), ypr.z());
}

}  // namespace

std::optional<AttitudeInitSource> parse_attitude_init_source(const std::string & name)
{
  if (name == "accelerometer") {
    return AttitudeInitSource::kAccelerometer;
  }
  if (name == "imu_orientation") {
    return AttitudeInitSource::kImuOrientation;
  }
  return std::nullopt;
}

TightlyCoupledEstimator::TightlyCoupledEstimator(const EstimatorConfig & config)
: config_(config)
{
  const auto & imu = config_.imu;
  preint_params_ = gtsam::PreintegrationCombinedParams::MakeSharedU(imu.gravity_mps2);
  preint_params_->setAccelerometerCovariance(
    gtsam::I_3x3 * imu.accel_noise_density * imu.accel_noise_density);
  preint_params_->setGyroscopeCovariance(
    gtsam::I_3x3 * imu.gyro_noise_density * imu.gyro_noise_density);
  preint_params_->setIntegrationCovariance(
    gtsam::I_3x3 * imu.integration_sigma * imu.integration_sigma);
  preint_params_->setBiasAccCovariance(
    gtsam::I_3x3 * imu.accel_bias_random_walk * imu.accel_bias_random_walk);
  preint_params_->setBiasOmegaCovariance(
    gtsam::I_3x3 * imu.gyro_bias_random_walk * imu.gyro_bias_random_walk);
  preint_params_->setBiasAccOmegaInit(gtsam::I_6x6 * 1.0e-5);

  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeThreshold = config_.relinearize_threshold;
  isam_params.relinearizeSkip = std::max(1, config_.relinearize_skip);
  isam_params.findUnusedFactorSlots = true;
  smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(
    config_.smoother_lag_s, isam_params);
}

TightlyCoupledEstimator::~TightlyCoupledEstimator() = default;

void TightlyCoupledEstimator::set_initial_state(
  double stamp_s, const gtsam::Pose3 & pose, const gtsam::Vector3 & velocity_world,
  const gtsam::imuBias::ConstantBias & bias)
{
  manual_init_stamp_ = stamp_s;
  manual_init_pose_ = pose;
  manual_init_velocity_ = velocity_world;
  manual_init_bias_ = bias;
}

std::vector<EstimatorOutput> TightlyCoupledEstimator::add_imu(
  double stamp_s, const gtsam::Vector3 & accel_body, const gtsam::Vector3 & gyro_body,
  const std::optional<gtsam::Rot3> & orientation)
{
  std::vector<EstimatorOutput> outputs;
  if (!accel_body.allFinite() || !gyro_body.allFinite() || !std::isfinite(stamp_s)) {
    return outputs;
  }
  ++counters_.imu_samples;

  if (!initialized_) {
    try_initialize(stamp_s, accel_body, gyro_body, orientation);
    last_imu_stamp_s_ = stamp_s;
    last_accel_ = accel_body;
    last_gyro_ = gyro_body;
    return outputs;
  }

  if (stamp_s <= last_imu_stamp_s_) {
    // Duplicate or out-of-order sample: keep the newest reading for the hold
    // but do not integrate negative/zero time.
    last_accel_ = accel_body;
    last_gyro_ = gyro_body;
    return outputs;
  }

  // Zero-order hold of the previous sample over (last_imu_stamp, stamp].
  // A grid state is created only once the IMU stream moves strictly past its
  // stamp, so measurements stamped exactly on the grid (which the replay
  // delivers after the IMU sample with the same stamp) join the same update.
  while (state_stamp(current_index_ + 1) < stamp_s) {
    const double boundary = state_stamp(current_index_ + 1);
    integrate(boundary - integrated_until_s_, last_accel_, last_gyro_);
    integrated_until_s_ = boundary;
    outputs.push_back(create_next_state());
  }
  integrate(stamp_s - integrated_until_s_, last_accel_, last_gyro_);
  integrated_until_s_ = stamp_s;

  last_imu_stamp_s_ = stamp_s;
  last_accel_ = accel_body;
  last_gyro_ = gyro_body;
  return outputs;
}

void TightlyCoupledEstimator::add_dvl(double stamp_s, const gtsam::Vector3 & velocity_dvl)
{
  add_dvl(stamp_s, velocity_dvl, -1.0);
}

void TightlyCoupledEstimator::add_dvl(
  double stamp_s, const gtsam::Vector3 & velocity_dvl, double sigma_mps)
{
  if (!config_.dvl.enabled || !initialized_) {
    return;
  }
  ++counters_.dvl_received;
  if (!velocity_dvl.allFinite() || velocity_dvl.norm() > config_.dvl.max_speed_mps) {
    ++counters_.dvl_rejected;
    return;
  }
  PendingMeasurement m{};
  m.kind = PendingMeasurement::Kind::kDvl;
  m.stamp_s = stamp_s;
  m.vector = velocity_dvl;
  m.omega = last_gyro_;
  m.scalar = sigma_mps > 0.0 ? sigma_mps : config_.dvl.velocity_sigma_mps;
  pending_.push_back(m);
}

void TightlyCoupledEstimator::add_depth(double stamp_s, double depth_m)
{
  if (!config_.depth.enabled || !initialized_ || !std::isfinite(depth_m)) {
    return;
  }
  ++counters_.depth_received;
  PendingMeasurement m{};
  m.kind = PendingMeasurement::Kind::kDepth;
  m.stamp_s = stamp_s;
  m.scalar = depth_m;
  pending_.push_back(m);
}

void TightlyCoupledEstimator::add_position(
  double stamp_s, const gtsam::Point3 & position_world, const gtsam::Matrix3 & covariance)
{
  if (!config_.position.enabled || !initialized_ || !position_world.allFinite()) {
    return;
  }
  ++counters_.position_received;
  PendingMeasurement m{};
  m.kind = PendingMeasurement::Kind::kPosition;
  m.stamp_s = stamp_s;
  m.vector = position_world;
  m.covariance = covariance.allFinite() ? covariance : gtsam::Matrix3::Zero();
  pending_.push_back(m);
}

std::optional<EstimatorOutput> TightlyCoupledEstimator::imu_rate_estimate() const
{
  if (!initialized_) {
    return std::nullopt;
  }
  const gtsam::NavState predicted = preint_->predict(current_state_, current_bias_);
  EstimatorOutput out;
  out.stamp_s = integrated_until_s_;
  out.state_index = current_index_;
  out.pose = predicted.pose();
  out.velocity_world = predicted.v();
  out.bias = current_bias_;
  return out;
}

std::optional<EstimatorOutput> TightlyCoupledEstimator::latest_state() const
{
  if (!initialized_) {
    return std::nullopt;
  }
  EstimatorOutput out;
  out.stamp_s = state_stamp(current_index_);
  out.state_index = current_index_;
  out.pose = current_state_.pose();
  out.velocity_world = current_state_.v();
  out.bias = current_bias_;
  return out;
}

void TightlyCoupledEstimator::try_initialize(
  double stamp_s, const gtsam::Vector3 & accel_body, const gtsam::Vector3 & gyro_body,
  const std::optional<gtsam::Rot3> & orientation)
{
  if (manual_init_stamp_) {
    if (stamp_s >= *manual_init_stamp_) {
      start_graph(
        stamp_s, gtsam::NavState(manual_init_pose_, manual_init_velocity_), manual_init_bias_);
    }
    return;
  }

  if (config_.init.attitude_source == AttitudeInitSource::kImuOrientation) {
    if (!orientation) {
      return;
    }
    const gtsam::Rot3 attitude =
      config_.init.keep_imu_yaw ? *orientation : strip_yaw(*orientation);
    start_graph(
      stamp_s, gtsam::NavState(gtsam::Pose3(attitude, gtsam::Point3::Zero()),
      gtsam::Vector3::Zero()), gtsam::imuBias::ConstantBias());
    return;
  }

  if (!init_window_start_s_) {
    init_window_start_s_ = stamp_s;
  }
  init_accel_sum_ += accel_body;
  init_gyro_sum_ += gyro_body;
  ++init_samples_;
  if (stamp_s - *init_window_start_s_ < config_.init.static_window_s) {
    return;
  }
  const gtsam::Vector3 f = init_accel_sum_ / static_cast<double>(init_samples_);
  const gtsam::Vector3 gyro_mean = init_gyro_sum_ / static_cast<double>(init_samples_);
  // Static specific force in body is R^T * (0, 0, g) for a z-up world.
  const double roll = std::atan2(f.y(), f.z());
  const double pitch = std::atan2(-f.x(), std::hypot(f.y(), f.z()));
  const gtsam::imuBias::ConstantBias bias(gtsam::Vector3::Zero(), gyro_mean);
  start_graph(
    stamp_s, gtsam::NavState(
      gtsam::Pose3(gtsam::Rot3::Ypr(0.0, pitch, roll), gtsam::Point3::Zero()),
      gtsam::Vector3::Zero()), bias);
}

void TightlyCoupledEstimator::start_graph(
  double stamp_s, const gtsam::NavState & state, const gtsam::imuBias::ConstantBias & bias)
{
  const auto & init = config_.init;
  t0_s_ = stamp_s;
  current_index_ = 0;
  integrated_until_s_ = stamp_s;
  current_state_ = state;
  current_bias_ = bias;
  preint_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(preint_params_, bias);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  // Pose3 tangent order: [roll, pitch, yaw (approx, body axes), x, y, z].
  gtsam::Vector6 pose_sigmas;
  pose_sigmas << init.roll_pitch_sigma_rad, init.roll_pitch_sigma_rad, init.yaw_sigma_rad,
    init.position_sigma_m, init.position_sigma_m, init.position_sigma_m;
  gtsam::Vector6 bias_sigmas;
  bias_sigmas << gtsam::Vector3::Constant(init.accel_bias_sigma),
    gtsam::Vector3::Constant(init.gyro_bias_sigma);
  graph.addPrior(X(0), state.pose(), gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas));
  graph.addPrior(
    V(0), state.v(), gtsam::noiseModel::Isotropic::Sigma(3, init.velocity_sigma_mps));
  graph.addPrior(B(0), bias, gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas));
  values.insert(X(0), state.pose());
  values.insert(V(0), state.v());
  values.insert(B(0), bias);

  initialized_ = true;
  ++counters_.states;
  run_update(graph, values, 0);
}

void TightlyCoupledEstimator::integrate(
  double dt, const gtsam::Vector3 & accel, const gtsam::Vector3 & gyro)
{
  if (dt <= 0.0) {
    return;
  }
  preint_->integrateMeasurement(accel, gyro, dt);
}

EstimatorOutput TightlyCoupledEstimator::create_next_state()
{
  const uint64_t k = current_index_ + 1;
  const gtsam::NavState predicted = preint_->predict(current_state_, current_bias_);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  graph.emplace_shared<gtsam::CombinedImuFactor>(
    X(k - 1), V(k - 1), X(k), V(k), B(k - 1), B(k), *preint_);
  values.insert(X(k), predicted.pose());
  values.insert(V(k), predicted.v());
  values.insert(B(k), current_bias_);

  current_index_ = k;
  attach_measurements(graph, k);
  ++counters_.states;
  run_update(graph, values, k);

  preint_->resetIntegrationAndSetBias(current_bias_);
  EstimatorOutput out;
  out.stamp_s = state_stamp(k);
  out.state_index = k;
  out.pose = current_state_.pose();
  out.velocity_world = current_state_.v();
  out.bias = current_bias_;
  return out;
}

int64_t TightlyCoupledEstimator::nearest_state_index(double stamp_s) const
{
  return static_cast<int64_t>(std::llround((stamp_s - t0_s_) / config_.state_period_s));
}

gtsam::SharedNoiseModel TightlyCoupledEstimator::robust(
  const gtsam::SharedNoiseModel & model, double huber_k) const
{
  if (huber_k <= 0.0) {
    return model;
  }
  return gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Huber::Create(huber_k), model);
}

void TightlyCoupledEstimator::attach_measurements(
  gtsam::NonlinearFactorGraph & graph, uint64_t newest_index)
{
  // Keys older than the smoother lag may already be marginalized; keep one
  // grid period of margin so we never reference a removed key.
  const double oldest_safe_stamp =
    state_stamp(newest_index) - config_.smoother_lag_s + config_.state_period_s;
  const gtsam::Vector3 velocity_world = current_state_.v();

  std::deque<PendingMeasurement> keep;
  for (const auto & m : pending_) {
    const int64_t raw_index = nearest_state_index(m.stamp_s);
    if (raw_index > static_cast<int64_t>(newest_index)) {
      keep.push_back(m);
      continue;
    }
    const uint64_t j = static_cast<uint64_t>(std::max<int64_t>(raw_index, 0));
    if (state_stamp(j) < oldest_safe_stamp) {
      ++counters_.measurements_too_old;
      continue;
    }
    // Shift positional measurements from their stamp to the grid stamp using
    // the latest velocity estimate (|dt| <= state_period / 2).
    const double dt = m.stamp_s - state_stamp(j);
    switch (m.kind) {
      case PendingMeasurement::Kind::kDvl:
        graph.emplace_shared<DvlVelocityFactor>(
          X(j), V(j), B(j), m.vector, m.omega, config_.dvl.body_R_dvl,
          config_.dvl.lever_arm_body,
          robust(gtsam::noiseModel::Isotropic::Sigma(3, m.scalar), config_.dvl.huber_k));
        ++counters_.dvl_applied;
        break;
      case PendingMeasurement::Kind::kDepth:
        graph.emplace_shared<PressureDepthFactor>(
          X(j), m.scalar + velocity_world.z() * dt, config_.depth.lever_arm_body,
          robust(gtsam::noiseModel::Isotropic::Sigma(1, config_.depth.depth_sigma_m),
          config_.depth.huber_k));
        ++counters_.depth_applied;
        break;
      case PendingMeasurement::Kind::kPosition: {
          gtsam::Matrix3 cov = 0.5 * (m.covariance + m.covariance.transpose());
          const double floor_var = config_.position.sigma_floor_m * config_.position.sigma_floor_m;
          for (int i = 0; i < 3; ++i) {
            cov(i, i) = std::max(cov(i, i), floor_var);
          }
          const Eigen::SelfAdjointEigenSolver<gtsam::Matrix3> eig(cov);
          gtsam::SharedNoiseModel model;
          if (eig.info() == Eigen::Success && eig.eigenvalues().minCoeff() > 0.0) {
            model = gtsam::noiseModel::Gaussian::Covariance(cov);
          } else {
            model = gtsam::noiseModel::Diagonal::Variances(cov.diagonal());
          }
          graph.emplace_shared<gtsam::GPSFactor>(
            X(j), gtsam::Point3(m.vector - velocity_world * dt),
            robust(model, config_.position.huber_k));
          ++counters_.position_applied;
          break;
        }
    }
  }
  pending_.swap(keep);
}

void TightlyCoupledEstimator::run_update(
  const gtsam::NonlinearFactorGraph & graph, const gtsam::Values & values,
  uint64_t new_state_index)
{
  gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
  const double stamp = state_stamp(new_state_index);
  timestamps[X(new_state_index)] = stamp;
  timestamps[V(new_state_index)] = stamp;
  timestamps[B(new_state_index)] = stamp;
  smoother_->update(graph, values, timestamps);
  ++counters_.smoother_updates;

  current_state_ = gtsam::NavState(
    smoother_->calculateEstimate<gtsam::Pose3>(X(new_state_index)),
    smoother_->calculateEstimate<gtsam::Vector3>(V(new_state_index)));
  current_bias_ =
    smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(new_state_index));
}

double TightlyCoupledEstimator::state_stamp(uint64_t index) const
{
  return t0_s_ + static_cast<double>(index) * config_.state_period_s;
}

}  // namespace aqua_graph
