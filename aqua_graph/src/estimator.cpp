// SPDX-License-Identifier: Apache-2.0
#include "aqua_graph/estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "aqua_graph/factors.hpp"

namespace aqua_graph
{

namespace
{

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

using StereoFactor = gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>;

// Stamps closer than this are the same instant (anchored measurements carry
// the exact stamp they requested a state for).
constexpr double kSameStampS = 1.0e-6;

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

  const auto & vis = config_.visual;
  stereo_calibration_ = boost::make_shared<gtsam::Cal3_S2Stereo>(
    vis.fx, vis.fy, 0.0, vis.cx, vis.cy, vis.baseline_m);

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
  if (!accel_body.allFinite() || !gyro_body.allFinite() || !std::isfinite(stamp_s)) {
    return {};
  }
  ++counters_.imu_samples;
  latest_gyro_ = gyro_body;

  if (!initialized_) {
    try_initialize(stamp_s, accel_body, gyro_body, orientation);
    if (initialized_) {
      held_imu_ = ImuSample{stamp_s, accel_body, gyro_body};
      integrated_until_s_ = stamp_s;
      latest_imu_s_ = stamp_s;
    }
    return {};
  }

  if (stamp_s <= latest_imu_s_) {
    // Duplicate stamp: keep the newest reading; never integrate backwards.
    if (!imu_buffer_.empty() && imu_buffer_.back().stamp_s == stamp_s) {
      imu_buffer_.back() = ImuSample{stamp_s, accel_body, gyro_body};
    }
    return {};
  }
  imu_buffer_.push_back(ImuSample{stamp_s, accel_body, gyro_body});
  latest_imu_s_ = stamp_s;
  return advance(stamp_s - config_.processing_delay_s);
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
  PendingMeasurement m;
  m.kind = PendingMeasurement::Kind::kDvl;
  m.stamp_s = stamp_s;
  m.anchored = config_.dvl.anchor_states;
  m.vector = velocity_dvl;
  m.omega = latest_gyro_;
  m.scalar = sigma_mps > 0.0 ? sigma_mps : config_.dvl.velocity_sigma_mps;
  enqueue(std::move(m));
}

void TightlyCoupledEstimator::add_depth(double stamp_s, double depth_m)
{
  if (!config_.depth.enabled || !initialized_ || !std::isfinite(depth_m)) {
    return;
  }
  ++counters_.depth_received;
  PendingMeasurement m;
  m.kind = PendingMeasurement::Kind::kDepth;
  m.stamp_s = stamp_s;
  m.scalar = depth_m;
  enqueue(std::move(m));
}

void TightlyCoupledEstimator::add_position(
  double stamp_s, const gtsam::Point3 & position_world, const gtsam::Matrix3 & covariance)
{
  if (!config_.position.enabled || !initialized_ || !position_world.allFinite()) {
    return;
  }
  ++counters_.position_received;
  PendingMeasurement m;
  m.kind = PendingMeasurement::Kind::kPosition;
  m.stamp_s = stamp_s;
  m.vector = position_world;
  m.covariance = covariance.allFinite() ? covariance : gtsam::Matrix3::Zero();
  enqueue(std::move(m));
}

void TightlyCoupledEstimator::add_stereo_frame(
  double stamp_s, const std::vector<StereoObservation> & observations)
{
  if (!config_.visual.enabled || !initialized_ || !std::isfinite(stamp_s)) {
    return;
  }
  ++counters_.stereo_frames;
  PendingMeasurement m;
  m.kind = PendingMeasurement::Kind::kStereo;
  m.stamp_s = stamp_s;
  m.anchored = true;
  m.stereo = observations;
  enqueue(std::move(m));
}

void TightlyCoupledEstimator::enqueue(PendingMeasurement measurement)
{
  // An anchored measurement newer than the newest state asks for a state at
  // its own stamp; an older one (it arrived too late) uses the nearest state.
  if (measurement.anchored) {
    if (measurement.stamp_s > states_.back().stamp_s + kSameStampS) {
      requested_stamps_.insert(measurement.stamp_s);
    } else {
      measurement.anchored = false;
    }
  }
  pending_.push_back(std::move(measurement));
}

std::optional<EstimatorOutput> TightlyCoupledEstimator::imu_rate_estimate() const
{
  if (!initialized_) {
    return std::nullopt;
  }
  // Propagate the newest state through the buffered, not yet integrated IMU
  // samples with the same zero-order hold the smoother uses.
  gtsam::PreintegratedCombinedMeasurements pim(preint_params_, current_bias_);
  double t = integrated_until_s_;
  ImuSample held = held_imu_;
  for (const auto & sample : imu_buffer_) {
    if (sample.stamp_s > t) {
      pim.integrateMeasurement(held.accel, held.gyro, sample.stamp_s - t);
      t = sample.stamp_s;
    }
    held = sample;
  }
  const gtsam::NavState predicted = pim.predict(current_state_, current_bias_);
  EstimatorOutput out;
  out.stamp_s = t;
  out.state_index = states_.back().index;
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
  out.stamp_s = states_.back().stamp_s;
  out.state_index = states_.back().index;
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
  current_state_ = state;
  current_bias_ = bias;
  preint_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(preint_params_, bias);

  UpdateBatch batch;
  // Pose3 tangent order: [roll, pitch, yaw (approx, body axes), x, y, z].
  gtsam::Vector6 pose_sigmas;
  pose_sigmas << init.roll_pitch_sigma_rad, init.roll_pitch_sigma_rad, init.yaw_sigma_rad,
    init.position_sigma_m, init.position_sigma_m, init.position_sigma_m;
  gtsam::Vector6 bias_sigmas;
  bias_sigmas << gtsam::Vector3::Constant(init.accel_bias_sigma),
    gtsam::Vector3::Constant(init.gyro_bias_sigma);
  batch.graph.addPrior(X(0), state.pose(), gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas));
  batch.graph.addPrior(
    V(0), state.v(), gtsam::noiseModel::Isotropic::Sigma(3, init.velocity_sigma_mps));
  batch.graph.addPrior(B(0), bias, gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas));
  batch.values.insert(X(0), state.pose());
  batch.values.insert(V(0), state.v());
  batch.values.insert(B(0), bias);

  initialized_ = true;
  states_.push_back(StateRecord{0, stamp_s});
  ++counters_.states;
  run_update(batch, states_.back());
}

std::vector<EstimatorOutput> TightlyCoupledEstimator::advance(double target_s)
{
  std::vector<EstimatorOutput> outputs;
  const double period = config_.state_period_s;
  for (;;) {
    const double last = states_.back().stamp_s;
    while (!requested_stamps_.empty() && *requested_stamps_.begin() <= last + kSameStampS) {
      requested_stamps_.erase(requested_stamps_.begin());
    }
    double next = last + period;
    bool anchored = false;
    // An anchor shortly after the grid point replaces it, so grid states do
    // not crowd measurement states.
    if (!requested_stamps_.empty() && *requested_stamps_.begin() < next + 0.3 * period) {
      next = *requested_stamps_.begin();
      anchored = true;
    }
    if (next > target_s) {
      break;
    }
    if (anchored) {
      requested_stamps_.erase(requested_stamps_.begin());
    }
    integrate_to(next);
    outputs.push_back(create_state(next));
  }
  return outputs;
}

void TightlyCoupledEstimator::integrate_to(double stamp_s)
{
  // Zero-order hold: each sample is held until the next sample's stamp.
  while (integrated_until_s_ < stamp_s) {
    const double next_sample =
      imu_buffer_.empty() ? std::numeric_limits<double>::infinity() :
      imu_buffer_.front().stamp_s;
    const double segment_end = std::min(stamp_s, next_sample);
    if (segment_end > integrated_until_s_) {
      preint_->integrateMeasurement(
        held_imu_.accel, held_imu_.gyro, segment_end - integrated_until_s_);
    }
    integrated_until_s_ = segment_end;
    if (segment_end >= next_sample) {
      held_imu_ = imu_buffer_.front();
      imu_buffer_.pop_front();
    }
  }
}

EstimatorOutput TightlyCoupledEstimator::create_state(double stamp_s)
{
  const uint64_t k = states_.back().index + 1;
  const gtsam::NavState predicted = preint_->predict(current_state_, current_bias_);

  UpdateBatch batch;
  batch.graph.emplace_shared<gtsam::CombinedImuFactor>(
    X(k - 1), V(k - 1), X(k), V(k), B(k - 1), B(k), *preint_);
  batch.values.insert(X(k), predicted.pose());
  batch.values.insert(V(k), predicted.v());
  batch.values.insert(B(k), current_bias_);

  states_.push_back(StateRecord{k, stamp_s});
  // Mirror the smoother's marginalization: keys older than the lag go away.
  while (!states_.empty() && states_.front().stamp_s < stamp_s - config_.smoother_lag_s) {
    states_.pop_front();
  }
  current_state_ = predicted;
  attach_measurements(batch, states_.back());
  ++counters_.states;
  run_update(batch, states_.back());

  preint_->resetIntegrationAndSetBias(current_bias_);
  EstimatorOutput out;
  out.stamp_s = stamp_s;
  out.state_index = k;
  out.pose = current_state_.pose();
  out.velocity_world = current_state_.v();
  out.bias = current_bias_;
  return out;
}

std::optional<TightlyCoupledEstimator::StateRecord>
TightlyCoupledEstimator::find_state(double stamp_s) const
{
  // Keep one grid period of margin to the window edge so a factor never
  // references a key the smoother is about to marginalize.
  const double oldest_safe =
    states_.back().stamp_s - config_.smoother_lag_s + config_.state_period_s;
  auto it = std::lower_bound(
    states_.begin(), states_.end(), stamp_s,
    [](const StateRecord & s, double t) {return s.stamp_s < t;});
  const StateRecord * best = nullptr;
  if (it != states_.end()) {
    best = &*it;
  }
  if (it != states_.begin()) {
    const StateRecord & before = *std::prev(it);
    if (!best || stamp_s - before.stamp_s <= best->stamp_s - stamp_s) {
      best = &before;
    }
  }
  if (!best || best->stamp_s < oldest_safe) {
    return std::nullopt;
  }
  return *best;
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
  UpdateBatch & batch, const StateRecord & newest)
{
  const gtsam::Vector3 velocity_world = current_state_.v();
  std::deque<PendingMeasurement> keep;
  for (auto & m : pending_) {
    // Wait until a state at or after the measurement exists, so the nearest
    // state is final (and an anchored state has been created).
    if (m.stamp_s > newest.stamp_s + kSameStampS) {
      keep.push_back(std::move(m));
      continue;
    }
    const auto state = find_state(m.stamp_s);
    if (!state) {
      ++counters_.measurements_too_old;
      continue;
    }
    const uint64_t j = state->index;
    // Shift positional measurements from their stamp to the state stamp
    // using the current velocity estimate (zero for anchored ones).
    const double dt = m.stamp_s - state->stamp_s;
    switch (m.kind) {
      case PendingMeasurement::Kind::kDvl:
        batch.graph.emplace_shared<DvlVelocityFactor>(
          X(j), V(j), B(j), m.vector, m.omega, config_.dvl.body_R_dvl,
          config_.dvl.lever_arm_body,
          robust(gtsam::noiseModel::Isotropic::Sigma(3, m.scalar), config_.dvl.huber_k));
        ++counters_.dvl_applied;
        break;
      case PendingMeasurement::Kind::kDepth:
        batch.graph.emplace_shared<PressureDepthFactor>(
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
          batch.graph.emplace_shared<gtsam::GPSFactor>(
            X(j), gtsam::Point3(m.vector - velocity_world * dt),
            robust(model, config_.position.huber_k));
          ++counters_.position_applied;
          break;
        }
      case PendingMeasurement::Kind::kStereo:
        attach_stereo(batch, *state, m.stereo);
        break;
    }
  }
  pending_.swap(keep);
}

gtsam::Pose3 TightlyCoupledEstimator::pose_estimate(
  uint64_t index, const UpdateBatch & batch) const
{
  if (batch.values.exists(X(index))) {
    return batch.values.at<gtsam::Pose3>(X(index));
  }
  return smoother_->calculateEstimate<gtsam::Pose3>(X(index));
}

void TightlyCoupledEstimator::attach_stereo(
  UpdateBatch & batch, const StateRecord & state, const std::vector<StereoObservation> & obs)
{
  const auto & vis = config_.visual;
  const double stamp = states_.back().stamp_s;
  const auto noise = robust(gtsam::noiseModel::Isotropic::Sigma(3, vis.pixel_sigma), vis.huber_k);
  const auto & smoother_stamps = smoother_->timestamps();
  const double bf = vis.fx * vis.baseline_m;

  for (const auto & o : obs) {
    const double disparity = o.u_left - o.u_right;
    if (!std::isfinite(disparity) || disparity < vis.min_disparity_px ||
      bf / disparity > vis.max_depth_m)
    {
      ++counters_.stereo_rejected;
      continue;
    }
    const gtsam::StereoPoint2 measurement(o.u_left, o.u_right, o.v);
    FeatureTrack & track = tracks_[o.id];
    track.last_seen_s = stamp;

    // A landmark that left the smoother window starts over as a new track.
    if (track.landmark && !smoother_stamps.count(*track.landmark)) {
      track.landmark.reset();
      track.first_state.reset();
    }

    if (track.landmark) {
      batch.graph.emplace_shared<StereoFactor>(
        measurement, noise, X(state.index), *track.landmark, stereo_calibration_,
        vis.body_P_camera);
      batch.timestamps[*track.landmark] = stamp;
      ++counters_.stereo_observations;
      continue;
    }

    // The first sighting's pose must still be safely inside the window.
    const double oldest_safe = stamp - config_.smoother_lag_s + config_.state_period_s;
    const bool first_usable = track.first_state && *track.first_state != state.index &&
      std::any_of(
      states_.begin(), states_.end(),
      [&](const StateRecord & s) {
        return s.index == *track.first_state && s.stamp_s >= oldest_safe;
      });
    if (!first_usable) {
      track.first_state = state.index;
      track.first_measurement = measurement;
      continue;
    }

    // Second sighting: triangulate from the first stereo observation with the
    // current estimate of that pose and add both reprojection factors.
    const gtsam::Pose3 world_P_camera =
      pose_estimate(*track.first_state, batch) * vis.body_P_camera;
    const gtsam::Point3 point =
      gtsam::StereoCamera(world_P_camera, stereo_calibration_).backproject(
      track.first_measurement);
    const gtsam::Key key = L(next_landmark_++);
    batch.values.insert(key, point);
    batch.timestamps[key] = stamp;
    batch.graph.emplace_shared<StereoFactor>(
      track.first_measurement, noise, X(*track.first_state), key, stereo_calibration_,
      vis.body_P_camera);
    batch.graph.emplace_shared<StereoFactor>(
      measurement, noise, X(state.index), key, stereo_calibration_, vis.body_P_camera);
    track.landmark = key;
    track.first_state.reset();
    ++counters_.landmarks_created;
    counters_.stereo_observations += 2;
  }

  // Forget tracks that have not been seen for a whole smoother window.
  for (auto it = tracks_.begin(); it != tracks_.end(); ) {
    if (it->second.last_seen_s < stamp - config_.smoother_lag_s) {
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }
}

void TightlyCoupledEstimator::run_update(UpdateBatch & batch, const StateRecord & newest)
{
  batch.timestamps[X(newest.index)] = newest.stamp_s;
  batch.timestamps[V(newest.index)] = newest.stamp_s;
  batch.timestamps[B(newest.index)] = newest.stamp_s;
  smoother_->update(batch.graph, batch.values, batch.timestamps);
  ++counters_.smoother_updates;

  current_state_ = gtsam::NavState(
    smoother_->calculateEstimate<gtsam::Pose3>(X(newest.index)),
    smoother_->calculateEstimate<gtsam::Vector3>(V(newest.index)));
  current_bias_ =
    smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(newest.index));
}

}  // namespace aqua_graph
