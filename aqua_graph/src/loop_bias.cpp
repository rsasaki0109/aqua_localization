// SPDX-License-Identifier: Apache-2.0
#include "aqua_graph/loop_bias.hpp"

#include <algorithm>
#include <cmath>

namespace aqua_graph
{

namespace
{

double yaw_of(const gtsam::Rot3 & r)
{
  const gtsam::Matrix3 m = r.matrix();
  return std::atan2(m(1, 0), m(0, 0));
}

double wrap(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

double vertical_component(const gtsam::Rot3 & world_R_body, const gtsam::Vector3 & gyro_bias)
{
  return world_R_body.unrotate(gtsam::Vector3::UnitZ()).dot(gyro_bias);
}

}  // namespace

LoopBiasEstimator::LoopBiasEstimator(LoopBiasConfig config)
: config_(config)
{
}

void LoopBiasEstimator::add_state(
  double stamp_s, const gtsam::Rot3 & world_R_body, const gtsam::Vector3 & gyro_bias)
{
  if (!history_.empty() && stamp_s <= history_.back().stamp_s) {
    return;
  }
  history_.push_back(Sample{stamp_s, world_R_body, vertical_component(world_R_body, gyro_bias)});
  while (!history_.empty() && history_.front().stamp_s < stamp_s - config_.history_s) {
    history_.pop_front();
  }
}

void LoopBiasEstimator::add_keyframe(std::uint32_t id, double stamp_s)
{
  keyframe_stamps_.emplace(id, stamp_s);
}

std::optional<LoopBiasEstimator::Sample> LoopBiasEstimator::sample_at(double stamp_s) const
{
  if (history_.empty() || stamp_s < history_.front().stamp_s ||
    stamp_s > history_.back().stamp_s)
  {
    return std::nullopt;
  }
  const auto it = std::lower_bound(
    history_.begin(), history_.end(), stamp_s,
    [](const Sample & s, double t) {return s.stamp_s < t;});
  if (it == history_.begin()) {
    return *it;
  }
  const Sample & b = *it;
  const Sample & a = *std::prev(it);
  const double f = (stamp_s - a.stamp_s) / std::max(b.stamp_s - a.stamp_s, 1e-9);
  return Sample{stamp_s, a.world_R_body.slerp(f, b.world_R_body),
    a.vertical_bias + f * (b.vertical_bias - a.vertical_bias)};
}

double LoopBiasEstimator::mean_vertical_bias(double t0, double t1) const
{
  double sum = 0.0;
  double weight = 0.0;
  for (std::size_t i = 1; i < history_.size(); ++i) {
    const double a = std::max(history_[i - 1].stamp_s, t0);
    const double b = std::min(history_[i].stamp_s, t1);
    if (b > a) {
      sum += history_[i - 1].vertical_bias * (b - a);
      weight += b - a;
    }
  }
  return weight > 0.0 ? sum / weight : 0.0;
}

std::optional<LoopBiasUpdate> LoopBiasEstimator::add_loop(
  std::uint32_t from_id, std::uint32_t to_id, const gtsam::Rot3 & from_R_to_measured)
{
  const auto from_stamp = keyframe_stamps_.find(from_id);
  const auto to_stamp = keyframe_stamps_.find(to_id);
  if (from_stamp == keyframe_stamps_.end() || to_stamp == keyframe_stamps_.end()) {
    return std::nullopt;
  }
  const double t0 = std::min(from_stamp->second, to_stamp->second);
  const double t1 = std::max(from_stamp->second, to_stamp->second);
  const double span = t1 - t0;
  if (span < config_.min_span_s) {
    return std::nullopt;
  }
  const auto from = sample_at(from_stamp->second);
  const auto to = sample_at(to_stamp->second);
  if (!from || !to) {
    return std::nullopt;
  }
  LoopBiasUpdate update;
  update.span_s = span;
  // Dead-reckoned yaw from `from` to `to` exceeds the true one by the
  // integral of (true - estimated) vertical bias between them.
  const double sign = to_stamp->second >= from_stamp->second ? 1.0 : -1.0;
  update.yaw_residual_rad = wrap(
    yaw_of(from->world_R_body.between(to->world_R_body)) - yaw_of(from_R_to_measured));
  update.observed_bias = mean_vertical_bias(t0, t1) + sign * update.yaw_residual_rad / span;
  update.observation_sigma = config_.yaw_sigma_rad / span;

  const double now = history_.back().stamp_s;
  if (!have_estimate_) {
    have_estimate_ = true;
    bias_ = history_.back().vertical_bias;
    variance_ = config_.initial_sigma * config_.initial_sigma;
  } else {
    variance_ += config_.bias_random_walk * config_.bias_random_walk *
      std::max(now - last_update_s_, 0.0);
  }
  last_update_s_ = now;
  const double r = update.observation_sigma * update.observation_sigma;
  const double gain = variance_ / (variance_ + r);
  bias_ += gain * (update.observed_bias - bias_);
  variance_ *= 1.0 - gain;
  update.bias = bias_;
  update.sigma = std::sqrt(variance_);
  return update;
}

gtsam::Vector3 LoopBiasEstimator::with_vertical_bias(
  const gtsam::Rot3 & world_R_body, const gtsam::Vector3 & gyro_bias, double vertical_bias)
{
  const gtsam::Vector3 up = world_R_body.unrotate(gtsam::Vector3::UnitZ());
  return gyro_bias + (vertical_bias - up.dot(gyro_bias)) * up;
}

}  // namespace aqua_graph
