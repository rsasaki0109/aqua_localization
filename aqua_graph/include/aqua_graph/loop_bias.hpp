// SPDX-License-Identifier: Apache-2.0
// Gyro-bias estimation from loop closures.
//
// Without an absolute heading, the gyro bias about the vertical axis is
// unobservable and integrates into yaw drift. A loop closure between
// keyframes a and b measures their relative yaw; the dead-reckoned relative
// yaw differs from it by the integral of (true bias - estimated bias) over
// [t_a, t_b]. Each loop therefore observes the mean vertical gyro bias over
// that interval, with sigma yaw_sigma_rad / (t_b - t_a). A scalar random-walk
// filter fuses these observations into the bias the estimator should use.
#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <unordered_map>

#include <gtsam/geometry/Rot3.h>

namespace aqua_graph
{

struct LoopBiasConfig
{
  // Yaw accuracy of one loop closure (relative rotation).
  double yaw_sigma_rad{0.05};
  // Loops spanning less time constrain the bias too weakly to use.
  double min_span_s{120.0};
  // Random walk of the vertical gyro bias between updates (rad/s/sqrt(s)).
  double bias_random_walk{5.0e-6};
  // Prior sigma of the vertical bias before the first loop (rad/s).
  double initial_sigma{1.0e-3};
  // History kept for looking up keyframe attitudes.
  double history_s{14400.0};
};

struct LoopBiasUpdate
{
  double span_s{0.0};
  // Dead-reckoned minus measured relative yaw.
  double yaw_residual_rad{0.0};
  // Mean vertical bias over the loop interval implied by this loop.
  double observed_bias{0.0};
  double observation_sigma{0.0};
  // Fused vertical bias and its sigma after the update.
  double bias{0.0};
  double sigma{0.0};
};

class LoopBiasEstimator
{
public:
  explicit LoopBiasEstimator(LoopBiasConfig config = {});

  // Record an estimator state: attitude (body to world, z up) and the gyro
  // bias it used. States must arrive in time order.
  void add_state(double stamp_s, const gtsam::Rot3 & world_R_body, const gtsam::Vector3 & gyro_bias);
  void add_keyframe(std::uint32_t id, double stamp_s);

  // Loop from keyframe `from_id` to `to_id`: measured rotation of `to` in the
  // `from` frame. Returns the fused bias when the loop is usable.
  std::optional<LoopBiasUpdate> add_loop(
    std::uint32_t from_id, std::uint32_t to_id, const gtsam::Rot3 & from_R_to_measured);

  // Gyro bias whose component along the current up axis is `vertical_bias`,
  // keeping the horizontal components of `gyro_bias`.
  static gtsam::Vector3 with_vertical_bias(
    const gtsam::Rot3 & world_R_body, const gtsam::Vector3 & gyro_bias, double vertical_bias);

private:
  struct Sample
  {
    double stamp_s;
    gtsam::Rot3 world_R_body;
    double vertical_bias;
  };

  std::optional<Sample> sample_at(double stamp_s) const;
  // Mean vertical bias used by the estimator over [t0, t1].
  double mean_vertical_bias(double t0, double t1) const;

  LoopBiasConfig config_;
  std::deque<Sample> history_;
  std::unordered_map<std::uint32_t, double> keyframe_stamps_;
  bool have_estimate_{false};
  double bias_{0.0};
  double variance_{0.0};
  double last_update_s_{0.0};
};

}  // namespace aqua_graph
