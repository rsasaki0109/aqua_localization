#include "aqua_imu_loc/imu_preprocessor.hpp"

#include <cmath>

namespace aqua_imu_loc
{

void ImuPreprocessor::configure(const ImuPreprocessorConfig & config)
{
  config_ = config;
}

std::optional<PredictionInterval> ImuPreprocessor::prediction_interval(double raw_dt) const
{
  if (!std::isfinite(raw_dt) || raw_dt < config_.min_prediction_dt) {
    return std::nullopt;
  }

  PredictionInterval interval;
  interval.dt = raw_dt;
  if (raw_dt > config_.max_prediction_dt) {
    interval.dt = config_.max_prediction_dt;
    interval.clamped = true;
  }
  return interval;
}

bool ImuPreprocessor::sample_is_finite(const ImuSample & sample) const
{
  return sample.linear_acceleration.allFinite() && sample.angular_velocity.allFinite();
}

}  // namespace aqua_imu_loc
