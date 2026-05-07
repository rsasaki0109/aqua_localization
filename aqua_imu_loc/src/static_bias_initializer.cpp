#include "aqua_imu_loc/static_bias_initializer.hpp"

#include <cmath>

namespace aqua_imu_loc
{

void StaticBiasInitializer::configure(const StaticBiasInitializerConfig & config)
{
  config_ = config;
  reset();
}

void StaticBiasInitializer::enable(bool enabled)
{
  enabled_ = enabled;
  if (!enabled_) {
    status_ = StaticBiasInitializerStatus::kDisabled;
  } else if (status_ == StaticBiasInitializerStatus::kDisabled) {
    reset();
  }
}

void StaticBiasInitializer::reset()
{
  if (enabled_) {
    status_ = StaticBiasInitializerStatus::kAccumulating;
  } else {
    status_ = StaticBiasInitializerStatus::kDisabled;
  }
  first_sample_seen_ = false;
  first_sample_seconds_ = 0.0;
  sample_count_ = 0;
  gyro_sum_.setZero();
  gyro_bias_.setZero();
}

StaticBiasInitializerStatus StaticBiasInitializer::add_sample(
  double stamp_seconds, const ImuSample & sample)
{
  if (status_ != StaticBiasInitializerStatus::kAccumulating) {
    return status_;
  }

  if (!std::isfinite(stamp_seconds) ||
    !sample.linear_acceleration.allFinite() ||
    !sample.angular_velocity.allFinite())
  {
    return status_;
  }

  if (!first_sample_seen_) {
    first_sample_seen_ = true;
    first_sample_seconds_ = stamp_seconds;
  }

  const double gyro_magnitude = sample.angular_velocity.norm();
  const double accel_magnitude_residual =
    std::abs(sample.linear_acceleration.norm() - config_.gravity_mps2);

  if (gyro_magnitude > config_.gyro_motion_threshold_radps ||
    accel_magnitude_residual > config_.accel_motion_threshold_mps2)
  {
    status_ = StaticBiasInitializerStatus::kAborted;
    return status_;
  }

  gyro_sum_ += sample.angular_velocity;
  ++sample_count_;

  const double elapsed = stamp_seconds - first_sample_seconds_;
  if (elapsed >= config_.window_seconds && sample_count_ >= config_.minimum_samples) {
    gyro_bias_ = gyro_sum_ / static_cast<double>(sample_count_);
    status_ = StaticBiasInitializerStatus::kReady;
  }

  return status_;
}

StaticBiasInitializerStatus StaticBiasInitializer::status() const
{
  return status_;
}

Eigen::Vector3d StaticBiasInitializer::gyro_bias() const
{
  return gyro_bias_;
}

std::size_t StaticBiasInitializer::sample_count() const
{
  return sample_count_;
}

}  // namespace aqua_imu_loc
