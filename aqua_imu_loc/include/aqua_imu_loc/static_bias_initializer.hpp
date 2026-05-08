#ifndef AQUA_IMU_LOC__STATIC_BIAS_INITIALIZER_HPP_
#define AQUA_IMU_LOC__STATIC_BIAS_INITIALIZER_HPP_

#include <cstddef>

#include <Eigen/Dense>

#include "aqua_imu_loc/additive_ukf.hpp"

namespace aqua_imu_loc
{

struct StaticBiasInitializerConfig
{
  // Time window over which IMU samples are averaged for bias initialization.
  double window_seconds{3.0};
  // Maximum gyro magnitude (rad/s) accepted before declaring motion and aborting.
  double gyro_motion_threshold_radps{0.10};
  // Maximum deviation of |accel| from gravity_mps2 accepted before declaring motion.
  double accel_motion_threshold_mps2{0.50};
  // Gravity used to filter accel-magnitude based motion detection.
  double gravity_mps2{9.80665};
  // Minimum sample count required before transitioning to ready, even if window expired.
  std::size_t minimum_samples{50};
};

enum class StaticBiasInitializerStatus
{
  kAccumulating,
  kReady,
  kAborted,
  kDisabled,
};

class StaticBiasInitializer
{
public:
  void configure(const StaticBiasInitializerConfig & config);
  void enable(bool enabled);
  void reset();

  // Add one IMU sample observed at time stamp_seconds. Returns the post-update status.
  StaticBiasInitializerStatus add_sample(
    double stamp_seconds, const ImuSample & sample);

  StaticBiasInitializerStatus status() const;
  Eigen::Vector3d gyro_bias() const;
  std::size_t sample_count() const;

private:
  StaticBiasInitializerConfig config_;
  StaticBiasInitializerStatus status_{StaticBiasInitializerStatus::kAccumulating};
  bool enabled_{true};
  bool first_sample_seen_{false};
  double first_sample_seconds_{0.0};
  std::size_t sample_count_{0};
  Eigen::Vector3d gyro_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
};

}  // namespace aqua_imu_loc

#endif  // AQUA_IMU_LOC__STATIC_BIAS_INITIALIZER_HPP_
