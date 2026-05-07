#ifndef AQUA_IMU_LOC__IMU_PREPROCESSOR_HPP_
#define AQUA_IMU_LOC__IMU_PREPROCESSOR_HPP_

#include <optional>

#include "aqua_imu_loc/additive_ukf.hpp"

namespace aqua_imu_loc
{

struct ImuPreprocessorConfig
{
  double min_prediction_dt{0.0005};
  double max_prediction_dt{0.05};
};

struct PredictionInterval
{
  double dt{0.0};
  bool clamped{false};
};

class ImuPreprocessor
{
public:
  void configure(const ImuPreprocessorConfig & config);

  std::optional<PredictionInterval> prediction_interval(double raw_dt) const;
  bool sample_is_finite(const ImuSample & sample) const;

private:
  ImuPreprocessorConfig config_;
};

}  // namespace aqua_imu_loc

#endif  // AQUA_IMU_LOC__IMU_PREPROCESSOR_HPP_
