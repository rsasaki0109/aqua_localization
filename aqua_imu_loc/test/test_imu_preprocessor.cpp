#include <limits>

#include <gtest/gtest.h>

#include "aqua_imu_loc/imu_preprocessor.hpp"

TEST(ImuPreprocessor, RejectsTooSmallAndNonFiniteDt)
{
  aqua_imu_loc::ImuPreprocessorConfig config;
  config.min_prediction_dt = 0.001;
  config.max_prediction_dt = 0.05;

  aqua_imu_loc::ImuPreprocessor preprocessor;
  preprocessor.configure(config);

  EXPECT_FALSE(preprocessor.prediction_interval(0.0005).has_value());
  EXPECT_FALSE(
    preprocessor.prediction_interval(std::numeric_limits<double>::quiet_NaN()).has_value());
}

TEST(ImuPreprocessor, AcceptsNormalDt)
{
  aqua_imu_loc::ImuPreprocessorConfig config;
  config.min_prediction_dt = 0.001;
  config.max_prediction_dt = 0.05;

  aqua_imu_loc::ImuPreprocessor preprocessor;
  preprocessor.configure(config);

  const auto interval = preprocessor.prediction_interval(0.01);

  ASSERT_TRUE(interval.has_value());
  EXPECT_DOUBLE_EQ(interval->dt, 0.01);
  EXPECT_FALSE(interval->clamped);
}

TEST(ImuPreprocessor, ClampsLargeDt)
{
  aqua_imu_loc::ImuPreprocessorConfig config;
  config.min_prediction_dt = 0.001;
  config.max_prediction_dt = 0.05;

  aqua_imu_loc::ImuPreprocessor preprocessor;
  preprocessor.configure(config);

  const auto interval = preprocessor.prediction_interval(1.0);

  ASSERT_TRUE(interval.has_value());
  EXPECT_DOUBLE_EQ(interval->dt, 0.05);
  EXPECT_TRUE(interval->clamped);
}

TEST(ImuPreprocessor, RejectsNonFiniteSamples)
{
  aqua_imu_loc::ImuPreprocessor preprocessor;
  preprocessor.configure({});

  aqua_imu_loc::ImuSample sample;
  EXPECT_TRUE(preprocessor.sample_is_finite(sample));

  sample.angular_velocity.x() = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(preprocessor.sample_is_finite(sample));
}
