#include <limits>

#include <gtest/gtest.h>

#include "aqua_imu_loc/pressure_depth_converter.hpp"

TEST(PressureDepthConverter, UsesConfiguredReferencePressure)
{
  aqua_imu_loc::PressureDepthConfig config;
  config.use_first_pressure_as_reference = false;
  config.reference_pressure_pa = 100000.0;
  config.water_density_kg_m3 = 1000.0;
  config.gravity_mps2 = 10.0;

  aqua_imu_loc::PressureDepthConverter converter;
  converter.configure(config);

  const auto depth = converter.pressure_to_depth(120000.0);

  ASSERT_TRUE(depth.has_value());
  EXPECT_NEAR(depth.value(), 2.0, 1.0e-12);
}

TEST(PressureDepthConverter, FirstPressureSampleCanInitializeReference)
{
  aqua_imu_loc::PressureDepthConfig config;
  config.use_first_pressure_as_reference = true;
  config.water_density_kg_m3 = 1000.0;
  config.gravity_mps2 = 10.0;

  aqua_imu_loc::PressureDepthConverter converter;
  converter.configure(config);

  const auto first_depth = converter.pressure_to_depth(110000.0);
  const auto second_depth = converter.pressure_to_depth(130000.0);

  ASSERT_TRUE(first_depth.has_value());
  ASSERT_TRUE(second_depth.has_value());
  EXPECT_NEAR(first_depth.value(), 0.0, 1.0e-12);
  EXPECT_NEAR(second_depth.value(), 2.0, 1.0e-12);
  EXPECT_NEAR(converter.reference_pressure_pa(), 110000.0, 1.0e-12);
}

TEST(PressureDepthConverter, ReconfigureResetsFirstPressureReference)
{
  aqua_imu_loc::PressureDepthConfig config;
  config.use_first_pressure_as_reference = true;
  config.water_density_kg_m3 = 1000.0;
  config.gravity_mps2 = 10.0;

  aqua_imu_loc::PressureDepthConverter converter;
  converter.configure(config);
  ASSERT_TRUE(converter.pressure_to_depth(110000.0).has_value());
  EXPECT_NEAR(converter.reference_pressure_pa(), 110000.0, 1.0e-12);

  converter.configure(config);
  const auto depth = converter.pressure_to_depth(120000.0);

  ASSERT_TRUE(depth.has_value());
  EXPECT_NEAR(depth.value(), 0.0, 1.0e-12);
  EXPECT_NEAR(converter.reference_pressure_pa(), 120000.0, 1.0e-12);
}

TEST(PressureDepthConverter, AppliesDepthOffsetAfterConversion)
{
  aqua_imu_loc::PressureDepthConfig config;
  config.use_first_pressure_as_reference = false;
  config.reference_pressure_pa = 100000.0;
  config.water_density_kg_m3 = 1000.0;
  config.gravity_mps2 = 10.0;
  config.depth_offset_m = -0.5;

  aqua_imu_loc::PressureDepthConverter converter;
  converter.configure(config);

  const auto depth = converter.pressure_to_depth(120000.0);

  ASSERT_TRUE(depth.has_value());
  EXPECT_NEAR(depth.value(), 1.5, 1.0e-12);
}

TEST(PressureDepthConverter, RejectsInvalidInputsAndConfig)
{
  aqua_imu_loc::PressureDepthConverter converter;
  aqua_imu_loc::PressureDepthConfig config;
  config.use_first_pressure_as_reference = false;
  config.water_density_kg_m3 = 0.0;
  converter.configure(config);

  EXPECT_FALSE(converter.pressure_to_depth(101325.0).has_value());

  config.water_density_kg_m3 = 1025.0;
  converter.configure(config);
  EXPECT_FALSE(
    converter.pressure_to_depth(std::numeric_limits<double>::quiet_NaN()).has_value());
}
