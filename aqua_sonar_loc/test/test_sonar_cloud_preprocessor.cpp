#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "aqua_sonar_loc/sonar_cloud_preprocessor.hpp"

namespace
{

sensor_msgs::msg::PointCloud2 make_cloud(const std::vector<std::array<float, 3>> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.is_dense = false;
  cloud.is_bigendian = false;
  cloud.point_step = 3 * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(static_cast<size_t>(cloud.row_step));

  const std::array<std::string, 3> names = {"x", "y", "z"};
  for (size_t i = 0; i < names.size(); ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i];
    field.offset = static_cast<uint32_t>(i * sizeof(float));
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }

  for (size_t i = 0; i < points.size(); ++i) {
    const size_t point_offset = i * static_cast<size_t>(cloud.point_step);
    for (size_t axis = 0; axis < 3; ++axis) {
      const float value = points[i][axis];
      std::memcpy(
        &cloud.data[point_offset + axis * sizeof(float)], &value, sizeof(float));
    }
  }
  return cloud;
}

}  // namespace

TEST(SonarCloudPreprocessor, AcceptsFiniteInRangeCloud)
{
  aqua_sonar_loc::SonarCloudPreprocessorConfig config;
  config.min_points = 2;
  config.max_range_m = 10.0;

  aqua_sonar_loc::SonarCloudPreprocessor preprocessor;
  preprocessor.configure(config);

  const auto cloud = make_cloud({{{1.0F, 0.0F, 0.0F}, {2.0F, 0.0F, 0.0F}}});
  const auto summary = preprocessor.summarize(cloud);

  EXPECT_TRUE(summary.accepted);
  EXPECT_EQ(summary.total_points, 2U);
  EXPECT_EQ(summary.finite_xyz_points, 2U);
  EXPECT_EQ(summary.in_range_points, 2U);
}

TEST(SonarCloudPreprocessor, RejectsMissingXyzFields)
{
  aqua_sonar_loc::SonarCloudPreprocessor preprocessor;
  preprocessor.configure({});

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.width = 3;
  cloud.height = 1;
  cloud.point_step = sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);

  const auto summary = preprocessor.summarize(cloud);

  EXPECT_FALSE(summary.accepted);
  EXPECT_EQ(summary.rejection_reason, "missing x/y/z fields");
}

TEST(SonarCloudPreprocessor, RejectsCloudBelowMinimumAfterFiltering)
{
  aqua_sonar_loc::SonarCloudPreprocessorConfig config;
  config.min_points = 2;
  config.max_range_m = 2.0;

  aqua_sonar_loc::SonarCloudPreprocessor preprocessor;
  preprocessor.configure(config);

  const auto cloud = make_cloud({{{1.0F, 0.0F, 0.0F}, {5.0F, 0.0F, 0.0F}}});
  const auto summary = preprocessor.summarize(cloud);

  EXPECT_FALSE(summary.accepted);
  EXPECT_EQ(summary.finite_xyz_points, 2U);
  EXPECT_EQ(summary.in_range_points, 1U);
}

TEST(SonarCloudPreprocessor, IgnoresNonFinitePoints)
{
  aqua_sonar_loc::SonarCloudPreprocessorConfig config;
  config.min_points = 1;

  aqua_sonar_loc::SonarCloudPreprocessor preprocessor;
  preprocessor.configure(config);

  const float nan = std::numeric_limits<float>::quiet_NaN();
  const auto cloud = make_cloud({{{nan, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}}});
  const auto summary = preprocessor.summarize(cloud);

  EXPECT_TRUE(summary.accepted);
  EXPECT_EQ(summary.finite_xyz_points, 1U);
  EXPECT_EQ(summary.in_range_points, 1U);
}
