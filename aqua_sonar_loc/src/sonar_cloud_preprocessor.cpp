#include "aqua_sonar_loc/sonar_cloud_preprocessor.hpp"

#include <cmath>
#include <cstring>

namespace aqua_sonar_loc
{

void SonarCloudPreprocessor::configure(const SonarCloudPreprocessorConfig & config)
{
  config_ = config;
}

CloudSummary SonarCloudPreprocessor::summarize(const sensor_msgs::msg::PointCloud2 & cloud) const
{
  CloudSummary summary;
  summary.total_points = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);

  if (summary.total_points == 0) {
    summary.rejection_reason = "empty cloud";
    return summary;
  }
  if (cloud.point_step == 0 || cloud.row_step == 0) {
    summary.rejection_reason = "invalid point step";
    return summary;
  }

  const auto fields = find_xyz_fields(cloud);
  if (config_.require_xyz && (!fields.x || !fields.y || !fields.z)) {
    summary.rejection_reason = "missing x/y/z fields";
    return summary;
  }
  if (!fields.x || !fields.y || !fields.z) {
    summary.accepted = summary.total_points >= config_.min_points;
    if (!summary.accepted) {
      summary.rejection_reason = "not enough points";
    }
    return summary;
  }

  for (size_t row = 0; row < cloud.height; ++row) {
    const size_t row_offset = row * static_cast<size_t>(cloud.row_step);
    for (size_t col = 0; col < cloud.width; ++col) {
      const size_t point_offset = row_offset + col * static_cast<size_t>(cloud.point_step);
      if (point_offset + cloud.point_step > cloud.data.size()) {
        continue;
      }

      const auto x = read_numeric_field(cloud, *fields.x, point_offset);
      const auto y = read_numeric_field(cloud, *fields.y, point_offset);
      const auto z = read_numeric_field(cloud, *fields.z, point_offset);
      if (!x.has_value() || !y.has_value() || !z.has_value()) {
        continue;
      }
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        continue;
      }

      ++summary.finite_xyz_points;
      const double range = std::sqrt((*x * *x) + (*y * *y) + (*z * *z));
      if (!config_.enable_range_filter || range <= config_.max_range_m) {
        ++summary.in_range_points;
      }
    }
  }

  summary.accepted = summary.in_range_points >= config_.min_points;
  if (!summary.accepted) {
    summary.rejection_reason = "not enough finite in-range points";
  }
  return summary;
}

SonarCloudPreprocessor::FieldLayout SonarCloudPreprocessor::find_xyz_fields(
  const sensor_msgs::msg::PointCloud2 & cloud) const
{
  FieldLayout layout;
  for (const auto & field : cloud.fields) {
    if (field.name == "x") {
      layout.x = &field;
    } else if (field.name == "y") {
      layout.y = &field;
    } else if (field.name == "z") {
      layout.z = &field;
    }
  }
  return layout;
}

std::optional<double> SonarCloudPreprocessor::read_numeric_field(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const sensor_msgs::msg::PointField & field,
  size_t point_offset) const
{
  const size_t offset = point_offset + static_cast<size_t>(field.offset);
  if (offset >= cloud.data.size()) {
    return std::nullopt;
  }

  switch (field.datatype) {
    case sensor_msgs::msg::PointField::FLOAT32: {
        if (offset + sizeof(float) > cloud.data.size()) {
          return std::nullopt;
        }
        float value = 0.0F;
        std::memcpy(&value, &cloud.data[offset], sizeof(float));
        return static_cast<double>(value);
      }
    case sensor_msgs::msg::PointField::FLOAT64: {
        if (offset + sizeof(double) > cloud.data.size()) {
          return std::nullopt;
        }
        double value = 0.0;
        std::memcpy(&value, &cloud.data[offset], sizeof(double));
        return value;
      }
    default:
      return std::nullopt;
  }
}

}  // namespace aqua_sonar_loc
