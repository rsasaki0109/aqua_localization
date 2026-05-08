#ifndef AQUA_SONAR_LOC__SONAR_CLOUD_PREPROCESSOR_HPP_
#define AQUA_SONAR_LOC__SONAR_CLOUD_PREPROCESSOR_HPP_

#include <cstddef>
#include <optional>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace aqua_sonar_loc
{

struct SonarCloudPreprocessorConfig
{
  bool require_xyz{true};
  bool enable_range_filter{true};
  double max_range_m{80.0};
  size_t min_points{20};
};

struct CloudSummary
{
  size_t total_points{0};
  size_t finite_xyz_points{0};
  size_t in_range_points{0};
  bool accepted{false};
  std::string rejection_reason;
};

class SonarCloudPreprocessor
{
public:
  void configure(const SonarCloudPreprocessorConfig & config);
  CloudSummary summarize(const sensor_msgs::msg::PointCloud2 & cloud) const;

private:
  struct FieldLayout
  {
    const sensor_msgs::msg::PointField * x{nullptr};
    const sensor_msgs::msg::PointField * y{nullptr};
    const sensor_msgs::msg::PointField * z{nullptr};
  };

  FieldLayout find_xyz_fields(const sensor_msgs::msg::PointCloud2 & cloud) const;
  std::optional<double> read_numeric_field(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const sensor_msgs::msg::PointField & field,
    size_t point_offset) const;

  SonarCloudPreprocessorConfig config_;
};

}  // namespace aqua_sonar_loc

#endif  // AQUA_SONAR_LOC__SONAR_CLOUD_PREPROCESSOR_HPP_
