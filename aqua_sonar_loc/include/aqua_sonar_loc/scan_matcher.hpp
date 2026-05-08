#ifndef AQUA_SONAR_LOC__SCAN_MATCHER_HPP_
#define AQUA_SONAR_LOC__SCAN_MATCHER_HPP_

#include <cstddef>
#include <deque>
#include <memory>
#include <string>

#include "aqua_sonar_loc/sonar_cloud_preprocessor.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace aqua_sonar_loc
{

struct ScanMatcherConfig
{
  std::string backend{"noop"};
  double max_correspondence_distance{1.0};
  int max_iterations{50};
  double transformation_epsilon{1.0e-6};
  // Quality gating thresholds applied after PCL ICP completes. A non-positive value
  // disables the gate. Rejected matches set ScanMatchResult::success=false with a
  // descriptive status string and do not advance the accumulated transform.
  double max_fitness_score{-1.0};            // sum of squared correspondences / count
  double max_translation_step_m{-1.0};       // ||T|| of per-step transform
  double max_rotation_step_rad{-1.0};        // angle of per-step rotation
  // When > 1, the icp/gicp backends match the new fan against the concatenation of the
  // last `submap_size` accepted fans (each kept in the most-recent fan's frame). This is
  // the "scan-to-submap" front end that fixes the geometric degeneracy of scan-to-scan
  // multibeam matching where a single fan gives almost no along-track structure. A value
  // of 1 (default) preserves the original scan-to-scan behavior exactly.
  std::size_t submap_size{1};
  // When true, the previous accepted current_to_previous transform is used as the
  // initial guess of the next registration (constant-velocity assumption between fans).
  // Crucial for submap_size > 1: without it, the submap rolls forward using ICP's own
  // zero-motion estimate, which creates a self-confirming feedback loop on degenerate
  // multibeam geometry. For pure scan-to-scan (submap_size == 1), the prior makes very
  // little difference because consecutive fans already overlap heavily.
  bool use_motion_prior{false};
};

struct ScanMatchResult
{
  bool success{false};
  bool converged{false};
  double fitness_score{0.0};
  std::string status;
  geometry_msgs::msg::Transform odom_to_base;
};

class ScanMatcher
{
public:
  virtual ~ScanMatcher() = default;
  virtual void configure(const ScanMatcherConfig & config) = 0;
  virtual ScanMatchResult match(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const CloudSummary & summary) = 0;
  virtual std::string backend_name() const = 0;
};

class NoopScanMatcher final : public ScanMatcher
{
public:
  void configure(const ScanMatcherConfig & config) override;
  ScanMatchResult match(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const CloudSummary & summary) override;
  std::string backend_name() const override;

private:
  ScanMatcherConfig config_;
};

class IcpScanMatcher final : public ScanMatcher
{
public:
  void configure(const ScanMatcherConfig & config) override;
  ScanMatchResult match(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const CloudSummary & summary) override;
  std::string backend_name() const override;

private:
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

  ScanMatcherConfig config_;
  std::deque<PointCloud::Ptr> previous_clouds_;
  Eigen::Matrix4f accumulated_transform_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f last_current_to_previous_{Eigen::Matrix4f::Identity()};
};

class GicpScanMatcher final : public ScanMatcher
{
public:
  void configure(const ScanMatcherConfig & config) override;
  ScanMatchResult match(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const CloudSummary & summary) override;
  std::string backend_name() const override;

private:
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

  ScanMatcherConfig config_;
  std::deque<PointCloud::Ptr> previous_clouds_;
  Eigen::Matrix4f accumulated_transform_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f last_current_to_previous_{Eigen::Matrix4f::Identity()};
};

std::unique_ptr<ScanMatcher> create_scan_matcher(const std::string & backend);

}  // namespace aqua_sonar_loc

#endif  // AQUA_SONAR_LOC__SCAN_MATCHER_HPP_
