#include "aqua_sonar_loc/scan_matcher.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

namespace aqua_sonar_loc
{

namespace
{
std::string lowercase(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

using PclPointCloud = pcl::PointCloud<pcl::PointXYZ>;

PclPointCloud::Ptr convert_cloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  auto converted = std::make_shared<PclPointCloud>();
  pcl::fromROSMsg(cloud, *converted);
  std::vector<int> finite_indices;
  pcl::removeNaNFromPointCloud(*converted, *converted, finite_indices);
  return converted;
}

geometry_msgs::msg::Transform accumulated_transform_msg(const Eigen::Matrix4f & accumulated)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = accumulated(0, 3);
  transform.translation.y = accumulated(1, 3);
  transform.translation.z = accumulated(2, 3);

  const Eigen::Matrix3f rotation = accumulated.block<3, 3>(0, 0);
  const Eigen::Quaternionf quaternion(rotation);
  transform.rotation.x = quaternion.x();
  transform.rotation.y = quaternion.y();
  transform.rotation.z = quaternion.z();
  transform.rotation.w = quaternion.w();
  return transform;
}

PclPointCloud::Ptr concatenate_clouds(const std::deque<PclPointCloud::Ptr> & clouds)
{
  auto out = std::make_shared<PclPointCloud>();
  std::size_t total = 0;
  for (const auto & cloud : clouds) {
    if (cloud) {
      total += cloud->size();
    }
  }
  out->reserve(total);
  for (const auto & cloud : clouds) {
    if (!cloud) {
      continue;
    }
    out->insert(out->end(), cloud->begin(), cloud->end());
  }
  return out;
}

template <typename PclRegistration>
ScanMatchResult run_pcl_registration(
  PclRegistration & registration,
  const ScanMatcherConfig & config,
  const std::string & backend_label,
  const sensor_msgs::msg::PointCloud2 & cloud,
  const CloudSummary & summary,
  std::deque<PclPointCloud::Ptr> & previous_clouds,
  Eigen::Matrix4f & accumulated_transform,
  Eigen::Matrix4f & last_current_to_previous)
{
  ScanMatchResult result;
  result.odom_to_base.rotation.w = 1.0;

  if (!summary.accepted) {
    result.status = summary.rejection_reason;
    return result;
  }

  auto current_cloud = convert_cloud(cloud);
  if (!current_cloud || current_cloud->empty()) {
    result.status = backend_label + " rejected empty converted cloud";
    return result;
  }

  const std::size_t submap_size = std::max<std::size_t>(config.submap_size, 1);

  if (previous_clouds.empty()) {
    previous_clouds.push_back(current_cloud);
    result.success = true;
    result.converged = true;
    result.fitness_score = 0.0;
    result.status = backend_label + " initialized";
    result.odom_to_base = accumulated_transform_msg(accumulated_transform);
    return result;
  }

  // The deque holds previous fans expressed in the most-recent successfully matched
  // fan's frame. The submap target is just the concatenation; for submap_size==1 this
  // reduces to a single-cloud target identical to the original scan-to-scan path.
  PclPointCloud::Ptr target =
    submap_size == 1 ? previous_clouds.back() : concatenate_clouds(previous_clouds);

  registration.setInputSource(current_cloud);
  registration.setInputTarget(target);
  registration.setMaxCorrespondenceDistance(config.max_correspondence_distance);
  registration.setMaximumIterations(config.max_iterations);
  registration.setTransformationEpsilon(config.transformation_epsilon);

  PclPointCloud aligned;
  if (config.use_motion_prior) {
    registration.align(aligned, last_current_to_previous);
  } else {
    registration.align(aligned);
  }

  result.converged = registration.hasConverged();
  result.fitness_score = registration.getFitnessScore();
  if (!result.converged || !std::isfinite(result.fitness_score)) {
    result.status = backend_label + " did not converge";
    return result;
  }

  if (config.max_fitness_score > 0.0 && result.fitness_score > config.max_fitness_score) {
    result.status = backend_label + " rejected: fitness_score above max_fitness_score";
    return result;
  }

  const Eigen::Matrix4f current_to_previous = registration.getFinalTransformation();
  const Eigen::Matrix4f previous_to_current = current_to_previous.inverse();

  const Eigen::Vector3f translation_step = previous_to_current.block<3, 1>(0, 3);
  if (config.max_translation_step_m > 0.0 &&
    translation_step.norm() > config.max_translation_step_m)
  {
    result.status = backend_label + " rejected: translation step above max_translation_step_m";
    return result;
  }

  const Eigen::Matrix3f rotation_step = previous_to_current.block<3, 3>(0, 0);
  const Eigen::AngleAxisf rotation_axis_angle(rotation_step);
  if (config.max_rotation_step_rad > 0.0 &&
    std::abs(rotation_axis_angle.angle()) > config.max_rotation_step_rad)
  {
    result.status = backend_label + " rejected: rotation step above max_rotation_step_rad";
    return result;
  }

  accumulated_transform = accumulated_transform * previous_to_current;
  last_current_to_previous = current_to_previous;

  // Roll the submap forward: every cached fan was expressed in the previous fan's frame;
  // transform each by previous_to_current so the deque + the new fan all live in the
  // current fan's frame. Then push the new fan as identity and trim to submap_size.
  if (submap_size > 1) {
    for (auto & cached : previous_clouds) {
      auto rotated = std::make_shared<PclPointCloud>();
      pcl::transformPointCloud(*cached, *rotated, previous_to_current);
      cached = rotated;
    }
  }
  previous_clouds.push_back(current_cloud);
  while (previous_clouds.size() > submap_size) {
    previous_clouds.pop_front();
  }

  result.success = true;
  result.status = backend_label + " converged";
  result.odom_to_base = accumulated_transform_msg(accumulated_transform);
  return result;
}
}  // namespace

void NoopScanMatcher::configure(const ScanMatcherConfig & config)
{
  config_ = config;
  config_.backend = "noop";
}

ScanMatchResult NoopScanMatcher::match(
  const sensor_msgs::msg::PointCloud2 &,
  const CloudSummary & summary)
{
  ScanMatchResult result;
  result.success = summary.accepted;
  result.converged = summary.accepted;
  result.fitness_score = 0.0;
  result.status = summary.accepted ? "noop identity transform" : summary.rejection_reason;
  result.odom_to_base.rotation.w = 1.0;
  return result;
}

std::string NoopScanMatcher::backend_name() const
{
  return config_.backend;
}

void IcpScanMatcher::configure(const ScanMatcherConfig & config)
{
  config_ = config;
  config_.backend = "icp";
  previous_clouds_.clear();
  accumulated_transform_.setIdentity();
  last_current_to_previous_.setIdentity();
}

ScanMatchResult IcpScanMatcher::match(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const CloudSummary & summary)
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  return run_pcl_registration(
    icp, config_, "icp", cloud, summary,
    previous_clouds_, accumulated_transform_, last_current_to_previous_);
}

std::string IcpScanMatcher::backend_name() const
{
  return config_.backend;
}

void GicpScanMatcher::configure(const ScanMatcherConfig & config)
{
  config_ = config;
  config_.backend = "gicp";
  previous_clouds_.clear();
  accumulated_transform_.setIdentity();
  last_current_to_previous_.setIdentity();
}

ScanMatchResult GicpScanMatcher::match(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const CloudSummary & summary)
{
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  return run_pcl_registration(
    gicp, config_, "gicp", cloud, summary,
    previous_clouds_, accumulated_transform_, last_current_to_previous_);
}

std::string GicpScanMatcher::backend_name() const
{
  return config_.backend;
}

std::unique_ptr<ScanMatcher> create_scan_matcher(const std::string & backend)
{
  const auto normalized = lowercase(backend);
  if (normalized == "noop" || normalized == "identity") {
    return std::make_unique<NoopScanMatcher>();
  }
  if (normalized == "icp") {
    return std::make_unique<IcpScanMatcher>();
  }
  if (normalized == "gicp") {
    return std::make_unique<GicpScanMatcher>();
  }
  return nullptr;
}

}  // namespace aqua_sonar_loc
