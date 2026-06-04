#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace aqua_sonar_loc
{

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

struct SubmapDescriptor
{
  bool valid{false};
  std::size_t point_count{0};
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
  Eigen::Vector3d extent{Eigen::Vector3d::Zero()};
};

struct Submap
{
  std::uint32_t id{0};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  PointCloud::Ptr cloud{std::make_shared<PointCloud>()};
  SubmapDescriptor descriptor;
};

struct MatchResult
{
  bool success{false};
  bool converged{false};
  double fitness{0.0};
  Eigen::Isometry3d candidate_to_current{Eigen::Isometry3d::Identity()};
  std::string status;
};

struct GateResult
{
  bool accepted{false};
  bool correction_pose_valid{false};
  Eigen::Isometry3d correction{Eigen::Isometry3d::Identity()};
  double correction_translation_m{std::numeric_limits<double>::quiet_NaN()};
  double correction_rotation_rad{std::numeric_limits<double>::quiet_NaN()};
  std::size_t consistency_support_count{0};
  std::size_t consistency_required_support_count{0};
  double consistency_nearest_translation_delta_m{
    std::numeric_limits<double>::quiet_NaN()};
  double consistency_nearest_rotation_delta_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double descriptor_centroid_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double descriptor_extent_ratio{std::numeric_limits<double>::quiet_NaN()};
  double descriptor_point_count_ratio{std::numeric_limits<double>::quiet_NaN()};
  std::string status;
};

struct SubmapManagerOptions
{
  int max_submaps{200};
  int min_points_per_submap{300};
  int max_points_per_submap{20000};
  double voxel_leaf_m{0.5};
};

enum class FinalizeSubmapStatus
{
  Empty,
  TooFewPoints,
  TooFewPointsAfterDownsample,
  Ready,
};

struct FinalizeSubmapResult
{
  FinalizeSubmapStatus status{FinalizeSubmapStatus::Empty};
  Submap submap;
  std::size_t raw_points{0};
  std::size_t final_points{0};
};

struct CandidateSelectionOptions
{
  int min_keyframe_separation{20};
  double max_distance_m{15.0};
  int max_per_keyframe{5};
  double descriptor_weight{0.0};
  double descriptor_centroid_scale_m{0.6};
  double descriptor_extent_scale{0.15};
  double descriptor_point_count_ratio_scale{0.10};
};

struct RegistrationOptions
{
  std::string backend{"gicp"};
  int max_iterations{60};
  double max_correspondence_distance_m{3.0};
  double transformation_epsilon{1.0e-6};
  double ndt_resolution_m{1.0};
  double ndt_step_size_m{0.1};
  double ndt_outlier_ratio{0.55};
};

struct GateOptions
{
  double max_fitness_score{2.0};
  double max_correction_translation_m{5.0};
  double max_correction_rotation_rad{0.5};
  double min_plan_view_separation_m{0.0};
  double max_short_plan_view_rotation_rad{0.0};
};

struct DescriptorGateOptions
{
  double max_centroid_distance_m{0.0};
  double max_extent_ratio{0.0};
  double min_point_count_ratio{0.0};
};

struct LoopSuppressionOptions
{
  int min_repeat_keyframe_gap{0};
  double max_consistency_translation_delta_m{0.0};
  double max_consistency_rotation_delta_rad{0.0};
  int min_consistency_support_count{1};
};

struct AcceptedLoop
{
  std::uint32_t from_id{0};
  std::uint32_t to_id{0};
  Eigen::Isometry3d correction{Eigen::Isometry3d::Identity()};
};

struct ConsistencyCheckResult
{
  bool consistent{true};
  std::size_t support_count{0};
  std::size_t required_support_count{0};
  double nearest_translation_delta_m{std::numeric_limits<double>::quiet_NaN()};
  double nearest_rotation_delta_rad{std::numeric_limits<double>::quiet_NaN()};
};

Eigen::Isometry3d pose_to_isometry(const geometry_msgs::msg::Pose & msg);
geometry_msgs::msg::Pose isometry_to_pose(const Eigen::Isometry3d & pose);
PointCloud::Ptr convert_cloud(const sensor_msgs::msg::PointCloud2 & msg);
PointCloud::Ptr downsample(const PointCloud & cloud, double voxel_leaf_m);
SubmapDescriptor describe_cloud(const PointCloud & cloud);
std::array<double, 36> diagonal_information(
  double translation_sigma_m,
  double rotation_sigma_rad);

class SubmapManager
{
public:
  explicit SubmapManager(SubmapManagerOptions options);

  void start_submap(std::uint32_t id, const rclcpp::Time & stamp, const Eigen::Isometry3d & pose);
  bool has_active_points() const;
  void append_points(const PointCloud & cloud);
  FinalizeSubmapResult finalize_current();
  void add_finalized_submap(const Submap & submap);

  const std::deque<Submap> & submaps() const;

private:
  SubmapManagerOptions options_;
  Submap current_submap_;
  std::deque<Submap> submaps_;
};

class LoopCandidateSelector
{
public:
  explicit LoopCandidateSelector(CandidateSelectionOptions options);

  std::vector<Submap> ranked_candidates(
    const std::deque<Submap> & submaps,
    const Submap & current) const;

private:
  CandidateSelectionOptions options_;
};

class RegistrationPipeline
{
public:
  explicit RegistrationPipeline(RegistrationOptions options);

  MatchResult match(
    const Submap & candidate,
    const Submap & current,
    const Eigen::Isometry3d & current_to_candidate_guess) const;

private:
  RegistrationOptions options_;
};

class DescriptorGateEvaluator
{
public:
  explicit DescriptorGateEvaluator(DescriptorGateOptions options);

  GateResult evaluate(const Submap & candidate, const Submap & current) const;

private:
  DescriptorGateOptions options_;
};

class LoopGateEvaluator
{
public:
  explicit LoopGateEvaluator(GateOptions options);

  GateResult evaluate(
    const Eigen::Isometry3d & guess,
    const MatchResult & result) const;

private:
  GateOptions options_;
};

class AcceptedLoopTracker
{
public:
  explicit AcceptedLoopTracker(LoopSuppressionOptions options);

  bool is_suppressed(std::uint32_t from_id, std::uint32_t to_id) const;
  ConsistencyCheckResult check_consistency(const Eigen::Isometry3d & correction) const;
  bool is_consistent(const Eigen::Isometry3d & correction) const;
  void record(std::uint32_t from_id, std::uint32_t to_id);
  void record(
    std::uint32_t from_id,
    std::uint32_t to_id,
    const Eigen::Isometry3d & correction);

  const std::vector<AcceptedLoop> & accepted_loops() const;

private:
  LoopSuppressionOptions options_;
  std::vector<AcceptedLoop> accepted_loops_;
};

}  // namespace aqua_sonar_loc
