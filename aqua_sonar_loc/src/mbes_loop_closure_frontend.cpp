#include "aqua_sonar_loc/mbes_loop_closure_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

namespace aqua_sonar_loc
{

namespace
{

template<typename RegistrationT>
MatchResult run_registration(
  RegistrationT & registration,
  const PointCloud::Ptr & candidate,
  const PointCloud::Ptr & current,
  const Eigen::Isometry3d & current_to_candidate_guess,
  const RegistrationOptions & options)
{
  MatchResult result;
  registration.setInputSource(current);
  registration.setInputTarget(candidate);
  registration.setMaximumIterations(options.max_iterations);
  registration.setMaxCorrespondenceDistance(options.max_correspondence_distance_m);
  registration.setTransformationEpsilon(options.transformation_epsilon);

  PointCloud aligned;
  registration.align(aligned, current_to_candidate_guess.matrix().cast<float>());
  result.converged = registration.hasConverged();
  result.fitness = registration.getFitnessScore();
  if (!result.converged || !std::isfinite(result.fitness)) {
    result.status = "registration did not converge";
    return result;
  }

  const Eigen::Matrix4f current_to_candidate_f = registration.getFinalTransformation();
  const Eigen::Isometry3d current_to_candidate(current_to_candidate_f.cast<double>());
  result.candidate_to_current = current_to_candidate.inverse();
  result.success = true;
  result.status = "registration converged";
  return result;
}

}  // namespace

Eigen::Isometry3d pose_to_isometry(const geometry_msgs::msg::Pose & msg)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() << msg.position.x, msg.position.y, msg.position.z;
  Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  if (q.norm() < 1.0e-9) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  pose.linear() = q.toRotationMatrix();
  return pose;
}

geometry_msgs::msg::Pose isometry_to_pose(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::Pose msg;
  const Eigen::Vector3d t = pose.translation();
  const Eigen::Quaterniond q(pose.linear());
  msg.position.x = t.x();
  msg.position.y = t.y();
  msg.position.z = t.z();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  return msg;
}

PointCloud::Ptr convert_cloud(const sensor_msgs::msg::PointCloud2 & msg)
{
  auto cloud = std::make_shared<PointCloud>();
  pcl::fromROSMsg(msg, *cloud);
  std::vector<int> finite_indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, finite_indices);
  return cloud;
}

PointCloud::Ptr downsample(const PointCloud & cloud, double voxel_leaf_m)
{
  auto out = std::make_shared<PointCloud>();
  if (voxel_leaf_m <= 0.0 || cloud.empty()) {
    *out = cloud;
    return out;
  }
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(
    static_cast<float>(voxel_leaf_m),
    static_cast<float>(voxel_leaf_m),
    static_cast<float>(voxel_leaf_m));
  voxel.setInputCloud(cloud.makeShared());
  voxel.filter(*out);
  return out;
}

std::array<double, 36> diagonal_information(
  double translation_sigma_m,
  double rotation_sigma_rad)
{
  std::array<double, 36> info{};
  const double translation_info = 1.0 / (translation_sigma_m * translation_sigma_m);
  const double rotation_info = 1.0 / (rotation_sigma_rad * rotation_sigma_rad);
  for (int i = 0; i < 3; ++i) {
    info[static_cast<std::size_t>(i * 6 + i)] = translation_info;
  }
  for (int i = 3; i < 6; ++i) {
    info[static_cast<std::size_t>(i * 6 + i)] = rotation_info;
  }
  return info;
}

SubmapManager::SubmapManager(SubmapManagerOptions options)
: options_(std::move(options))
{
}

void SubmapManager::start_submap(
  std::uint32_t id,
  const rclcpp::Time & stamp,
  const Eigen::Isometry3d & pose)
{
  current_submap_ = Submap{};
  current_submap_.id = id;
  current_submap_.stamp = stamp;
  current_submap_.pose = pose;
  current_submap_.cloud = std::make_shared<PointCloud>();
}

bool SubmapManager::has_active_points() const
{
  return current_submap_.cloud && !current_submap_.cloud->empty();
}

void SubmapManager::append_points(const PointCloud & cloud)
{
  if (!current_submap_.cloud) {
    return;
  }
  if (static_cast<int>(current_submap_.cloud->size()) >= options_.max_points_per_submap) {
    return;
  }

  const int remaining = options_.max_points_per_submap -
    static_cast<int>(current_submap_.cloud->size());
  const int count = std::min<int>(remaining, static_cast<int>(cloud.size()));
  current_submap_.cloud->insert(
    current_submap_.cloud->end(), cloud.begin(), cloud.begin() + count);
}

FinalizeSubmapResult SubmapManager::finalize_current()
{
  FinalizeSubmapResult result;
  result.submap = current_submap_;
  if (!current_submap_.cloud || current_submap_.cloud->empty()) {
    result.status = FinalizeSubmapStatus::Empty;
    return result;
  }

  result.raw_points = current_submap_.cloud->size();
  if (static_cast<int>(result.raw_points) < options_.min_points_per_submap) {
    result.status = FinalizeSubmapStatus::TooFewPoints;
    return result;
  }

  current_submap_.cloud = downsample(*current_submap_.cloud, options_.voxel_leaf_m);
  result.submap = current_submap_;
  result.final_points = current_submap_.cloud->size();
  if (static_cast<int>(result.final_points) < options_.min_points_per_submap) {
    result.status = FinalizeSubmapStatus::TooFewPointsAfterDownsample;
    return result;
  }

  result.status = FinalizeSubmapStatus::Ready;
  return result;
}

void SubmapManager::add_finalized_submap(const Submap & submap)
{
  submaps_.push_back(submap);
  while (static_cast<int>(submaps_.size()) > options_.max_submaps) {
    submaps_.pop_front();
  }
}

const std::deque<Submap> & SubmapManager::submaps() const
{
  return submaps_;
}

LoopCandidateSelector::LoopCandidateSelector(CandidateSelectionOptions options)
: options_(std::move(options))
{
}

std::vector<Submap> LoopCandidateSelector::ranked_candidates(
  const std::deque<Submap> & submaps,
  const Submap & current) const
{
  std::vector<Submap> candidates;
  for (const auto & candidate : submaps) {
    if (current.id <= candidate.id + static_cast<std::uint32_t>(options_.min_keyframe_separation)) {
      continue;
    }
    const double distance =
      (current.pose.translation() - candidate.pose.translation()).norm();
    if (options_.max_distance_m > 0.0 && distance > options_.max_distance_m) {
      continue;
    }
    candidates.push_back(candidate);
  }
  std::sort(candidates.begin(), candidates.end(), [&current](const Submap & a, const Submap & b) {
    const double da = (current.pose.translation() - a.pose.translation()).squaredNorm();
    const double db = (current.pose.translation() - b.pose.translation()).squaredNorm();
    return da < db;
  });
  return candidates;
}

RegistrationPipeline::RegistrationPipeline(RegistrationOptions options)
: options_(std::move(options))
{
}

MatchResult RegistrationPipeline::match(
  const Submap & candidate,
  const Submap & current,
  const Eigen::Isometry3d & current_to_candidate_guess) const
{
  if (options_.backend == "icp") {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    return run_registration(
      icp, candidate.cloud, current.cloud, current_to_candidate_guess, options_);
  }
  if (options_.backend == "ndt") {
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setResolution(static_cast<float>(options_.ndt_resolution_m));
    ndt.setStepSize(options_.ndt_step_size_m);
    ndt.setOulierRatio(options_.ndt_outlier_ratio);
    return run_registration(
      ndt, candidate.cloud, current.cloud, current_to_candidate_guess, options_);
  }

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  return run_registration(
    gicp, candidate.cloud, current.cloud, current_to_candidate_guess, options_);
}

LoopGateEvaluator::LoopGateEvaluator(GateOptions options)
: options_(std::move(options))
{
}

GateResult LoopGateEvaluator::evaluate(
  const Eigen::Isometry3d & guess,
  const MatchResult & result) const
{
  GateResult gate;
  if (!result.success) {
    gate.status = result.status.empty() ? "registration failed" : result.status;
    return gate;
  }
  const Eigen::Isometry3d correction = guess.inverse() * result.candidate_to_current;
  gate.correction_translation_m = correction.translation().norm();
  const Eigen::AngleAxisd aa(correction.linear());
  gate.correction_rotation_rad = std::abs(aa.angle());

  if (options_.max_fitness_score > 0.0 && result.fitness > options_.max_fitness_score) {
    gate.status = "fitness score exceeds gate";
    return gate;
  }
  if (options_.max_correction_translation_m > 0.0 &&
    gate.correction_translation_m > options_.max_correction_translation_m)
  {
    gate.status = "translation correction exceeds gate";
    return gate;
  }
  if (options_.max_correction_rotation_rad > 0.0 &&
    gate.correction_rotation_rad > options_.max_correction_rotation_rad)
  {
    gate.status = "rotation correction exceeds gate";
    return gate;
  }
  gate.accepted = true;
  gate.status = "accepted";
  return gate;
}

}  // namespace aqua_sonar_loc
