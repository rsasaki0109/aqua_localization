#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include "aqua_msgs/msg/loop_closure_status.hpp"
#include "aqua_msgs/msg/pose_graph_keyframe.hpp"
#include "aqua_msgs/msg/pose_graph_loop_constraint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace aqua_sonar_loc
{

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

struct Submap
{
  std::uint32_t id{0};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  PointCloud::Ptr cloud{std::make_shared<PointCloud>()};
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
  double correction_translation_m{std::numeric_limits<double>::quiet_NaN()};
  double correction_rotation_rad{std::numeric_limits<double>::quiet_NaN()};
  std::string status;
};

namespace
{

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

template<typename RegistrationT>
MatchResult run_registration(
  RegistrationT & registration,
  const PointCloud::Ptr & candidate,
  const PointCloud::Ptr & current,
  const Eigen::Isometry3d & current_to_candidate_guess,
  int max_iterations,
  double max_correspondence_distance,
  double transformation_epsilon)
{
  MatchResult result;
  registration.setInputSource(current);
  registration.setInputTarget(candidate);
  registration.setMaximumIterations(max_iterations);
  registration.setMaxCorrespondenceDistance(max_correspondence_distance);
  registration.setTransformationEpsilon(transformation_epsilon);

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

class MbesLoopClosureNode : public rclcpp::Node
{
public:
  explicit MbesLoopClosureNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mbes_loop_closure", options)
  {
    load_parameters();

    keyframe_sub_ = create_subscription<aqua_msgs::msg::PoseGraphKeyframe>(
      keyframe_topic_, rclcpp::QoS(20).transient_local(),
      [this](const aqua_msgs::msg::PoseGraphKeyframe::SharedPtr msg) {
        on_keyframe(*msg);
      });
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        on_points(*msg);
      });
    loop_pub_ = create_publisher<aqua_msgs::msg::PoseGraphLoopConstraint>(
      loop_constraint_topic_, rclcpp::QoS(10));
    status_pub_ = create_publisher<aqua_msgs::msg::LoopClosureStatus>(
      status_topic_, rclcpp::QoS(10));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(),
      "mbes_loop_closure started: points=%s keyframes=%s loops=%s status=%s markers=%s backend=%s",
      points_topic_.c_str(), keyframe_topic_.c_str(),
      loop_constraint_topic_.c_str(), status_topic_.c_str(), marker_topic_.c_str(),
      backend_.c_str());
  }

private:
  void load_parameters()
  {
    points_topic_ = declare_parameter<std::string>(
      "topics.points", "/aqua_sonar_loc/points_filtered");
    keyframe_topic_ = declare_parameter<std::string>(
      "topics.keyframe", "/aqua_pose_graph/keyframe");
    loop_constraint_topic_ = declare_parameter<std::string>(
      "topics.loop_constraint", "/aqua_pose_graph/loop_constraint");
    status_topic_ = declare_parameter<std::string>(
      "topics.status", "/mbes_loop_closure/status");
    marker_topic_ = declare_parameter<std::string>(
      "topics.markers", "/mbes_loop_closure/markers");
    map_frame_ = declare_parameter<std::string>("frames.map", "map");

    max_submaps_ = declare_parameter<int>("submaps.max_submaps", 200);
    min_points_per_submap_ = declare_parameter<int>("submaps.min_points", 300);
    max_points_per_submap_ = declare_parameter<int>("submaps.max_points", 20000);
    voxel_leaf_m_ = declare_parameter<double>("submaps.voxel_leaf_m", 0.5);

    min_keyframe_separation_ =
      declare_parameter<int>("candidates.min_keyframe_separation", 20);
    candidate_max_distance_m_ =
      declare_parameter<double>("candidates.max_distance_m", 15.0);
    max_candidates_per_keyframe_ =
      declare_parameter<int>("candidates.max_per_keyframe", 5);

    backend_ = declare_parameter<std::string>("registration.backend", "gicp");
    max_iterations_ = declare_parameter<int>("registration.max_iterations", 60);
    max_correspondence_distance_m_ =
      declare_parameter<double>("registration.max_correspondence_distance_m", 3.0);
    transformation_epsilon_ =
      declare_parameter<double>("registration.transformation_epsilon", 1.0e-6);
    ndt_resolution_m_ = declare_parameter<double>("registration.ndt.resolution_m", 1.0);
    ndt_step_size_m_ = declare_parameter<double>("registration.ndt.step_size_m", 0.1);
    ndt_outlier_ratio_ =
      declare_parameter<double>("registration.ndt.outlier_ratio", 0.55);

    max_fitness_score_ = declare_parameter<double>("gates.max_fitness_score", 2.0);
    max_correction_translation_m_ =
      declare_parameter<double>("gates.max_correction_translation_m", 5.0);
    max_correction_rotation_rad_ =
      declare_parameter<double>("gates.max_correction_rotation_rad", 0.5);

    loop_translation_sigma_m_ =
      declare_parameter<double>("loop.translation_sigma_m", 2.0);
    loop_rotation_sigma_rad_ =
      declare_parameter<double>("loop.rotation_sigma_rad", 0.35);
    optimize_after_insert_ =
      declare_parameter<bool>("loop.optimize_after_insert", true);
  }

  void on_keyframe(const aqua_msgs::msg::PoseGraphKeyframe & msg)
  {
    if (current_submap_.cloud && !current_submap_.cloud->empty()) {
      finalize_current_submap();
    }

    current_submap_ = Submap{};
    current_submap_.id = msg.id;
    current_submap_.stamp = rclcpp::Time(msg.header.stamp);
    current_submap_.pose = pose_to_isometry(msg.pose);
    current_submap_.cloud = std::make_shared<PointCloud>();
  }

  void on_points(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if (!current_submap_.cloud) {
      return;
    }
    if (static_cast<int>(current_submap_.cloud->size()) >= max_points_per_submap_) {
      return;
    }

    const auto cloud = convert_cloud(msg);
    if (!cloud || cloud->empty()) {
      return;
    }
    const int remaining = max_points_per_submap_ -
      static_cast<int>(current_submap_.cloud->size());
    const int count = std::min<int>(remaining, static_cast<int>(cloud->size()));
    current_submap_.cloud->insert(
      current_submap_.cloud->end(), cloud->begin(), cloud->begin() + count);
  }

  void finalize_current_submap()
  {
    if (static_cast<int>(current_submap_.cloud->size()) < min_points_per_submap_) {
      RCLCPP_DEBUG(
        get_logger(), "drop submap %u: only %zu points",
        current_submap_.id, current_submap_.cloud->size());
      return;
    }

    current_submap_.cloud = downsample(*current_submap_.cloud, voxel_leaf_m_);
    if (static_cast<int>(current_submap_.cloud->size()) < min_points_per_submap_) {
      RCLCPP_DEBUG(
        get_logger(), "drop submap %u after voxel filter: only %zu points",
        current_submap_.id, current_submap_.cloud->size());
      return;
    }

    try_loop_closure(current_submap_);
    submaps_.push_back(current_submap_);
    while (static_cast<int>(submaps_.size()) > max_submaps_) {
      submaps_.pop_front();
    }
  }

  void try_loop_closure(const Submap & current)
  {
    int tested = 0;
    for (const auto & candidate : ranked_candidates(current)) {
      if (tested >= max_candidates_per_keyframe_) {
        break;
      }
      ++tested;

      const Eigen::Isometry3d candidate_to_current_guess =
        candidate.pose.inverse() * current.pose;
      const Eigen::Isometry3d current_to_candidate_guess =
        candidate_to_current_guess.inverse();
      MatchResult result =
        match_submaps(candidate, current, current_to_candidate_guess);
      const GateResult gate = evaluate_gates(candidate_to_current_guess, result);
      publish_status(candidate, current, result, gate);
      publish_candidate_marker(candidate, current, gate);
      if (!gate.accepted) {
        RCLCPP_DEBUG(
          get_logger(), "rejected loop candidate %u -> %u: %s fitness=%.4f",
          candidate.id, current.id, gate.status.c_str(), result.fitness);
        continue;
      }

      publish_loop_constraint(candidate, current, result.candidate_to_current);
      return;
    }
    if (tested == 0) {
      publish_no_candidate_status(current);
    }
  }

  std::vector<Submap> ranked_candidates(const Submap & current) const
  {
    std::vector<Submap> candidates;
    for (const auto & candidate : submaps_) {
      if (current.id <= candidate.id + static_cast<std::uint32_t>(min_keyframe_separation_)) {
        continue;
      }
      const double distance =
        (current.pose.translation() - candidate.pose.translation()).norm();
      if (candidate_max_distance_m_ > 0.0 && distance > candidate_max_distance_m_) {
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

  MatchResult match_submaps(
    const Submap & candidate,
    const Submap & current,
    const Eigen::Isometry3d & current_to_candidate_guess) const
  {
    if (backend_ == "icp") {
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      return run_registration(
        icp, candidate.cloud, current.cloud, current_to_candidate_guess,
        max_iterations_, max_correspondence_distance_m_, transformation_epsilon_);
    }
    if (backend_ == "ndt") {
      pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
      ndt.setResolution(static_cast<float>(ndt_resolution_m_));
      ndt.setStepSize(ndt_step_size_m_);
      ndt.setOulierRatio(ndt_outlier_ratio_);
      return run_registration(
        ndt, candidate.cloud, current.cloud, current_to_candidate_guess,
        max_iterations_, max_correspondence_distance_m_, transformation_epsilon_);
    }

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    return run_registration(
      gicp, candidate.cloud, current.cloud, current_to_candidate_guess,
      max_iterations_, max_correspondence_distance_m_, transformation_epsilon_);
  }

  GateResult evaluate_gates(
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

    if (max_fitness_score_ > 0.0 && result.fitness > max_fitness_score_) {
      gate.status = "fitness score exceeds gate";
      return gate;
    }
    if (max_correction_translation_m_ > 0.0 &&
      gate.correction_translation_m > max_correction_translation_m_)
    {
      gate.status = "translation correction exceeds gate";
      return gate;
    }
    if (max_correction_rotation_rad_ > 0.0 &&
      gate.correction_rotation_rad > max_correction_rotation_rad_)
    {
      gate.status = "rotation correction exceeds gate";
      return gate;
    }
    gate.accepted = true;
    gate.status = "accepted";
    return gate;
  }

  void publish_loop_constraint(
    const Submap & candidate,
    const Submap & current,
    const Eigen::Isometry3d & candidate_to_current)
  {
    aqua_msgs::msg::PoseGraphLoopConstraint msg;
    msg.header.stamp = current.stamp;
    msg.header.frame_id = map_frame_;
    msg.from_id = candidate.id;
    msg.to_id = current.id;
    msg.relative_pose = isometry_to_pose(candidate_to_current);
    msg.information =
      diagonal_information(loop_translation_sigma_m_, loop_rotation_sigma_rad_);
    msg.optimize_after_insert = optimize_after_insert_;
    loop_pub_->publish(msg);
    RCLCPP_INFO(
      get_logger(), "published MBES loop constraint %u -> %u",
      candidate.id, current.id);
  }

  void publish_status(
    const Submap & candidate,
    const Submap & current,
    const MatchResult & result,
    const GateResult & gate)
  {
    aqua_msgs::msg::LoopClosureStatus msg;
    msg.header.stamp = current.stamp;
    msg.header.frame_id = map_frame_;
    msg.current_id = current.id;
    msg.candidate_id = candidate.id;
    msg.accepted = gate.accepted;
    msg.converged = result.converged;
    msg.fitness_score = result.fitness;
    msg.correction_translation_m = gate.correction_translation_m;
    msg.correction_rotation_rad = gate.correction_rotation_rad;
    msg.status = gate.status;
    status_pub_->publish(msg);
  }

  geometry_msgs::msg::Point marker_point(const Eigen::Vector3d & p) const
  {
    geometry_msgs::msg::Point msg;
    msg.x = p.x();
    msg.y = p.y();
    msg.z = p.z();
    return msg;
  }

  void publish_candidate_marker(
    const Submap & candidate,
    const Submap & current,
    const GateResult & gate)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = current.stamp;
    marker.header.frame_id = map_frame_;
    marker.ns = gate.accepted ? "mbes_loop_closure/accepted" : "mbes_loop_closure/rejected";
    marker.id = static_cast<int>(marker_sequence_++);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = gate.accepted ? 0.12 : 0.05;
    marker.color.a = gate.accepted ? 1.0F : 0.55F;
    marker.color.r = gate.accepted ? 0.10F : 1.0F;
    marker.color.g = gate.accepted ? 0.95F : 0.20F;
    marker.color.b = gate.accepted ? 0.30F : 0.10F;
    marker.points.push_back(marker_point(candidate.pose.translation()));
    marker.points.push_back(marker_point(current.pose.translation()));

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(marker);
    marker_pub_->publish(markers);
  }

  void publish_no_candidate_status(const Submap & current)
  {
    aqua_msgs::msg::LoopClosureStatus msg;
    msg.header.stamp = current.stamp;
    msg.header.frame_id = map_frame_;
    msg.current_id = current.id;
    msg.candidate_id = std::numeric_limits<std::uint32_t>::max();
    msg.accepted = false;
    msg.converged = false;
    msg.fitness_score = std::numeric_limits<double>::quiet_NaN();
    msg.correction_translation_m = std::numeric_limits<double>::quiet_NaN();
    msg.correction_rotation_rad = std::numeric_limits<double>::quiet_NaN();
    msg.status = "no candidate submaps";
    status_pub_->publish(msg);
  }

  std::string points_topic_;
  std::string keyframe_topic_;
  std::string loop_constraint_topic_;
  std::string status_topic_;
  std::string marker_topic_;
  std::string map_frame_;

  int max_submaps_{200};
  int min_points_per_submap_{300};
  int max_points_per_submap_{20000};
  double voxel_leaf_m_{0.5};

  int min_keyframe_separation_{20};
  double candidate_max_distance_m_{15.0};
  int max_candidates_per_keyframe_{5};

  std::string backend_{"gicp"};
  int max_iterations_{60};
  double max_correspondence_distance_m_{3.0};
  double transformation_epsilon_{1.0e-6};
  double ndt_resolution_m_{1.0};
  double ndt_step_size_m_{0.1};
  double ndt_outlier_ratio_{0.55};

  double max_fitness_score_{2.0};
  double max_correction_translation_m_{5.0};
  double max_correction_rotation_rad_{0.5};

  double loop_translation_sigma_m_{2.0};
  double loop_rotation_sigma_rad_{0.35};
  bool optimize_after_insert_{true};
  std::uint32_t marker_sequence_{0};

  Submap current_submap_;
  std::deque<Submap> submaps_;

  rclcpp::Subscription<aqua_msgs::msg::PoseGraphKeyframe>::SharedPtr keyframe_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Publisher<aqua_msgs::msg::PoseGraphLoopConstraint>::SharedPtr loop_pub_;
  rclcpp::Publisher<aqua_msgs::msg::LoopClosureStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace aqua_sonar_loc

#ifndef AQUA_SONAR_LOC_DISABLE_MBES_LOOP_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aqua_sonar_loc::MbesLoopClosureNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
