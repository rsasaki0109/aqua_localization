#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "aqua_sonar_loc/mbes_loop_closure_frontend.hpp"
#include "aqua_msgs/msg/loop_closure_status.hpp"
#include "aqua_msgs/msg/pose_graph_keyframe.hpp"
#include "aqua_msgs/msg/pose_graph_loop_constraint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace aqua_sonar_loc
{

namespace
{

geometry_msgs::msg::Point marker_point(const Eigen::Vector3d & p)
{
  geometry_msgs::msg::Point msg;
  msg.x = p.x();
  msg.y = p.y();
  msg.z = p.z();
  return msg;
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
      registration_options_.backend.c_str());
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

    submap_options_.max_submaps = declare_parameter<int>("submaps.max_submaps", 200);
    submap_options_.min_points_per_submap = declare_parameter<int>("submaps.min_points", 300);
    submap_options_.max_points_per_submap = declare_parameter<int>("submaps.max_points", 20000);
    submap_options_.voxel_leaf_m = declare_parameter<double>("submaps.voxel_leaf_m", 0.5);

    candidate_options_.min_keyframe_separation =
      declare_parameter<int>("candidates.min_keyframe_separation", 20);
    candidate_options_.max_distance_m =
      declare_parameter<double>("candidates.max_distance_m", 15.0);
    candidate_options_.max_per_keyframe =
      declare_parameter<int>("candidates.max_per_keyframe", 5);

    registration_options_.backend = declare_parameter<std::string>("registration.backend", "gicp");
    registration_options_.max_iterations =
      declare_parameter<int>("registration.max_iterations", 60);
    registration_options_.max_correspondence_distance_m =
      declare_parameter<double>("registration.max_correspondence_distance_m", 3.0);
    registration_options_.transformation_epsilon =
      declare_parameter<double>("registration.transformation_epsilon", 1.0e-6);
    registration_options_.ndt_resolution_m =
      declare_parameter<double>("registration.ndt.resolution_m", 1.0);
    registration_options_.ndt_step_size_m =
      declare_parameter<double>("registration.ndt.step_size_m", 0.1);
    registration_options_.ndt_outlier_ratio =
      declare_parameter<double>("registration.ndt.outlier_ratio", 0.55);

    gate_options_.max_fitness_score =
      declare_parameter<double>("gates.max_fitness_score", 2.0);
    gate_options_.max_correction_translation_m =
      declare_parameter<double>("gates.max_correction_translation_m", 5.0);
    gate_options_.max_correction_rotation_rad =
      declare_parameter<double>("gates.max_correction_rotation_rad", 0.5);

    descriptor_options_.max_centroid_distance_m =
      declare_parameter<double>("descriptor.max_centroid_distance_m", 0.0);
    descriptor_options_.max_extent_ratio =
      declare_parameter<double>("descriptor.max_extent_ratio", 0.0);
    descriptor_options_.min_point_count_ratio =
      declare_parameter<double>("descriptor.min_point_count_ratio", 0.0);

    loop_translation_sigma_m_ =
      declare_parameter<double>("loop.translation_sigma_m", 2.0);
    loop_rotation_sigma_rad_ =
      declare_parameter<double>("loop.rotation_sigma_rad", 0.35);
    optimize_after_insert_ =
      declare_parameter<bool>("loop.optimize_after_insert", true);
    loop_suppression_options_.min_repeat_keyframe_gap =
      declare_parameter<int>("loop.min_repeat_keyframe_gap", 5);

    submap_manager_ = SubmapManager(submap_options_);
    accepted_loop_tracker_ = AcceptedLoopTracker(loop_suppression_options_);
  }

  void on_keyframe(const aqua_msgs::msg::PoseGraphKeyframe & msg)
  {
    if (submap_manager_.has_active_points()) {
      finalize_current_submap();
    }

    submap_manager_.start_submap(
      msg.id, rclcpp::Time(msg.header.stamp), pose_to_isometry(msg.pose));
  }

  void on_points(const sensor_msgs::msg::PointCloud2 & msg)
  {
    const auto cloud = convert_cloud(msg);
    if (!cloud || cloud->empty()) {
      return;
    }
    submap_manager_.append_points(*cloud);
  }

  void finalize_current_submap()
  {
    const auto result = submap_manager_.finalize_current();
    if (result.status == FinalizeSubmapStatus::TooFewPoints) {
      publish_submap_status(
        result.submap,
        "too few raw points: " + std::to_string(result.raw_points) + " < " +
        std::to_string(submap_options_.min_points_per_submap));
      RCLCPP_DEBUG(
        get_logger(), "drop submap %u: only %zu points",
        result.submap.id, result.raw_points);
      return;
    }
    if (result.status == FinalizeSubmapStatus::TooFewPointsAfterDownsample) {
      publish_submap_status(
        result.submap,
        "too few downsampled points: " + std::to_string(result.final_points) + " < " +
        std::to_string(submap_options_.min_points_per_submap));
      RCLCPP_DEBUG(
        get_logger(), "drop submap %u after voxel filter: only %zu points",
        result.submap.id, result.final_points);
      return;
    }
    if (result.status != FinalizeSubmapStatus::Ready) {
      return;
    }

    try_loop_closure(result.submap);
    submap_manager_.add_finalized_submap(result.submap);
  }

  void try_loop_closure(const Submap & current)
  {
    int tested = 0;
    const LoopCandidateSelector selector(candidate_options_);
    const DescriptorGateEvaluator descriptor_gate(descriptor_options_);
    const RegistrationPipeline registration(registration_options_);
    const LoopGateEvaluator gate_evaluator(gate_options_);

    for (const auto & candidate : selector.ranked_candidates(submap_manager_.submaps(), current)) {
      if (tested >= candidate_options_.max_per_keyframe) {
        break;
      }
      ++tested;

      const Eigen::Isometry3d candidate_to_current_guess =
        candidate.pose.inverse() * current.pose;
      const Eigen::Isometry3d current_to_candidate_guess =
        candidate_to_current_guess.inverse();
      const GateResult descriptor_result = descriptor_gate.evaluate(candidate, current);
      if (!descriptor_result.accepted) {
        MatchResult result;
        result.fitness = std::numeric_limits<double>::quiet_NaN();
        result.status = descriptor_result.status;
        publish_status(candidate, current, result, descriptor_result);
        publish_candidate_marker(candidate, current, descriptor_result);
        RCLCPP_DEBUG(
          get_logger(), "rejected loop candidate %u -> %u: %s",
          candidate.id, current.id, descriptor_result.status.c_str());
        continue;
      }

      MatchResult result =
        registration.match(candidate, current, current_to_candidate_guess);
      GateResult gate = gate_evaluator.evaluate(candidate_to_current_guess, result);
      gate.descriptor_centroid_distance_m =
        descriptor_result.descriptor_centroid_distance_m;
      gate.descriptor_extent_ratio = descriptor_result.descriptor_extent_ratio;
      gate.descriptor_point_count_ratio = descriptor_result.descriptor_point_count_ratio;
      if (gate.accepted &&
        accepted_loop_tracker_.is_suppressed(candidate.id, current.id))
      {
        gate.accepted = false;
        gate.status = "duplicate loop suppressed";
      }
      publish_status(candidate, current, result, gate);
      publish_candidate_marker(candidate, current, gate);
      if (!gate.accepted) {
        RCLCPP_DEBUG(
          get_logger(), "rejected loop candidate %u -> %u: %s fitness=%.4f",
          candidate.id, current.id, gate.status.c_str(), result.fitness);
        continue;
      }

      publish_loop_constraint(candidate, current, result.candidate_to_current);
      accepted_loop_tracker_.record(candidate.id, current.id);
      return;
    }
    if (tested == 0) {
      publish_submap_status(current, "no candidate submaps");
    }
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
    msg.descriptor_centroid_distance_m = gate.descriptor_centroid_distance_m;
    msg.descriptor_extent_ratio = gate.descriptor_extent_ratio;
    msg.descriptor_point_count_ratio = gate.descriptor_point_count_ratio;
    msg.status = gate.status;
    status_pub_->publish(msg);
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

  void publish_submap_status(const Submap & current, const std::string & status)
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
    msg.descriptor_centroid_distance_m = std::numeric_limits<double>::quiet_NaN();
    msg.descriptor_extent_ratio = std::numeric_limits<double>::quiet_NaN();
    msg.descriptor_point_count_ratio = std::numeric_limits<double>::quiet_NaN();
    msg.status = status;
    status_pub_->publish(msg);
  }

  std::string points_topic_;
  std::string keyframe_topic_;
  std::string loop_constraint_topic_;
  std::string status_topic_;
  std::string marker_topic_;
  std::string map_frame_;

  SubmapManagerOptions submap_options_;
  CandidateSelectionOptions candidate_options_;
  RegistrationOptions registration_options_;
  GateOptions gate_options_;
  DescriptorGateOptions descriptor_options_;
  LoopSuppressionOptions loop_suppression_options_;

  double loop_translation_sigma_m_{2.0};
  double loop_rotation_sigma_rad_{0.35};
  bool optimize_after_insert_{true};
  std::uint32_t marker_sequence_{0};

  SubmapManager submap_manager_{submap_options_};
  AcceptedLoopTracker accepted_loop_tracker_{loop_suppression_options_};

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
