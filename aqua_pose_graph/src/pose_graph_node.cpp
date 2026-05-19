// SPDX-License-Identifier: Apache-2.0
#include "aqua_pose_graph/pose_graph.hpp"

#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <aqua_msgs/msg/pose_graph_loop_constraint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace aqua_pose_graph {

namespace {

Eigen::Isometry3d odometry_to_isometry(const nav_msgs::msg::Odometry & msg)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  pose.translation() << p.x, p.y, p.z;
  Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
  if (quat.norm() < 1e-9) {
    quat = Eigen::Quaterniond::Identity();
  } else {
    quat.normalize();
  }
  pose.linear() = quat.toRotationMatrix();
  return pose;
}

Eigen::Isometry3d pose_msg_to_isometry(const geometry_msgs::msg::Pose & msg)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() << msg.position.x, msg.position.y, msg.position.z;
  Eigen::Quaterniond quat(
    msg.orientation.w,
    msg.orientation.x,
    msg.orientation.y,
    msg.orientation.z);
  if (quat.norm() < 1e-9) {
    quat = Eigen::Quaterniond::Identity();
  } else {
    quat.normalize();
  }
  pose.linear() = quat.toRotationMatrix();
  return pose;
}

Eigen::Matrix<double, 6, 6> covariance_to_eigen(
  const std::array<double, 36> & cov)
{
  Eigen::Matrix<double, 6, 6> out;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      out(i, j) = cov[static_cast<size_t>(i * 6 + j)];
    }
  }
  return out;
}

Eigen::Matrix<double, 6, 6> information_to_eigen(
  const std::array<double, 36> & info)
{
  Eigen::Matrix<double, 6, 6> out;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      out(i, j) = info[static_cast<size_t>(i * 6 + j)];
    }
  }
  return out;
}

bool has_valid_information_diagonal(const Eigen::Matrix<double, 6, 6> & info)
{
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(info(i, i)) || info(i, i) <= 0.0) {
      return false;
    }
  }
  return true;
}

geometry_msgs::msg::PoseStamped pose_to_msg(
  const Keyframe & kf,
  const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = frame_id;
  ps.header.stamp.sec = static_cast<int32_t>(std::floor(kf.stamp_seconds));
  ps.header.stamp.nanosec = static_cast<uint32_t>(
    (kf.stamp_seconds - std::floor(kf.stamp_seconds)) * 1e9);
  const Eigen::Vector3d t = kf.pose.translation();
  const Eigen::Quaterniond q(kf.pose.linear());
  ps.pose.position.x = t.x();
  ps.pose.position.y = t.y();
  ps.pose.position.z = t.z();
  ps.pose.orientation.x = q.x();
  ps.pose.orientation.y = q.y();
  ps.pose.orientation.z = q.z();
  ps.pose.orientation.w = q.w();
  return ps;
}

}  // namespace

class PoseGraphNode : public rclcpp::Node
{
public:
  PoseGraphNode()
  : Node("aqua_pose_graph")
  {
    PoseGraphConfig cfg;
    cfg.keyframe_translation_m = declare_parameter<double>(
      "keyframe.translation_m", cfg.keyframe_translation_m);
    cfg.keyframe_rotation_rad = declare_parameter<double>(
      "keyframe.rotation_rad", cfg.keyframe_rotation_rad);
    cfg.default_translation_information = declare_parameter<double>(
      "edges.default_translation_information",
      cfg.default_translation_information);
    cfg.default_rotation_information = declare_parameter<double>(
      "edges.default_rotation_information",
      cfg.default_rotation_information);
    cfg.optimization_iterations = declare_parameter<int>(
      "optimization.iterations", cfg.optimization_iterations);
    cfg.optimize_every_n_keyframes = declare_parameter<int>(
      "optimization.optimize_every_n_keyframes",
      cfg.optimize_every_n_keyframes);

    odom_topic_ = declare_parameter<std::string>(
      "topics.odometry", "/aqua_imu_loc/odometry");
    path_topic_ = declare_parameter<std::string>(
      "topics.path", "/aqua_pose_graph/path");
    keyframe_count_topic_ = declare_parameter<std::string>(
      "topics.keyframe_count", "/aqua_pose_graph/keyframe_count");
    loop_constraint_topic_ = declare_parameter<std::string>(
      "topics.loop_constraint", "/aqua_pose_graph/loop_constraint");
    loop_constraint_count_topic_ = declare_parameter<std::string>(
      "topics.loop_constraint_count", "/aqua_pose_graph/loop_constraint_count");
    map_frame_ = declare_parameter<std::string>("frames.map", "map");

    graph_ = std::make_unique<PoseGraph>(cfg);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(50),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        on_odometry(*msg);
      });
    loop_constraint_sub_ = create_subscription<aqua_msgs::msg::PoseGraphLoopConstraint>(
      loop_constraint_topic_, rclcpp::QoS(20),
      [this](const aqua_msgs::msg::PoseGraphLoopConstraint::SharedPtr msg) {
        on_loop_constraint(*msg);
      });
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10).transient_local());
    keyframe_count_pub_ = create_publisher<std_msgs::msg::UInt32>(
      keyframe_count_topic_, rclcpp::QoS(10).transient_local());
    loop_constraint_count_pub_ = create_publisher<std_msgs::msg::UInt32>(
      loop_constraint_count_topic_, rclcpp::QoS(10).transient_local());

    optimize_srv_ = create_service<std_srvs::srv::Trigger>(
      "/aqua_pose_graph/optimize",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const double chi2 = graph_->optimize();
        publish_path();
        response->success = true;
        response->message = "optimized; chi2=" + std::to_string(chi2);
      });
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "/aqua_pose_graph/reset",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        graph_->reset();
        publish_path();
        publish_keyframe_count();
        publish_loop_constraint_count();
        response->success = true;
        response->message = "graph reset";
      });

    RCLCPP_INFO(
      get_logger(),
      "aqua_pose_graph started: odometry=%s loop_constraints=%s path=%s frame=%s "
      "keyframe_thresholds=(t=%.2fm, r=%.2frad) optimize_every=%d",
      odom_topic_.c_str(), loop_constraint_topic_.c_str(),
      path_topic_.c_str(), map_frame_.c_str(),
      cfg.keyframe_translation_m, cfg.keyframe_rotation_rad,
      cfg.optimize_every_n_keyframes);
  }

private:
  void on_odometry(const nav_msgs::msg::Odometry & msg)
  {
    const Eigen::Isometry3d pose = odometry_to_isometry(msg);
    const Eigen::Matrix<double, 6, 6> cov =
      covariance_to_eigen(msg.pose.covariance);
    const double t = static_cast<double>(msg.header.stamp.sec)
      + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    const bool added = graph_->add_odometry_sample(t, pose, cov);
    if (added) {
      publish_keyframe_count();
      publish_path();
    }
  }

  void on_loop_constraint(const aqua_msgs::msg::PoseGraphLoopConstraint & msg)
  {
    if (msg.from_id == msg.to_id) {
      RCLCPP_WARN(
        get_logger(),
        "rejected loop constraint with identical keyframe id %u",
        msg.from_id);
      return;
    }

    LoopConstraint constraint;
    constraint.from_id = static_cast<std::size_t>(msg.from_id);
    constraint.to_id = static_cast<std::size_t>(msg.to_id);
    constraint.relative_pose = pose_msg_to_isometry(msg.relative_pose);
    constraint.information = information_to_eigen(msg.information);

    if (!has_valid_information_diagonal(constraint.information)) {
      RCLCPP_WARN(
        get_logger(),
        "rejected loop constraint %u -> %u: information diagonal must be finite and positive",
        msg.from_id, msg.to_id);
      return;
    }

    if (!graph_->add_loop_constraint(constraint)) {
      RCLCPP_WARN(
        get_logger(),
        "rejected loop constraint %u -> %u: keyframe id not found",
        msg.from_id, msg.to_id);
      return;
    }

    double chi2 = 0.0;
    if (msg.optimize_after_insert) {
      chi2 = graph_->optimize();
    }
    publish_loop_constraint_count();
    publish_path();

    RCLCPP_INFO(
      get_logger(),
      "accepted loop constraint %u -> %u (total=%zu optimize=%s chi2=%.6f)",
      msg.from_id, msg.to_id, graph_->loop_constraint_count(),
      msg.optimize_after_insert ? "true" : "false", chi2);
  }

  void publish_keyframe_count()
  {
    std_msgs::msg::UInt32 msg;
    msg.data = static_cast<uint32_t>(graph_->keyframes().size());
    keyframe_count_pub_->publish(msg);
  }

  void publish_loop_constraint_count()
  {
    std_msgs::msg::UInt32 msg;
    msg.data = static_cast<uint32_t>(graph_->loop_constraint_count());
    loop_constraint_count_pub_->publish(msg);
  }

  void publish_path()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();
    path.poses.reserve(graph_->keyframes().size());
    for (const auto & kf : graph_->keyframes()) {
      path.poses.push_back(pose_to_msg(kf, map_frame_));
    }
    path_pub_->publish(path);
  }

  std::unique_ptr<PoseGraph> graph_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<aqua_msgs::msg::PoseGraphLoopConstraint>::SharedPtr
    loop_constraint_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr keyframe_count_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr loop_constraint_count_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr optimize_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  std::string odom_topic_;
  std::string path_topic_;
  std::string keyframe_count_topic_;
  std::string loop_constraint_topic_;
  std::string loop_constraint_count_topic_;
  std::string map_frame_;
};

}  // namespace aqua_pose_graph

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aqua_pose_graph::PoseGraphNode>());
  rclcpp::shutdown();
  return 0;
}
