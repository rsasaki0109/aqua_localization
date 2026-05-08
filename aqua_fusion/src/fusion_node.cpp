#include <memory>
#include <optional>
#include <string>

#include "aqua_msgs/msg/fusion_status.hpp"
#include "aqua_fusion/loosely_coupled_fuser.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace aqua_fusion
{

class FusionNode : public rclcpp::Node
{
public:
  explicit FusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("aqua_fusion", options)
  {
    load_parameters();

    imu_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      imu_odometry_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FusionNode::on_imu_odometry, this, std::placeholders::_1));
    sonar_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      sonar_odometry_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FusionNode::on_sonar_odometry, this, std::placeholders::_1));
    fused_odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(fused_odometry_topic_, rclcpp::SystemDefaultsQoS());
    status_pub_ =
      create_publisher<aqua_msgs::msg::FusionStatus>(status_topic_, rclcpp::SystemDefaultsQoS());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(
      get_logger(), "aqua_fusion started: imu=%s sonar=%s fused=%s mode=%s",
      imu_odometry_topic_.c_str(), sonar_odometry_topic_.c_str(), fused_odometry_topic_.c_str(),
      fusion_mode_.c_str());
  }

private:
  void load_parameters()
  {
    imu_odometry_topic_ =
      declare_parameter<std::string>("topics.imu_odometry", "/aqua_imu_loc/odometry");
    sonar_odometry_topic_ =
      declare_parameter<std::string>("topics.sonar_odometry", "/aqua_sonar_loc/odometry");
    fused_odometry_topic_ =
      declare_parameter<std::string>("topics.fused_odometry", "/aqua_fusion/odometry");
    status_topic_ = declare_parameter<std::string>("topics.status", "/aqua_fusion/status");

    map_frame_ = declare_parameter<std::string>("frames.map", "map");
    odom_frame_ = declare_parameter<std::string>("frames.odom", "odom");
    base_frame_ = declare_parameter<std::string>("frames.base_link", "base_link");

    fusion_mode_ = declare_parameter<std::string>("fusion.mode", "loosely_coupled");
    publish_tf_ = declare_parameter<bool>("publish.tf", false);

    LooselyCoupledFuserConfig config;
    config.sonar_pose_weight = declare_parameter<double>("fusion.sonar_pose_weight", 0.35);
    config.max_sonar_age_s = declare_parameter<double>("fusion.max_sonar_age_s", 1.0);
    config.sonar_position_variance_floor =
      declare_parameter<double>("fusion.sonar_position_variance_floor", 0.25);
    config.sonar_yaw_variance_floor =
      declare_parameter<double>("fusion.sonar_yaw_variance_floor", 0.05);
    config.use_sonar_orientation = declare_parameter<bool>("fusion.use_sonar_orientation", false);
    fuser_.configure(config);

    if (fusion_mode_ != "loosely_coupled") {
      RCLCPP_WARN(
        get_logger(), "Fusion mode '%s' is not implemented; using loosely_coupled.",
        fusion_mode_.c_str());
      fusion_mode_ = "loosely_coupled";
    }
  }

  void on_imu_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto result = fuser_.fuse_with_status(*msg, latest_sonar_odometry_);
    fused_odom_pub_->publish(result.odometry);
    status_pub_->publish(make_status_msg(result));
    if (publish_tf_) {
      publish_transforms(result.odometry);
    }
  }

  void on_sonar_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_sonar_odometry_ = *msg;
  }

  void publish_transforms(const nav_msgs::msg::Odometry & odometry)
  {
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = odometry.header.stamp;
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.child_frame_id = odom_frame_;
    map_to_odom.transform.rotation.w = 1.0;

    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.stamp = odometry.header.stamp;
    odom_to_base.header.frame_id = odom_frame_;
    odom_to_base.child_frame_id = base_frame_;
    odom_to_base.transform.translation.x = odometry.pose.pose.position.x;
    odom_to_base.transform.translation.y = odometry.pose.pose.position.y;
    odom_to_base.transform.translation.z = odometry.pose.pose.position.z;
    odom_to_base.transform.rotation = odometry.pose.pose.orientation;

    tf_broadcaster_->sendTransform(map_to_odom);
    tf_broadcaster_->sendTransform(odom_to_base);
  }

  aqua_msgs::msg::FusionStatus make_status_msg(const FusionResult & result) const
  {
    aqua_msgs::msg::FusionStatus status;
    status.header = result.odometry.header;
    status.mode = fusion_mode_;
    status.used_sonar = result.used_sonar;
    status.sonar_available = result.sonar_available;
    status.sonar_age = result.sonar_age_s;
    status.sonar_pose_weight = result.sonar_pose_weight;
    status.status = result.status;
    return status;
  }

  LooselyCoupledFuser fuser_;
  std::optional<nav_msgs::msg::Odometry> latest_sonar_odometry_;

  std::string imu_odometry_topic_;
  std::string sonar_odometry_topic_;
  std::string fused_odometry_topic_;
  std::string status_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string fusion_mode_;
  bool publish_tf_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sonar_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
  rclcpp::Publisher<aqua_msgs::msg::FusionStatus>::SharedPtr status_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace aqua_fusion

#ifndef AQUA_FUSION_DISABLE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aqua_fusion::FusionNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
