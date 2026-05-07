#include <memory>
#include <optional>
#include <string>

#include "aqua_msgs/msg/scan_matching_status.hpp"
#include "aqua_sonar_loc/scan_matcher.hpp"
#include "aqua_sonar_loc/sonar_cloud_preprocessor.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace aqua_sonar_loc
{

class SonarLocNode : public rclcpp::Node
{
public:
  explicit SonarLocNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("aqua_sonar_loc", options)
  {
    load_parameters();

    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SonarLocNode::on_points, this, std::placeholders::_1));
    filtered_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      filtered_points_topic_, rclcpp::SensorDataQoS());
    odometry_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, rclcpp::SystemDefaultsQoS());
    status_pub_ = create_publisher<aqua_msgs::msg::ScanMatchingStatus>(
      status_topic_, rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO(
      get_logger(),
      "aqua_sonar_loc started: points=%s filtered=%s odom=%s sonar_frame=%s backend=%s",
      points_topic_.c_str(), filtered_points_topic_.c_str(), odometry_topic_.c_str(),
      sonar_frame_.c_str(), scan_matcher_->backend_name().c_str());
  }

private:
  void load_parameters()
  {
    points_topic_ = declare_parameter<std::string>("topics.points", "/sonar/points");
    filtered_points_topic_ =
      declare_parameter<std::string>("topics.filtered_points", "/aqua_sonar_loc/points_filtered");
    odometry_topic_ = declare_parameter<std::string>("topics.odometry", "/aqua_sonar_loc/odometry");
    status_topic_ =
      declare_parameter<std::string>("topics.status", "/aqua_sonar_loc/status");

    map_frame_ = declare_parameter<std::string>("frames.map", "map");
    odom_frame_ = declare_parameter<std::string>("frames.odom", "odom");
    base_frame_ = declare_parameter<std::string>("frames.base_link", "base_link");
    sonar_frame_ = declare_parameter<std::string>("frames.sonar", "sonar_link");

    ScanMatcherConfig scan_matcher_config;
    scan_matcher_config.backend = declare_parameter<std::string>("scan_matching.backend", "noop");
    scan_matcher_config.max_correspondence_distance =
      declare_parameter<double>("scan_matching.max_correspondence_distance", 1.0);
    scan_matcher_config.max_iterations =
      declare_parameter<int>("scan_matching.max_iterations", 50);
    scan_matcher_config.transformation_epsilon =
      declare_parameter<double>("scan_matching.transformation_epsilon", 1.0e-6);
    scan_matcher_config.max_fitness_score =
      declare_parameter<double>("scan_matching.max_fitness_score", -1.0);
    scan_matcher_config.max_translation_step_m =
      declare_parameter<double>("scan_matching.max_translation_step_m", -1.0);
    scan_matcher_config.max_rotation_step_rad =
      declare_parameter<double>("scan_matching.max_rotation_step_rad", -1.0);
    const int submap_size_param =
      declare_parameter<int>("scan_matching.submap_size", 1);
    scan_matcher_config.submap_size =
      submap_size_param > 0 ? static_cast<std::size_t>(submap_size_param) : std::size_t{1};
    scan_matcher_config.use_motion_prior =
      declare_parameter<bool>("scan_matching.use_motion_prior", false);
    scan_matcher_ = create_scan_matcher(scan_matcher_config.backend);
    if (!scan_matcher_) {
      RCLCPP_WARN(
        get_logger(), "Scan matching backend '%s' is not implemented; falling back to noop.",
        scan_matcher_config.backend.c_str());
      scan_matcher_config.backend = "noop";
      scan_matcher_ = create_scan_matcher(scan_matcher_config.backend);
    }
    scan_matcher_->configure(scan_matcher_config);

    SonarCloudPreprocessorConfig config;
    config.require_xyz = declare_parameter<bool>("preprocessing.require_xyz", true);
    config.enable_range_filter =
      declare_parameter<bool>("preprocessing.enable_range_filter", true);
    config.max_range_m = declare_parameter<double>("preprocessing.max_range_m", 80.0);
    const int min_points = declare_parameter<int>("preprocessing.min_points", 20);
    config.min_points = min_points > 0 ? static_cast<size_t>(min_points) : 1U;
    preprocessor_.configure(config);
  }

  void on_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto summary = preprocessor_.summarize(*msg);
    if (!summary.accepted) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected sonar cloud: %s total=%zu finite_xyz=%zu in_range=%zu",
        summary.rejection_reason.c_str(), summary.total_points, summary.finite_xyz_points,
        summary.in_range_points);
      status_pub_->publish(make_status(*msg, summary, std::nullopt));
      return;
    }

    RCLCPP_DEBUG(
      get_logger(), "Accepted sonar cloud: total=%zu finite_xyz=%zu in_range=%zu",
      summary.total_points, summary.finite_xyz_points, summary.in_range_points);
    filtered_points_pub_->publish(*msg);

    const auto match_result = scan_matcher_->match(*msg, summary);
    if (!match_result.success) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Scan matching failed: backend=%s status=%s",
        scan_matcher_->backend_name().c_str(), match_result.status.c_str());
      status_pub_->publish(make_status(*msg, summary, match_result));
      return;
    }
    odometry_pub_->publish(make_odometry(*msg, match_result));
    status_pub_->publish(make_status(*msg, summary, match_result));
  }

  aqua_msgs::msg::ScanMatchingStatus make_status(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const CloudSummary & summary,
    const std::optional<ScanMatchResult> & match_result) const
  {
    aqua_msgs::msg::ScanMatchingStatus status;
    status.header.stamp = cloud.header.stamp;
    status.header.frame_id = cloud.header.frame_id.empty() ? sonar_frame_ : cloud.header.frame_id;
    status.backend = scan_matcher_->backend_name();
    status.success = match_result.has_value() && match_result->success;
    status.converged = match_result.has_value() && match_result->converged;
    status.total_points = static_cast<uint32_t>(summary.total_points);
    status.finite_xyz_points = static_cast<uint32_t>(summary.finite_xyz_points);
    status.in_range_points = static_cast<uint32_t>(summary.in_range_points);
    status.fitness_score = match_result.has_value() ? match_result->fitness_score : 0.0;
    if (match_result.has_value()) {
      status.status = match_result->status;
    } else {
      status.status = summary.rejection_reason;
    }
    return status;
  }

  nav_msgs::msg::Odometry make_odometry(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const ScanMatchResult & match_result) const
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = cloud.header.stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = match_result.odom_to_base.translation.x;
    odometry.pose.pose.position.y = match_result.odom_to_base.translation.y;
    odometry.pose.pose.position.z = match_result.odom_to_base.translation.z;
    odometry.pose.pose.orientation = match_result.odom_to_base.rotation;

    for (auto & value : odometry.pose.covariance) {
      value = 0.0;
    }
    for (auto & value : odometry.twist.covariance) {
      value = 0.0;
    }

    odometry.pose.covariance[0] = 0.25;
    odometry.pose.covariance[7] = 0.25;
    odometry.pose.covariance[14] = 0.25;
    odometry.pose.covariance[21] = 0.10;
    odometry.pose.covariance[28] = 0.10;
    odometry.pose.covariance[35] = 0.10;
    return odometry;
  }

  SonarCloudPreprocessor preprocessor_;
  std::unique_ptr<ScanMatcher> scan_matcher_;
  std::string points_topic_;
  std::string filtered_points_topic_;
  std::string odometry_topic_;
  std::string status_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string sonar_frame_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<aqua_msgs::msg::ScanMatchingStatus>::SharedPtr status_pub_;
};

}  // namespace aqua_sonar_loc

#ifndef AQUA_SONAR_LOC_DISABLE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aqua_sonar_loc::SonarLocNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
