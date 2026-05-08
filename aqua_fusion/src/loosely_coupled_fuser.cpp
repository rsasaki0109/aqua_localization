#include "aqua_fusion/loosely_coupled_fuser.hpp"

#include <algorithm>
#include <cmath>

namespace aqua_fusion
{

void LooselyCoupledFuser::configure(const LooselyCoupledFuserConfig & config)
{
  config_ = config;
  config_.sonar_pose_weight = std::clamp(config_.sonar_pose_weight, 0.0, 1.0);
  config_.max_sonar_age_s = std::max(0.0, config_.max_sonar_age_s);
  config_.sonar_position_variance_floor = std::max(0.0, config_.sonar_position_variance_floor);
  config_.sonar_yaw_variance_floor = std::max(0.0, config_.sonar_yaw_variance_floor);
}

nav_msgs::msg::Odometry LooselyCoupledFuser::fuse(
  const nav_msgs::msg::Odometry & imu_odometry,
  const std::optional<nav_msgs::msg::Odometry> & sonar_odometry) const
{
  return fuse_with_status(imu_odometry, sonar_odometry).odometry;
}

FusionResult LooselyCoupledFuser::fuse_with_status(
  const nav_msgs::msg::Odometry & imu_odometry,
  const std::optional<nav_msgs::msg::Odometry> & sonar_odometry) const
{
  FusionResult result;
  result.odometry = imu_odometry;
  result.sonar_pose_weight = config_.sonar_pose_weight;

  if (!sonar_odometry.has_value()) {
    result.status = "sonar_unavailable";
    return result;
  }

  result.sonar_available = true;
  result.sonar_age_s = sonar_age_s(imu_odometry, sonar_odometry.value());
  if (!sonar_is_usable(imu_odometry, sonar_odometry.value())) {
    result.status = "sonar_rejected";
    return result;
  }

  nav_msgs::msg::Odometry fused = imu_odometry;
  const auto & sonar = sonar_odometry.value();
  const double imu_weight = 1.0 - config_.sonar_pose_weight;
  const double sonar_weight = config_.sonar_pose_weight;

  fused.pose.pose.position.x =
    imu_weight * imu_odometry.pose.pose.position.x + sonar_weight * sonar.pose.pose.position.x;
  fused.pose.pose.position.y =
    imu_weight * imu_odometry.pose.pose.position.y + sonar_weight * sonar.pose.pose.position.y;
  fused.pose.pose.position.z =
    imu_weight * imu_odometry.pose.pose.position.z + sonar_weight * sonar.pose.pose.position.z;

  if (config_.use_sonar_orientation) {
    fused.pose.pose.orientation = sonar.pose.pose.orientation;
  }

  fused.pose.covariance[0] = std::max(
    std::min(imu_odometry.pose.covariance[0], sonar.pose.covariance[0]),
    config_.sonar_position_variance_floor);
  fused.pose.covariance[7] = std::max(
    std::min(imu_odometry.pose.covariance[7], sonar.pose.covariance[7]),
    config_.sonar_position_variance_floor);
  fused.pose.covariance[14] = std::max(
    std::min(imu_odometry.pose.covariance[14], sonar.pose.covariance[14]),
    config_.sonar_position_variance_floor);
  fused.pose.covariance[35] = std::max(
    std::min(imu_odometry.pose.covariance[35], sonar.pose.covariance[35]),
    config_.sonar_yaw_variance_floor);
  result.odometry = fused;
  result.used_sonar = true;
  result.status = "sonar_fused";
  return result;
}

double LooselyCoupledFuser::sonar_age_s(
  const nav_msgs::msg::Odometry & imu_odometry,
  const nav_msgs::msg::Odometry & sonar_odometry) const
{
  const rclcpp::Time imu_stamp(imu_odometry.header.stamp);
  const rclcpp::Time sonar_stamp(sonar_odometry.header.stamp);
  return std::abs((imu_stamp - sonar_stamp).seconds());
}

bool LooselyCoupledFuser::sonar_is_usable(
  const nav_msgs::msg::Odometry & imu_odometry,
  const nav_msgs::msg::Odometry & sonar_odometry) const
{
  if (sonar_age_s(imu_odometry, sonar_odometry) > config_.max_sonar_age_s) {
    return false;
  }

  const auto & position = sonar_odometry.pose.pose.position;
  const auto & orientation = sonar_odometry.pose.pose.orientation;
  return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
         std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
         std::isfinite(orientation.z) && std::isfinite(orientation.w);
}

}  // namespace aqua_fusion
