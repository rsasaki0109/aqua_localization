#ifndef AQUA_FUSION__LOOSELY_COUPLED_FUSER_HPP_
#define AQUA_FUSION__LOOSELY_COUPLED_FUSER_HPP_

#include <optional>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/time.hpp"

namespace aqua_fusion
{

struct LooselyCoupledFuserConfig
{
  double sonar_pose_weight{0.35};
  double max_sonar_age_s{1.0};
  double sonar_position_variance_floor{0.25};
  double sonar_yaw_variance_floor{0.05};
  bool use_sonar_orientation{false};
};

struct FusionResult
{
  nav_msgs::msg::Odometry odometry;
  bool used_sonar{false};
  bool sonar_available{false};
  double sonar_age_s{0.0};
  double sonar_pose_weight{0.0};
  std::string status;
};

class LooselyCoupledFuser
{
public:
  void configure(const LooselyCoupledFuserConfig & config);

  nav_msgs::msg::Odometry fuse(
    const nav_msgs::msg::Odometry & imu_odometry,
    const std::optional<nav_msgs::msg::Odometry> & sonar_odometry) const;
  FusionResult fuse_with_status(
    const nav_msgs::msg::Odometry & imu_odometry,
    const std::optional<nav_msgs::msg::Odometry> & sonar_odometry) const;

private:
  double sonar_age_s(
    const nav_msgs::msg::Odometry & imu_odometry,
    const nav_msgs::msg::Odometry & sonar_odometry) const;
  bool sonar_is_usable(
    const nav_msgs::msg::Odometry & imu_odometry,
    const nav_msgs::msg::Odometry & sonar_odometry) const;

  LooselyCoupledFuserConfig config_;
};

}  // namespace aqua_fusion

#endif  // AQUA_FUSION__LOOSELY_COUPLED_FUSER_HPP_
