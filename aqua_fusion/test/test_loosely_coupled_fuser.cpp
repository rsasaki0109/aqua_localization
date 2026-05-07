#include <optional>

#include <gtest/gtest.h>

#include "aqua_fusion/loosely_coupled_fuser.hpp"

namespace
{

nav_msgs::msg::Odometry make_odometry(double stamp_s, double x, double y, double z)
{
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp.sec = static_cast<int32_t>(stamp_s);
  odometry.header.stamp.nanosec =
    static_cast<uint32_t>((stamp_s - static_cast<int32_t>(stamp_s)) * 1.0e9);
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "base_link";
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.position.y = y;
  odometry.pose.pose.position.z = z;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.pose.covariance[0] = 0.1;
  odometry.pose.covariance[7] = 0.1;
  odometry.pose.covariance[14] = 0.1;
  odometry.pose.covariance[35] = 0.01;
  return odometry;
}

}  // namespace

TEST(LooselyCoupledFuser, ReturnsImuWhenSonarMissing)
{
  aqua_fusion::LooselyCoupledFuser fuser;
  fuser.configure({});

  const auto imu = make_odometry(10.0, 1.0, 2.0, 3.0);
  const auto fused = fuser.fuse(imu, std::nullopt);

  EXPECT_DOUBLE_EQ(fused.pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(fused.pose.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(fused.pose.pose.position.z, 3.0);
}

TEST(LooselyCoupledFuser, BlendsFreshSonarPosition)
{
  aqua_fusion::LooselyCoupledFuserConfig config;
  config.sonar_pose_weight = 0.25;
  config.max_sonar_age_s = 1.0;

  aqua_fusion::LooselyCoupledFuser fuser;
  fuser.configure(config);

  const auto imu = make_odometry(10.0, 0.0, 0.0, 0.0);
  const auto sonar = make_odometry(10.2, 4.0, -4.0, 2.0);
  const auto fused = fuser.fuse(imu, sonar);

  EXPECT_DOUBLE_EQ(fused.pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(fused.pose.pose.position.y, -1.0);
  EXPECT_DOUBLE_EQ(fused.pose.pose.position.z, 0.5);
}

TEST(LooselyCoupledFuser, ReportsFreshSonarFusionStatus)
{
  aqua_fusion::LooselyCoupledFuserConfig config;
  config.sonar_pose_weight = 0.25;
  config.max_sonar_age_s = 1.0;

  aqua_fusion::LooselyCoupledFuser fuser;
  fuser.configure(config);

  const auto imu = make_odometry(10.0, 0.0, 0.0, 0.0);
  const auto sonar = make_odometry(10.2, 4.0, 0.0, 0.0);
  const auto result = fuser.fuse_with_status(imu, sonar);

  EXPECT_TRUE(result.sonar_available);
  EXPECT_TRUE(result.used_sonar);
  EXPECT_NEAR(result.sonar_age_s, 0.2, 1.0e-9);
  EXPECT_DOUBLE_EQ(result.sonar_pose_weight, 0.25);
  EXPECT_EQ(result.status, "sonar_fused");
}

TEST(LooselyCoupledFuser, IgnoresStaleSonar)
{
  aqua_fusion::LooselyCoupledFuserConfig config;
  config.sonar_pose_weight = 1.0;
  config.max_sonar_age_s = 0.5;

  aqua_fusion::LooselyCoupledFuser fuser;
  fuser.configure(config);

  const auto imu = make_odometry(10.0, 1.0, 0.0, 0.0);
  const auto sonar = make_odometry(8.0, 100.0, 0.0, 0.0);
  const auto fused = fuser.fuse(imu, sonar);

  EXPECT_DOUBLE_EQ(fused.pose.pose.position.x, 1.0);
}

TEST(LooselyCoupledFuser, ReportsStaleSonarRejection)
{
  aqua_fusion::LooselyCoupledFuserConfig config;
  config.sonar_pose_weight = 1.0;
  config.max_sonar_age_s = 0.5;

  aqua_fusion::LooselyCoupledFuser fuser;
  fuser.configure(config);

  const auto imu = make_odometry(10.0, 1.0, 0.0, 0.0);
  const auto sonar = make_odometry(8.0, 100.0, 0.0, 0.0);
  const auto result = fuser.fuse_with_status(imu, sonar);

  EXPECT_TRUE(result.sonar_available);
  EXPECT_FALSE(result.used_sonar);
  EXPECT_DOUBLE_EQ(result.sonar_age_s, 2.0);
  EXPECT_EQ(result.status, "sonar_rejected");
}

TEST(LooselyCoupledFuser, AppliesCovarianceFloors)
{
  aqua_fusion::LooselyCoupledFuserConfig config;
  config.sonar_position_variance_floor = 0.25;
  config.sonar_yaw_variance_floor = 0.05;

  aqua_fusion::LooselyCoupledFuser fuser;
  fuser.configure(config);

  const auto imu = make_odometry(10.0, 0.0, 0.0, 0.0);
  const auto sonar = make_odometry(10.0, 0.0, 0.0, 0.0);
  const auto fused = fuser.fuse(imu, sonar);

  EXPECT_DOUBLE_EQ(fused.pose.covariance[0], 0.25);
  EXPECT_DOUBLE_EQ(fused.pose.covariance[7], 0.25);
  EXPECT_DOUBLE_EQ(fused.pose.covariance[14], 0.25);
  EXPECT_DOUBLE_EQ(fused.pose.covariance[35], 0.05);
}
