#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "aqua_msgs/msg/fusion_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#define AQUA_FUSION_DISABLE_MAIN
#include "../src/fusion_node.cpp"

namespace
{

using namespace std::chrono_literals;

nav_msgs::msg::Odometry make_odometry(
  const rclcpp::Time & stamp, double x, double y, double z)
{
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "base_link";
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.position.y = y;
  odometry.pose.pose.position.z = z;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.pose.covariance[0] = 0.1;
  odometry.pose.covariance[7] = 0.1;
  odometry.pose.covariance[14] = 0.1;
  odometry.pose.covariance[35] = 0.1;
  return odometry;
}

template<typename Predicate>
bool spin_until(
  rclcpp::Executor & executor,
  const Predicate & predicate,
  std::chrono::milliseconds timeout = 5s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

class FusionNodeRuntimeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(FusionNodeRuntimeTest, PublishesFusedOdometryStatusAndTf)
{
  constexpr auto kImuTopic = "/aqua_fusion_runtime_test/imu_odometry";
  constexpr auto kSonarTopic = "/aqua_fusion_runtime_test/sonar_odometry";
  constexpr auto kFusedTopic = "/aqua_fusion_runtime_test/fused_odometry";
  constexpr auto kStatusTopic = "/aqua_fusion_runtime_test/status";

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("topics.imu_odometry", std::string(kImuTopic)),
    rclcpp::Parameter("topics.sonar_odometry", std::string(kSonarTopic)),
    rclcpp::Parameter("topics.fused_odometry", std::string(kFusedTopic)),
    rclcpp::Parameter("topics.status", std::string(kStatusTopic)),
    rclcpp::Parameter("publish.tf", true),
    rclcpp::Parameter("fusion.sonar_pose_weight", 0.25),
    rclcpp::Parameter("fusion.max_sonar_age_s", 10.0),
  });

  auto fusion_node = std::make_shared<aqua_fusion::FusionNode>(options);
  auto test_node = std::make_shared<rclcpp::Node>("aqua_fusion_runtime_test");

  std::vector<nav_msgs::msg::Odometry> fused_messages;
  std::vector<aqua_msgs::msg::FusionStatus> status_messages;

  auto imu_pub =
    test_node->create_publisher<nav_msgs::msg::Odometry>(kImuTopic, rclcpp::SensorDataQoS());
  auto sonar_pub =
    test_node->create_publisher<nav_msgs::msg::Odometry>(kSonarTopic, rclcpp::SensorDataQoS());
  auto fused_sub = test_node->create_subscription<nav_msgs::msg::Odometry>(
    kFusedTopic, 10,
    [&fused_messages](const nav_msgs::msg::Odometry::SharedPtr msg) {
      fused_messages.push_back(*msg);
    });
  auto status_sub = test_node->create_subscription<aqua_msgs::msg::FusionStatus>(
    kStatusTopic, 10,
    [&status_messages](const aqua_msgs::msg::FusionStatus::SharedPtr msg) {
      status_messages.push_back(*msg);
    });

  tf2_ros::Buffer tf_buffer(test_node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer, test_node, false);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(fusion_node);
  executor.add_node(test_node);

  ASSERT_TRUE(spin_until(executor, [&]() {
    return imu_pub->get_subscription_count() > 0 && sonar_pub->get_subscription_count() > 0;
  }));

  ASSERT_TRUE(spin_until(executor, [&]() {
    const rclcpp::Time stamp = test_node->now();
    const auto sonar = make_odometry(stamp, 2.0, -4.0, 1.0);
    const auto imu = make_odometry(stamp, 0.0, 0.0, 0.0);
    sonar_pub->publish(sonar);
    imu_pub->publish(imu);
    return !fused_messages.empty() && !status_messages.empty();
  }));

  const auto & fused = fused_messages.back();
  EXPECT_NEAR(fused.pose.pose.position.x, 0.5, 1.0e-6);
  EXPECT_NEAR(fused.pose.pose.position.y, -1.0, 1.0e-6);
  EXPECT_NEAR(fused.pose.pose.position.z, 0.25, 1.0e-6);

  const auto & status = status_messages.back();
  EXPECT_TRUE(status.used_sonar);
  EXPECT_TRUE(status.sonar_available);
  EXPECT_EQ(status.status, "sonar_fused");

  ASSERT_TRUE(spin_until(executor, [&]() {
    try {
      const auto transform = tf_buffer.lookupTransform("odom", "base_link", tf2::TimePointZero);
      EXPECT_NEAR(transform.transform.translation.x, 0.5, 1.0e-6);
      EXPECT_NEAR(transform.transform.translation.y, -1.0, 1.0e-6);
      EXPECT_NEAR(transform.transform.translation.z, 0.25, 1.0e-6);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }));
}

}  // namespace
