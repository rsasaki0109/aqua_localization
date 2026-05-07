#include <array>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "aqua_msgs/msg/scan_matching_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#define AQUA_SONAR_LOC_DISABLE_MAIN
#include "../src/sonar_loc_node.cpp"

namespace
{

using namespace std::chrono_literals;

sensor_msgs::msg::PointCloud2 make_cloud(
  const rclcpp::Time & stamp,
  const std::vector<std::array<float, 3>> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = "sonar_link";
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.is_dense = false;
  cloud.is_bigendian = false;
  cloud.point_step = 3 * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(static_cast<size_t>(cloud.row_step));

  const std::array<std::string, 3> names = {"x", "y", "z"};
  for (size_t i = 0; i < names.size(); ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i];
    field.offset = static_cast<uint32_t>(i * sizeof(float));
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }

  for (size_t i = 0; i < points.size(); ++i) {
    const size_t point_offset = i * static_cast<size_t>(cloud.point_step);
    for (size_t axis = 0; axis < 3; ++axis) {
      const float value = points[i][axis];
      std::memcpy(&cloud.data[point_offset + axis * sizeof(float)], &value, sizeof(float));
    }
  }
  return cloud;
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

class SonarLocNodeRuntimeTest : public ::testing::Test
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

TEST_F(SonarLocNodeRuntimeTest, PublishesFilteredCloudOdometryAndStatus)
{
  constexpr auto kPointsTopic = "/aqua_sonar_runtime_test/points";
  constexpr auto kFilteredTopic = "/aqua_sonar_runtime_test/points_filtered";
  constexpr auto kOdometryTopic = "/aqua_sonar_runtime_test/odometry";
  constexpr auto kStatusTopic = "/aqua_sonar_runtime_test/status";

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("topics.points", std::string(kPointsTopic)),
    rclcpp::Parameter("topics.filtered_points", std::string(kFilteredTopic)),
    rclcpp::Parameter("topics.odometry", std::string(kOdometryTopic)),
    rclcpp::Parameter("topics.status", std::string(kStatusTopic)),
    rclcpp::Parameter("scan_matching.backend", std::string("noop")),
    rclcpp::Parameter("preprocessing.min_points", 3),
    rclcpp::Parameter("preprocessing.max_range_m", 10.0),
  });

  auto sonar_node = std::make_shared<aqua_sonar_loc::SonarLocNode>(options);
  auto test_node = std::make_shared<rclcpp::Node>("aqua_sonar_runtime_test");

  std::vector<sensor_msgs::msg::PointCloud2> filtered_messages;
  std::vector<nav_msgs::msg::Odometry> odometry_messages;
  std::vector<aqua_msgs::msg::ScanMatchingStatus> status_messages;

  auto points_pub =
    test_node->create_publisher<sensor_msgs::msg::PointCloud2>(kPointsTopic, rclcpp::SensorDataQoS());
  auto filtered_sub = test_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    kFilteredTopic, rclcpp::SensorDataQoS(),
    [&filtered_messages](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      filtered_messages.push_back(*msg);
    });
  auto odometry_sub = test_node->create_subscription<nav_msgs::msg::Odometry>(
    kOdometryTopic, 10,
    [&odometry_messages](const nav_msgs::msg::Odometry::SharedPtr msg) {
      odometry_messages.push_back(*msg);
    });
  auto status_sub = test_node->create_subscription<aqua_msgs::msg::ScanMatchingStatus>(
    kStatusTopic, 10,
    [&status_messages](const aqua_msgs::msg::ScanMatchingStatus::SharedPtr msg) {
      status_messages.push_back(*msg);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sonar_node);
  executor.add_node(test_node);

  ASSERT_TRUE(spin_until(executor, [&]() {
    return points_pub->get_subscription_count() > 0;
  }));

  const auto cloud = make_cloud(
    test_node->now(),
    {{{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}}});
  points_pub->publish(cloud);

  ASSERT_TRUE(spin_until(executor, [&]() {
    return !filtered_messages.empty() && !odometry_messages.empty() && !status_messages.empty();
  }));

  const auto & filtered = filtered_messages.back();
  EXPECT_EQ(filtered.header.frame_id, "sonar_link");
  EXPECT_EQ(filtered.width, 3U);
  EXPECT_EQ(filtered.point_step, 3U * sizeof(float));

  const auto & odometry = odometry_messages.back();
  EXPECT_EQ(odometry.header.frame_id, "odom");
  EXPECT_EQ(odometry.child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(odometry.pose.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(odometry.pose.pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(odometry.pose.pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(odometry.pose.pose.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(odometry.pose.covariance[0], 0.25);
  EXPECT_DOUBLE_EQ(odometry.pose.covariance[35], 0.10);

  const auto & status = status_messages.back();
  EXPECT_EQ(status.backend, "noop");
  EXPECT_TRUE(status.success);
  EXPECT_TRUE(status.converged);
  EXPECT_EQ(status.total_points, 3U);
  EXPECT_EQ(status.finite_xyz_points, 3U);
  EXPECT_EQ(status.in_range_points, 3U);
  EXPECT_EQ(status.status, "noop identity transform");
}

}  // namespace
