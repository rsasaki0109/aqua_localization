#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "aqua_msgs/msg/estimator_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#define AQUA_IMU_LOC_DISABLE_MAIN
#include "../src/imu_loc_node.cpp"

namespace
{

using namespace std::chrono_literals;

sensor_msgs::msg::Imu make_stationary_imu(const rclcpp::Time & stamp)
{
  sensor_msgs::msg::Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = "base_link";
  imu.orientation.w = 1.0;
  imu.linear_acceleration.z = 9.80665;
  return imu;
}

sensor_msgs::msg::FluidPressure make_pressure(
  const rclcpp::Time & stamp, double fluid_pressure)
{
  sensor_msgs::msg::FluidPressure pressure;
  pressure.header.stamp = stamp;
  pressure.header.frame_id = "pressure_link";
  pressure.fluid_pressure = fluid_pressure;
  return pressure;
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

class ImuLocNodeRuntimeTest : public ::testing::Test
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

TEST_F(ImuLocNodeRuntimeTest, PublishesOdometryStatusTfAndHandlesReset)
{
  constexpr auto kImuTopic = "/aqua_imu_runtime_test/imu";
  constexpr auto kPressureTopic = "/aqua_imu_runtime_test/pressure";
  constexpr auto kOdometryTopic = "/aqua_imu_runtime_test/odometry";
  constexpr auto kStatusTopic = "/aqua_imu_runtime_test/status";
  constexpr auto kResetService = "/aqua_imu_runtime_test/reset";

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("topics.imu", std::string(kImuTopic)),
    rclcpp::Parameter("topics.pressure", std::string(kPressureTopic)),
    rclcpp::Parameter("topics.odometry", std::string(kOdometryTopic)),
    rclcpp::Parameter("topics.status", std::string(kStatusTopic)),
    rclcpp::Parameter("services.reset", std::string(kResetService)),
    rclcpp::Parameter("publish.tf", true),
    rclcpp::Parameter("pressure.use_first_pressure_as_reference", false),
    rclcpp::Parameter("pressure.reference_pressure_pa", 101325.0),
    rclcpp::Parameter("pressure.depth_variance", 0.01),
    rclcpp::Parameter("dynamics.enable_linear_drag", false),
  });

  auto imu_node = std::make_shared<aqua_imu_loc::ImuLocNode>(options);
  auto test_node = std::make_shared<rclcpp::Node>("aqua_imu_runtime_test");

  std::vector<nav_msgs::msg::Odometry> odometry_messages;
  std::vector<aqua_msgs::msg::EstimatorStatus> status_messages;

  auto imu_pub =
    test_node->create_publisher<sensor_msgs::msg::Imu>(kImuTopic, rclcpp::SensorDataQoS());
  auto pressure_pub = test_node->create_publisher<sensor_msgs::msg::FluidPressure>(
    kPressureTopic, rclcpp::SensorDataQoS());
  auto odometry_sub = test_node->create_subscription<nav_msgs::msg::Odometry>(
    kOdometryTopic, 10,
    [&odometry_messages](const nav_msgs::msg::Odometry::SharedPtr msg) {
      odometry_messages.push_back(*msg);
    });
  auto status_sub = test_node->create_subscription<aqua_msgs::msg::EstimatorStatus>(
    kStatusTopic, 10,
    [&status_messages](const aqua_msgs::msg::EstimatorStatus::SharedPtr msg) {
      status_messages.push_back(*msg);
    });
  auto reset_client = test_node->create_client<std_srvs::srv::Trigger>(kResetService);

  tf2_ros::Buffer tf_buffer(test_node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer, test_node, false);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(imu_node);
  executor.add_node(test_node);

  ASSERT_TRUE(spin_until(executor, [&]() {
    return imu_pub->get_subscription_count() > 0 &&
           pressure_pub->get_subscription_count() > 0 &&
           reset_client->service_is_ready();
  }));

  const auto start = test_node->now();
  imu_pub->publish(make_stationary_imu(start));
  imu_pub->publish(make_stationary_imu(start + rclcpp::Duration::from_seconds(0.01)));

  const double depth_m = 2.0;
  const double pressure_pa = 101325.0 + 1025.0 * 9.80665 * depth_m;
  pressure_pub->publish(make_pressure(start + rclcpp::Duration::from_seconds(0.02), pressure_pa));

  ASSERT_TRUE(spin_until(executor, [&]() {
    return !odometry_messages.empty() && !status_messages.empty() &&
           status_messages.back().initialized && status_messages.back().update_count >= 2U &&
           odometry_messages.back().pose.pose.position.z < -1.0;
  }));

  const auto & odometry = odometry_messages.back();
  EXPECT_NEAR(-odometry.pose.pose.position.z, depth_m, 0.3);
  EXPECT_EQ(odometry.header.frame_id, "odom");
  EXPECT_EQ(odometry.child_frame_id, "base_link");

  const auto & status = status_messages.back();
  EXPECT_EQ(status.estimator_name, "aqua_imu_loc");
  EXPECT_EQ(status.backend, "additive_ukf");
  EXPECT_EQ(status.status, "running");
  EXPECT_GT(status.position_covariance_trace, 0.0);

  ASSERT_TRUE(spin_until(executor, [&]() {
    try {
      const auto transform = tf_buffer.lookupTransform("odom", "base_link", tf2::TimePointZero);
      EXPECT_NEAR(transform.transform.translation.z, odometry.pose.pose.position.z, 0.3);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }));

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response_future = reset_client->async_send_request(request);
  ASSERT_TRUE(spin_until(executor, [&]() {
    return response_future.wait_for(0ms) == std::future_status::ready;
  }));

  const auto response = response_future.get();
  ASSERT_TRUE(response->success);
  EXPECT_EQ(response->message, "aqua_imu_loc estimator state reset");

  ASSERT_TRUE(spin_until(executor, [&]() {
    return !status_messages.empty() && !status_messages.back().initialized &&
           status_messages.back().update_count == 0U &&
           status_messages.back().status == "waiting_for_imu";
  }));
}

}  // namespace
