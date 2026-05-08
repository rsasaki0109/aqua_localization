#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_msgs/msg/float64.hpp"

#define AQUA_IMU_LOC_DEPTH_TO_PRESSURE_DISABLE_MAIN
#include "../src/depth_to_pressure_node.cpp"

namespace
{

using namespace std::chrono_literals;

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

class DepthToPressureNodeTest : public ::testing::Test
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

TEST_F(DepthToPressureNodeTest, ConvertsPositiveDownDepthToFluidPressure)
{
  constexpr auto kDepthTopic = "/aqua_depth_to_pressure_test/depth";
  constexpr auto kPressureTopic = "/aqua_depth_to_pressure_test/pressure";
  constexpr double kReferencePressure = 101325.0;
  constexpr double kDensity = 1025.0;
  constexpr double kGravity = 9.80665;
  constexpr double kDepth = 2.0;

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("topics.depth", std::string(kDepthTopic)),
    rclcpp::Parameter("topics.pressure", std::string(kPressureTopic)),
    rclcpp::Parameter("frame_id", std::string("pressure_link")),
    rclcpp::Parameter("reference_pressure_pa", kReferencePressure),
    rclcpp::Parameter("water_density_kg_m3", kDensity),
    rclcpp::Parameter("gravity_mps2", kGravity),
    rclcpp::Parameter("pressure_variance_pa2", 4.0),
  });

  auto adapter_node = std::make_shared<aqua_imu_loc::DepthToPressureNode>(options);
  auto test_node = std::make_shared<rclcpp::Node>("aqua_depth_to_pressure_test");

  std::vector<sensor_msgs::msg::FluidPressure> pressure_messages;
  auto depth_pub =
    test_node->create_publisher<std_msgs::msg::Float64>(kDepthTopic, rclcpp::SensorDataQoS());
  auto pressure_sub = test_node->create_subscription<sensor_msgs::msg::FluidPressure>(
    kPressureTopic, rclcpp::SensorDataQoS(),
    [&pressure_messages](const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
      pressure_messages.push_back(*msg);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter_node);
  executor.add_node(test_node);

  ASSERT_TRUE(spin_until(executor, [&]() {
    return depth_pub->get_subscription_count() > 0;
  }));

  std_msgs::msg::Float64 depth;
  depth.data = kDepth;
  depth_pub->publish(depth);

  ASSERT_TRUE(spin_until(executor, [&]() { return !pressure_messages.empty(); }));

  const auto & pressure = pressure_messages.back();
  const double expected_pressure = kReferencePressure + kDensity * kGravity * kDepth;
  EXPECT_EQ(pressure.header.frame_id, "pressure_link");
  EXPECT_NEAR(pressure.fluid_pressure, expected_pressure, 1.0e-6);
  EXPECT_DOUBLE_EQ(pressure.variance, 4.0);
}

TEST_F(DepthToPressureNodeTest, RejectsDepthBelowMinimum)
{
  constexpr auto kDepthTopic = "/aqua_depth_to_pressure_reject_test/depth";
  constexpr auto kPressureTopic = "/aqua_depth_to_pressure_reject_test/pressure";

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("topics.depth", std::string(kDepthTopic)),
    rclcpp::Parameter("topics.pressure", std::string(kPressureTopic)),
    rclcpp::Parameter("min_depth_m", -0.5),
  });

  auto adapter_node = std::make_shared<aqua_imu_loc::DepthToPressureNode>(options);
  auto test_node = std::make_shared<rclcpp::Node>("aqua_depth_to_pressure_reject_test");

  std::vector<sensor_msgs::msg::FluidPressure> pressure_messages;
  auto depth_pub =
    test_node->create_publisher<std_msgs::msg::Float64>(kDepthTopic, rclcpp::SensorDataQoS());
  auto pressure_sub = test_node->create_subscription<sensor_msgs::msg::FluidPressure>(
    kPressureTopic, rclcpp::SensorDataQoS(),
    [&pressure_messages](const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
      pressure_messages.push_back(*msg);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter_node);
  executor.add_node(test_node);

  ASSERT_TRUE(spin_until(executor, [&]() {
    return depth_pub->get_subscription_count() > 0;
  }));

  std_msgs::msg::Float64 depth;
  depth.data = -2.0;
  depth_pub->publish(depth);

  executor.spin_some();
  std::this_thread::sleep_for(50ms);
  executor.spin_some();

  EXPECT_TRUE(pressure_messages.empty());
}

}  // namespace
