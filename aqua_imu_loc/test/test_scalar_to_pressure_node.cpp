#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

#define AQUA_IMU_LOC_SCALAR_TO_PRESSURE_DISABLE_MAIN
#include "../src/scalar_to_pressure_node.cpp"

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

class ScalarToPressureNodeTest : public ::testing::Test
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

TEST_F(ScalarToPressureNodeTest, PassesThroughFloat64Pressure)
{
  constexpr auto kScalarTopic = "/aqua_scalar_pressure_test/scalar";
  constexpr auto kPressureTopic = "/aqua_scalar_pressure_test/pressure";

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("topics.scalar", std::string(kScalarTopic)),
    rclcpp::Parameter("topics.pressure", std::string(kPressureTopic)),
    rclcpp::Parameter("input_type", std::string("float64")),
    rclcpp::Parameter("mode", std::string("pressure_pa")),
    rclcpp::Parameter("pressure_offset_pa", 10.0),
    rclcpp::Parameter("pressure_variance_pa2", 4.0),
  });

  auto adapter_node = std::make_shared<aqua_imu_loc::ScalarToPressureNode>(options);
  auto test_node = std::make_shared<rclcpp::Node>("aqua_scalar_pressure_test");

  std::vector<sensor_msgs::msg::FluidPressure> pressure_messages;
  auto scalar_pub =
    test_node->create_publisher<std_msgs::msg::Float64>(kScalarTopic, rclcpp::SensorDataQoS());
  auto pressure_sub = test_node->create_subscription<sensor_msgs::msg::FluidPressure>(
    kPressureTopic, rclcpp::SensorDataQoS(),
    [&pressure_messages](const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
      pressure_messages.push_back(*msg);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter_node);
  executor.add_node(test_node);

  ASSERT_TRUE(spin_until(executor, [&]() { return scalar_pub->get_subscription_count() > 0; }));

  std_msgs::msg::Float64 scalar;
  scalar.data = 101325.0;
  scalar_pub->publish(scalar);

  ASSERT_TRUE(spin_until(executor, [&]() { return !pressure_messages.empty(); }));

  const auto & pressure = pressure_messages.back();
  EXPECT_NEAR(pressure.fluid_pressure, 101335.0, 1.0e-6);
  EXPECT_DOUBLE_EQ(pressure.variance, 4.0);
}

TEST_F(ScalarToPressureNodeTest, ConvertsNtnuBarometerFloat32ToPressure)
{
  constexpr auto kScalarTopic = "/aqua_scalar_ntnu_test/scalar";
  constexpr auto kPressureTopic = "/aqua_scalar_ntnu_test/pressure";
  constexpr double kReferencePressure = 101325.0;
  constexpr double kDensity = 1025.0;
  constexpr double kGravity = 9.80665;
  constexpr double kDepth = 3.0;
  constexpr double kBaroOffset = 100.0;
  constexpr double kBaroScale = 2.0;
  const double barometer_measurement = kBaroOffset - kDepth * kBaroScale;

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("topics.scalar", std::string(kScalarTopic)),
    rclcpp::Parameter("topics.pressure", std::string(kPressureTopic)),
    rclcpp::Parameter("input_type", std::string("float32")),
    rclcpp::Parameter("mode", std::string("ntnu_barometer")),
    rclcpp::Parameter("reference_pressure_pa", kReferencePressure),
    rclcpp::Parameter("water_density_kg_m3", kDensity),
    rclcpp::Parameter("gravity_mps2", kGravity),
    rclcpp::Parameter("barometer_pressure_offset", kBaroOffset),
    rclcpp::Parameter("barometer_pressure_scale", kBaroScale),
  });

  auto adapter_node = std::make_shared<aqua_imu_loc::ScalarToPressureNode>(options);
  auto test_node = std::make_shared<rclcpp::Node>("aqua_scalar_ntnu_test");

  std::vector<sensor_msgs::msg::FluidPressure> pressure_messages;
  auto scalar_pub =
    test_node->create_publisher<std_msgs::msg::Float32>(kScalarTopic, rclcpp::SensorDataQoS());
  auto pressure_sub = test_node->create_subscription<sensor_msgs::msg::FluidPressure>(
    kPressureTopic, rclcpp::SensorDataQoS(),
    [&pressure_messages](const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
      pressure_messages.push_back(*msg);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter_node);
  executor.add_node(test_node);

  ASSERT_TRUE(spin_until(executor, [&]() { return scalar_pub->get_subscription_count() > 0; }));

  std_msgs::msg::Float32 scalar;
  scalar.data = static_cast<float>(barometer_measurement);
  scalar_pub->publish(scalar);

  ASSERT_TRUE(spin_until(executor, [&]() { return !pressure_messages.empty(); }));

  const auto & pressure = pressure_messages.back();
  const double expected_pressure = kReferencePressure + kDensity * kGravity * kDepth;
  EXPECT_NEAR(pressure.fluid_pressure, expected_pressure, 1.0e-3);
}

TEST_F(ScalarToPressureNodeTest, RejectsConvertedDepthOutsideRange)
{
  constexpr auto kScalarTopic = "/aqua_scalar_reject_test/scalar";
  constexpr auto kPressureTopic = "/aqua_scalar_reject_test/pressure";

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("topics.scalar", std::string(kScalarTopic)),
    rclcpp::Parameter("topics.pressure", std::string(kPressureTopic)),
    rclcpp::Parameter("mode", std::string("depth_m")),
    rclcpp::Parameter("max_depth_m", 1.0),
  });

  auto adapter_node = std::make_shared<aqua_imu_loc::ScalarToPressureNode>(options);
  auto test_node = std::make_shared<rclcpp::Node>("aqua_scalar_reject_test");

  std::vector<sensor_msgs::msg::FluidPressure> pressure_messages;
  auto scalar_pub =
    test_node->create_publisher<std_msgs::msg::Float64>(kScalarTopic, rclcpp::SensorDataQoS());
  auto pressure_sub = test_node->create_subscription<sensor_msgs::msg::FluidPressure>(
    kPressureTopic, rclcpp::SensorDataQoS(),
    [&pressure_messages](const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
      pressure_messages.push_back(*msg);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter_node);
  executor.add_node(test_node);

  ASSERT_TRUE(spin_until(executor, [&]() { return scalar_pub->get_subscription_count() > 0; }));

  std_msgs::msg::Float64 scalar;
  scalar.data = 5.0;
  scalar_pub->publish(scalar);

  executor.spin_some();
  std::this_thread::sleep_for(50ms);
  executor.spin_some();

  EXPECT_TRUE(pressure_messages.empty());
}

}  // namespace
