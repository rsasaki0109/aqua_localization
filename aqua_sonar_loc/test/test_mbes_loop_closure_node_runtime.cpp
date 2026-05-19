#include <array>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "aqua_msgs/msg/loop_closure_status.hpp"
#include "aqua_msgs/msg/pose_graph_keyframe.hpp"
#include "aqua_msgs/msg/pose_graph_loop_constraint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#define AQUA_SONAR_LOC_DISABLE_MBES_LOOP_MAIN
#include "../src/mbes_loop_closure_node.cpp"

namespace
{

using namespace std::chrono_literals;

sensor_msgs::msg::PointCloud2 make_cloud(
  const rclcpp::Time & stamp,
  const std::vector<std::array<float, 3>> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = "norbit";
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

aqua_msgs::msg::PoseGraphKeyframe make_keyframe(
  const rclcpp::Time & stamp,
  uint32_t id,
  double x)
{
  aqua_msgs::msg::PoseGraphKeyframe msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "map";
  msg.id = id;
  msg.pose.position.x = x;
  msg.pose.orientation.w = 1.0;
  return msg;
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

std::vector<std::array<float, 3>> asymmetric_points()
{
  return {{
    {0.0F, 0.0F, 0.0F},
    {1.0F, 0.2F, 0.1F},
    {-0.4F, 1.2F, 0.3F},
    {1.6F, -0.7F, 0.8F},
    {-1.3F, -0.3F, 1.0F},
    {0.3F, 1.9F, -0.4F},
    {2.0F, 1.1F, 0.5F},
    {-0.8F, 0.8F, -0.7F},
    {0.7F, -1.5F, 1.2F},
    {1.2F, 0.5F, -1.1F},
  }};
}

class MbesLoopClosureNodeRuntimeTest : public ::testing::Test
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

TEST_F(MbesLoopClosureNodeRuntimeTest, PublishesLoopConstraintForRepeatedSubmap)
{
  constexpr auto kPointsTopic = "/mbes_loop_runtime_test/points";
  constexpr auto kKeyframeTopic = "/mbes_loop_runtime_test/keyframe";
  constexpr auto kLoopTopic = "/mbes_loop_runtime_test/loop_constraint";
  constexpr auto kStatusTopic = "/mbes_loop_runtime_test/status";
  constexpr auto kMarkerTopic = "/mbes_loop_runtime_test/markers";

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("topics.points", std::string(kPointsTopic)),
    rclcpp::Parameter("topics.keyframe", std::string(kKeyframeTopic)),
    rclcpp::Parameter("topics.loop_constraint", std::string(kLoopTopic)),
    rclcpp::Parameter("topics.status", std::string(kStatusTopic)),
    rclcpp::Parameter("topics.markers", std::string(kMarkerTopic)),
    rclcpp::Parameter("submaps.min_points", 5),
    rclcpp::Parameter("submaps.max_points", 1000),
    rclcpp::Parameter("submaps.voxel_leaf_m", 0.0),
    rclcpp::Parameter("candidates.min_keyframe_separation", 0),
    rclcpp::Parameter("candidates.max_distance_m", 10.0),
    rclcpp::Parameter("candidates.max_per_keyframe", 2),
    rclcpp::Parameter("registration.backend", std::string("icp")),
    rclcpp::Parameter("registration.max_correspondence_distance_m", 2.0),
    rclcpp::Parameter("registration.max_iterations", 100),
    rclcpp::Parameter("registration.transformation_epsilon", 1.0e-10),
    rclcpp::Parameter("gates.max_fitness_score", 1.0e-4),
    rclcpp::Parameter("gates.max_correction_translation_m", 1.0),
    rclcpp::Parameter("gates.max_correction_rotation_rad", 0.2),
    rclcpp::Parameter("loop.translation_sigma_m", 1.5),
    rclcpp::Parameter("loop.rotation_sigma_rad", 0.25),
    rclcpp::Parameter("loop.optimize_after_insert", false),
  });

  auto loop_node = std::make_shared<aqua_sonar_loc::MbesLoopClosureNode>(options);
  auto test_node = std::make_shared<rclcpp::Node>("mbes_loop_runtime_test");

  std::vector<aqua_msgs::msg::PoseGraphLoopConstraint> loop_messages;
  std::vector<aqua_msgs::msg::LoopClosureStatus> status_messages;
  std::vector<visualization_msgs::msg::MarkerArray> marker_messages;
  auto keyframe_pub = test_node->create_publisher<aqua_msgs::msg::PoseGraphKeyframe>(
    kKeyframeTopic, rclcpp::QoS(10).transient_local());
  auto points_pub = test_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    kPointsTopic, rclcpp::SensorDataQoS());
  auto loop_sub = test_node->create_subscription<aqua_msgs::msg::PoseGraphLoopConstraint>(
    kLoopTopic, 10,
    [&loop_messages](const aqua_msgs::msg::PoseGraphLoopConstraint::SharedPtr msg) {
      loop_messages.push_back(*msg);
    });
  auto status_sub = test_node->create_subscription<aqua_msgs::msg::LoopClosureStatus>(
    kStatusTopic, 10,
    [&status_messages](const aqua_msgs::msg::LoopClosureStatus::SharedPtr msg) {
      status_messages.push_back(*msg);
    });
  auto marker_sub = test_node->create_subscription<visualization_msgs::msg::MarkerArray>(
    kMarkerTopic, 10,
    [&marker_messages](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
      marker_messages.push_back(*msg);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(loop_node);
  executor.add_node(test_node);

  ASSERT_TRUE(spin_until(executor, [&]() {
    return keyframe_pub->get_subscription_count() > 0 &&
           points_pub->get_subscription_count() > 0;
  }));

  const auto points = asymmetric_points();
  keyframe_pub->publish(make_keyframe(test_node->now(), 0, 0.0));
  ASSERT_TRUE(spin_until(executor, []() {return true;}, 100ms));
  points_pub->publish(make_cloud(test_node->now(), points));
  ASSERT_TRUE(spin_until(executor, []() {return true;}, 100ms));

  keyframe_pub->publish(make_keyframe(test_node->now(), 1, 0.0));
  ASSERT_TRUE(spin_until(executor, []() {return true;}, 100ms));
  points_pub->publish(make_cloud(test_node->now(), points));
  ASSERT_TRUE(spin_until(executor, []() {return true;}, 100ms));

  keyframe_pub->publish(make_keyframe(test_node->now(), 2, 0.0));
  ASSERT_TRUE(spin_until(executor, [&]() {return !loop_messages.empty();}));

  const auto & loop = loop_messages.back();
  EXPECT_EQ(loop.from_id, 0U);
  EXPECT_EQ(loop.to_id, 1U);
  EXPECT_FALSE(loop.optimize_after_insert);
  EXPECT_NEAR(loop.relative_pose.position.x, 0.0, 1e-3);
  EXPECT_NEAR(loop.relative_pose.position.y, 0.0, 1e-3);
  EXPECT_NEAR(loop.relative_pose.position.z, 0.0, 1e-3);
  EXPECT_NEAR(loop.relative_pose.orientation.w, 1.0, 1e-3);
  EXPECT_NEAR(loop.information[0], 1.0 / (1.5 * 1.5), 1e-9);
  EXPECT_NEAR(loop.information[35], 1.0 / (0.25 * 0.25), 1e-9);

  ASSERT_FALSE(status_messages.empty());
  const auto accepted_status = std::find_if(
    status_messages.begin(), status_messages.end(),
    [](const aqua_msgs::msg::LoopClosureStatus & msg) {return msg.accepted;});
  ASSERT_NE(accepted_status, status_messages.end());
  EXPECT_EQ(accepted_status->current_id, 1U);
  EXPECT_EQ(accepted_status->candidate_id, 0U);
  EXPECT_TRUE(accepted_status->converged);
  EXPECT_NEAR(accepted_status->fitness_score, 0.0, 1e-4);
  EXPECT_NEAR(accepted_status->correction_translation_m, 0.0, 1e-3);
  EXPECT_NEAR(accepted_status->correction_rotation_rad, 0.0, 1e-3);
  EXPECT_EQ(accepted_status->status, "accepted");

  ASSERT_FALSE(marker_messages.empty());
  const auto marker_array = std::find_if(
    marker_messages.begin(), marker_messages.end(),
    [](const visualization_msgs::msg::MarkerArray & msg) {return !msg.markers.empty();});
  ASSERT_NE(marker_array, marker_messages.end());
  const auto & marker = marker_array->markers.front();
  EXPECT_EQ(marker.ns, "mbes_loop_closure/accepted");
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::LINE_STRIP);
  ASSERT_EQ(marker.points.size(), 2U);
  EXPECT_NEAR(marker.points[0].x, 0.0, 1e-9);
  EXPECT_NEAR(marker.points[1].x, 0.0, 1e-9);
  EXPECT_GT(marker.color.g, marker.color.r);

  const auto loop_count_after_first_accept = loop_messages.size();
  points_pub->publish(make_cloud(test_node->now(), points));
  ASSERT_TRUE(spin_until(executor, []() {return true;}, 100ms));
  keyframe_pub->publish(make_keyframe(test_node->now(), 3, 0.0));
  ASSERT_TRUE(spin_until(executor, [&]() {
    return std::any_of(
      status_messages.begin(), status_messages.end(),
      [](const aqua_msgs::msg::LoopClosureStatus & msg) {
        return msg.status == "duplicate loop suppressed";
      });
  }));

  EXPECT_EQ(loop_messages.size(), loop_count_after_first_accept);
}

}  // namespace
