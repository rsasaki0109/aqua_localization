// SPDX-License-Identifier: Apache-2.0
#include "aqua_replay/message_registry.hpp"

#include <cstring>
#include <map>
#include <type_traits>
#include <utility>

#include "aqua_msgs/msg/estimator_status.hpp"
#include "aqua_msgs/msg/fusion_status.hpp"
#include "aqua_msgs/msg/graph_estimator_status.hpp"
#include "aqua_msgs/msg/loop_closure_status.hpp"
#include "aqua_msgs/msg/pose_graph_keyframe.hpp"
#include "aqua_msgs/msg/pose_graph_loop_constraint.hpp"
#include "aqua_msgs/msg/scan_matching_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace aqua_replay
{

namespace
{

template<typename T, typename = void>
struct HasHeader : std::false_type {};

template<typename T>
struct HasHeader<T, std::void_t<decltype(std::declval<T>().header.stamp)>>: std::true_type {};

int64_t to_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
}

template<typename T>
std::optional<int64_t> stamp_of(const T & message)
{
  if constexpr (HasHeader<T>::value) {
    return to_ns(message.header.stamp);
  } else if constexpr (std::is_same_v<T, tf2_msgs::msg::TFMessage>) {
    if (message.transforms.empty()) {
      return std::nullopt;
    }
    return to_ns(message.transforms.front().header.stamp);
  } else {
    (void)message;
    return std::nullopt;
  }
}

template<typename T>
TypeSupport make_support(const std::string & type_name)
{
  TypeSupport support;
  support.type_name = type_name;
  support.deserialize = [](const rclcpp::SerializedMessage & serialized) -> AnyMessage {
      auto message = std::make_shared<T>();
      rclcpp::Serialization<T> serialization;
      serialization.deserialize_message(&serialized, message.get());
      return message;
    };
  support.header_stamp_ns = [](const void * message) {
      return stamp_of(*static_cast<const T *>(message));
    };
  support.create_publisher = [](
    rclcpp::Node & node, const std::string & topic, const rclcpp::QoS & qos)
    -> rclcpp::PublisherBase::SharedPtr {
      return node.create_publisher<T>(topic, qos);
    };
  support.publish = [](rclcpp::PublisherBase & publisher, const AnyMessage & message) {
      // unique_ptr publish keeps intra-process delivery to a single copy.
      static_cast<rclcpp::Publisher<T> &>(publisher).publish(
        std::make_unique<T>(*static_cast<const T *>(message.get())));
    };
  support.create_recorder = [](
    rclcpp::Node & node, const std::string & topic, const rclcpp::QoS & qos,
    SerializedCallback callback) -> rclcpp::SubscriptionBase::SharedPtr {
      auto capacity = std::make_shared<size_t>(256);
      return node.create_subscription<T>(
        topic, qos,
        [callback = std::move(callback), capacity](std::shared_ptr<const T> message) {
          // The CDR serializer skips alignment padding, so serializing into a
          // fresh heap buffer leaves garbage bytes that change between runs.
          // Serialize into a zero-filled buffer that is already large enough
          // (retrying once at the exact size if the serializer had to grow it).
          rclcpp::Serialization<T> serialization;
          for (;;) {
            auto serialized = std::make_shared<rclcpp::SerializedMessage>(*capacity);
            auto & raw = serialized->get_rcl_serialized_message();
            std::memset(raw.buffer, 0, raw.buffer_capacity);
            const size_t zeroed = raw.buffer_capacity;
            serialization.serialize_message(message.get(), serialized.get());
            if (raw.buffer_capacity == zeroed) {
              callback(serialized);
              return;
            }
            *capacity = raw.buffer_capacity;
          }
        });
    };
  return support;
}

const std::map<std::string, TypeSupport> & registry()
{
  static const std::map<std::string, TypeSupport> types = [] {
      std::map<std::string, TypeSupport> m;
      auto add = [&m](TypeSupport support) {
          const std::string name = support.type_name;
          m.emplace(name, std::move(support));
        };
      add(make_support<aqua_msgs::msg::EstimatorStatus>("aqua_msgs/msg/EstimatorStatus"));
      add(make_support<aqua_msgs::msg::FusionStatus>("aqua_msgs/msg/FusionStatus"));
      add(make_support<aqua_msgs::msg::GraphEstimatorStatus>(
          "aqua_msgs/msg/GraphEstimatorStatus"));
      add(make_support<aqua_msgs::msg::LoopClosureStatus>("aqua_msgs/msg/LoopClosureStatus"));
      add(make_support<aqua_msgs::msg::PoseGraphKeyframe>("aqua_msgs/msg/PoseGraphKeyframe"));
      add(make_support<aqua_msgs::msg::PoseGraphLoopConstraint>(
          "aqua_msgs/msg/PoseGraphLoopConstraint"));
      add(make_support<aqua_msgs::msg::ScanMatchingStatus>("aqua_msgs/msg/ScanMatchingStatus"));
      add(make_support<geometry_msgs::msg::PoseStamped>("geometry_msgs/msg/PoseStamped"));
      add(make_support<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "geometry_msgs/msg/PoseWithCovarianceStamped"));
      add(make_support<geometry_msgs::msg::TwistStamped>("geometry_msgs/msg/TwistStamped"));
      add(make_support<geometry_msgs::msg::TwistWithCovarianceStamped>(
          "geometry_msgs/msg/TwistWithCovarianceStamped"));
      add(make_support<geometry_msgs::msg::Vector3Stamped>("geometry_msgs/msg/Vector3Stamped"));
      add(make_support<nav_msgs::msg::Odometry>("nav_msgs/msg/Odometry"));
      add(make_support<nav_msgs::msg::Path>("nav_msgs/msg/Path"));
      add(make_support<sensor_msgs::msg::CameraInfo>("sensor_msgs/msg/CameraInfo"));
      add(make_support<sensor_msgs::msg::FluidPressure>("sensor_msgs/msg/FluidPressure"));
      add(make_support<sensor_msgs::msg::Image>("sensor_msgs/msg/Image"));
      add(make_support<sensor_msgs::msg::Imu>("sensor_msgs/msg/Imu"));
      add(make_support<sensor_msgs::msg::MagneticField>("sensor_msgs/msg/MagneticField"));
      add(make_support<sensor_msgs::msg::NavSatFix>("sensor_msgs/msg/NavSatFix"));
      add(make_support<sensor_msgs::msg::PointCloud2>("sensor_msgs/msg/PointCloud2"));
      add(make_support<std_msgs::msg::Float32>("std_msgs/msg/Float32"));
      add(make_support<std_msgs::msg::Float64>("std_msgs/msg/Float64"));
      add(make_support<std_msgs::msg::UInt32>("std_msgs/msg/UInt32"));
      add(make_support<tf2_msgs::msg::TFMessage>("tf2_msgs/msg/TFMessage"));
      add(make_support<visualization_msgs::msg::MarkerArray>(
          "visualization_msgs/msg/MarkerArray"));
      return m;
    }();
  return types;
}

}  // namespace

const TypeSupport * find_type_support(const std::string & type_name)
{
  const auto & types = registry();
  const auto it = types.find(type_name);
  return it == types.end() ? nullptr : &it->second;
}

std::vector<std::string> supported_type_names()
{
  std::vector<std::string> names;
  for (const auto & entry : registry()) {
    names.push_back(entry.first);
  }
  return names;
}

std::optional<std::string> default_type_for_topic(const std::string & topic)
{
  static const std::map<std::string, std::string> defaults = {
    {"/aqua_imu_loc/odometry", "nav_msgs/msg/Odometry"},
    {"/aqua_imu_loc/status", "aqua_msgs/msg/EstimatorStatus"},
    {"/aqua_sonar_loc/odometry", "nav_msgs/msg/Odometry"},
    {"/aqua_sonar_loc/status", "aqua_msgs/msg/ScanMatchingStatus"},
    {"/aqua_sonar_loc/points_filtered", "sensor_msgs/msg/PointCloud2"},
    {"/aqua_fusion/odometry", "nav_msgs/msg/Odometry"},
    {"/aqua_fusion/status", "aqua_msgs/msg/FusionStatus"},
    {"/aqua_pose_graph/path", "nav_msgs/msg/Path"},
    {"/aqua_pose_graph/keyframe", "aqua_msgs/msg/PoseGraphKeyframe"},
    {"/aqua_pose_graph/keyframe_count", "std_msgs/msg/UInt32"},
    {"/aqua_pose_graph/loop_constraint", "aqua_msgs/msg/PoseGraphLoopConstraint"},
    {"/aqua_pose_graph/loop_constraint_count", "std_msgs/msg/UInt32"},
    {"/aqua_pose_graph/optimization_count", "std_msgs/msg/UInt32"},
    {"/aqua_pose_graph/optimization_chi2", "std_msgs/msg/Float64"},
    {"/mbes_loop_closure/status", "aqua_msgs/msg/LoopClosureStatus"},
    {"/mbes_loop_closure/markers", "visualization_msgs/msg/MarkerArray"},
    {"/aqua_graph/odometry", "nav_msgs/msg/Odometry"},
    {"/aqua_graph/status", "aqua_msgs/msg/GraphEstimatorStatus"},
    {"/ground_truth/odometry", "nav_msgs/msg/Odometry"},
    {"/tf", "tf2_msgs/msg/TFMessage"},
  };
  const auto it = defaults.find(topic);
  if (it == defaults.end()) {
    return std::nullopt;
  }
  return it->second;
}

}  // namespace aqua_replay
