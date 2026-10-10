// SPDX-License-Identifier: Apache-2.0
// Type-erased publish / deserialize / record support for the message types
// the offline replay harness can move between a bag and in-process nodes.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"

namespace aqua_replay
{

using AnyMessage = std::shared_ptr<void>;
using SerializedCallback = std::function<void (std::shared_ptr<rclcpp::SerializedMessage>)>;

struct TypeSupport
{
  std::string type_name;
  std::function<AnyMessage(const rclcpp::SerializedMessage &)> deserialize;
  // Header stamp (or first transform stamp for tf2_msgs/TFMessage), if any.
  std::function<std::optional<int64_t>(const void *)> header_stamp_ns;
  std::function<rclcpp::PublisherBase::SharedPtr(
      rclcpp::Node &, const std::string &, const rclcpp::QoS &)> create_publisher;
  std::function<void (rclcpp::PublisherBase &, const AnyMessage &)> publish;
  std::function<rclcpp::SubscriptionBase::SharedPtr(
      rclcpp::Node &, const std::string &, const rclcpp::QoS &, SerializedCallback)>
  create_recorder;
};

// nullptr when the type is not supported.
const TypeSupport * find_type_support(const std::string & type_name);
std::vector<std::string> supported_type_names();
// Message type of the stack's well-known output topics.
std::optional<std::string> default_type_for_topic(const std::string & topic);

}  // namespace aqua_replay
