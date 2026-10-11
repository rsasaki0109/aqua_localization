// SPDX-License-Identifier: Apache-2.0
// Online stereo frontend: synchronizes compressed (or raw mono8) left/right
// images, runs StereoTracker, and publishes aqua_msgs/StereoFeatures stamped
// with the left image's header stamp. Also publishes the per-frame
// processing time so real-time margins can be measured on the target.
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/imgcodecs.hpp>

#include "aqua_msgs/msg/stereo_features.hpp"
#include "aqua_visual_frontend/stereo_tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/float64.hpp"

namespace aqua_visual_frontend
{

class StereoFrontendNode : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::CompressedImage;
  using Policy = message_filters::sync_policies::ApproximateTime<Image, Image>;

  explicit StereoFrontendNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("stereo_frontend", options)
  {
    const auto left_topic = declare_parameter<std::string>(
      "topics.left", "/camera/left/image_dehazed/compressed");
    const auto right_topic = declare_parameter<std::string>(
      "topics.right", "/camera/right/image_dehazed/compressed");
    const auto features_topic = declare_parameter<std::string>(
      "topics.features", "/aqua_visual_frontend/stereo_features");
    const auto timing_topic = declare_parameter<std::string>(
      "topics.processing_time", "/aqua_visual_frontend/processing_time_ms");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_left");
    publish_every_ = std::max<int>(1, declare_parameter<int>("publish_every", 1));
    const double sync_slop_s = declare_parameter<double>("sync_slop_s", 0.005);
    const int queue = declare_parameter<int>("qos.image_depth", 5);

    StereoTrackerConfig config;
    config.max_features = declare_parameter<int>("max_features", config.max_features);
    config.min_distance_px = declare_parameter<int>("min_distance_px", config.min_distance_px);
    config.quality_level = declare_parameter<double>("quality_level", config.quality_level);
    config.lk_window = declare_parameter<int>("lk_window", config.lk_window);
    config.lk_levels = declare_parameter<int>("lk_levels", config.lk_levels);
    config.max_backtrack_px =
      declare_parameter<double>("max_backtrack_px", config.max_backtrack_px);
    config.fundamental_threshold_px =
      declare_parameter<double>("fundamental_threshold_px", config.fundamental_threshold_px);
    config.max_epipolar_px = declare_parameter<double>("max_epipolar_px", config.max_epipolar_px);
    config.min_disparity_px =
      declare_parameter<double>("min_disparity_px", config.min_disparity_px);
    config.max_disparity_px =
      declare_parameter<double>("max_disparity_px", config.max_disparity_px);
    config.initial_disparity_px =
      declare_parameter<double>("initial_disparity_px", config.initial_disparity_px);
    tracker_ = std::make_unique<StereoTracker>(config);

    features_pub_ = create_publisher<aqua_msgs::msg::StereoFeatures>(features_topic, 10);
    timing_pub_ = create_publisher<std_msgs::msg::Float64>(timing_topic, 10);

    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
    qos.depth = static_cast<size_t>(std::max(1, queue));
    left_sub_.subscribe(this, left_topic, qos);
    right_sub_.subscribe(this, right_topic, qos);
    sync_ = std::make_unique<message_filters::Synchronizer<Policy>>(
      Policy(10), left_sub_, right_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_s));
    sync_->registerCallback(
      std::bind(
        &StereoFrontendNode::on_pair, this, std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(), "stereo_frontend: left=%s right=%s -> %s (max_features=%d, publish_every=%d)",
      left_topic.c_str(), right_topic.c_str(), features_topic.c_str(), config.max_features,
      publish_every_);
  }

private:
  static cv::Mat decode(const Image & msg)
  {
    return cv::imdecode(msg.data, cv::IMREAD_GRAYSCALE);
  }

  void on_pair(const Image::ConstSharedPtr & left_msg, const Image::ConstSharedPtr & right_msg)
  {
    const auto start = std::chrono::steady_clock::now();
    const cv::Mat left = decode(*left_msg);
    const cv::Mat right = decode(*right_msg);
    if (left.empty() || right.empty() || left.size() != right.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "skipping undecodable or mismatched stereo pair");
      return;
    }
    const auto tracks = tracker_->process(left, right);
    ++processed_;
    // Track on every frame (short baselines track best) but only emit every
    // Nth frame, which bounds the backend's state rate.
    if ((processed_ - 1) % publish_every_ == 0) {
      aqua_msgs::msg::StereoFeatures msg;
      msg.header.stamp = left_msg->header.stamp;
      msg.header.frame_id = frame_id_;
      msg.ids.reserve(tracks.size());
      msg.u_left.reserve(tracks.size());
      msg.u_right.reserve(tracks.size());
      msg.v.reserve(tracks.size());
      for (const auto & t : tracks) {
        msg.ids.push_back(t.id);
        msg.u_left.push_back(t.u_left);
        msg.u_right.push_back(t.u_right);
        msg.v.push_back(t.v);
      }
      features_pub_->publish(msg);
    }
    std_msgs::msg::Float64 timing;
    timing.data = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    timing_pub_->publish(timing);
  }

  std::string frame_id_;
  int publish_every_{1};
  long processed_{0};
  std::unique_ptr<StereoTracker> tracker_;
  message_filters::Subscriber<Image> left_sub_;
  message_filters::Subscriber<Image> right_sub_;
  std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::Publisher<aqua_msgs::msg::StereoFeatures>::SharedPtr features_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr timing_pub_;
};

}  // namespace aqua_visual_frontend

RCLCPP_COMPONENTS_REGISTER_NODE(aqua_visual_frontend::StereoFrontendNode)
