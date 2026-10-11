// SPDX-License-Identifier: Apache-2.0
// Online stereo frontend: synchronizes left/right images (CompressedImage or
// raw sensor_msgs/Image), optionally rectifies a fisheye pair from a Kalibr
// camchain, runs StereoTracker, and publishes aqua_msgs/StereoFeatures
// stamped with the left image's header stamp. Also publishes the per-frame
// processing time so real-time margins can be measured on the target.
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "aqua_msgs/msg/stereo_features.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "aqua_visual_frontend/stereo_rectifier.hpp"
#include "aqua_visual_frontend/stereo_tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64.hpp"

namespace aqua_visual_frontend
{

class StereoFrontendNode : public rclcpp::Node
{
public:
  using Compressed = sensor_msgs::msg::CompressedImage;
  using Raw = sensor_msgs::msg::Image;

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
    // "compressed" (sensor_msgs/CompressedImage) or "raw" (sensor_msgs/Image).
    const auto input_format = declare_parameter<std::string>("input_format", "compressed");
    // Kalibr camchain of a raw pinhole-equidistant pair; empty = the input is
    // already rectified.
    const auto camchain = declare_parameter<std::string>("rectify.camchain", "");
    const auto left_camera = declare_parameter<std::string>("rectify.left_camera", "cam0");
    const auto right_camera = declare_parameter<std::string>("rectify.right_camera", "cam1");
    const double balance = declare_parameter<double>("rectify.balance", 0.0);

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
    if (!camchain.empty()) {
      rectifier_ = std::make_unique<StereoRectifier>(camchain, left_camera, right_camera, balance);
      const auto & g = rectifier_->geometry();
      // The backend's stereo model must use this geometry (aqua_graph
      // visual.fx/fy/cx/cy/baseline_m, body_T_camera rotated by R_left).
      RCLCPP_INFO(
        get_logger(),
        "rectifying %s (left) / %s (right) from %s: fx=%.3f fy=%.3f cx=%.3f cy=%.3f "
        "baseline=%.5f m", left_camera.c_str(), right_camera.c_str(), camchain.c_str(), g.fx,
        g.fy, g.cx, g.cy, g.baseline_m);
    }

    features_pub_ = create_publisher<aqua_msgs::msg::StereoFeatures>(features_topic, 10);
    timing_pub_ = create_publisher<std_msgs::msg::Float64>(timing_topic, 10);

    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
    qos.depth = static_cast<size_t>(std::max(1, queue));
    if (input_format == "compressed") {
      compressed_.subscribe(this, left_topic, right_topic, qos, sync_slop_s);
    } else if (input_format == "raw") {
      raw_.subscribe(this, left_topic, right_topic, qos, sync_slop_s);
    } else {
      throw std::invalid_argument("input_format must be 'compressed' or 'raw'");
    }

    RCLCPP_INFO(
      get_logger(), "stereo_frontend: left=%s right=%s -> %s (max_features=%d, publish_every=%d)",
      left_topic.c_str(), right_topic.c_str(), features_topic.c_str(), config.max_features,
      publish_every_);
  }

private:
  template<typename Msg>
  struct PairSync
  {
    using Policy = message_filters::sync_policies::ApproximateTime<Msg, Msg>;
    message_filters::Subscriber<Msg> left;
    message_filters::Subscriber<Msg> right;
    std::unique_ptr<message_filters::Synchronizer<Policy>> sync;
    StereoFrontendNode * node{nullptr};

    void subscribe(
      StereoFrontendNode * owner, const std::string & left_topic,
      const std::string & right_topic, const rmw_qos_profile_t & qos, double slop_s)
    {
      node = owner;
      left.subscribe(owner, left_topic, qos);
      right.subscribe(owner, right_topic, qos);
      sync = std::make_unique<message_filters::Synchronizer<Policy>>(Policy(10), left, right);
      sync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(slop_s));
      sync->registerCallback(&PairSync::on_pair, this);
    }

    void on_pair(const typename Msg::ConstSharedPtr & l, const typename Msg::ConstSharedPtr & r)
    {
      node->on_pair(l->header.stamp, decode(*l), decode(*r));
    }
  };

  static cv::Mat decode(const Compressed & msg)
  {
    return cv::imdecode(msg.data, cv::IMREAD_GRAYSCALE);
  }

  // Grayscale view of a raw image; empty for unsupported encodings.
  static cv::Mat decode(const Raw & msg)
  {
    namespace enc = sensor_msgs::image_encodings;
    const auto & e = msg.encoding;
    const int rows = static_cast<int>(msg.height);
    const int cols = static_cast<int>(msg.width);
    if (msg.is_bigendian || msg.data.size() < static_cast<size_t>(msg.step) * msg.height) {
      return {};
    }
    auto view = [&](int type) {
        return cv::Mat(rows, cols, type, const_cast<uint8_t *>(msg.data.data()), msg.step);
      };
    cv::Mat gray;
    if (e == enc::MONO8 || e == enc::TYPE_8UC1) {
      gray = view(CV_8UC1).clone();
    } else if (e == enc::MONO16 || e == enc::TYPE_16UC1) {
      view(CV_16UC1).convertTo(gray, CV_8U, 1.0 / 256.0);
    } else if (e == enc::BGR8) {
      cv::cvtColor(view(CV_8UC3), gray, cv::COLOR_BGR2GRAY);
    } else if (e == enc::RGB8) {
      cv::cvtColor(view(CV_8UC3), gray, cv::COLOR_RGB2GRAY);
    } else if (e == enc::BGRA8) {
      cv::cvtColor(view(CV_8UC4), gray, cv::COLOR_BGRA2GRAY);
    } else if (e == enc::RGBA8) {
      cv::cvtColor(view(CV_8UC4), gray, cv::COLOR_RGBA2GRAY);
    } else if (e == enc::BAYER_RGGB8) {
      cv::cvtColor(view(CV_8UC1), gray, cv::COLOR_BayerBG2GRAY);
    } else if (e == enc::BAYER_BGGR8) {
      cv::cvtColor(view(CV_8UC1), gray, cv::COLOR_BayerRG2GRAY);
    } else if (e == enc::BAYER_GBRG8) {
      cv::cvtColor(view(CV_8UC1), gray, cv::COLOR_BayerGR2GRAY);
    } else if (e == enc::BAYER_GRBG8) {
      cv::cvtColor(view(CV_8UC1), gray, cv::COLOR_BayerGB2GRAY);
    }
    return gray;
  }

  void on_pair(const builtin_interfaces::msg::Time & stamp, cv::Mat left, cv::Mat right)
  {
    const auto start = std::chrono::steady_clock::now();
    if (left.empty() || right.empty() || left.size() != right.size() ||
      (rectifier_ && left.size() != rectifier_->size()))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "skipping undecodable, mismatched or wrongly sized stereo pair");
      return;
    }
    if (rectifier_) {
      cv::Mat rect_left, rect_right;
      rectifier_->rectify_left(left, rect_left);
      rectifier_->rectify_right(right, rect_right);
      left = rect_left;
      right = rect_right;
    }
    const auto tracks = tracker_->process(left, right);
    ++processed_;
    // Track on every frame (short baselines track best) but only emit every
    // Nth frame, which bounds the backend's state rate.
    if ((processed_ - 1) % publish_every_ == 0) {
      aqua_msgs::msg::StereoFeatures msg;
      msg.header.stamp = stamp;
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
  std::unique_ptr<StereoRectifier> rectifier_;
  PairSync<Compressed> compressed_;
  PairSync<Raw> raw_;
  rclcpp::Publisher<aqua_msgs::msg::StereoFeatures>::SharedPtr features_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr timing_pub_;
};

}  // namespace aqua_visual_frontend

RCLCPP_COMPONENTS_REGISTER_NODE(aqua_visual_frontend::StereoFrontendNode)
