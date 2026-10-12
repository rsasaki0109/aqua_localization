// SPDX-License-Identifier: Apache-2.0
// Online stereo frontend: synchronizes left/right images (CompressedImage or
// raw sensor_msgs/Image), optionally rectifies a fisheye pair from a Kalibr
// camchain, runs StereoTracker, and publishes aqua_msgs/StereoFeatures
// stamped with the left image's header stamp. With mono: true it tracks the
// left topic alone (optionally undistorting a pinhole-equidistant camera) and
// publishes u_right = NaN, which aqua_graph treats as monocular observations. Also publishes the per-frame
// processing time so real-time margins can be measured on the target.
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "aqua_msgs/msg/feature_descriptors.hpp"
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
    // Added to the image stamp of every published frame, e.g. Kalibr's
    // timeshift_cam_imu (t_imu = t_cam + shift).
    stamp_offset_s_ = declare_parameter<double>("stamp_offset_s", 0.0);
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
    const bool mono = declare_parameter<bool>("mono", false);
    // Monocular pinhole-equidistant camera to undistort ([fx, fy, cx, cy],
    // [k1..k4]); empty = the input is already undistorted.
    const auto mono_intrinsics =
      declare_parameter<std::vector<double>>("undistort.intrinsics", std::vector<double>{});
    const auto mono_distortion =
      declare_parameter<std::vector<double>>("undistort.distortion", std::vector<double>{});
    const auto mono_resolution =
      declare_parameter<std::vector<int64_t>>("undistort.resolution", std::vector<int64_t>{});
    const double mono_balance = declare_parameter<double>("undistort.balance", 0.0);

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
    config.clahe_clip_limit =
      declare_parameter<double>("clahe_clip_limit", config.clahe_clip_limit);
    config.clahe_tile_grid = declare_parameter<int>("clahe_tile_grid", config.clahe_tile_grid);
    config.saturation_level = declare_parameter<int>("saturation_level", config.saturation_level);
    config.reuse_tracks = declare_parameter<bool>("reuse.enabled", config.reuse_tracks);
    config.descriptors = declare_parameter<bool>("descriptors.enabled", config.descriptors);
    const auto descriptors_topic = declare_parameter<std::string>(
      "topics.descriptors", "/aqua_visual_frontend/feature_descriptors");
    config.reuse_max_hamming =
      declare_parameter<int>("reuse.max_hamming", config.reuse_max_hamming);
    config.reuse_ratio = declare_parameter<double>("reuse.ratio", config.reuse_ratio);
    config.reuse_max_retired =
      declare_parameter<int>("reuse.max_retired", config.reuse_max_retired);
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

    if (mono && !mono_intrinsics.empty()) {
      if (mono_intrinsics.size() != 4 || mono_distortion.size() != 4 ||
        mono_resolution.size() != 2)
      {
        throw std::invalid_argument(
                "undistort.intrinsics/distortion need 4 values, undistort.resolution 2");
      }
      const cv::Matx33d K(
        mono_intrinsics[0], 0.0, mono_intrinsics[2], 0.0, mono_intrinsics[1], mono_intrinsics[3],
        0.0, 0.0, 1.0);
      const cv::Vec4d D(mono_distortion[0], mono_distortion[1], mono_distortion[2],
        mono_distortion[3]);
      const cv::Size size(static_cast<int>(mono_resolution[0]),
        static_cast<int>(mono_resolution[1]));
      cv::Matx33d P;
      cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
        K, D, size, cv::Matx33d::eye(), P, mono_balance);
      cv::fisheye::initUndistortRectifyMap(
        K, D, cv::Matx33d::eye(), P, size, CV_32FC1, undistort_map1_, undistort_map2_);
      // The backend's camera model must use these (aqua_graph visual.fx/fy/cx/cy).
      RCLCPP_INFO(
        get_logger(), "undistorting the monocular camera: fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
        P(0, 0), P(1, 1), P(0, 2), P(1, 2));
    }

    features_pub_ = create_publisher<aqua_msgs::msg::StereoFeatures>(features_topic, 10);
    if (config.descriptors) {
      descriptors_pub_ =
        create_publisher<aqua_msgs::msg::FeatureDescriptors>(descriptors_topic, 10);
    }
    timing_pub_ = create_publisher<std_msgs::msg::Float64>(timing_topic, 10);

    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
    qos.depth = static_cast<size_t>(std::max(1, queue));
    if (mono) {
      const auto sub_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos);
      if (input_format == "compressed") {
        mono_compressed_ = create_subscription<Compressed>(
          left_topic, sub_qos, [this](Compressed::ConstSharedPtr msg) {
            on_pair(msg->header.stamp, decode(*msg), cv::Mat());
          });
      } else if (input_format == "raw") {
        mono_raw_ = create_subscription<Raw>(
          left_topic, sub_qos, [this](Raw::ConstSharedPtr msg) {
            on_pair(msg->header.stamp, decode(*msg), cv::Mat());
          });
      } else {
        throw std::invalid_argument("input_format must be 'compressed' or 'raw'");
      }
    } else if (input_format == "compressed") {
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
    if (right.empty() && !left.empty()) {
      // Monocular frame.
      if (!undistort_map1_.empty()) {
        if (left.size() != undistort_map1_.size()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "skipping wrongly sized monocular image");
          return;
        }
        cv::Mat undistorted;
        cv::remap(left, undistorted, undistort_map1_, undistort_map2_, cv::INTER_LINEAR);
        left = undistorted;
      }
    } else if (left.empty() || right.empty() || left.size() != right.size() ||
      (rectifier_ && left.size() != rectifier_->size()))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "skipping undecodable, mismatched or wrongly sized stereo pair");
      return;
    }
    if (rectifier_ && !right.empty()) {
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
      if (descriptors_pub_) {
        // Before the features, so a backend has them when the ids appear.
        aqua_msgs::msg::FeatureDescriptors d;
        d.header.stamp = stamp;
        d.header.frame_id = frame_id_;
        d.descriptor_bytes = 32;
        for (const auto & [id, row] : tracker_->take_new_descriptors()) {
          if (row.cols != 32 || row.type() != CV_8U) {
            continue;
          }
          d.ids.push_back(id);
          d.descriptors.insert(d.descriptors.end(), row.data, row.data + 32);
        }
        if (!d.ids.empty()) {
          descriptors_pub_->publish(d);
        }
      }
      aqua_msgs::msg::StereoFeatures msg;
      msg.header.stamp = stamp;
      if (stamp_offset_s_ != 0.0) {
        msg.header.stamp =
          (rclcpp::Time(stamp) + rclcpp::Duration::from_seconds(stamp_offset_s_));
      }
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
  double stamp_offset_s_{0.0};
  long processed_{0};
  std::unique_ptr<StereoTracker> tracker_;
  std::unique_ptr<StereoRectifier> rectifier_;
  PairSync<Compressed> compressed_;
  PairSync<Raw> raw_;
  rclcpp::Subscription<Compressed>::SharedPtr mono_compressed_;
  rclcpp::Subscription<Raw>::SharedPtr mono_raw_;
  cv::Mat undistort_map1_;
  cv::Mat undistort_map2_;
  rclcpp::Publisher<aqua_msgs::msg::StereoFeatures>::SharedPtr features_pub_;
  rclcpp::Publisher<aqua_msgs::msg::FeatureDescriptors>::SharedPtr descriptors_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr timing_pub_;
};

}  // namespace aqua_visual_frontend

RCLCPP_COMPONENTS_REGISTER_NODE(aqua_visual_frontend::StereoFrontendNode)
