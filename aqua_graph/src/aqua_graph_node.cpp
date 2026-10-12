// SPDX-License-Identifier: Apache-2.0
// ROS 2 front end for the tightly-coupled factor-graph estimator.
//
// Subscribes to IMU, fluid pressure, DVL body velocity, and an optional
// absolute-position odometry (e.g. a visual frontend), and publishes the
// estimate as nav_msgs/Odometry plus per-state GraphEstimatorStatus.
// Measurements can arrive out of order across topics: the estimator attaches
// each one to its nearest grid state as long as that state is still inside
// the smoother window. The IMU stream itself must be time-ordered.
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "aqua_graph/estimator.hpp"
#include "aqua_graph/factors.hpp"
#include "aqua_graph/loop_bias.hpp"
#include "aqua_imu_loc/pressure_depth_converter.hpp"
#include "aqua_msgs/msg/graph_estimator_status.hpp"
#include "aqua_msgs/msg/pose_graph_keyframe.hpp"
#include "aqua_msgs/msg/pose_graph_loop_constraint.hpp"
#include "aqua_msgs/msg/stereo_features.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace aqua_graph
{

namespace
{

double to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
}

builtin_interfaces::msg::Time to_stamp(double seconds)
{
  builtin_interfaces::msg::Time stamp;
  const auto ns = static_cast<int64_t>(std::llround(seconds * 1.0e9));
  stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return stamp;
}

geometry_msgs::msg::Vector3 to_msg(const gtsam::Vector3 & v)
{
  geometry_msgs::msg::Vector3 out;
  out.x = v.x();
  out.y = v.y();
  out.z = v.z();
  return out;
}

}  // namespace

class AquaGraphNode : public rclcpp::Node
{
public:
  explicit AquaGraphNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("aqua_graph", options)
  {
    const std::string imu_topic = declare_parameter<std::string>("topics.imu", "/imu/data");
    const std::string pressure_topic =
      declare_parameter<std::string>("topics.pressure", "/pressure");
    const std::string dvl_topic = declare_parameter<std::string>("topics.dvl_velocity", "");
    // Same DVL velocity as TwistWithCovarianceStamped; the message's linear
    // covariance sets the factor sigma when dvl.use_message_covariance is set.
    const std::string dvl_cov_topic =
      declare_parameter<std::string>("topics.dvl_velocity_with_covariance", "");
    const std::string heading_topic = declare_parameter<std::string>("topics.heading", "");
    const std::string position_topic =
      declare_parameter<std::string>("topics.position_odometry", "");
    const std::string stereo_topic =
      declare_parameter<std::string>("topics.stereo_features", "");
    const std::string odometry_topic =
      declare_parameter<std::string>("topics.odometry", "/aqua_graph/odometry");
    const std::string status_topic =
      declare_parameter<std::string>("topics.status", "/aqua_graph/status");
    map_frame_ = declare_parameter<std::string>("frames.map", "map");
    base_frame_ = declare_parameter<std::string>("frames.base_link", "base_link");
    publish_imu_rate_ = declare_parameter<bool>("publish.imu_rate", true);
    const int sensor_depth = declare_parameter<int>("qos.sensor_depth", 200);
    // Loop closures (e.g. MBES bathymetric loops) -> vertical gyro bias. Off
    // unless a loop topic is set; see loop_bias.hpp.
    const std::string loop_topic = declare_parameter<std::string>("loop_bias.loop_topic", "");
    const std::string keyframe_topic =
      declare_parameter<std::string>("loop_bias.keyframe_topic", "/aqua_pose_graph/keyframe");
    LoopBiasConfig loop_bias;
    loop_bias.yaw_sigma_rad =
      declare_parameter<double>("loop_bias.yaw_sigma_rad", loop_bias.yaw_sigma_rad);
    loop_bias.min_span_s = declare_parameter<double>("loop_bias.min_span_s", loop_bias.min_span_s);
    loop_bias.bias_random_walk =
      declare_parameter<double>("loop_bias.bias_random_walk", loop_bias.bias_random_walk);
    loop_bias.initial_sigma =
      declare_parameter<double>("loop_bias.initial_sigma", loop_bias.initial_sigma);
    loop_bias_ = std::make_unique<LoopBiasEstimator>(loop_bias);

    EstimatorConfig config;
    config.state_period_s = declare_parameter<double>("graph.state_period_s", config.state_period_s);
    config.smoother_lag_s = declare_parameter<double>("graph.smoother_lag_s", config.smoother_lag_s);
    config.processing_delay_s =
      declare_parameter<double>("graph.processing_delay_s", config.processing_delay_s);
    config.relinearize_threshold =
      declare_parameter<double>("graph.relinearize_threshold", config.relinearize_threshold);
    config.relinearize_skip =
      declare_parameter<int>("graph.relinearize_skip", config.relinearize_skip);

    auto & imu = config.imu;
    imu.gravity_mps2 = declare_parameter<double>("imu.gravity_mps2", imu.gravity_mps2);
    imu.accel_noise_density =
      declare_parameter<double>("imu.accel_noise_density", imu.accel_noise_density);
    imu.gyro_noise_density =
      declare_parameter<double>("imu.gyro_noise_density", imu.gyro_noise_density);
    imu.accel_bias_random_walk =
      declare_parameter<double>("imu.accel_bias_random_walk", imu.accel_bias_random_walk);
    imu.gyro_bias_random_walk =
      declare_parameter<double>("imu.gyro_bias_random_walk", imu.gyro_bias_random_walk);
    imu.integration_sigma =
      declare_parameter<double>("imu.integration_sigma", imu.integration_sigma);

    auto & init = config.init;
    const std::string attitude_source =
      declare_parameter<std::string>("init.attitude_source", "accelerometer");
    const auto parsed = parse_attitude_init_source(attitude_source);
    if (!parsed) {
      throw std::invalid_argument(
              "init.attitude_source must be 'accelerometer' or 'imu_orientation'");
    }
    init.attitude_source = *parsed;
    init.static_window_s = declare_parameter<double>("init.static_window_s", init.static_window_s);
    init.keep_imu_yaw = declare_parameter<bool>("init.keep_imu_yaw", init.keep_imu_yaw);
    init.gyro_bias_from_static_window = declare_parameter<bool>(
      "init.gyro_bias_from_static_window", init.gyro_bias_from_static_window);
    init.position_sigma_m =
      declare_parameter<double>("init.position_sigma_m", init.position_sigma_m);
    init.roll_pitch_sigma_rad =
      declare_parameter<double>("init.roll_pitch_sigma_rad", init.roll_pitch_sigma_rad);
    init.yaw_sigma_rad = declare_parameter<double>("init.yaw_sigma_rad", init.yaw_sigma_rad);
    init.velocity_sigma_mps =
      declare_parameter<double>("init.velocity_sigma_mps", init.velocity_sigma_mps);
    init.accel_bias_sigma =
      declare_parameter<double>("init.accel_bias_sigma", init.accel_bias_sigma);
    init.gyro_bias_sigma = declare_parameter<double>("init.gyro_bias_sigma", init.gyro_bias_sigma);
    init.bootstrap_velocity_sigma_mps = declare_parameter<double>(
      "init.bootstrap_velocity_sigma_mps", init.bootstrap_velocity_sigma_mps);
    init.bootstrap_landmarks =
      declare_parameter<int>("init.bootstrap_landmarks", init.bootstrap_landmarks);

    auto & dvl = config.dvl;
    dvl.enabled = !dvl_topic.empty() || !dvl_cov_topic.empty();
    use_dvl_covariance_ = declare_parameter<bool>("dvl.use_message_covariance", false);
    // Added to DVL header stamps (sensor latency calibration): the true
    // measurement time is stamp + time_offset_s.
    dvl_time_offset_s_ = declare_parameter<double>("dvl.time_offset_s", 0.0);
    dvl.anchor_states = declare_parameter<bool>("dvl.anchor_states", dvl.anchor_states);
    const auto dvl_rpy = declare_parameter<std::vector<double>>(
      "dvl.mount.rotation_rpy_rad", {0.0, 0.0, 0.0});
    const auto dvl_lever = declare_parameter<std::vector<double>>(
      "dvl.lever_arm_m", {0.0, 0.0, 0.0});
    dvl.body_R_dvl = rotation_from_rpy(dvl_rpy, "dvl.mount.rotation_rpy_rad");
    dvl.lever_arm_body = point_from(dvl_lever, "dvl.lever_arm_m");
    dvl.velocity_sigma_mps =
      declare_parameter<double>("dvl.velocity_sigma_mps", dvl.velocity_sigma_mps);
    dvl.huber_k = declare_parameter<double>("dvl.huber_k", dvl.huber_k);
    dvl.max_speed_mps = declare_parameter<double>("dvl.max_speed_mps", dvl.max_speed_mps);

    auto & depth = config.depth;
    depth.enabled = !pressure_topic.empty();
    depth.depth_sigma_m = declare_parameter<double>("pressure.depth_sigma_m", depth.depth_sigma_m);
    depth.huber_k = declare_parameter<double>("pressure.huber_k", depth.huber_k);
    depth.lever_arm_body = point_from(
      declare_parameter<std::vector<double>>("pressure.lever_arm_m", {0.0, 0.0, 0.0}),
      "pressure.lever_arm_m");
    aqua_imu_loc::PressureDepthConfig pressure_config;
    pressure_config.use_first_pressure_as_reference = declare_parameter<bool>(
      "pressure.use_first_pressure_as_reference", pressure_config.use_first_pressure_as_reference);
    pressure_config.reference_pressure_pa = declare_parameter<double>(
      "pressure.reference_pressure_pa", pressure_config.reference_pressure_pa);
    pressure_config.water_density_kg_m3 = declare_parameter<double>(
      "pressure.water_density_kg_m3", pressure_config.water_density_kg_m3);
    pressure_config.depth_offset_m =
      declare_parameter<double>("pressure.depth_offset_m", pressure_config.depth_offset_m);
    pressure_config.gravity_mps2 = imu.gravity_mps2;
    pressure_converter_.configure(pressure_config);

    depth.hold_initial_depth =
      declare_parameter<bool>("depth.hold_initial_depth", depth.hold_initial_depth);
    depth.hold_sigma_m = declare_parameter<double>("depth.hold_sigma_m", depth.hold_sigma_m);

    auto & heading = config.heading;
    heading.enabled = !heading_topic.empty();
    heading.axis_body = point_from(
      declare_parameter<std::vector<double>>("heading.axis_body", {1.0, 0.0, 0.0}),
      "heading.axis_body");
    // Heading of the measuring sensor's x axis is mapped to axis_body via this
    // offset: measured = heading(sensor x) + offset_rad.
    heading_offset_rad_ = declare_parameter<double>("heading.offset_rad", 0.0);
    heading.sigma_rad = declare_parameter<double>("heading.sigma_rad", heading.sigma_rad);
    heading.huber_k = declare_parameter<double>("heading.huber_k", heading.huber_k);
    heading.align_initial_yaw =
      declare_parameter<bool>("heading.align_initial_yaw", heading.align_initial_yaw);
    use_heading_covariance_ = declare_parameter<bool>("heading.use_message_covariance", false);

    auto & position = config.position;
    position.enabled = !position_topic.empty();
    position.sigma_floor_m =
      declare_parameter<double>("position.sigma_floor_m", position.sigma_floor_m);
    position.huber_k = declare_parameter<double>("position.huber_k", position.huber_k);
    use_position_covariance_ = declare_parameter<bool>("position.use_message_covariance", true);

    auto & visual = config.visual;
    visual.enabled = !stereo_topic.empty();
    visual.fx = declare_parameter<double>("visual.camera.fx", visual.fx);
    visual.fy = declare_parameter<double>("visual.camera.fy", visual.fy);
    visual.cx = declare_parameter<double>("visual.camera.cx", visual.cx);
    visual.cy = declare_parameter<double>("visual.camera.cy", visual.cy);
    visual.baseline_m = declare_parameter<double>("visual.camera.baseline_m", visual.baseline_m);
    // body_P_camera as a row-major 4x4 homogeneous matrix (left camera in the
    // body/IMU frame); identity by default.
    const auto body_T_camera = declare_parameter<std::vector<double>>(
      "visual.body_T_camera",
      {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    visual.body_P_camera = pose_from_matrix(body_T_camera, "visual.body_T_camera");
    visual.pixel_sigma = declare_parameter<double>("visual.pixel_sigma", visual.pixel_sigma);
    visual.huber_k = declare_parameter<double>("visual.huber_k", visual.huber_k);
    visual.min_disparity_px =
      declare_parameter<double>("visual.min_disparity_px", visual.min_disparity_px);
    visual.max_depth_m = declare_parameter<double>("visual.max_depth_m", visual.max_depth_m);
    visual.landmark_prior_sigma_m =
      declare_parameter<double>("visual.landmark_prior_sigma_m", visual.landmark_prior_sigma_m);
    // Monocular features (u_right = NaN, e.g. stereo_frontend mono: true).
    visual.mono_min_observations =
      declare_parameter<int>("visual.mono.min_observations", visual.mono_min_observations);
    visual.mono_min_parallax_rad =
      declare_parameter<double>("visual.mono.min_parallax_rad", visual.mono_min_parallax_rad);
    visual.mono_max_reprojection_px = declare_parameter<double>(
      "visual.mono.max_reprojection_px", visual.mono_max_reprojection_px);
    visual.mono_min_depth_m =
      declare_parameter<double>("visual.mono.min_depth_m", visual.mono_min_depth_m);

    estimator_ = std::make_unique<TightlyCoupledEstimator>(config);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(sensor_depth));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, sensor_qos, [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {on_imu(*msg);});
    if (!pressure_topic.empty()) {
      pressure_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
        pressure_topic, sensor_qos,
        [this](sensor_msgs::msg::FluidPressure::ConstSharedPtr msg) {on_pressure(*msg);});
    }
    if (!dvl_topic.empty()) {
      dvl_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        dvl_topic, sensor_qos,
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {on_dvl(*msg);});
    }
    if (!dvl_cov_topic.empty()) {
      dvl_cov_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        dvl_cov_topic, sensor_qos,
        [this](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg) {
          on_dvl_cov(*msg);
        });
    }
    if (!heading_topic.empty()) {
      heading_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        heading_topic, sensor_qos,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {on_heading(*msg);});
    }
    if (!position_topic.empty()) {
      position_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        position_topic, rclcpp::QoS(50),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_position(*msg);});
    }
    if (!stereo_topic.empty()) {
      stereo_sub_ = create_subscription<aqua_msgs::msg::StereoFeatures>(
        stereo_topic, rclcpp::QoS(50),
        [this](aqua_msgs::msg::StereoFeatures::ConstSharedPtr msg) {on_stereo(*msg);});
    }
    if (!loop_topic.empty()) {
      loop_bias_enabled_ = true;
      keyframe_sub_ = create_subscription<aqua_msgs::msg::PoseGraphKeyframe>(
        keyframe_topic, rclcpp::QoS(1000),
        [this](aqua_msgs::msg::PoseGraphKeyframe::ConstSharedPtr msg) {
          loop_bias_->add_keyframe(msg->id, to_seconds(msg->header.stamp));
        });
      loop_sub_ = create_subscription<aqua_msgs::msg::PoseGraphLoopConstraint>(
        loop_topic, rclcpp::QoS(100),
        [this](aqua_msgs::msg::PoseGraphLoopConstraint::ConstSharedPtr msg) {on_loop(*msg);});
    }
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic, rclcpp::QoS(100));
    status_pub_ =
      create_publisher<aqua_msgs::msg::GraphEstimatorStatus>(status_topic, rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(),
      "aqua_graph started: imu=%s pressure=%s dvl=%s heading=%s position=%s stereo=%s odom=%s "
      "grid=%.3fs lag=%.1fs",
      imu_topic.c_str(), pressure_topic.empty() ? "<disabled>" : pressure_topic.c_str(),
      !dvl_topic.empty() ? dvl_topic.c_str() :
      dvl_cov_topic.empty() ? "<disabled>" : dvl_cov_topic.c_str(),
      heading_topic.empty() ? "<disabled>" : heading_topic.c_str(),
      position_topic.empty() ? "<disabled>" : position_topic.c_str(),
      stereo_topic.empty() ? "<disabled>" : stereo_topic.c_str(), odometry_topic.c_str(),
      config.state_period_s, config.smoother_lag_s);
  }

private:
  gtsam::Rot3 rotation_from_rpy(const std::vector<double> & rpy, const std::string & name)
  {
    if (rpy.size() != 3) {
      throw std::invalid_argument(name + " must have 3 elements");
    }
    return gtsam::Rot3::Ypr(rpy[2], rpy[1], rpy[0]);
  }

  gtsam::Point3 point_from(const std::vector<double> & xyz, const std::string & name)
  {
    if (xyz.size() != 3) {
      throw std::invalid_argument(name + " must have 3 elements");
    }
    return gtsam::Point3(xyz[0], xyz[1], xyz[2]);
  }

  gtsam::Pose3 pose_from_matrix(const std::vector<double> & m, const std::string & name)
  {
    if (m.size() != 16) {
      throw std::invalid_argument(name + " must have 16 elements (row-major 4x4)");
    }
    gtsam::Matrix3 r;
    r << m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10];
    // Re-orthonormalize: calibration files often carry 4-digit rotations.
    const Eigen::JacobiSVD<gtsam::Matrix3> svd(r, Eigen::ComputeFullU | Eigen::ComputeFullV);
    gtsam::Matrix3 rotation = svd.matrixU() * svd.matrixV().transpose();
    if (rotation.determinant() < 0.0) {
      throw std::invalid_argument(name + " rotation is not proper");
    }
    return gtsam::Pose3(gtsam::Rot3(rotation), gtsam::Point3(m[3], m[7], m[11]));
  }

  void on_stereo(const aqua_msgs::msg::StereoFeatures & msg)
  {
    const size_t n = msg.ids.size();
    if (msg.u_left.size() != n || msg.u_right.size() != n || msg.v.size() != n) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected stereo features with mismatched arrays.");
      return;
    }
    std::vector<StereoObservation> observations(n);
    for (size_t i = 0; i < n; ++i) {
      observations[i] = StereoObservation{msg.ids[i], msg.u_left[i], msg.u_right[i], msg.v[i]};
    }
    estimator_->add_stereo_frame(to_seconds(msg.header.stamp), observations);
  }

  void on_imu(const sensor_msgs::msg::Imu & msg)
  {
    const double t = to_seconds(msg.header.stamp);
    const gtsam::Vector3 accel(
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    const gtsam::Vector3 gyro(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    std::optional<gtsam::Rot3> orientation;
    const auto & q = msg.orientation;
    const double qn = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (msg.orientation_covariance[0] >= 0.0 && qn > 0.5 && std::isfinite(qn)) {
      orientation = gtsam::Rot3::Quaternion(q.w / qn, q.x / qn, q.y / qn, q.z / qn);
    }
    const bool was_initialized = estimator_->initialized();
    const auto states = estimator_->add_imu(t, accel, gyro, orientation);
    last_gyro_ = gyro;
    if (!was_initialized && estimator_->initialized()) {
      RCLCPP_INFO(get_logger(), "aqua_graph initialized at t=%.3f", t);
    }
    for (const auto & state : states) {
      publish_status(state);
      if (!publish_imu_rate_) {
        publish_odometry(state);
      }
    }
    if (publish_imu_rate_) {
      if (const auto estimate = estimator_->imu_rate_estimate()) {
        publish_odometry(*estimate);
      }
    }
  }

  void on_loop(const aqua_msgs::msg::PoseGraphLoopConstraint & msg)
  {
    if (!last_output_) {
      return;
    }
    const auto & q = msg.relative_pose.orientation;
    const auto update = loop_bias_->add_loop(
      msg.from_id, msg.to_id, gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z));
    if (!update) {
      return;
    }
    const gtsam::Rot3 & attitude = last_output_->pose.rotation();
    const gtsam::Vector3 bias = LoopBiasEstimator::with_vertical_bias(
      attitude, last_output_->bias.gyroscope(), update->bias);
    estimator_->set_gyro_bias(bias, "gyro bias from loop closure");
    RCLCPP_INFO(
      get_logger(),
      "loop %u -> %u over %.0f s: yaw residual %.2f deg, observed vertical gyro bias %.1f deg/h "
      "(+-%.1f), fused %.1f deg/h (+-%.1f)",
      msg.from_id, msg.to_id, update->span_s, update->yaw_residual_rad * 180.0 / M_PI,
      update->observed_bias * 180.0 / M_PI * 3600.0,
      update->observation_sigma * 180.0 / M_PI * 3600.0, update->bias * 180.0 / M_PI * 3600.0,
      update->sigma * 180.0 / M_PI * 3600.0);
  }

  void on_pressure(const sensor_msgs::msg::FluidPressure & msg)
  {
    if (const auto depth = pressure_converter_.pressure_to_depth(msg.fluid_pressure)) {
      estimator_->add_depth(to_seconds(msg.header.stamp), *depth);
    }
  }

  void on_dvl(const geometry_msgs::msg::TwistStamped & msg)
  {
    const auto & v = msg.twist.linear;
    estimator_->add_dvl(
      to_seconds(msg.header.stamp) + dvl_time_offset_s_, gtsam::Vector3(v.x, v.y, v.z));
  }

  void on_dvl_cov(const geometry_msgs::msg::TwistWithCovarianceStamped & msg)
  {
    const auto & v = msg.twist.twist.linear;
    double sigma = -1.0;
    if (use_dvl_covariance_) {
      const auto & c = msg.twist.covariance;
      const double var = std::max({c[0], c[7], c[14]});
      if (std::isfinite(var) && var > 0.0) {
        sigma = std::sqrt(var);
      }
    }
    estimator_->add_dvl(
      to_seconds(msg.header.stamp) + dvl_time_offset_s_, gtsam::Vector3(v.x, v.y, v.z), sigma);
  }

  void on_heading(const sensor_msgs::msg::Imu & msg)
  {
    const auto & q = msg.orientation;
    const double qn = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!std::isfinite(qn) || qn < 0.5) {
      return;
    }
    const gtsam::Rot3 r = gtsam::Rot3::Quaternion(q.w / qn, q.x / qn, q.y / qn, q.z / qn);
    const double heading =
      heading_of_axis(r, gtsam::Vector3::UnitX()) + heading_offset_rad_;
    double sigma = -1.0;
    if (use_heading_covariance_) {
      const double var = msg.orientation_covariance[8];
      if (std::isfinite(var) && var > 0.0) {
        sigma = std::sqrt(var);
      }
    }
    estimator_->add_heading(to_seconds(msg.header.stamp), heading, sigma);
  }

  void on_position(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose.position;
    gtsam::Matrix3 cov = gtsam::Matrix3::Zero();
    if (use_position_covariance_) {
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          cov(r, c) = msg.pose.covariance[static_cast<size_t>(r * 6 + c)];
        }
      }
    }
    estimator_->add_position(to_seconds(msg.header.stamp), gtsam::Point3(p.x, p.y, p.z), cov);
  }

  void publish_odometry(const EstimatorOutput & state)
  {
    if (loop_bias_enabled_) {
      last_output_ = state;
      if (state.stamp_s >= last_history_s_ + 0.05) {
        last_history_s_ = state.stamp_s;
        loop_bias_->add_state(state.stamp_s, state.pose.rotation(), state.bias.gyroscope());
      }
    }
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = to_stamp(state.stamp_s);
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;
    const gtsam::Point3 & t = state.pose.translation();
    const gtsam::Quaternion q = state.pose.rotation().toQuaternion();
    odom.pose.pose.position.x = t.x();
    odom.pose.pose.position.y = t.y();
    odom.pose.pose.position.z = t.z();
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    // REP-105: twist in the child (body) frame.
    odom.twist.twist.linear = to_msg(state.pose.rotation().unrotate(state.velocity_world));
    odom.twist.twist.angular = to_msg(last_gyro_ - state.bias.gyroscope());
    odom_pub_->publish(odom);
  }

  void publish_status(const EstimatorOutput & state)
  {
    const auto & c = estimator_->counters();
    aqua_msgs::msg::GraphEstimatorStatus status;
    status.header.stamp = to_stamp(state.stamp_s);
    status.header.frame_id = map_frame_;
    status.state_index = state.state_index;
    status.imu_samples = c.imu_samples;
    status.dvl_received = c.dvl_received;
    status.dvl_applied = c.dvl_applied;
    status.dvl_rejected = c.dvl_rejected;
    status.depth_received = c.depth_received;
    status.depth_applied = c.depth_applied;
    status.position_received = c.position_received;
    status.position_applied = c.position_applied;
    status.stereo_frames = c.stereo_frames;
    // Monocular projection factors count as visual observations too.
    status.stereo_observations = c.stereo_observations + c.mono_observations;
    status.landmarks_created = c.landmarks_created;
    status.measurements_too_old = c.measurements_too_old;
    status.smoother_updates = c.smoother_updates;
    status.smoother_resets = c.smoother_resets;
    if (c.smoother_resets != reported_resets_) {
      reported_resets_ = c.smoother_resets;
      RCLCPP_WARN(
        get_logger(), "smoother re-anchored (#%lu) at t=%.3f: %s",
        static_cast<unsigned long>(c.smoother_resets), state.stamp_s,
        estimator_->last_reset_reason().c_str());
    }
    status.heading_received = c.heading_received;
    status.heading_applied = c.heading_applied;
    status.velocity_world = to_msg(state.velocity_world);
    status.accel_bias = to_msg(state.bias.accelerometer());
    status.gyro_bias = to_msg(state.bias.gyroscope());
    status_pub_->publish(status);
  }

  std::unique_ptr<TightlyCoupledEstimator> estimator_;
  aqua_imu_loc::PressureDepthConverter pressure_converter_;
  std::string map_frame_;
  std::string base_frame_;
  bool publish_imu_rate_{true};
  bool loop_bias_enabled_{false};
  std::unique_ptr<LoopBiasEstimator> loop_bias_;
  std::optional<EstimatorOutput> last_output_;
  double last_history_s_{-1.0e18};
  rclcpp::Subscription<aqua_msgs::msg::PoseGraphKeyframe>::SharedPtr keyframe_sub_;
  rclcpp::Subscription<aqua_msgs::msg::PoseGraphLoopConstraint>::SharedPtr loop_sub_;
  bool use_position_covariance_{true};
  bool use_dvl_covariance_{false};
  bool use_heading_covariance_{false};
  double dvl_time_offset_s_{0.0};
  uint64_t reported_resets_{0};
  double heading_offset_rad_{0.0};
  gtsam::Vector3 last_gyro_{gtsam::Vector3::Zero()};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_cov_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr heading_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_sub_;
  rclcpp::Subscription<aqua_msgs::msg::StereoFeatures>::SharedPtr stereo_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<aqua_msgs::msg::GraphEstimatorStatus>::SharedPtr status_pub_;
};

}  // namespace aqua_graph

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(aqua_graph::AquaGraphNode)
