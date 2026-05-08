#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "aqua_imu_loc/additive_ukf.hpp"
#include "aqua_imu_loc/imu_preprocessor.hpp"
#include "aqua_imu_loc/pressure_depth_converter.hpp"
#include "aqua_imu_loc/static_bias_initializer.hpp"
#include "aqua_msgs/msg/estimator_status.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace aqua_imu_loc
{
class ImuLocNode : public rclcpp::Node
{
public:
  explicit ImuLocNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("aqua_imu_loc", options)
  {
    load_parameters();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ImuLocNode::on_imu, this, std::placeholders::_1));
    if (!pressure_topic_.empty()) {
      pressure_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
        pressure_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ImuLocNode::on_pressure, this, std::placeholders::_1));
    }
    if (!current_velocity_topic_.empty()) {
      current_velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        current_velocity_topic_, rclcpp::SystemDefaultsQoS(),
        std::bind(&ImuLocNode::on_current_velocity, this, std::placeholders::_1));
    }
    if (!sonar_odometry_topic_.empty()) {
      sonar_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        sonar_odometry_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ImuLocNode::on_sonar_odometry, this, std::placeholders::_1));
    }
    if (!dvl_velocity_topic_.empty()) {
      dvl_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        dvl_velocity_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ImuLocNode::on_dvl_velocity, this, std::placeholders::_1));
    }
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, rclcpp::SystemDefaultsQoS());
    status_pub_ =
      create_publisher<aqua_msgs::msg::EstimatorStatus>(status_topic_, rclcpp::SystemDefaultsQoS());
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      reset_service_,
      std::bind(&ImuLocNode::on_reset, this, std::placeholders::_1, std::placeholders::_2));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(
      get_logger(),
      "aqua_imu_loc started: imu=%s pressure=%s current_velocity=%s sonar=%s dvl=%s odom=%s reset=%s frames=%s->%s->%s",
      imu_topic_.c_str(),
      pressure_topic_.empty() ? "<disabled>" : pressure_topic_.c_str(),
      current_velocity_topic_.empty() ? "<disabled>" : current_velocity_topic_.c_str(),
      sonar_odometry_topic_.empty() ? "<disabled>" : sonar_odometry_topic_.c_str(),
      dvl_velocity_topic_.empty() ? "<disabled>" : dvl_velocity_topic_.c_str(),
      odometry_topic_.c_str(),
      reset_service_.c_str(), map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  void load_parameters()
  {
    imu_topic_ = declare_parameter<std::string>("topics.imu", "/imu/data");
    pressure_topic_ = declare_parameter<std::string>("topics.pressure", "/pressure");
    current_velocity_topic_ = declare_parameter<std::string>("topics.current_velocity", "");
    sonar_odometry_topic_ = declare_parameter<std::string>("topics.sonar_odometry", "");
    dvl_velocity_topic_ = declare_parameter<std::string>("topics.dvl_velocity", "");
    odometry_topic_ = declare_parameter<std::string>("topics.odometry", "/aqua_imu_loc/odometry");
    status_topic_ = declare_parameter<std::string>("topics.status", "/aqua_imu_loc/status");
    reset_service_ = declare_parameter<std::string>("services.reset", "/aqua_imu_loc/reset");

    map_frame_ = declare_parameter<std::string>("frames.map", "map");
    odom_frame_ = declare_parameter<std::string>("frames.odom", "odom");
    base_frame_ = declare_parameter<std::string>("frames.base_link", "base_link");

    publish_tf_ = declare_parameter<bool>("publish.tf", true);
    publish_odometry_ = declare_parameter<bool>("publish.odometry", true);

    const double alpha = declare_parameter<double>("ukf.alpha", 0.2);
    const double beta = declare_parameter<double>("ukf.beta", 2.0);
    const double kappa = declare_parameter<double>("ukf.kappa", 0.0);
    ImuPreprocessorConfig imu_preprocessor_config;
    imu_preprocessor_config.max_prediction_dt =
      declare_parameter<double>("ukf.max_prediction_dt", 0.05);
    imu_preprocessor_config.min_prediction_dt =
      declare_parameter<double>("ukf.min_prediction_dt", 0.0005);
    imu_preprocessor_.configure(imu_preprocessor_config);

    filter_.configure(alpha, beta, kappa);
    initial_covariance_diagonal_ = vector_parameter(
      "ukf.initial_covariance_diagonal",
      {0.25, 0.25, 0.25, 0.10, 0.10, 0.10, 0.05, 0.05, 0.10, 0.05, 0.05, 0.05,
        0.01, 0.01, 0.01});
    filter_.set_initial_covariance(initial_covariance_diagonal_);
    filter_.set_process_noise(vector_parameter(
      "ukf.process_noise_diagonal",
      {0.02, 0.02, 0.02, 0.20, 0.20, 0.20, 0.01, 0.01, 0.02, 0.0005, 0.0005,
        0.0005, 0.0001, 0.0001, 0.0001}));

    depth_variance_ = declare_parameter<double>("pressure.depth_variance", 0.04);

    dynamics_.gravity_mps2 = declare_parameter<double>("dynamics.gravity_mps2", 9.80665);
    dynamics_.enable_linear_drag = declare_parameter<bool>("dynamics.enable_linear_drag", true);
    dynamics_.linear_drag_coeff = declare_parameter<double>("dynamics.linear_drag_coeff", 0.05);
    dynamics_.enable_buoyancy = declare_parameter<bool>("dynamics.enable_buoyancy", false);
    dynamics_.buoyancy_accel_z_mps2 =
      declare_parameter<double>("dynamics.buoyancy_accel_z_mps2", 0.0);

    const auto current_velocity =
      vector_parameter("dynamics.current_velocity_xyz", {0.0, 0.0, 0.0}, 3);
    dynamics_.current_velocity =
      Eigen::Vector3d(current_velocity[0], current_velocity[1], current_velocity[2]);

    pressure_config_.use_first_pressure_as_reference =
      declare_parameter<bool>("pressure.use_first_pressure_as_reference", true);
    pressure_config_.reference_pressure_pa =
      declare_parameter<double>("pressure.reference_pressure_pa", 101325.0);
    pressure_config_.water_density_kg_m3 =
      declare_parameter<double>("pressure.water_density_kg_m3", 1025.0);
    pressure_config_.gravity_mps2 = dynamics_.gravity_mps2;
    pressure_config_.depth_offset_m = declare_parameter<double>("pressure.depth_offset_m", 0.0);
    pressure_converter_.configure(pressure_config_);

    StaticBiasInitializerConfig bias_config;
    bias_config.window_seconds =
      declare_parameter<double>("init.static_bias.window_seconds", 3.0);
    bias_config.gyro_motion_threshold_radps =
      declare_parameter<double>("init.static_bias.gyro_motion_threshold_radps", 0.10);
    bias_config.accel_motion_threshold_mps2 =
      declare_parameter<double>("init.static_bias.accel_motion_threshold_mps2", 0.50);
    bias_config.minimum_samples = static_cast<size_t>(std::max<long>(
      1, declare_parameter<long>("init.static_bias.minimum_samples", 50)));
    bias_config.gravity_mps2 = dynamics_.gravity_mps2;
    static_bias_initializer_.configure(bias_config);
    static_bias_initializer_.enable(
      declare_parameter<bool>("init.static_bias.enable", true));

    // Static IMU mounting rotation. Some bags carry the IMU mounted such that
    // the body-Z axis is not the up axis (e.g. AQUALOC reads gravity on the
    // body-Y axis). When set, accel and angular_velocity are pre-rotated by
    // R_mount before being handed to the UKF, so the UKF's REP-145 assumption
    // (body-Z up, accel ~ +g when stationary level) holds.
    {
      const auto rpy = vector_parameter(
        "imu.mount.rotation_rpy_rad", {0.0, 0.0, 0.0}, 3);
      const Eigen::AngleAxisd roll(rpy[0], Eigen::Vector3d::UnitX());
      const Eigen::AngleAxisd pitch(rpy[1], Eigen::Vector3d::UnitY());
      const Eigen::AngleAxisd yaw(rpy[2], Eigen::Vector3d::UnitZ());
      imu_mount_rotation_ = (yaw * pitch * roll).toRotationMatrix();
    }

    use_orientation_yaw_ = declare_parameter<bool>("imu.use_orientation_yaw", false);
    orientation_yaw_variance_rad2_ =
      declare_parameter<double>("imu.orientation_yaw_variance_rad2", 0.05);
    orientation_yaw_subsample_ = static_cast<size_t>(std::max<long>(
      1, declare_parameter<long>("imu.orientation_yaw_subsample", 5)));

    use_ahrs_gyro_bias_z_ = declare_parameter<bool>("imu.use_ahrs_gyro_bias_z", false);
    ahrs_gyro_bias_z_variance_rad2_s2_ =
      declare_parameter<double>("imu.ahrs_gyro_bias_z_variance_rad2_s2", 1.0e-3);
    ahrs_gyro_bias_z_subsample_ = static_cast<size_t>(std::max<long>(
      1, declare_parameter<long>("imu.ahrs_gyro_bias_z_subsample", 10)));
    ahrs_gyro_bias_z_max_dt_s_ =
      declare_parameter<double>("imu.ahrs_gyro_bias_z_max_dt_s", 0.5);

    use_ahrs_gyro_bias_xyz_ = declare_parameter<bool>("imu.use_ahrs_gyro_bias_xyz", false);
    {
      const auto vx = declare_parameter<double>(
        "imu.ahrs_gyro_bias_xyz_variance_rad2_s2.x", 1.0e-2);
      const auto vy = declare_parameter<double>(
        "imu.ahrs_gyro_bias_xyz_variance_rad2_s2.y", 1.0e-2);
      const auto vz = declare_parameter<double>(
        "imu.ahrs_gyro_bias_xyz_variance_rad2_s2.z", 1.0e-2);
      ahrs_gyro_bias_xyz_variance_ = Eigen::Vector3d(vx, vy, vz);
    }
    ahrs_gyro_bias_xyz_subsample_ = static_cast<size_t>(std::max<long>(
      1, declare_parameter<long>("imu.ahrs_gyro_bias_xyz_subsample", 20)));
    ahrs_gyro_bias_xyz_max_dt_s_ =
      declare_parameter<double>("imu.ahrs_gyro_bias_xyz_max_dt_s", 0.5);

    // Surface-vessel pseudo-depth: when there is no pressure sensor (e.g. a
    // surface boat with a multibeam sounder mounted from the hull) we can pin z
    // at zero so accel-derived vertical drift does not blow up. This is
    // equivalent to a synthetic depth-0 measurement at every Nth IMU step.
    surface_assumption_enable_ =
      declare_parameter<bool>("imu.surface_assumption.enable", false);
    surface_assumption_variance_ =
      declare_parameter<double>("imu.surface_assumption.depth_variance", 0.04);
    surface_assumption_subsample_ = static_cast<size_t>(std::max<long>(
      1, declare_parameter<long>("imu.surface_assumption.subsample", 10)));

    // Sonar tightly-coupled fusion knobs. Position covariance from
    // /aqua_sonar_loc/odometry is honoured directly when it satisfies the
    // floor; otherwise the floor is used. The variance floor protects against
    // an over-confident sonar publishing a near-zero covariance that would
    // collapse the UKF position uncertainty.
    sonar_position_variance_floor_ =
      declare_parameter<double>("imu.sonar.position_variance_floor", 0.04);
    sonar_max_age_s_ =
      declare_parameter<double>("imu.sonar.max_age_s", 1.0);

    // DVL velocity observation knobs. mount.rotation_rpy_rad pre-rotates the
    // raw DVL sample into base_link before the body-frame measurement update.
    {
      const auto rpy = vector_parameter(
        "imu.dvl.mount.rotation_rpy_rad", {0.0, 0.0, 0.0}, 3);
      const Eigen::AngleAxisd roll(rpy[0], Eigen::Vector3d::UnitX());
      const Eigen::AngleAxisd pitch(rpy[1], Eigen::Vector3d::UnitY());
      const Eigen::AngleAxisd yaw(rpy[2], Eigen::Vector3d::UnitZ());
      dvl_mount_rotation_ = (yaw * pitch * roll).toRotationMatrix();
    }
    dvl_velocity_variance_floor_ =
      declare_parameter<double>("imu.dvl.velocity_variance_floor", 0.01);
    dvl_max_age_s_ =
      declare_parameter<double>("imu.dvl.max_age_s", 0.5);
  }

  void on_dvl_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    const Eigen::Vector3d v_raw(
      msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    if (!v_raw.allFinite()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected non-finite DVL velocity.");
      return;
    }
    if (last_imu_stamp_valid_) {
      const rclcpp::Time stamp(msg->header.stamp);
      const double age = (last_imu_stamp_ - stamp).seconds();
      if (age > dvl_max_age_s_) {
        RCLCPP_DEBUG(
          get_logger(), "Skipping DVL velocity: %.3fs older than latest IMU sample", age);
        return;
      }
    }
    const Eigen::Vector3d v_body = dvl_mount_rotation_ * v_raw;
    // Diagonal isotropic covariance — TwistStamped does not carry one, so we
    // expose the floor as the published variance. Future work: use a
    // Twist*WithCovariance*Stamped variant when the upstream driver provides
    // per-axis variance.
    const Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * dvl_velocity_variance_floor_;
    filter_.update_body_velocity(v_body, cov);
    ++update_count_;
  }

  void on_sonar_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const Eigen::Vector3d position(
      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    if (!position.allFinite()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected non-finite sonar position.");
      return;
    }
    // Reject stale samples relative to the latest IMU stamp; this keeps
    // sonar-position observations from leaking across long bag pauses.
    if (last_imu_stamp_valid_) {
      const rclcpp::Time stamp(msg->header.stamp);
      const double age = (last_imu_stamp_ - stamp).seconds();
      if (age > sonar_max_age_s_) {
        RCLCPP_DEBUG(
          get_logger(), "Skipping sonar pose: %.3fs older than latest IMU sample", age);
        return;
      }
    }
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    // Read the 3x3 position block from the 6x6 row-major pose covariance.
    cov(0, 0) = msg->pose.covariance[0];
    cov(0, 1) = msg->pose.covariance[1];
    cov(0, 2) = msg->pose.covariance[2];
    cov(1, 0) = msg->pose.covariance[6];
    cov(1, 1) = msg->pose.covariance[7];
    cov(1, 2) = msg->pose.covariance[8];
    cov(2, 0) = msg->pose.covariance[12];
    cov(2, 1) = msg->pose.covariance[13];
    cov(2, 2) = msg->pose.covariance[14];
    // Ensure the diagonal stays at or above the configured floor so an
    // overconfident sonar cannot collapse the UKF position uncertainty.
    for (int i = 0; i < 3; ++i) {
      cov(i, i) = std::max(cov(i, i), sonar_position_variance_floor_);
    }
    filter_.update_position(position, cov);
    ++update_count_;
  }

  std::vector<double> vector_parameter(
    const std::string & name, const std::vector<double> & default_value,
    size_t expected_size = kStateDim)
  {
    auto value = declare_parameter<std::vector<double>>(name, default_value);
    if (value.size() != expected_size) {
      RCLCPP_WARN(
        get_logger(), "Parameter '%s' must have %zu elements; using defaults.",
        name.c_str(), expected_size);
      value = default_value;
    }
    return value;
  }

  void on_reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    reset_estimator_state();
    response->success = true;
    response->message = "aqua_imu_loc estimator state reset";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    status_pub_->publish(make_status_msg(now()));
  }

  void reset_estimator_state()
  {
    filter_.set_state(StateVector::Zero());
    filter_.set_initial_covariance(initial_covariance_diagonal_);
    pressure_converter_.configure(pressure_config_);
    static_bias_initializer_.reset();
    orientation_yaw_offset_ = 0.0;
    orientation_yaw_offset_set_ = false;
    orientation_sample_count_ = 0;
    ahrs_yaw_rate_initialized_ = false;
    ahrs_yaw_rate_last_ = 0.0;
    ahrs_yaw_rate_last_stamp_s_ = 0.0;
    ahrs_gyro_bias_z_sample_count_ = 0;
    ahrs_gyro_bias_z_observation_active_ = false;
    ahrs_gyro_bias_z_last_observed_ = std::numeric_limits<double>::quiet_NaN();
    ahrs_xyz_initialized_ = false;
    ahrs_xyz_prev_q_ = tf2::Quaternion::getIdentity();
    ahrs_xyz_prev_stamp_s_ = 0.0;
    ahrs_gyro_bias_xyz_sample_count_ = 0;
    ahrs_gyro_bias_xyz_observation_active_ = false;
    ahrs_gyro_bias_xyz_last_observed_ =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    last_imu_stamp_valid_ = false;
    last_prediction_dt_ = 0.0;
    update_count_ = 0;
    surface_assumption_sample_count_ = 0;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const rclcpp::Time stamp = msg->header.stamp;
    if (!last_imu_stamp_valid_) {
      last_imu_stamp_ = stamp;
      last_imu_stamp_valid_ = true;
      publish(stamp);
      return;
    }

    double dt = (stamp - last_imu_stamp_).seconds();
    last_imu_stamp_ = stamp;

    const auto interval = imu_preprocessor_.prediction_interval(dt);
    if (!interval.has_value()) {
      return;
    }
    if (interval->clamped) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Large IMU dt %.4f s clamped to %.4f s", dt, interval->dt);
    }

    ImuSample sample;
    sample.linear_acceleration = imu_mount_rotation_ * Eigen::Vector3d(
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    sample.angular_velocity = imu_mount_rotation_ * Eigen::Vector3d(
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    if (!imu_preprocessor_.sample_is_finite(sample)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected non-finite IMU sample.");
      return;
    }

    maybe_initialize_static_bias(stamp, sample);

    filter_.predict(interval->dt, sample, dynamics_);
    last_prediction_dt_ = interval->dt;
    ++update_count_;

    maybe_apply_orientation_yaw(*msg);
    maybe_apply_ahrs_gyro_bias_z(stamp, *msg);
    maybe_apply_ahrs_gyro_bias_xyz(stamp, *msg, sample);
    maybe_apply_surface_assumption();

    publish(stamp);
  }

  void maybe_apply_surface_assumption()
  {
    if (!surface_assumption_enable_) {
      return;
    }
    ++surface_assumption_sample_count_;
    if (surface_assumption_sample_count_ < surface_assumption_subsample_) {
      return;
    }
    surface_assumption_sample_count_ = 0;
    filter_.update_depth(0.0, surface_assumption_variance_);
  }

  void maybe_apply_ahrs_gyro_bias_xyz(
    const rclcpp::Time & stamp,
    const sensor_msgs::msg::Imu & msg,
    const ImuSample & sample)
  {
    if (!use_ahrs_gyro_bias_xyz_) {
      return;
    }
    if (msg.orientation_covariance[0] < 0.0) {
      return;
    }
    tf2::Quaternion q_curr(
      msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    if (q_curr.length2() <= 1.0e-9) {
      return;
    }
    q_curr.normalize();

    const double t_now = stamp.seconds();
    if (!ahrs_xyz_initialized_) {
      ahrs_xyz_prev_q_ = q_curr;
      ahrs_xyz_prev_stamp_s_ = t_now;
      ahrs_xyz_initialized_ = true;
      ++ahrs_gyro_bias_xyz_sample_count_;
      return;
    }

    const double dt = t_now - ahrs_xyz_prev_stamp_s_;
    if (dt <= 0.0 || dt > ahrs_gyro_bias_xyz_max_dt_s_) {
      ahrs_xyz_prev_q_ = q_curr;
      ahrs_xyz_prev_stamp_s_ = t_now;
      ++ahrs_gyro_bias_xyz_sample_count_;
      return;
    }

    // Body-frame relative rotation from previous sample to current sample.
    tf2::Quaternion delta = ahrs_xyz_prev_q_.inverse() * q_curr;
    delta.normalize();
    const double w = delta.w();
    const double sign = (w >= 0.0) ? 1.0 : -1.0;
    Eigen::Vector3d omega_body(
      2.0 * sign * delta.x() / dt,
      2.0 * sign * delta.y() / dt,
      2.0 * sign * delta.z() / dt);

    ahrs_xyz_prev_q_ = q_curr;
    ahrs_xyz_prev_stamp_s_ = t_now;
    ++ahrs_gyro_bias_xyz_sample_count_;
    if (ahrs_gyro_bias_xyz_sample_count_ % ahrs_gyro_bias_xyz_subsample_ != 0) {
      return;
    }

    const Eigen::Vector3d observed_bias = sample.angular_velocity - omega_body;
    if (!observed_bias.allFinite()) {
      return;
    }
    filter_.update_gyro_bias_xyz(observed_bias, ahrs_gyro_bias_xyz_variance_);
    ahrs_gyro_bias_xyz_last_observed_ = observed_bias;
    ahrs_gyro_bias_xyz_observation_active_ = true;
  }

  void maybe_apply_ahrs_gyro_bias_z(const rclcpp::Time & stamp, const sensor_msgs::msg::Imu & msg)
  {
    if (!use_ahrs_gyro_bias_z_) {
      return;
    }
    if (msg.orientation_covariance[0] < 0.0) {
      return;
    }
    tf2::Quaternion q(
      msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    if (q.length2() <= 1.0e-9) {
      return;
    }
    q.normalize();
    double roll;
    double pitch;
    double yaw_absolute;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_absolute);
    if (!std::isfinite(yaw_absolute)) {
      return;
    }

    const double t_now = stamp.seconds();
    if (!ahrs_yaw_rate_initialized_) {
      ahrs_yaw_rate_last_ = yaw_absolute;
      ahrs_yaw_rate_last_stamp_s_ = t_now;
      ahrs_yaw_rate_initialized_ = true;
      ++ahrs_gyro_bias_z_sample_count_;
      return;
    }

    const double dt = t_now - ahrs_yaw_rate_last_stamp_s_;
    const double yaw_step = normalize_angle(yaw_absolute - ahrs_yaw_rate_last_);
    ahrs_yaw_rate_last_ = yaw_absolute;
    ahrs_yaw_rate_last_stamp_s_ = t_now;
    ++ahrs_gyro_bias_z_sample_count_;
    if (dt <= 0.0 || dt > ahrs_gyro_bias_z_max_dt_s_) {
      return;
    }
    if (ahrs_gyro_bias_z_sample_count_ % ahrs_gyro_bias_z_subsample_ != 0) {
      return;
    }

    const double ahrs_yaw_rate = yaw_step / dt;
    const double observed_bias = msg.angular_velocity.z - ahrs_yaw_rate;
    if (!std::isfinite(observed_bias)) {
      return;
    }
    filter_.update_gyro_bias_z(observed_bias, ahrs_gyro_bias_z_variance_rad2_s2_);
    ahrs_gyro_bias_z_last_observed_ = observed_bias;
    ahrs_gyro_bias_z_observation_active_ = true;
  }

  void maybe_apply_orientation_yaw(const sensor_msgs::msg::Imu & msg)
  {
    if (!use_orientation_yaw_) {
      return;
    }
    if (orientation_yaw_variance_rad2_ <= 0.0) {
      return;
    }
    if (msg.orientation_covariance[0] < 0.0) {
      // Convention: orientation_covariance[0] == -1.0 means orientation is unavailable.
      return;
    }
    ++orientation_sample_count_;
    if (orientation_sample_count_ % orientation_yaw_subsample_ != 0) {
      return;
    }

    tf2::Quaternion q(
      msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    if (q.length2() <= 1.0e-9) {
      return;
    }
    q.normalize();
    double roll;
    double pitch;
    double yaw_absolute;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_absolute);
    if (!std::isfinite(yaw_absolute)) {
      return;
    }
    if (!orientation_yaw_offset_set_) {
      orientation_yaw_offset_ = yaw_absolute;
      orientation_yaw_offset_set_ = true;
      return;
    }
    const double yaw_relative = normalize_angle(yaw_absolute - orientation_yaw_offset_);
    filter_.update_yaw(yaw_relative, orientation_yaw_variance_rad2_);
  }

  void maybe_initialize_static_bias(const rclcpp::Time & stamp, const ImuSample & sample)
  {
    const auto previous_status = static_bias_initializer_.status();
    if (previous_status != StaticBiasInitializerStatus::kAccumulating) {
      return;
    }
    const double stamp_seconds = stamp.seconds();
    const auto status = static_bias_initializer_.add_sample(stamp_seconds, sample);
    if (status == StaticBiasInitializerStatus::kReady &&
      previous_status == StaticBiasInitializerStatus::kAccumulating)
    {
      const Eigen::Vector3d gyro_bias = static_bias_initializer_.gyro_bias();
      StateVector seeded = filter_.state();
      seeded.segment<3>(12) = gyro_bias;
      filter_.set_state(seeded);
      RCLCPP_INFO(
        get_logger(),
        "Static bias initialized after %zu samples: gyro_bias=[%.5f, %.5f, %.5f] rad/s",
        static_bias_initializer_.sample_count(),
        gyro_bias.x(), gyro_bias.y(), gyro_bias.z());
    } else if (status == StaticBiasInitializerStatus::kAborted &&
      previous_status == StaticBiasInitializerStatus::kAccumulating)
    {
      RCLCPP_WARN(
        get_logger(),
        "Static bias initialization aborted: motion detected during warmup; gyro_bias remains zero.");
    }
  }

  void on_pressure(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
  {
    const bool reference_was_initialized = pressure_converter_.reference_initialized();
    const auto depth_m = pressure_converter_.pressure_to_depth(msg->fluid_pressure);
    if (!depth_m.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected non-finite pressure sample.");
      return;
    }

    if (!reference_was_initialized && pressure_converter_.reference_initialized()) {
      RCLCPP_INFO(
        get_logger(), "Pressure zero-depth reference initialized at %.2f Pa",
        pressure_converter_.reference_pressure_pa());
    }

    filter_.update_depth(depth_m.value(), depth_variance_);
    ++update_count_;
    publish(msg->header.stamp);
  }

  void on_current_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    const Eigen::Vector3d current_velocity(
      msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    if (!current_velocity.allFinite()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected non-finite current velocity sample.");
      return;
    }

    if (!msg->header.frame_id.empty() && msg->header.frame_id != odom_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Current velocity frame '%s' differs from odom frame '%s'; using values without TF transform.",
        msg->header.frame_id.c_str(), odom_frame_.c_str());
    }

    dynamics_.current_velocity = current_velocity;
  }

  void publish(const rclcpp::Time & stamp)
  {
    if (publish_odometry_) {
      odom_pub_->publish(make_odometry_msg(stamp));
    }
    if (publish_tf_) {
      publish_transforms(stamp);
    }
    status_pub_->publish(make_status_msg(stamp));
  }

  nav_msgs::msg::Odometry make_odometry_msg(const rclcpp::Time & stamp) const
  {
    const auto & state = filter_.state();
    const auto & covariance = filter_.covariance();
    const Eigen::Vector3d rpy = state.segment<3>(6);

    tf2::Quaternion quaternion;
    quaternion.setRPY(rpy.x(), rpy.y(), rpy.z());
    quaternion.normalize();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = state.x();
    odom.pose.pose.position.y = state.y();
    odom.pose.pose.position.z = state.z();
    odom.pose.pose.orientation.x = quaternion.x();
    odom.pose.pose.orientation.y = quaternion.y();
    odom.pose.pose.orientation.z = quaternion.z();
    odom.pose.pose.orientation.w = quaternion.w();
    odom.twist.twist.linear.x = state(3);
    odom.twist.twist.linear.y = state(4);
    odom.twist.twist.linear.z = state(5);

    for (auto & value : odom.pose.covariance) {
      value = 0.0;
    }
    for (auto & value : odom.twist.covariance) {
      value = 0.0;
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        odom.pose.covariance[static_cast<size_t>(i * 6 + j)] = covariance(i, j);
        odom.pose.covariance[static_cast<size_t>((i + 3) * 6 + (j + 3))] =
          covariance(i + 6, j + 6);
        odom.twist.covariance[static_cast<size_t>(i * 6 + j)] = covariance(i + 3, j + 3);
      }
    }

    return odom;
  }

  void publish_transforms(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = stamp;
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.child_frame_id = odom_frame_;
    map_to_odom.transform.rotation.w = 1.0;

    const auto odom_msg = make_odometry_msg(stamp);
    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header = odom_msg.header;
    odom_to_base.child_frame_id = base_frame_;
    odom_to_base.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_to_base.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_to_base.transform.translation.z = odom_msg.pose.pose.position.z;
    odom_to_base.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(map_to_odom);
    tf_broadcaster_->sendTransform(odom_to_base);
  }

  aqua_msgs::msg::EstimatorStatus make_status_msg(const rclcpp::Time & stamp) const
  {
    const auto & state = filter_.state();
    const auto & covariance = filter_.covariance();

    aqua_msgs::msg::EstimatorStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = odom_frame_;
    status.estimator_name = "aqua_imu_loc";
    status.backend = "additive_ukf";
    status.initialized = last_imu_stamp_valid_;
    status.update_count = update_count_;
    status.last_prediction_dt = last_prediction_dt_;
    status.position_covariance_trace = covariance(0, 0) + covariance(1, 1) + covariance(2, 2);
    status.orientation_covariance_trace = covariance(6, 6) + covariance(7, 7) + covariance(8, 8);
    status.status = status.initialized ? "running" : "waiting_for_imu";
    status.accel_bias = {state(9), state(10), state(11)};
    status.gyro_bias = {state(12), state(13), state(14)};
    status.ahrs_gyro_bias_z_enabled = use_ahrs_gyro_bias_z_;
    status.ahrs_gyro_bias_z_active = ahrs_gyro_bias_z_observation_active_;
    status.ahrs_gyro_bias_z_last_observed = ahrs_gyro_bias_z_last_observed_;
    return status;
  }

  AdditiveUkf filter_;
  DynamicsParams dynamics_;
  ImuPreprocessor imu_preprocessor_;
  PressureDepthConverter pressure_converter_;
  PressureDepthConfig pressure_config_;
  StaticBiasInitializer static_bias_initializer_;
  std::vector<double> initial_covariance_diagonal_;

  std::string imu_topic_;
  std::string pressure_topic_;
  std::string current_velocity_topic_;
  std::string odometry_topic_;
  std::string status_topic_;
  std::string reset_service_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  bool publish_tf_{true};
  bool publish_odometry_{true};
  bool last_imu_stamp_valid_{false};
  bool use_orientation_yaw_{false};
  bool use_ahrs_gyro_bias_z_{false};
  bool use_ahrs_gyro_bias_xyz_{false};
  bool ahrs_yaw_rate_initialized_{false};
  bool ahrs_gyro_bias_z_observation_active_{false};
  bool ahrs_xyz_initialized_{false};
  bool ahrs_gyro_bias_xyz_observation_active_{false};

  double depth_variance_{0.04};
  double last_prediction_dt_{0.0};
  double orientation_yaw_variance_rad2_{0.05};
  double orientation_yaw_offset_{0.0};
  double ahrs_gyro_bias_z_variance_rad2_s2_{1.0e-3};
  double ahrs_gyro_bias_z_max_dt_s_{0.5};
  double ahrs_gyro_bias_z_last_observed_{std::numeric_limits<double>::quiet_NaN()};
  double ahrs_yaw_rate_last_{0.0};
  double ahrs_yaw_rate_last_stamp_s_{0.0};
  double ahrs_gyro_bias_xyz_max_dt_s_{0.5};
  double ahrs_xyz_prev_stamp_s_{0.0};
  Eigen::Vector3d ahrs_gyro_bias_xyz_variance_{
    Eigen::Vector3d::Constant(1.0e-2)};
  Eigen::Vector3d ahrs_gyro_bias_xyz_last_observed_{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  tf2::Quaternion ahrs_xyz_prev_q_{tf2::Quaternion::getIdentity()};
  bool orientation_yaw_offset_set_{false};
  uint64_t update_count_{0};
  std::size_t orientation_yaw_subsample_{5};
  std::size_t orientation_sample_count_{0};
  std::size_t ahrs_gyro_bias_z_subsample_{10};
  std::size_t ahrs_gyro_bias_z_sample_count_{0};
  std::size_t ahrs_gyro_bias_xyz_subsample_{20};
  std::size_t ahrs_gyro_bias_xyz_sample_count_{0};
  bool surface_assumption_enable_{false};
  double surface_assumption_variance_{0.04};
  std::size_t surface_assumption_subsample_{10};
  std::size_t surface_assumption_sample_count_{0};
  Eigen::Matrix3d imu_mount_rotation_{Eigen::Matrix3d::Identity()};
  std::string sonar_odometry_topic_;
  double sonar_position_variance_floor_{0.04};
  double sonar_max_age_s_{1.0};
  std::string dvl_velocity_topic_;
  Eigen::Matrix3d dvl_mount_rotation_{Eigen::Matrix3d::Identity()};
  double dvl_velocity_variance_floor_{0.01};
  double dvl_max_age_s_{0.5};

  rclcpp::Time last_imu_stamp_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr current_velocity_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sonar_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<aqua_msgs::msg::EstimatorStatus>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
}  // namespace aqua_imu_loc

#ifndef AQUA_IMU_LOC_DISABLE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aqua_imu_loc::ImuLocNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
