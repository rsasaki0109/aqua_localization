// SPDX-License-Identifier: Apache-2.0
// Tightly-coupled IMU + DVL + pressure + stereo-vision (+ optional absolute
// position) fixed-lag smoother built on GTSAM.
//
// States (pose, world velocity, IMU bias) are created on a regular time grid
// and, additionally, exactly at the stamps of "anchoring" measurements (stereo
// frames, and DVL samples when dvl.anchor_states is set), so those
// measurements constrain the state at their own time instead of the nearest
// grid point. IMU samples between consecutive states are folded into a
// CombinedImuFactor. Depth and position measurements attach to the nearest
// state, shifted to its stamp with the current velocity estimate. Stereo
// feature tracks become landmarks with GenericStereoFactor reprojection
// constraints once a track has been seen in two frames.
//
// To allow states at measurement stamps, IMU samples are buffered and states
// are only created up to (latest IMU stamp - processing_delay_s). Anchoring
// measurements must therefore arrive no later than processing_delay_s after
// the IMU stream passes their stamp; later ones fall back to nearest-state
// attachment. The estimator is a plain C++ library with no ROS dependency so
// it can be driven by a ROS node, the deterministic offline replay harness,
// or synthetic unit tests with identical results.
#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace gtsam
{
class IncrementalFixedLagSmoother;
}  // namespace gtsam

namespace aqua_graph
{

struct ImuNoiseConfig
{
  double gravity_mps2{9.80665};
  // Continuous-time noise densities.
  double accel_noise_density{0.02};   // m/s^2/sqrt(Hz)
  double gyro_noise_density{0.002};   // rad/s/sqrt(Hz)
  double accel_bias_random_walk{1.0e-3};  // m/s^3/sqrt(Hz)
  double gyro_bias_random_walk{1.0e-4};   // rad/s^2/sqrt(Hz)
  double integration_sigma{1.0e-4};
};

enum class AttitudeInitSource
{
  // Average the first init.static_window_s of accelerometer samples to get
  // roll/pitch (yaw = 0). Assumes the vehicle is roughly static at start.
  kAccelerometer,
  // Use the orientation reported with the first IMU sample (AHRS output).
  kImuOrientation,
};

struct InitConfig
{
  AttitudeInitSource attitude_source{AttitudeInitSource::kAccelerometer};
  double static_window_s{0.5};
  // Take the mean gyro over the static window as the initial gyro bias.
  // Disable for platforms that start under way (a turn rate would be
  // frozen in as bias); the bias then starts at zero.
  bool gyro_bias_from_static_window{true};
  // When true the initial yaw from the AHRS orientation is kept; otherwise the
  // initial yaw is zeroed so the trajectory starts heading along +x.
  bool keep_imu_yaw{false};
  double position_sigma_m{1.0e-3};
  double roll_pitch_sigma_rad{0.05};
  double yaw_sigma_rad{1.0e-3};
  double velocity_sigma_mps{0.2};
  double accel_bias_sigma{0.1};
  double gyro_bias_sigma{0.01};
};

struct DvlConfig
{
  bool enabled{true};
  // Create a state at every DVL stamp instead of using the nearest grid state.
  bool anchor_states{true};
  gtsam::Rot3 body_R_dvl{};
  gtsam::Point3 lever_arm_body{0.0, 0.0, 0.0};
  double velocity_sigma_mps{0.02};
  // Huber threshold in sigmas; <= 0 disables the robust loss. Keep it loose:
  // a tight threshold also suppresses genuine large innovations after drift.
  double huber_k{3.0};
  double max_speed_mps{5.0};
};

struct DepthConfig
{
  bool enabled{true};
  gtsam::Point3 lever_arm_body{0.0, 0.0, 0.0};
  double depth_sigma_m{0.05};
  // Pressure is the most reliable underwater measurement; a robust loss here
  // stops depth from pulling a drifted vertical channel back.
  double huber_k{0.0};
  // Surface vessels: constrain every state to the initial depth (sigma
  // hold_sigma_m) instead of (or in addition to) pressure.
  bool hold_initial_depth{false};
  double hold_sigma_m{0.2};
};

struct HeadingConfig
{
  // Off by default: with align_initial_yaw, initialization waits for the
  // first heading, which must not happen on platforms without one.
  bool enabled{false};
  // Body axis whose world heading is measured (e.g. the GNSS antenna
  // baseline). Must not be close to vertical.
  gtsam::Vector3 axis_body{1.0, 0.0, 0.0};
  double sigma_rad{0.01};
  double huber_k{3.0};
  // Start the graph with the measured heading instead of yaw = 0, so the
  // world frame is the heading sensor's (e.g. ENU) frame.
  bool align_initial_yaw{true};
};

struct PositionConfig
{
  bool enabled{true};
  // Lower bound applied to each axis' sigma, whatever the incoming covariance.
  double sigma_floor_m{0.02};
  double huber_k{3.0};
};

struct VisualConfig
{
  bool enabled{true};
  // Rectified stereo pinhole model (left camera) and baseline.
  double fx{655.0};
  double fy{655.0};
  double cx{306.0};
  double cy{256.0};
  double baseline_m{0.12};
  // Pose of the left camera in the body (IMU) frame.
  gtsam::Pose3 body_P_camera{};
  double pixel_sigma{1.0};
  double huber_k{1.345};
  double min_disparity_px{1.0};
  double max_depth_m{20.0};
  // Weak isotropic prior on each new landmark at its triangulated position.
  // GTSAM's stereo factor returns zero Jacobians for points behind the
  // camera, so a landmark whose observations all fail cheirality is
  // otherwise unconstrained and makes the system singular. <= 0 disables.
  double landmark_prior_sigma_m{10.0};
};

struct EstimatorConfig
{
  ImuNoiseConfig imu;
  InitConfig init;
  DvlConfig dvl;
  DepthConfig depth;
  PositionConfig position;
  HeadingConfig heading;
  VisualConfig visual;
  // Maximum spacing of the state grid between anchoring measurements.
  double state_period_s{0.1};
  // States are created up to (latest IMU stamp - processing_delay_s), so
  // anchoring measurements may lag the IMU stream by up to this much.
  double processing_delay_s{0.05};
  // Fixed-lag smoother window.
  double smoother_lag_s{5.0};
  // iSAM2 relinearization.
  double relinearize_threshold{0.01};
  int relinearize_skip{1};
};

struct EstimatorOutput
{
  double stamp_s{0.0};
  uint64_t state_index{0};
  gtsam::Pose3 pose{};
  gtsam::Vector3 velocity_world{gtsam::Vector3::Zero()};
  gtsam::imuBias::ConstantBias bias{};
};

struct EstimatorCounters
{
  uint64_t imu_samples{0};
  uint64_t states{0};
  uint64_t dvl_received{0};
  uint64_t dvl_applied{0};
  uint64_t dvl_rejected{0};
  uint64_t depth_received{0};
  uint64_t depth_applied{0};
  uint64_t position_received{0};
  uint64_t position_applied{0};
  uint64_t heading_received{0};
  uint64_t heading_applied{0};
  uint64_t stereo_frames{0};
  uint64_t stereo_observations{0};
  uint64_t stereo_rejected{0};
  uint64_t landmarks_created{0};
  // Measurements whose state already left the smoother window.
  uint64_t measurements_too_old{0};
  uint64_t smoother_updates{0};
  // Times the smoother hit an indeterminate system and was re-anchored at
  // the latest estimate (see run_update).
  uint64_t smoother_resets{0};
};

struct StereoObservation
{
  uint64_t id{0};
  double u_left{0.0};
  double u_right{0.0};
  double v{0.0};
};

class TightlyCoupledEstimator
{
public:
  explicit TightlyCoupledEstimator(const EstimatorConfig & config = EstimatorConfig());
  ~TightlyCoupledEstimator();

  TightlyCoupledEstimator(const TightlyCoupledEstimator &) = delete;
  TightlyCoupledEstimator & operator=(const TightlyCoupledEstimator &) = delete;

  // Skip automatic initialization and start from a known state at the first
  // IMU sample at or after stamp_s.
  void set_initial_state(
    double stamp_s, const gtsam::Pose3 & pose, const gtsam::Vector3 & velocity_world,
    const gtsam::imuBias::ConstantBias & bias);

  // Feed one IMU sample. Returns the estimates of any states created as a
  // result (states lag the IMU stream by processing_delay_s).
  std::vector<EstimatorOutput> add_imu(
    double stamp_s, const gtsam::Vector3 & accel_body, const gtsam::Vector3 & gyro_body,
    const std::optional<gtsam::Rot3> & orientation = std::nullopt);

  void add_dvl(double stamp_s, const gtsam::Vector3 & velocity_dvl);
  // Optional per-measurement sigma override (<= 0 keeps the configured one).
  void add_dvl(double stamp_s, const gtsam::Vector3 & velocity_dvl, double sigma_mps);
  void add_depth(double stamp_s, double depth_m);
  void add_position(
    double stamp_s, const gtsam::Point3 & position_world, const gtsam::Matrix3 & covariance);
  // World heading of config.heading.axis_body (sigma <= 0: configured one).
  void add_heading(double stamp_s, double heading_rad, double sigma_rad = -1.0);
  // Replace the gyro bias estimate (e.g. one derived from loop closures) and
  // re-anchor the smoother at the newest state with it.
  void set_gyro_bias(const gtsam::Vector3 & gyro_bias, const std::string & reason);
  // One rectified stereo frame of tracked features (ids persist across frames).
  void add_stereo_frame(double stamp_s, const std::vector<StereoObservation> & observations);

  bool initialized() const {return initialized_;}

  // Newest state propagated with the buffered IMU samples up to the latest one.
  std::optional<EstimatorOutput> imu_rate_estimate() const;
  // Latest smoothed estimate of the newest state.
  std::optional<EstimatorOutput> latest_state() const;

  const EstimatorCounters & counters() const {return counters_;}
  // what() of the exception behind the latest smoother reset (empty if none).
  const std::string & last_reset_reason() const {return last_reset_reason_;}
  const EstimatorConfig & config() const {return config_;}

private:
  struct ImuSample
  {
    double stamp_s{0.0};
    gtsam::Vector3 accel{gtsam::Vector3::Zero()};
    gtsam::Vector3 gyro{gtsam::Vector3::Zero()};
  };

  struct StateRecord
  {
    uint64_t index{0};
    double stamp_s{0.0};
  };

  struct PendingMeasurement
  {
    enum class Kind {kDvl, kDepth, kPosition, kHeading, kStereo};
    Kind kind{Kind::kDvl};
    double stamp_s{0.0};
    bool anchored{false};
    gtsam::Vector3 vector{gtsam::Vector3::Zero()};
    gtsam::Vector3 omega{gtsam::Vector3::Zero()};
    double scalar{0.0};
    gtsam::Matrix3 covariance{gtsam::Matrix3::Zero()};
    std::vector<StereoObservation> stereo;
  };

  struct FeatureTrack
  {
    std::optional<gtsam::Key> landmark;
    // First observation, held until the second one initializes the landmark.
    std::optional<uint64_t> first_state;
    gtsam::StereoPoint2 first_measurement;
    double last_seen_s{0.0};
  };

  struct UpdateBatch
  {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
  };

  void try_initialize(
    double stamp_s, const gtsam::Vector3 & accel_body, const gtsam::Vector3 & gyro_body,
    const std::optional<gtsam::Rot3> & orientation);
  void start_graph(double stamp_s, const gtsam::NavState & state,
    const gtsam::imuBias::ConstantBias & bias);
  void enqueue(PendingMeasurement measurement);
  std::vector<EstimatorOutput> advance(double target_s);
  void integrate_to(double stamp_s);
  EstimatorOutput create_state(double stamp_s);
  // Nearest in-window state to a stamp (exact match for anchored ones).
  std::optional<StateRecord> find_state(double stamp_s) const;
  void attach_measurements(UpdateBatch & batch, const StateRecord & newest);
  void attach_stereo(
    UpdateBatch & batch, const StateRecord & state, const std::vector<StereoObservation> & obs);
  gtsam::Pose3 pose_estimate(uint64_t index, const UpdateBatch & batch) const;
  gtsam::SharedNoiseModel robust(
    const gtsam::SharedNoiseModel & model, double huber_k) const;
  void add_state_priors(
    UpdateBatch & batch, uint64_t index, const gtsam::NavState & state,
    const gtsam::imuBias::ConstantBias & bias, double yaw_sigma_rad,
    double position_sigma_m) const;
  void make_smoother();
  void run_update(UpdateBatch & batch, const StateRecord & newest);
  void reset_smoother(StateRecord newest);

  EstimatorConfig config_;
  EstimatorCounters counters_;
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> preint_params_;
  std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> preint_;
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
  gtsam::Cal3_S2Stereo::shared_ptr stereo_calibration_;

  bool initialized_{false};
  std::optional<double> manual_init_stamp_;
  gtsam::Pose3 manual_init_pose_;
  gtsam::Vector3 manual_init_velocity_{gtsam::Vector3::Zero()};
  gtsam::imuBias::ConstantBias manual_init_bias_;

  // Static-window initialization accumulators.
  std::optional<double> init_window_start_s_;
  gtsam::Vector3 init_accel_sum_{gtsam::Vector3::Zero()};
  gtsam::Vector3 init_gyro_sum_{gtsam::Vector3::Zero()};
  uint64_t init_samples_{0};
  std::optional<double> latest_heading_rad_;
  std::string last_reset_reason_;

  // IMU stream: the sample held over [integrated_until_s_, next sample) plus
  // the not-yet-integrated samples after it.
  ImuSample held_imu_;
  std::deque<ImuSample> imu_buffer_;
  double integrated_until_s_{0.0};
  double latest_imu_s_{0.0};
  gtsam::Vector3 latest_gyro_{gtsam::Vector3::Zero()};

  std::deque<StateRecord> states_;  // in the smoother window, oldest first
  std::set<double> requested_stamps_;
  std::deque<PendingMeasurement> pending_;
  std::map<uint64_t, FeatureTrack> tracks_;
  uint64_t next_landmark_{0};

  gtsam::NavState current_state_;
  gtsam::imuBias::ConstantBias current_bias_;
};

// Parse "kAccelerometer"-style names used in YAML ("accelerometer",
// "imu_orientation"). Returns nullopt for unknown names.
std::optional<AttitudeInitSource> parse_attitude_init_source(const std::string & name);

}  // namespace aqua_graph
