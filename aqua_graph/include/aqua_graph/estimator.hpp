// SPDX-License-Identifier: Apache-2.0
// Tightly-coupled IMU + DVL + pressure (+ optional absolute position)
// fixed-lag smoother built on GTSAM.
//
// States (pose, world velocity, IMU bias) are created on a fixed time grid
// driven by the IMU stream. IMU samples between grid states are folded into a
// CombinedImuFactor; DVL, depth, and position measurements are attached to the
// grid state nearest to their stamp. The estimator is a plain C++ library with
// no ROS dependency so it can be driven by a ROS node, the deterministic
// offline replay harness, or synthetic unit tests with identical results.
//
// Callers must feed measurements in non-decreasing stamp order across all
// sensors (the replay harness and the node's reorder buffer guarantee this).
#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
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
};

struct PositionConfig
{
  bool enabled{true};
  // Lower bound applied to each axis' sigma, whatever the incoming covariance.
  double sigma_floor_m{0.02};
  double huber_k{3.0};
};

struct EstimatorConfig
{
  ImuNoiseConfig imu;
  InitConfig init;
  DvlConfig dvl;
  DepthConfig depth;
  PositionConfig position;
  // Spacing of the state grid. Measurements attach to the nearest grid state.
  double state_period_s{0.1};
  // Fixed-lag smoother window.
  double smoother_lag_s{5.0};
  // IMU samples with dt above this are treated as gaps and integrated with
  // the previous sample held (covariance still grows with dt).
  double max_imu_dt_s{0.1};
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
  // Measurements whose nearest state already left the smoother window.
  uint64_t measurements_too_old{0};
  uint64_t smoother_updates{0};
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

  // Feed one IMU sample. Returns the estimates of any grid states created
  // while integrating this sample (normally zero or one).
  std::vector<EstimatorOutput> add_imu(
    double stamp_s, const gtsam::Vector3 & accel_body, const gtsam::Vector3 & gyro_body,
    const std::optional<gtsam::Rot3> & orientation = std::nullopt);

  void add_dvl(double stamp_s, const gtsam::Vector3 & velocity_dvl);
  // Optional per-measurement sigma override (<= 0 keeps the configured one).
  void add_dvl(double stamp_s, const gtsam::Vector3 & velocity_dvl, double sigma_mps);
  void add_depth(double stamp_s, double depth_m);
  void add_position(
    double stamp_s, const gtsam::Point3 & position_world, const gtsam::Matrix3 & covariance);

  bool initialized() const {return initialized_;}

  // Latest grid-state estimate propagated with the IMU samples received since.
  std::optional<EstimatorOutput> imu_rate_estimate() const;
  // Latest smoothed estimate of the newest grid state.
  std::optional<EstimatorOutput> latest_state() const;

  const EstimatorCounters & counters() const {return counters_;}
  const EstimatorConfig & config() const {return config_;}

private:
  struct PendingMeasurement
  {
    enum class Kind {kDvl, kDepth, kPosition};
    Kind kind{Kind::kDvl};
    double stamp_s{0.0};
    gtsam::Vector3 vector{gtsam::Vector3::Zero()};
    gtsam::Vector3 omega{gtsam::Vector3::Zero()};
    double scalar{0.0};
    gtsam::Matrix3 covariance{gtsam::Matrix3::Zero()};
  };

  void try_initialize(
    double stamp_s, const gtsam::Vector3 & accel_body, const gtsam::Vector3 & gyro_body,
    const std::optional<gtsam::Rot3> & orientation);
  void start_graph(double stamp_s, const gtsam::NavState & state,
    const gtsam::imuBias::ConstantBias & bias);
  void integrate(double dt, const gtsam::Vector3 & accel, const gtsam::Vector3 & gyro);
  EstimatorOutput create_next_state();
  int64_t nearest_state_index(double stamp_s) const;
  void attach_measurements(gtsam::NonlinearFactorGraph & graph, uint64_t newest_index);
  gtsam::SharedNoiseModel robust(
    const gtsam::SharedNoiseModel & model, double huber_k) const;
  void run_update(
    const gtsam::NonlinearFactorGraph & graph, const gtsam::Values & values,
    uint64_t new_state_index);
  double state_stamp(uint64_t index) const;

  EstimatorConfig config_;
  EstimatorCounters counters_;
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> preint_params_;
  std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> preint_;
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;

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

  double t0_s_{0.0};
  uint64_t current_index_{0};
  double last_imu_stamp_s_{0.0};
  gtsam::Vector3 last_accel_{gtsam::Vector3::Zero()};
  gtsam::Vector3 last_gyro_{gtsam::Vector3::Zero()};
  double integrated_until_s_{0.0};

  gtsam::NavState current_state_;
  gtsam::imuBias::ConstantBias current_bias_;
  std::deque<PendingMeasurement> pending_;
};

// Parse "kAccelerometer"-style names used in YAML ("accelerometer",
// "imu_orientation"). Returns nullopt for unknown names.
std::optional<AttitudeInitSource> parse_attitude_init_source(const std::string & name);

}  // namespace aqua_graph
