#ifndef AQUA_IMU_LOC__ADDITIVE_UKF_HPP_
#define AQUA_IMU_LOC__ADDITIVE_UKF_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Dense>

namespace aqua_imu_loc
{

constexpr int kStateDim = 15;
constexpr double kPi = 3.14159265358979323846;

using StateVector = Eigen::Matrix<double, kStateDim, 1>;
using StateMatrix = Eigen::Matrix<double, kStateDim, kStateDim>;

struct ImuSample
{
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

struct DynamicsParams
{
  double gravity_mps2{9.80665};
  bool enable_linear_drag{true};
  double linear_drag_coeff{0.05};
  bool enable_buoyancy{false};
  double buoyancy_accel_z_mps2{0.0};
  Eigen::Vector3d current_velocity{Eigen::Vector3d::Zero()};
};

class AdditiveUkf
{
public:
  void configure(double alpha, double beta, double kappa);
  void set_state(const StateVector & state);
  void set_initial_covariance(const std::vector<double> & diagonal);
  void set_process_noise(const std::vector<double> & diagonal);

  void predict(double dt, const ImuSample & imu, const DynamicsParams & dynamics);
  void update_depth(double depth_m, double variance);
  void update_yaw(double yaw_rad, double variance);
  void update_gyro_bias_z(double observed_bias_rad_s, double variance);
  void update_gyro_bias_xyz(
    const Eigen::Vector3d & observed_bias_rad_s,
    const Eigen::Vector3d & variance_diagonal);

  const StateVector & state() const;
  const StateMatrix & covariance() const;

private:
  std::vector<StateVector> make_sigma_points() const;
  StateVector process_model(
    const StateVector & state, double dt, const ImuSample & imu,
    const DynamicsParams & dynamics) const;
  StateVector weighted_mean(const std::vector<StateVector> & sigma_points) const;
  double depth_measurement_model(const StateVector & state) const;

  StateVector state_{StateVector::Zero()};
  StateMatrix covariance_{StateMatrix::Identity() * 1.0e-3};
  StateMatrix process_noise_{StateMatrix::Identity() * 1.0e-4};
  double alpha_{0.2};
  double beta_{2.0};
  double kappa_{0.0};
  double lambda_{0.0};
  std::vector<double> mean_weights_;
  std::vector<double> covariance_weights_;
};

double normalize_angle(double angle);
Eigen::Vector3d normalize_angles(const Eigen::Vector3d & angles);
Eigen::Matrix3d rotation_from_rpy(const Eigen::Vector3d & rpy);
Eigen::Vector3d euler_rates_from_body_rates(
  const Eigen::Vector3d & rpy, const Eigen::Vector3d & body_rates);

}  // namespace aqua_imu_loc

#endif  // AQUA_IMU_LOC__ADDITIVE_UKF_HPP_
