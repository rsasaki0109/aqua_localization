// New hydrostatic model; not recovered code from user-reported commit 51cea95.
#pragma once
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/Vec.h"
#include <cmath>
#include <stdexcept>

class HydrostaticPressure {
public:
  static constexpr double density = 1025.0, gravity = 9.80665, sigma_depth = 0.2;
  std::shared_ptr<ov_type::Vec> offset;
  // World z is up. A constant unknown offset absorbs atmospheric pressure and
  // the VIO translation gauge. Sensor is assumed co-located with the IMU.
  void update(const std::shared_ptr<ov_msckf::State>& state, double pressure_pa) {
    if (!std::isfinite(pressure_pa) || pressure_pa < 80000 || pressure_pa > 1e8)
      throw std::runtime_error("registered FluidPressure Pa range violated; no unit guessing");
    double observation = -pressure_pa / (density * gravity);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 3), R(1, 1);
    H(0, 2) = 1; R(0, 0) = sigma_depth * sigma_depth;
    Eigen::VectorXd residual(1);
    if (!offset) {
      offset = std::make_shared<ov_type::Vec>(1);
      Eigen::VectorXd value(1); value(0) = observation - state->_imu->pos()(2);
      offset->set_value(value); offset->set_fej(value); residual(0) = 0;
      // Augment with the anchor's uncertainty and cross-covariances. Treating
      // the first estimated altitude as an exact independent anchor is invalid.
      ov_msckf::StateHelper::initialize_invertible(state, offset, {state->_imu->p()},
          H, Eigen::MatrixXd::Identity(1, 1), R, residual);
    } else {
      Eigen::MatrixXd fullH = Eigen::MatrixXd::Zero(1, 4);
      fullH(0, 2) = fullH(0, 3) = 1;
      residual(0) = observation - state->_imu->pos()(2) - offset->value()(0);
      ov_msckf::StateHelper::EKFUpdate(state, {state->_imu->p(), offset}, fullH, residual, R);
    }
  }
};
