#ifndef AQUA_IMU_LOC__FEEDBACK_ODOMETRY_BUFFER_HPP_
#define AQUA_IMU_LOC__FEEDBACK_ODOMETRY_BUFFER_HPP_

#include <cstddef>
#include <deque>
#include <vector>

#include <Eigen/Dense>

namespace aqua_imu_loc
{

// One buffered absolute-position observation from an external odometry
// source (sonar registration feedback).
struct FeedbackObservation
{
  double stamp_s{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Zero()};
};

// Stamp-ordered staging buffer for sonar odometry feedback into the IMU UKF.
// Subscription callbacks insert observations in whatever order DDS delivers
// them; the IMU step drains everything at or before the current IMU stamp in
// message-stamp order, so the sequence of measurement updates applied to the
// filter does not depend on callback arrival order.
class FeedbackOdometryBuffer
{
public:
  // Insert keeping ascending stamp order. Observations with equal stamps keep
  // their insertion order (stable), so replays that deliver duplicates apply
  // them identically.
  void push(const FeedbackObservation & observation);

  // Remove and return every observation with stamp <= `stamp_s`, ascending.
  std::vector<FeedbackObservation> drain_through(double stamp_s);

  std::size_t pending() const;
  void clear();

private:
  std::deque<FeedbackObservation> entries_;
};

}  // namespace aqua_imu_loc

#endif  // AQUA_IMU_LOC__FEEDBACK_ODOMETRY_BUFFER_HPP_
