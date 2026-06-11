#ifndef AQUA_IMU_LOC__POSITION_HISTORY_BUFFER_HPP_
#define AQUA_IMU_LOC__POSITION_HISTORY_BUFFER_HPP_

#include <deque>
#include <optional>

#include <Eigen/Dense>

namespace aqua_imu_loc
{

// Short history of (stamp, filter position) used to evaluate delayed absolute
// position measurements (visual / sonar odometry) at their measurement time
// instead of against the current state. Without this, a measurement that is
// `age` seconds old drags the estimate backward along the trajectory by
// roughly vehicle_speed * age.
//
// Two invariants matter:
//
// 1. lookup(t) interpolates between the two entries that bracket `t`. A stamp
//    newer than the newest entry clamps to the newest entry; a stamp older
//    than the oldest entry returns nullopt so the caller can fall back to an
//    uncompensated update.
//
// 2. shift_from(t, delta) folds an applied filter correction back into every
//    entry at or after `t`. Without this, a second delayed measurement
//    arriving after a correction looks up the *uncorrected* past position,
//    re-measures the error the filter already absorbed, and re-applies it —
//    a positive feedback that destroys the trajectory once several late
//    samples queue up.
class PositionHistoryBuffer
{
public:
  void configure(double horizon_s);

  // Append the current filter position; trims entries older than the horizon
  // relative to `stamp_s`. Out-of-order pushes (stamp <= newest) are ignored.
  void push(double stamp_s, const Eigen::Vector3d & position);

  // Interpolated position at `stamp_s`, or nullopt when the buffer is empty
  // or `stamp_s` predates the oldest entry.
  std::optional<Eigen::Vector3d> lookup(double stamp_s) const;

  // Add `delta` to every entry with stamp >= `stamp_s`.
  void shift_from(double stamp_s, const Eigen::Vector3d & delta);

  void clear();
  std::size_t size() const;

private:
  struct Entry
  {
    double stamp_s;
    Eigen::Vector3d position;
  };

  std::deque<Entry> entries_;
  double horizon_s_{2.0};
};

}  // namespace aqua_imu_loc

#endif  // AQUA_IMU_LOC__POSITION_HISTORY_BUFFER_HPP_
