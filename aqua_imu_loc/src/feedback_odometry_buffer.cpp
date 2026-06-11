#include "aqua_imu_loc/feedback_odometry_buffer.hpp"

#include <algorithm>

namespace aqua_imu_loc
{

void FeedbackOdometryBuffer::push(const FeedbackObservation & observation)
{
  const auto insert_at = std::upper_bound(
    entries_.begin(), entries_.end(), observation.stamp_s,
    [](double value, const FeedbackObservation & entry) {return value < entry.stamp_s;});
  entries_.insert(insert_at, observation);
}

std::vector<FeedbackObservation> FeedbackOdometryBuffer::drain_through(double stamp_s)
{
  std::vector<FeedbackObservation> drained;
  while (!entries_.empty() && entries_.front().stamp_s <= stamp_s) {
    drained.push_back(entries_.front());
    entries_.pop_front();
  }
  return drained;
}

std::size_t FeedbackOdometryBuffer::pending() const
{
  return entries_.size();
}

void FeedbackOdometryBuffer::clear()
{
  entries_.clear();
}

}  // namespace aqua_imu_loc
