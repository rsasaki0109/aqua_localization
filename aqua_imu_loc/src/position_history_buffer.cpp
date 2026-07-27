#include "aqua_imu_loc/position_history_buffer.hpp"

#include <algorithm>

namespace aqua_imu_loc
{

void PositionHistoryBuffer::configure(double horizon_s)
{
  horizon_s_ = std::max(horizon_s, 0.0);
}

void PositionHistoryBuffer::push(double stamp_s, const Eigen::Vector3d & position)
{
  if (!entries_.empty() && stamp_s <= entries_.back().stamp_s) {
    return;
  }
  entries_.push_back({stamp_s, position});
  const double cutoff = stamp_s - horizon_s_;
  while (!entries_.empty() && entries_.front().stamp_s < cutoff) {
    entries_.pop_front();
  }
}

std::optional<Eigen::Vector3d> PositionHistoryBuffer::lookup(double stamp_s) const
{
  if (entries_.empty() || stamp_s < entries_.front().stamp_s) {
    return std::nullopt;
  }
  if (stamp_s >= entries_.back().stamp_s) {
    return entries_.back().position;
  }
  const auto upper = std::upper_bound(
    entries_.begin(), entries_.end(), stamp_s,
    [](double value, const Entry & entry) {return value < entry.stamp_s;});
  const auto & after = *upper;
  const auto & before = *std::prev(upper);
  const double span = after.stamp_s - before.stamp_s;
  if (span <= 0.0) {
    return before.position;
  }
  const double ratio = (stamp_s - before.stamp_s) / span;
  return before.position + ratio * (after.position - before.position);
}

void PositionHistoryBuffer::shift_from(double stamp_s, const Eigen::Vector3d & delta)
{
  for (auto it = entries_.rbegin(); it != entries_.rend(); ++it) {
    if (it->stamp_s < stamp_s) {
      break;
    }
    it->position += delta;
  }
}

void PositionHistoryBuffer::clear()
{
  entries_.clear();
}

std::size_t PositionHistoryBuffer::size() const
{
  return entries_.size();
}

}  // namespace aqua_imu_loc
