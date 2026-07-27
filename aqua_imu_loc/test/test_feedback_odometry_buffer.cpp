#include <gtest/gtest.h>

#include <vector>

#include <Eigen/Dense>

#include "aqua_imu_loc/feedback_odometry_buffer.hpp"

namespace
{

using aqua_imu_loc::FeedbackObservation;
using aqua_imu_loc::FeedbackOdometryBuffer;

FeedbackObservation make_observation(double stamp_s, double x)
{
  FeedbackObservation observation;
  observation.stamp_s = stamp_s;
  observation.position = Eigen::Vector3d(x, 0.0, 0.0);
  observation.covariance = Eigen::Matrix3d::Identity() * 0.04;
  return observation;
}

TEST(FeedbackOdometryBufferTest, DrainsInStampOrderRegardlessOfArrivalOrder)
{
  FeedbackOdometryBuffer buffer;
  buffer.push(make_observation(100.3, 3.0));
  buffer.push(make_observation(100.1, 1.0));
  buffer.push(make_observation(100.2, 2.0));

  const auto drained = buffer.drain_through(100.4);
  ASSERT_EQ(drained.size(), 3u);
  EXPECT_DOUBLE_EQ(drained[0].stamp_s, 100.1);
  EXPECT_DOUBLE_EQ(drained[1].stamp_s, 100.2);
  EXPECT_DOUBLE_EQ(drained[2].stamp_s, 100.3);
  EXPECT_EQ(buffer.pending(), 0u);
}

TEST(FeedbackOdometryBufferTest, DrainStopsAtRequestedStamp)
{
  FeedbackOdometryBuffer buffer;
  buffer.push(make_observation(100.1, 1.0));
  buffer.push(make_observation(100.2, 2.0));
  buffer.push(make_observation(100.5, 5.0));

  const auto drained = buffer.drain_through(100.2);
  ASSERT_EQ(drained.size(), 2u);
  EXPECT_DOUBLE_EQ(drained.back().stamp_s, 100.2);
  EXPECT_EQ(buffer.pending(), 1u);

  // The future-stamped observation drains once the clock catches up.
  const auto rest = buffer.drain_through(100.5);
  ASSERT_EQ(rest.size(), 1u);
  EXPECT_DOUBLE_EQ(rest.front().stamp_s, 100.5);
}

TEST(FeedbackOdometryBufferTest, EqualStampsKeepInsertionOrder)
{
  FeedbackOdometryBuffer buffer;
  buffer.push(make_observation(100.1, 1.0));
  buffer.push(make_observation(100.1, 2.0));
  buffer.push(make_observation(100.1, 3.0));

  const auto drained = buffer.drain_through(100.1);
  ASSERT_EQ(drained.size(), 3u);
  EXPECT_DOUBLE_EQ(drained[0].position.x(), 1.0);
  EXPECT_DOUBLE_EQ(drained[1].position.x(), 2.0);
  EXPECT_DOUBLE_EQ(drained[2].position.x(), 3.0);
}

TEST(FeedbackOdometryBufferTest, RepeatedShuffledInsertionsDrainIdentically)
{
  // Determinism at the buffer level: any arrival order of the same message
  // set produces the same drained sequence.
  const std::vector<std::vector<int>> arrival_orders = {
    {0, 1, 2, 3, 4},
    {4, 3, 2, 1, 0},
    {2, 0, 4, 1, 3},
  };
  std::vector<std::vector<double>> drained_stamps;
  for (const auto & order : arrival_orders) {
    FeedbackOdometryBuffer buffer;
    for (const int index : order) {
      buffer.push(make_observation(100.0 + 0.1 * index, static_cast<double>(index)));
    }
    std::vector<double> stamps;
    for (const auto & observation : buffer.drain_through(200.0)) {
      stamps.push_back(observation.stamp_s);
    }
    drained_stamps.push_back(stamps);
  }
  EXPECT_EQ(drained_stamps[0], drained_stamps[1]);
  EXPECT_EQ(drained_stamps[0], drained_stamps[2]);
}

TEST(FeedbackOdometryBufferTest, ClearEmptiesBuffer)
{
  FeedbackOdometryBuffer buffer;
  buffer.push(make_observation(100.1, 1.0));
  buffer.clear();
  EXPECT_EQ(buffer.pending(), 0u);
  EXPECT_TRUE(buffer.drain_through(200.0).empty());
}

}  // namespace
