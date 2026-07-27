#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "aqua_imu_loc/position_history_buffer.hpp"

namespace
{

using aqua_imu_loc::PositionHistoryBuffer;

PositionHistoryBuffer make_linear_track()
{
  // Position moves along +x at 1 m/s, sampled at 10 Hz.
  PositionHistoryBuffer buffer;
  buffer.configure(2.0);
  for (int i = 0; i <= 10; ++i) {
    const double t = 100.0 + 0.1 * i;
    buffer.push(t, Eigen::Vector3d(0.1 * i, 0.0, 0.0));
  }
  return buffer;
}

TEST(PositionHistoryBufferTest, LookupExactStampReturnsStoredPosition)
{
  const auto buffer = make_linear_track();
  const auto position = buffer.lookup(100.5);
  ASSERT_TRUE(position.has_value());
  EXPECT_NEAR(position->x(), 0.5, 1e-12);
  EXPECT_NEAR(position->y(), 0.0, 1e-12);
  EXPECT_NEAR(position->z(), 0.0, 1e-12);
}

TEST(PositionHistoryBufferTest, LookupInterpolatesBetweenStamps)
{
  const auto buffer = make_linear_track();
  const auto position = buffer.lookup(100.55);
  ASSERT_TRUE(position.has_value());
  EXPECT_NEAR(position->x(), 0.55, 1e-12);
}

TEST(PositionHistoryBufferTest, LookupOlderThanBufferReturnsNullopt)
{
  const auto buffer = make_linear_track();
  EXPECT_FALSE(buffer.lookup(99.0).has_value());
  PositionHistoryBuffer empty;
  EXPECT_FALSE(empty.lookup(100.0).has_value());
}

TEST(PositionHistoryBufferTest, LookupNewerThanBufferClampsToNewest)
{
  const auto buffer = make_linear_track();
  const auto position = buffer.lookup(200.0);
  ASSERT_TRUE(position.has_value());
  EXPECT_NEAR(position->x(), 1.0, 1e-12);
}

TEST(PositionHistoryBufferTest, PushTrimsEntriesOlderThanHorizon)
{
  PositionHistoryBuffer buffer;
  buffer.configure(0.5);
  buffer.push(100.0, Eigen::Vector3d::Zero());
  buffer.push(100.4, Eigen::Vector3d::UnitX());
  EXPECT_EQ(buffer.size(), 2u);
  buffer.push(100.6, Eigen::Vector3d::UnitX() * 2.0);
  EXPECT_EQ(buffer.size(), 2u);
  EXPECT_FALSE(buffer.lookup(100.0).has_value());
}

TEST(PositionHistoryBufferTest, ShiftFromAppliesCorrectionToTailOnly)
{
  auto buffer = make_linear_track();
  const Eigen::Vector3d delta(0.0, 1.0, 0.0);
  buffer.shift_from(100.5, delta);

  // Entries before the measurement stamp are untouched.
  const auto before = buffer.lookup(100.2);
  ASSERT_TRUE(before.has_value());
  EXPECT_NEAR(before->y(), 0.0, 1e-12);

  // Entries at and after the measurement stamp carry the correction.
  const auto at = buffer.lookup(100.5);
  ASSERT_TRUE(at.has_value());
  EXPECT_NEAR(at->y(), 1.0, 1e-12);
  const auto after = buffer.lookup(101.0);
  ASSERT_TRUE(after.has_value());
  EXPECT_NEAR(after->y(), 1.0, 1e-12);
}

TEST(PositionHistoryBufferTest, ShiftFromStopsDelayedMeasurementReapplication)
{
  // Reproduces the positive-feedback flaw the shift fix kills: two delayed
  // measurements of the same external track arrive after the filter already
  // absorbed the first one's correction. With shift_from, the innovation
  // computed for the second measurement is only the *new* information, not
  // the already-applied correction again.
  auto buffer = make_linear_track();

  // External source says the position at t=100.5 was actually 0.7, i.e. the
  // filter was 0.2 behind. The filter applies the full correction.
  const Eigen::Vector3d measurement_1(0.7, 0.0, 0.0);
  const auto history_1 = buffer.lookup(100.5);
  ASSERT_TRUE(history_1.has_value());
  const Eigen::Vector3d innovation_1 = measurement_1 - *history_1;
  EXPECT_NEAR(innovation_1.x(), 0.2, 1e-12);
  buffer.shift_from(100.5, innovation_1);

  // A second sample from the same source at t=100.6 (same 0.2 offset track).
  // The corrected history already contains the absorbed 0.2, so the residual
  // innovation is zero — without shift_from it would be 0.2 again and the
  // correction would double up.
  const Eigen::Vector3d measurement_2(0.8, 0.0, 0.0);
  const auto history_2 = buffer.lookup(100.6);
  ASSERT_TRUE(history_2.has_value());
  const Eigen::Vector3d innovation_2 = measurement_2 - *history_2;
  EXPECT_NEAR(innovation_2.x(), 0.0, 1e-12);
}

TEST(PositionHistoryBufferTest, OutOfOrderPushIsIgnored)
{
  auto buffer = make_linear_track();
  const auto size_before = buffer.size();
  buffer.push(100.5, Eigen::Vector3d::Constant(99.0));
  EXPECT_EQ(buffer.size(), size_before);
  const auto position = buffer.lookup(100.5);
  ASSERT_TRUE(position.has_value());
  EXPECT_NEAR(position->x(), 0.5, 1e-12);
}

}  // namespace
