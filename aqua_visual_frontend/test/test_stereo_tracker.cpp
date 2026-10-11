// SPDX-License-Identifier: Apache-2.0
// StereoTracker on synthetic rectified pairs: a random texture shifted by a
// known disparity (right image) and translated between frames.
#include <gtest/gtest.h>

#include <cmath>
#include <map>

#include <opencv2/imgproc.hpp>

#include "aqua_visual_frontend/stereo_tracker.hpp"

namespace
{

cv::Mat texture(int width, int height)
{
  cv::Mat noise(height, width, CV_8UC1);
  cv::RNG rng(42);
  rng.fill(noise, cv::RNG::UNIFORM, 0, 255);
  cv::Mat smooth;
  cv::GaussianBlur(noise, smooth, cv::Size(0, 0), 2.0);
  cv::normalize(smooth, smooth, 0, 255, cv::NORM_MINMAX);
  return smooth;
}

// Crop of the big texture at integer offset (x0, y0).
cv::Mat view(const cv::Mat & world, int x0, int y0, int width, int height)
{
  return world(cv::Rect(x0, y0, width, height)).clone();
}

}  // namespace

TEST(StereoTracker, RecoversDisparityAndKeepsIdsAcrossFrames)
{
  const int w = 320;
  const int h = 240;
  const int disparity = 12;
  const cv::Mat world = texture(600, 400);
  aqua_visual_frontend::StereoTrackerConfig config;
  config.max_features = 80;
  aqua_visual_frontend::StereoTracker tracker(config);

  // The right camera sees the scene shifted left by the disparity:
  // u_right = u_left - disparity.
  std::map<std::uint64_t, cv::Point2f> first;
  const auto f0 = tracker.process(view(world, 100, 80, w, h), view(world, 100 + disparity, 80, w, h));
  ASSERT_GT(f0.size(), 40U);
  for (const auto & t : f0) {
    EXPECT_NEAR(t.u_left - t.u_right, disparity, 0.2) << "id " << t.id;
    first[t.id] = cv::Point2f(t.u_left, t.v);
  }

  // Camera moves +3 px in x and +2 px in y: features move -3/-2 px.
  const auto f1 = tracker.process(view(world, 103, 82, w, h), view(world, 103 + disparity, 82, w, h));
  // LK is less accurate within half a window of the border, so require
  // sub-0.2 px motion for nearly all (not every) persisting feature.
  size_t persisted = 0;
  size_t accurate = 0;
  for (const auto & t : f1) {
    EXPECT_NEAR(t.u_left - t.u_right, disparity, 0.5);
    const auto it = first.find(t.id);
    if (it == first.end()) {
      continue;
    }
    ++persisted;
    if (std::abs(t.u_left - (it->second.x - 3.0F)) < 0.2F &&
      std::abs(t.v - (it->second.y - 2.0F)) < 0.2F)
    {
      ++accurate;
    }
  }
  EXPECT_GT(persisted, 30U);
  EXPECT_GE(accurate * 100, persisted * 95);
}

TEST(StereoTracker, ResetStartsNewTracksWithFreshIds)
{
  const cv::Mat world = texture(400, 300);
  aqua_visual_frontend::StereoTracker tracker;
  const cv::Mat left = view(world, 40, 30, 320, 240);
  const cv::Mat right = view(world, 48, 30, 320, 240);
  const auto a = tracker.process(left, right);
  ASSERT_FALSE(a.empty());
  const auto ids_after_first = tracker.next_id();
  tracker.reset();
  const auto b = tracker.process(left, right);
  ASSERT_FALSE(b.empty());
  for (const auto & t : b) {
    EXPECT_GE(t.id, ids_after_first);
  }
}

TEST(StereoTracker, RejectsPairsWithoutValidDisparity)
{
  const cv::Mat world = texture(400, 300);
  aqua_visual_frontend::StereoTracker tracker;
  // Identical images: zero disparity is below min_disparity_px.
  const cv::Mat img = view(world, 40, 30, 320, 240);
  EXPECT_TRUE(tracker.process(img, img).empty());
}
