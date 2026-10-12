// SPDX-License-Identifier: Apache-2.0
// StereoTracker on synthetic rectified pairs: a random texture shifted by a
// known disparity (right image) and translated between frames.
#include <gtest/gtest.h>

#include <cmath>
#include <unordered_map>
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

TEST(StereoTracker, SaturatedRegionGetsNoFeatures)
{
  cv::Mat world = texture(400, 300);
  world.convertTo(world, CV_8UC1, 200.0 / 255.0);
  // A blown-out band (sun glare) across the top of both views.
  world(cv::Rect(0, 0, 400, 90)).setTo(255);
  aqua_visual_frontend::StereoTrackerConfig config;
  config.saturation_level = 250;
  aqua_visual_frontend::StereoTracker tracker(config);
  // The views start at y = 30, so the glare covers image rows 0-59.
  const auto f = tracker.process(view(world, 40, 30, 320, 240), view(world, 48, 30, 320, 240));
  ASSERT_GT(f.size(), 20U);
  for (const auto & t : f) {
    EXPECT_GE(t.v, 60.0F + config.min_distance_px - 1.0F) << "id " << t.id;
  }
}

TEST(StereoTracker, ClaheKeepsDisparity)
{
  cv::Mat world = texture(400, 300);
  // Low-contrast, hazy version of the texture.
  world.convertTo(world, CV_8UC1, 40.0 / 255.0, 100.0);
  aqua_visual_frontend::StereoTrackerConfig config;
  config.clahe_clip_limit = 2.0;
  aqua_visual_frontend::StereoTracker tracker(config);
  const auto f = tracker.process(view(world, 40, 30, 320, 240), view(world, 48, 30, 320, 240));
  ASSERT_GT(f.size(), 40U);
  for (const auto & t : f) {
    EXPECT_NEAR(t.u_left - t.u_right, 8.0F, 0.3F) << "id " << t.id;
  }
}

TEST(StereoTracker, MonocularTracksKeepIdsAndHaveNoRightCoordinate)
{
  aqua_visual_frontend::StereoTracker tracker;
  const cv::Mat world = texture(400, 300);
  const auto first = tracker.process(view(world, 40, 30, 320, 240), cv::Mat());
  ASSERT_GT(first.size(), 50U);
  for (const auto & t : first) {
    EXPECT_TRUE(std::isnan(t.u_right));
  }
  // Shift the view by 3 px: the same ids come back, 3 px to the left.
  const auto second = tracker.process(view(world, 43, 30, 320, 240), cv::Mat());
  int matched = 0;
  for (const auto & a : first) {
    for (const auto & b : second) {
      if (a.id == b.id) {
        EXPECT_NEAR(a.u_left - b.u_left, 3.0, 0.5);
        EXPECT_NEAR(a.v - b.v, 0.0, 0.5);
        ++matched;
      }
    }
  }
  EXPECT_GT(matched, static_cast<int>(first.size()) / 2);
}

TEST(StereoTracker, ReusesIdsOfLostTracksWhenTheSceneReturns)
{
  aqua_visual_frontend::StereoTrackerConfig config;
  config.reuse_tracks = true;
  aqua_visual_frontend::StereoTracker tracker(config);
  const cv::Mat scene_a = texture(400, 300);
  cv::Mat scene_b = texture(400, 300);
  cv::flip(scene_b, scene_b, -1);  // a different scene

  const auto first = tracker.process(view(scene_a, 40, 30, 320, 240), cv::Mat());
  ASSERT_GT(first.size(), 50U);
  // Everything is lost on the other scene, then scene A comes back.
  tracker.process(view(scene_b, 40, 30, 320, 240), cv::Mat());
  const std::uint64_t reused_on_b = tracker.reused_tracks();
  const auto back = tracker.process(view(scene_a, 40, 30, 320, 240), cv::Mat());

  std::unordered_map<std::uint64_t, cv::Point2f> before;
  for (const auto & t : first) {
    before[t.id] = cv::Point2f(t.u_left, t.v);
  }
  int reused = 0;
  int misplaced = 0;
  for (const auto & t : back) {
    const auto it = before.find(t.id);
    if (it == before.end()) {
      continue;
    }
    ++reused;
    if (cv::norm(it->second - cv::Point2f(t.u_left, t.v)) > 2.0) {
      ++misplaced;
    }
  }
  EXPECT_GT(reused, static_cast<int>(first.size()) / 2);
  EXPECT_LE(misplaced, reused / 10);
  // A different scene takes over few ids (the backend checks geometry).
  EXPECT_LT(reused_on_b, first.size() / 10);
  // Nearly all of them came back through descriptor reuse (a stray KLT
  // track can survive the scene change).
  EXPECT_GE(tracker.reused_tracks() - reused_on_b, static_cast<std::uint64_t>(reused * 9 / 10));
}
