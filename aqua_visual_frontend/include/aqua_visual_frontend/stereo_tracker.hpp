// SPDX-License-Identifier: Apache-2.0
// VINS-style stereo feature tracker for rectified image pairs. Corners are
// detected in the left image, tracked frame to frame with pyramidal
// Lucas-Kanade (forward-backward check + fundamental-matrix RANSAC), topped
// up away from existing tracks, and matched into the right image with LK
// along the epipolar line. Every surviving feature keeps a persistent id.
// Same algorithm and defaults as aqua_replay/scripts/make_stereo_feature_track.py.
#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace aqua_visual_frontend
{

struct StereoTrackerConfig
{
  int max_features{200};
  int min_distance_px{20};
  double quality_level{0.01};
  int lk_window{21};
  int lk_levels{3};
  double max_backtrack_px{0.5};
  double fundamental_threshold_px{1.0};
  double max_epipolar_px{1.5};
  double min_disparity_px{1.0};
  double max_disparity_px{200.0};
  double initial_disparity_px{0.0};
  // Contrast-limited histogram equalization of both images before tracking
  // (hazy, low-contrast water); 0 disables it.
  double clahe_clip_limit{0.0};
  int clahe_tile_grid{8};
  // Pixels at or above this level (sun glare, strobes) and a min_distance_px
  // band around them get no new corners and drop tracks; 256 disables it.
  int saturation_level{256};
};

struct StereoTrack
{
  std::uint64_t id{0};
  float u_left{0.0F};
  float v{0.0F};
  float u_right{0.0F};
};

class StereoTracker
{
public:
  explicit StereoTracker(StereoTrackerConfig config = {});

  // Track into a new rectified grayscale pair; returns the features that
  // also matched into the right image. With an empty right image
  // (monocular) it returns every left-image track with u_right = NaN.
  std::vector<StereoTrack> process(const cv::Mat & left, const cv::Mat & right);
  // Forget all tracks (e.g. after a camera dropout); ids keep increasing.
  void reset();
  std::uint64_t next_id() const {return next_id_;}

private:
  // LK with a forward-backward check; returns tracked points and a mask.
  void flow(
    const cv::Mat & img0, const cv::Mat & img1, const std::vector<cv::Point2f> & pts,
    const std::vector<cv::Point2f> * guess, std::vector<cv::Point2f> & out,
    std::vector<bool> & ok) const;

  // Grayscale image the tracker works on (CLAHE applied when enabled).
  cv::Mat prepare(const cv::Mat & image) const;
  // 255 where corners may be detected and tracks may live.
  cv::Mat valid_mask(const cv::Mat & image) const;

  StereoTrackerConfig config_;
  cv::Ptr<cv::CLAHE> clahe_;
  cv::Mat prev_left_;
  std::vector<cv::Point2f> prev_pts_;
  std::vector<std::uint64_t> ids_;
  std::uint64_t next_id_{0};
  double disparity_guess_{0.0};
};

}  // namespace aqua_visual_frontend
