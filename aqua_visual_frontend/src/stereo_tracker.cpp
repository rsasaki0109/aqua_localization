// SPDX-License-Identifier: Apache-2.0
#include "aqua_visual_frontend/stereo_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace aqua_visual_frontend
{

StereoTracker::StereoTracker(StereoTrackerConfig config)
: config_(std::move(config)), disparity_guess_(config_.initial_disparity_px)
{
}

void StereoTracker::reset()
{
  prev_left_.release();
  prev_pts_.clear();
  ids_.clear();
}

void StereoTracker::flow(
  const cv::Mat & img0, const cv::Mat & img1, const std::vector<cv::Point2f> & pts,
  const std::vector<cv::Point2f> * guess, std::vector<cv::Point2f> & out,
  std::vector<bool> & ok) const
{
  out.clear();
  ok.assign(pts.size(), false);
  if (pts.empty()) {
    return;
  }
  const cv::Size window(config_.lk_window, config_.lk_window);
  const cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01);
  std::vector<uchar> st1;
  std::vector<uchar> st2;
  std::vector<float> err;
  int flags = 0;
  if (guess != nullptr) {
    out = *guess;
    flags = cv::OPTFLOW_USE_INITIAL_FLOW;
  }
  cv::calcOpticalFlowPyrLK(
    img0, img1, pts, out, st1, err, window, config_.lk_levels, criteria, flags);
  std::vector<cv::Point2f> back = pts;
  cv::calcOpticalFlowPyrLK(
    img1, img0, out, back, st2, err, window, config_.lk_levels, criteria,
    cv::OPTFLOW_USE_INITIAL_FLOW);
  const float w = static_cast<float>(img1.cols);
  const float h = static_cast<float>(img1.rows);
  for (size_t i = 0; i < pts.size(); ++i) {
    const cv::Point2f & p = out[i];
    const bool inside = p.x >= 0.0F && p.x < w - 1.0F && p.y >= 0.0F && p.y < h - 1.0F;
    const double backtrack = cv::norm(back[i] - pts[i]);
    ok[i] = st1[i] == 1 && st2[i] == 1 && backtrack < config_.max_backtrack_px && inside;
  }
}

std::vector<StereoTrack> StereoTracker::process(const cv::Mat & left, const cv::Mat & right)
{
  const auto & c = config_;
  std::vector<cv::Point2f> pts;
  std::vector<std::uint64_t> ids;

  // 1) Temporal tracking in the left image.
  if (!prev_left_.empty() && !prev_pts_.empty()) {
    std::vector<cv::Point2f> cur;
    std::vector<bool> ok;
    flow(prev_left_, left, prev_pts_, nullptr, cur, ok);
    std::vector<cv::Point2f> prev_kept;
    for (size_t i = 0; i < ok.size(); ++i) {
      if (ok[i]) {
        prev_kept.push_back(prev_pts_[i]);
        pts.push_back(cur[i]);
        ids.push_back(ids_[i]);
      }
    }
    if (pts.size() >= 8) {
      std::vector<uchar> inliers;
      cv::findFundamentalMat(
        prev_kept, pts, cv::FM_RANSAC, c.fundamental_threshold_px, 0.99, inliers);
      if (inliers.size() == pts.size()) {
        std::vector<cv::Point2f> kept_pts;
        std::vector<std::uint64_t> kept_ids;
        for (size_t i = 0; i < inliers.size(); ++i) {
          if (inliers[i]) {
            kept_pts.push_back(pts[i]);
            kept_ids.push_back(ids[i]);
          }
        }
        pts = std::move(kept_pts);
        ids = std::move(kept_ids);
      }
    }
  }

  // 2) Top up with new corners away from existing tracks.
  const int missing = c.max_features - static_cast<int>(pts.size());
  if (missing > 0) {
    cv::Mat mask(left.size(), CV_8UC1, cv::Scalar(255));
    for (const auto & p : pts) {
      cv::circle(
        mask, cv::Point(static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y))),
        c.min_distance_px, cv::Scalar(0), -1);
    }
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(left, corners, missing, c.quality_level, c.min_distance_px, mask);
    for (const auto & corner : corners) {
      pts.push_back(corner);
      ids.push_back(next_id_++);
    }
  }

  prev_left_ = left.clone();
  prev_pts_ = pts;
  ids_ = ids;

  // 3) Stereo matching into the right image along the epipolar line.
  std::vector<StereoTrack> out;
  if (pts.empty()) {
    return out;
  }
  std::vector<cv::Point2f> guess = pts;
  for (auto & g : guess) {
    g.x -= static_cast<float>(disparity_guess_);
  }
  std::vector<cv::Point2f> right_pts;
  std::vector<bool> ok;
  flow(left, right, pts, &guess, right_pts, ok);
  std::vector<double> disparities;
  for (size_t i = 0; i < pts.size(); ++i) {
    const double disparity = pts[i].x - right_pts[i].x;
    if (!ok[i] || std::abs(pts[i].y - right_pts[i].y) >= c.max_epipolar_px ||
      disparity <= c.min_disparity_px || disparity >= c.max_disparity_px)
    {
      continue;
    }
    disparities.push_back(disparity);
    out.push_back(StereoTrack{ids[i], pts[i].x, pts[i].y, right_pts[i].x});
  }
  if (disparities.size() >= 10) {
    // numpy.median: mean of the two middle values for an even count.
    std::sort(disparities.begin(), disparities.end());
    const size_t n = disparities.size();
    disparity_guess_ = n % 2 ? disparities[n / 2] :
      0.5 * (disparities[n / 2 - 1] + disparities[n / 2]);
  }
  return out;
}

}  // namespace aqua_visual_frontend
