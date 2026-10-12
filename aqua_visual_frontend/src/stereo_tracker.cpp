// SPDX-License-Identifier: Apache-2.0
#include "aqua_visual_frontend/stereo_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace aqua_visual_frontend
{

StereoTracker::StereoTracker(StereoTrackerConfig config)
: config_(std::move(config)), disparity_guess_(config_.initial_disparity_px)
{
  if (config_.clahe_clip_limit > 0.0) {
    clahe_ = cv::createCLAHE(
      config_.clahe_clip_limit, cv::Size(config_.clahe_tile_grid, config_.clahe_tile_grid));
  }
  if (config_.reuse_tracks || config_.descriptors) {
    orb_ = cv::ORB::create();
  }
}

std::vector<std::pair<std::uint64_t, cv::Mat>> StereoTracker::take_new_descriptors()
{
  return std::exchange(new_descriptors_, {});
}

void StereoTracker::retire(std::uint64_t id)
{
  const auto it = descriptors_.find(id);
  if (it == descriptors_.end()) {
    return;
  }
  retired_.emplace_back(id, it->second);
  descriptors_.erase(it);
  while (static_cast<int>(retired_.size()) > config_.reuse_max_retired) {
    retired_.pop_front();
  }
}

std::vector<std::uint64_t> StereoTracker::assign_ids(
  const cv::Mat & image, const std::vector<cv::Point2f> & corners)
{
  std::vector<std::uint64_t> ids(corners.size(), 0);
  std::vector<bool> assigned(corners.size(), false);
  if ((config_.reuse_tracks || config_.descriptors) && !corners.empty()) {
    std::vector<cv::KeyPoint> keypoints;
    for (size_t i = 0; i < corners.size(); ++i) {
      cv::KeyPoint kp(corners[i], 31.0F);
      kp.class_id = static_cast<int>(i);
      keypoints.push_back(kp);
    }
    cv::Mat desc;
    orb_->compute(image, keypoints, desc);
    if (config_.reuse_tracks && !retired_.empty() && !keypoints.empty()) {
      cv::Mat retired_desc;
      for (const auto & r : retired_) {
        retired_desc.push_back(r.second);
      }
      std::vector<std::vector<cv::DMatch>> matches;
      cv::BFMatcher(cv::NORM_HAMMING).knnMatch(desc, retired_desc, matches, 2);
      std::vector<bool> taken(retired_.size(), false);
      std::vector<size_t> used;
      for (size_t k = 0; k < matches.size(); ++k) {
        if (matches[k].empty()) {
          continue;
        }
        const cv::DMatch & best = matches[k][0];
        const bool distinct = matches[k].size() < 2 ||
          best.distance < config_.reuse_ratio * matches[k][1].distance;
        if (best.distance > config_.reuse_max_hamming || !distinct ||
          taken[static_cast<size_t>(best.trainIdx)])
        {
          continue;
        }
        taken[static_cast<size_t>(best.trainIdx)] = true;
        const auto i = static_cast<size_t>(keypoints[k].class_id);
        ids[i] = retired_[static_cast<size_t>(best.trainIdx)].first;
        assigned[i] = true;
        ++reused_tracks_;
      }
      // Drop the reused entries from the retired set.
      std::deque<std::pair<std::uint64_t, cv::Mat>> kept;
      for (size_t r = 0; r < retired_.size(); ++r) {
        if (!taken[r]) {
          kept.push_back(std::move(retired_[r]));
        }
      }
      retired_ = std::move(kept);
    }
    for (size_t k = 0; k < keypoints.size(); ++k) {
      const auto i = static_cast<size_t>(keypoints[k].class_id);
      if (!assigned[i]) {
        ids[i] = next_id_++;
        assigned[i] = true;
      }
      descriptors_[ids[i]] = desc.row(static_cast<int>(k)).clone();
      if (config_.descriptors) {
        new_descriptors_.emplace_back(ids[i], descriptors_[ids[i]]);
      }
    }
  }
  for (size_t i = 0; i < corners.size(); ++i) {
    if (!assigned[i]) {
      ids[i] = next_id_++;
    }
  }
  return ids;
}

cv::Mat StereoTracker::prepare(const cv::Mat & image) const
{
  if (clahe_.empty()) {
    return image;
  }
  cv::Mat out;
  clahe_->apply(image, out);
  return out;
}

cv::Mat StereoTracker::valid_mask(const cv::Mat & image) const
{
  cv::Mat mask(image.size(), CV_8UC1, cv::Scalar(255));
  if (config_.saturation_level <= 255) {
    cv::Mat saturated;
    cv::threshold(image, saturated, config_.saturation_level - 1, 255, cv::THRESH_BINARY);
    if (cv::countNonZero(saturated) > 0) {
      const int k = 2 * config_.min_distance_px + 1;
      cv::dilate(saturated, saturated, cv::getStructuringElement(cv::MORPH_ELLIPSE, {k, k}));
      mask.setTo(0, saturated);
    }
  }
  return mask;
}

void StereoTracker::reset()
{
  for (const auto id : ids_) {
    retire(id);
  }
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

std::vector<StereoTrack> StereoTracker::process(
  const cv::Mat & raw_left, const cv::Mat & raw_right)
{
  const auto & c = config_;
  // Saturation is judged on the raw image, before CLAHE stretches it.
  const cv::Mat valid = valid_mask(raw_left);
  const cv::Mat left = prepare(raw_left);
  const bool mono = raw_right.empty();
  const cv::Mat right = mono ? cv::Mat() : prepare(raw_right);
  std::vector<cv::Point2f> pts;
  std::vector<std::uint64_t> ids;

  // 1) Temporal tracking in the left image.
  if (!prev_left_.empty() && !prev_pts_.empty()) {
    std::vector<cv::Point2f> cur;
    std::vector<bool> ok;
    flow(prev_left_, left, prev_pts_, nullptr, cur, ok);
    std::vector<cv::Point2f> prev_kept;
    for (size_t i = 0; i < ok.size(); ++i) {
      if (ok[i] && valid.at<uchar>(
          static_cast<int>(cur[i].y), static_cast<int>(cur[i].x)) != 0)
      {
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

  if (config_.reuse_tracks) {
    std::vector<std::uint64_t> kept = ids;
    std::sort(kept.begin(), kept.end());
    for (const auto id : ids_) {
      if (!std::binary_search(kept.begin(), kept.end(), id)) {
        retire(id);
      }
    }
  }

  // 2) Top up with new corners away from existing tracks.
  const int missing = c.max_features - static_cast<int>(pts.size());
  if (missing > 0) {
    cv::Mat mask = valid.clone();
    for (const auto & p : pts) {
      cv::circle(
        mask, cv::Point(static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y))),
        c.min_distance_px, cv::Scalar(0), -1);
    }
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(left, corners, missing, c.quality_level, c.min_distance_px, mask);
    const std::vector<std::uint64_t> corner_ids = assign_ids(left, corners);
    for (size_t i = 0; i < corners.size(); ++i) {
      pts.push_back(corners[i]);
      ids.push_back(corner_ids[i]);
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
  if (mono) {
    // Monocular: every left-image track, with no right-image coordinate.
    for (size_t i = 0; i < pts.size(); ++i) {
      out.push_back(
        StereoTrack{ids[i], pts[i].x, pts[i].y, std::numeric_limits<float>::quiet_NaN()});
    }
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
