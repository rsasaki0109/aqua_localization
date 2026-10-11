// SPDX-License-Identifier: Apache-2.0
// Online rectification of an equidistant (fisheye) stereo pair from a Kalibr
// camchain, so raw camera streams can feed StereoTracker without an offline
// conversion step. Same geometry as
// aqua_localization/scripts/convert_ntnu_stereo_bag.py.
#pragma once

#include <string>

#include <opencv2/core.hpp>

namespace aqua_visual_frontend
{

struct RectifiedStereoGeometry
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  double baseline_m{0.0};
  // Rotation from the left camera's raw frame to its rectified frame
  // (p_rect = R_left * p_cam).
  cv::Matx33d R_left{cv::Matx33d::eye()};
};

class StereoRectifier
{
public:
  // camchain: Kalibr YAML with two pinhole-equidistant cameras, the second
  // carrying T_cn_cnm1. left_name/right_name pick which camchain entries
  // ("cam0"/"cam1") are the left and right images; the left camera must
  // actually be on the left (the other camera at +x after rectification),
  // otherwise construction throws. balance as in cv::fisheye::stereoRectify.
  StereoRectifier(
    const std::string & camchain_path, const std::string & left_name,
    const std::string & right_name, double balance = 0.0);

  void rectify_left(const cv::Mat & raw, cv::Mat & out) const;
  void rectify_right(const cv::Mat & raw, cv::Mat & out) const;
  const RectifiedStereoGeometry & geometry() const {return geometry_;}
  cv::Size size() const {return size_;}

  // Rectified pixel of a raw pixel (for tests and diagnostics).
  cv::Point2d rectify_point_left(const cv::Point2d & raw) const;
  cv::Point2d rectify_point_right(const cv::Point2d & raw) const;

private:
  cv::Size size_;
  cv::Matx33d K_left_, K_right_, R_left_, R_right_;
  cv::Vec4d D_left_, D_right_;
  cv::Mat P_left_, P_right_;
  cv::Mat map_left_x_, map_left_y_, map_right_x_, map_right_y_;
  RectifiedStereoGeometry geometry_;
};

}  // namespace aqua_visual_frontend
