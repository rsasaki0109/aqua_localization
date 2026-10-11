// SPDX-License-Identifier: Apache-2.0
#include "aqua_visual_frontend/stereo_rectifier.hpp"

#include <stdexcept>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace aqua_visual_frontend
{

namespace
{

struct KalibrCamera
{
  cv::Matx33d K;
  cv::Vec4d D;
  cv::Size size;
};

KalibrCamera load_camera(const YAML::Node & cam, const std::string & name)
{
  if (!cam) {
    throw std::invalid_argument("camchain has no entry " + name);
  }
  if (cam["camera_model"].as<std::string>() != "pinhole" ||
    cam["distortion_model"].as<std::string>() != "equidistant")
  {
    throw std::invalid_argument(name + ": only pinhole-equidistant cameras are supported");
  }
  const auto k = cam["intrinsics"].as<std::vector<double>>();
  const auto d = cam["distortion_coeffs"].as<std::vector<double>>();
  const auto r = cam["resolution"].as<std::vector<int>>();
  if (k.size() != 4 || d.size() != 4 || r.size() != 2) {
    throw std::invalid_argument(name + ": malformed intrinsics/distortion/resolution");
  }
  KalibrCamera out;
  out.K = cv::Matx33d(k[0], 0, k[2], 0, k[1], k[3], 0, 0, 1);
  out.D = cv::Vec4d(d[0], d[1], d[2], d[3]);
  out.size = cv::Size(r[0], r[1]);
  return out;
}

cv::Matx44d load_transform(const YAML::Node & rows)
{
  cv::Matx44d T;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      T(i, j) = rows[i][j].as<double>();
    }
  }
  return T;
}

}  // namespace

StereoRectifier::StereoRectifier(
  const std::string & camchain_path, const std::string & left_name,
  const std::string & right_name, double balance)
{
  const YAML::Node chain = YAML::LoadFile(camchain_path);
  const KalibrCamera left = load_camera(chain[left_name], left_name);
  const KalibrCamera right = load_camera(chain[right_name], right_name);
  if (left.size != right.size) {
    throw std::invalid_argument("stereo cameras must share a resolution");
  }
  // Kalibr stores T_cn_cnm1 on the second camera: p_cam1 = T * p_cam0.
  cv::Matx44d T_r_l;
  if (chain[right_name]["T_cn_cnm1"]) {
    T_r_l = load_transform(chain[right_name]["T_cn_cnm1"]);
  } else if (chain[left_name]["T_cn_cnm1"]) {
    T_r_l = load_transform(chain[left_name]["T_cn_cnm1"]).inv();
  } else {
    throw std::invalid_argument("camchain has no T_cn_cnm1 between the stereo cameras");
  }
  const cv::Matx33d R = T_r_l.get_minor<3, 3>(0, 0);
  const cv::Vec3d t(T_r_l(0, 3), T_r_l(1, 3), T_r_l(2, 3));
  // With the right camera at +x of the left one, p_right = R p_left + t has
  // t_x < 0. Otherwise the configured left camera is really on the right.
  if (t[0] >= 0.0) {
    throw std::invalid_argument(
            left_name + " is not the left camera of the pair (swap left/right)");
  }

  size_ = left.size;
  K_left_ = left.K;
  K_right_ = right.K;
  D_left_ = left.D;
  D_right_ = right.D;
  cv::Mat R_l, R_r, Q;
  cv::fisheye::stereoRectify(
    cv::Mat(left.K), cv::Mat(left.D), cv::Mat(right.K), cv::Mat(right.D), size_, cv::Mat(R),
    cv::Mat(t), R_l, R_r, P_left_, P_right_, Q, cv::CALIB_ZERO_DISPARITY, size_, balance, 1.0);
  R_left_ = cv::Matx33d(R_l);
  R_right_ = cv::Matx33d(R_r);
  cv::fisheye::initUndistortRectifyMap(
    cv::Mat(left.K), cv::Mat(left.D), R_l, P_left_, size_, CV_16SC2, map_left_x_, map_left_y_);
  cv::fisheye::initUndistortRectifyMap(
    cv::Mat(right.K), cv::Mat(right.D), R_r, P_right_, size_, CV_16SC2, map_right_x_,
    map_right_y_);

  geometry_.fx = P_left_.at<double>(0, 0);
  geometry_.fy = P_left_.at<double>(1, 1);
  geometry_.cx = P_left_.at<double>(0, 2);
  geometry_.cy = P_left_.at<double>(1, 2);
  geometry_.baseline_m = -P_right_.at<double>(0, 3) / P_right_.at<double>(0, 0);
  geometry_.R_left = R_left_;
}

void StereoRectifier::rectify_left(const cv::Mat & raw, cv::Mat & out) const
{
  cv::remap(raw, out, map_left_x_, map_left_y_, cv::INTER_LINEAR);
}

void StereoRectifier::rectify_right(const cv::Mat & raw, cv::Mat & out) const
{
  cv::remap(raw, out, map_right_x_, map_right_y_, cv::INTER_LINEAR);
}

cv::Point2d StereoRectifier::rectify_point_left(const cv::Point2d & raw) const
{
  std::vector<cv::Point2d> in{raw}, out;
  cv::fisheye::undistortPoints(
    in, out, cv::Mat(K_left_), cv::Mat(D_left_), cv::Mat(R_left_), P_left_);
  return out.front();
}

cv::Point2d StereoRectifier::rectify_point_right(const cv::Point2d & raw) const
{
  std::vector<cv::Point2d> in{raw}, out;
  cv::fisheye::undistortPoints(
    in, out, cv::Mat(K_right_), cv::Mat(D_right_), cv::Mat(R_right_), P_right_);
  return out.front();
}

}  // namespace aqua_visual_frontend
