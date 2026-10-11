// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "aqua_visual_frontend/stereo_rectifier.hpp"

namespace
{

using aqua_visual_frontend::StereoRectifier;

// Kalibr-style pair laid out like the NTNU Alphasense rig: cam1 is the left
// camera (cam0 sits at +x of it, so T_cn_cnm1 has +x translation), slightly
// rotated against each other, both equidistant.
const cv::Matx33d kRot = [] {
    cv::Matx33d R;
    cv::Rodrigues(cv::Vec3d(0.01, -0.02, 0.005), R);
    return R;
  }();
const cv::Vec3d kTrans(0.11, 0.002, -0.001);

std::string write_camchain()
{
  const std::string path = ::testing::TempDir() + "camchain_test.yaml";
  std::ofstream out(path);
  out.precision(17);
  out << "cam0:\n"
      << "  camera_model: pinhole\n  distortion_model: equidistant\n"
      << "  intrinsics: [350.0, 351.0, 360.0, 270.0]\n"
      << "  distortion_coeffs: [-0.02, 0.01, -0.005, 0.001]\n"
      << "  resolution: [720, 540]\n";
  out << "cam1:\n"
      << "  camera_model: pinhole\n  distortion_model: equidistant\n"
      << "  intrinsics: [352.0, 352.5, 355.0, 265.0]\n"
      << "  distortion_coeffs: [-0.015, 0.008, -0.004, 0.0005]\n"
      << "  resolution: [720, 540]\n"
      << "  T_cn_cnm1:\n";
  for (int i = 0; i < 3; ++i) {
    out << "  - [" << kRot(i, 0) << ", " << kRot(i, 1) << ", " << kRot(i, 2) << ", "
        << kTrans[i] << "]\n";
  }
  out << "  - [0.0, 0.0, 0.0, 1.0]\n";
  return path;
}

// Raw pixel of a point given in the left (cam1) raw frame.
struct RawViews
{
  cv::Point2d left;
  cv::Point2d right;
};

RawViews project(const cv::Vec3d & p_left)
{
  // p_cam1 = R p_cam0 + t  =>  p_cam0 = R^T (p_cam1 - t).
  const cv::Matx33d R_rl = kRot.t();
  const cv::Vec3d t_rl = -(kRot.t() * kTrans);
  cv::Vec3d r_rl;
  cv::Rodrigues(R_rl, r_rl);
  const cv::Matx33d K0(350.0, 0, 360.0, 0, 351.0, 270.0, 0, 0, 1);
  const cv::Matx33d K1(352.0, 0, 355.0, 0, 352.5, 265.0, 0, 0, 1);
  const cv::Vec4d D0(-0.02, 0.01, -0.005, 0.001);
  const cv::Vec4d D1(-0.015, 0.008, -0.004, 0.0005);
  std::vector<cv::Point3d> obj{cv::Point3d(p_left[0], p_left[1], p_left[2])};
  std::vector<cv::Point2d> left, right;
  cv::fisheye::projectPoints(obj, left, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K1, D1);
  cv::fisheye::projectPoints(obj, right, r_rl, t_rl, K0, D0);
  return {left.front(), right.front()};
}

TEST(StereoRectifier, RowsAlignAndDisparityMatchesDepth)
{
  const StereoRectifier rect(write_camchain(), "cam1", "cam0");
  const auto & g = rect.geometry();
  ASSERT_GT(g.fx, 100.0);
  // Baseline is the norm of the camera offset.
  EXPECT_NEAR(g.baseline_m, cv::norm(kTrans), 1e-6);

  int checked = 0;
  for (double x = -1.5; x <= 1.5; x += 0.5) {
    for (double y = -1.0; y <= 1.0; y += 0.5) {
      for (double z : {2.0, 5.0, 12.0}) {
        const cv::Vec3d p(x, y, z);
        const RawViews raw = project(p);
        const cv::Point2d l = rect.rectify_point_left(raw.left);
        const cv::Point2d r = rect.rectify_point_right(raw.right);
        // Rectified rows coincide.
        EXPECT_NEAR(l.y, r.y, 1e-3) << x << " " << y << " " << z;
        // Disparity is fx * b / Z in the rectified left frame.
        const double z_rect = (g.R_left * p)[2];
        EXPECT_NEAR(l.x - r.x, g.fx * g.baseline_m / z_rect, 1e-3);
        // And the rectified left pixel is the pinhole projection.
        const cv::Vec3d q = g.R_left * p;
        EXPECT_NEAR(l.x, g.fx * q[0] / q[2] + g.cx, 1e-3);
        EXPECT_NEAR(l.y, g.fy * q[1] / q[2] + g.cy, 1e-3);
        ++checked;
      }
    }
  }
  EXPECT_EQ(checked, 7 * 5 * 3);
}

TEST(StereoRectifier, RemappedImagePutsBlobAtRectifiedPixel)
{
  const StereoRectifier rect(write_camchain(), "cam1", "cam0");
  const RawViews raw = project(cv::Vec3d(0.4, -0.2, 3.0));
  cv::Mat img(rect.size(), CV_8UC1, cv::Scalar(0));
  cv::circle(img, cv::Point(cvRound(raw.left.x), cvRound(raw.left.y)), 3, cv::Scalar(255), -1);
  cv::GaussianBlur(img, img, cv::Size(9, 9), 2.0);
  cv::Mat out;
  rect.rectify_left(img, out);
  ASSERT_EQ(out.size(), rect.size());
  cv::Point max_loc;
  cv::minMaxLoc(out, nullptr, nullptr, nullptr, &max_loc);
  const cv::Point2d expected = rect.rectify_point_left(
    cv::Point2d(cvRound(raw.left.x), cvRound(raw.left.y)));
  EXPECT_NEAR(max_loc.x, expected.x, 1.5);
  EXPECT_NEAR(max_loc.y, expected.y, 1.5);
}

TEST(StereoRectifier, RejectsSwappedPair)
{
  EXPECT_THROW(StereoRectifier(write_camchain(), "cam0", "cam1"), std::invalid_argument);
}

TEST(StereoRectifier, RejectsMissingCamera)
{
  EXPECT_THROW(StereoRectifier(write_camchain(), "cam1", "cam2"), std::invalid_argument);
}

}  // namespace
