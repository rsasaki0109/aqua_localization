// SPDX-License-Identifier: Apache-2.0
#include "aqua_pose_graph/pose_graph.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <gtest/gtest.h>

using aqua_pose_graph::Keyframe;
using aqua_pose_graph::LoopConstraint;
using aqua_pose_graph::PoseGraph;
using aqua_pose_graph::PoseGraphConfig;

namespace {

Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() << x, y, z;
  return t;
}

Eigen::Matrix<double, 6, 6> tight_information()
{
  Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Zero();
  for (int i = 0; i < 3; ++i) {
    info(i, i) = 10000.0;          // 1 / (0.01 m)^2
  }
  for (int i = 3; i < 6; ++i) {
    info(i, i) = 100000.0;
  }
  return info;
}

}  // namespace

TEST(PoseGraph, FirstSampleSeedsAKeyframe)
{
  PoseGraphConfig cfg;
  PoseGraph graph(cfg);
  EXPECT_TRUE(graph.add_odometry_sample(
    0.0, translation(0.0, 0.0, 0.0),
    Eigen::Matrix<double, 6, 6>::Identity()));
  EXPECT_EQ(graph.keyframes().size(), 1u);
  EXPECT_EQ(graph.edge_count(), 0u);
}

TEST(PoseGraph, AddsKeyframeOnlyAfterTranslationThreshold)
{
  PoseGraphConfig cfg;
  cfg.keyframe_translation_m = 0.5;
  cfg.keyframe_rotation_rad = M_PI;  // disable rotation trigger
  PoseGraph graph(cfg);

  graph.add_odometry_sample(
    0.0, translation(0.0, 0.0, 0.0),
    Eigen::Matrix<double, 6, 6>::Identity());

  // 0.4 m: under threshold, no new keyframe.
  EXPECT_FALSE(graph.add_odometry_sample(
    0.1, translation(0.4, 0.0, 0.0),
    Eigen::Matrix<double, 6, 6>::Identity()));
  EXPECT_EQ(graph.keyframes().size(), 1u);

  // 0.6 m: new keyframe, edge created.
  EXPECT_TRUE(graph.add_odometry_sample(
    0.2, translation(0.6, 0.0, 0.0),
    Eigen::Matrix<double, 6, 6>::Identity()));
  EXPECT_EQ(graph.keyframes().size(), 2u);
  EXPECT_EQ(graph.edge_count(), 1u);
}

TEST(PoseGraph, OptimizeOnChainPreservesPoses)
{
  PoseGraphConfig cfg;
  cfg.keyframe_translation_m = 1.0;
  cfg.keyframe_rotation_rad = M_PI;
  cfg.optimize_every_n_keyframes = 0;
  PoseGraph graph(cfg);

  // Linear motion at 1m steps gives keyframes at (0,0,0),(1,0,0),(2,0,0),(3,0,0)
  for (int i = 0; i <= 3; ++i) {
    graph.add_odometry_sample(
      0.1 * i, translation(static_cast<double>(i), 0.0, 0.0),
      Eigen::Matrix<double, 6, 6>::Identity());
  }
  ASSERT_EQ(graph.keyframes().size(), 4u);

  graph.optimize();

  // Without loop closures the chain optimization is a no-op against the
  // odometry-only initialisation. Verify keyframe poses match the input.
  Eigen::Isometry3d pose;
  for (int i = 0; i <= 3; ++i) {
    ASSERT_TRUE(graph.keyframe_pose(static_cast<size_t>(i), &pose));
    EXPECT_NEAR(pose.translation().x(), static_cast<double>(i), 1e-6);
    EXPECT_NEAR(pose.translation().y(), 0.0, 1e-6);
    EXPECT_NEAR(pose.translation().z(), 0.0, 1e-6);
  }
}

TEST(PoseGraph, LoopConstraintBendsTrajectory)
{
  PoseGraphConfig cfg;
  cfg.keyframe_translation_m = 1.0;
  cfg.keyframe_rotation_rad = M_PI;
  cfg.optimize_every_n_keyframes = 0;
  PoseGraph graph(cfg);

  // 5-keyframe chain marching forward 1m at a time.
  for (int i = 0; i <= 4; ++i) {
    graph.add_odometry_sample(
      0.1 * i, translation(static_cast<double>(i), 0.0, 0.0),
      Eigen::Matrix<double, 6, 6>::Identity());
  }
  ASSERT_EQ(graph.keyframes().size(), 5u);

  // Insert a loop constraint claiming kf 4 is back at the same place as kf 0
  // with high information weight (cm-tight). Optimisation should pull the
  // tail back toward the head.
  LoopConstraint loop;
  loop.from_id = 0;
  loop.to_id = 4;
  loop.relative_pose = Eigen::Isometry3d::Identity();
  loop.information = tight_information();
  ASSERT_TRUE(graph.add_loop_constraint(loop));

  graph.optimize();

  Eigen::Isometry3d kf0;
  Eigen::Isometry3d kf4;
  ASSERT_TRUE(graph.keyframe_pose(0, &kf0));
  ASSERT_TRUE(graph.keyframe_pose(4, &kf4));
  // kf0 is fixed (the seed), so kf4 should have collapsed close to it under
  // the loop constraint. Tolerance is loose because the chain edges still
  // resist with their default information.
  EXPECT_LT((kf4.translation() - kf0.translation()).norm(), 1.0);
}

TEST(PoseGraph, ResetClearsState)
{
  PoseGraphConfig cfg;
  PoseGraph graph(cfg);
  graph.add_odometry_sample(
    0.0, translation(0.0, 0.0, 0.0),
    Eigen::Matrix<double, 6, 6>::Identity());
  graph.add_odometry_sample(
    0.1, translation(1.0, 0.0, 0.0),
    Eigen::Matrix<double, 6, 6>::Identity());
  ASSERT_EQ(graph.keyframes().size(), 2u);
  graph.reset();
  EXPECT_EQ(graph.keyframes().size(), 0u);
  EXPECT_EQ(graph.edge_count(), 0u);
}
