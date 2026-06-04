#include <cmath>
#include <deque>
#include <memory>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "aqua_sonar_loc/mbes_loop_closure_frontend.hpp"

namespace
{

aqua_sonar_loc::Submap make_submap(std::uint32_t id, double x)
{
  aqua_sonar_loc::Submap submap;
  submap.id = id;
  submap.pose.translation().x() = x;
  submap.cloud = std::make_shared<aqua_sonar_loc::PointCloud>();
  return submap;
}

aqua_sonar_loc::PointCloud make_cloud(int points)
{
  aqua_sonar_loc::PointCloud cloud;
  for (int i = 0; i < points; ++i) {
    cloud.push_back(pcl::PointXYZ(static_cast<float>(i), 0.0F, 0.0F));
  }
  return cloud;
}

aqua_sonar_loc::PointCloud make_shifted_cloud(int points, float x_offset)
{
  aqua_sonar_loc::PointCloud cloud;
  for (int i = 0; i < points; ++i) {
    cloud.push_back(pcl::PointXYZ(static_cast<float>(i) + x_offset, 0.0F, 0.0F));
  }
  return cloud;
}

aqua_sonar_loc::Submap make_described_submap(
  std::uint32_t id,
  const aqua_sonar_loc::PointCloud & cloud)
{
  auto submap = make_submap(id, 0.0);
  *submap.cloud = cloud;
  submap.descriptor = aqua_sonar_loc::describe_cloud(*submap.cloud);
  return submap;
}

}  // namespace

TEST(MbesLoopClosureFrontendTest, CandidateSelectorFiltersAndRanksByDistance)
{
  aqua_sonar_loc::CandidateSelectionOptions options;
  options.min_keyframe_separation = 3;
  options.max_distance_m = 10.0;
  aqua_sonar_loc::LoopCandidateSelector selector(options);

  std::deque<aqua_sonar_loc::Submap> history;
  history.push_back(make_submap(0, 5.0));
  history.push_back(make_submap(1, 2.0));
  history.push_back(make_submap(2, 20.0));
  history.push_back(make_submap(9, 1.0));
  const auto current = make_submap(10, 0.0);

  const auto candidates = selector.ranked_candidates(history, current);

  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_EQ(candidates[0].id, 1U);
  EXPECT_EQ(candidates[1].id, 0U);
}

TEST(MbesLoopClosureFrontendTest, CandidateSelectorCanRankByDescriptorSimilarity)
{
  aqua_sonar_loc::CandidateSelectionOptions options;
  options.min_keyframe_separation = 0;
  options.max_distance_m = 10.0;
  options.descriptor_weight = 1.0;
  aqua_sonar_loc::LoopCandidateSelector selector(options);

  auto near_mismatch = make_described_submap(0, make_shifted_cloud(10, 100.0F));
  near_mismatch.pose.translation().x() = 1.0;
  auto far_match = make_described_submap(1, make_cloud(10));
  far_match.pose.translation().x() = 8.0;

  std::deque<aqua_sonar_loc::Submap> history;
  history.push_back(near_mismatch);
  history.push_back(far_match);
  auto current = make_described_submap(10, make_cloud(10));

  const auto candidates = selector.ranked_candidates(history, current);

  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_EQ(candidates[0].id, 1U);
  EXPECT_EQ(candidates[1].id, 0U);
}

TEST(MbesLoopClosureFrontendTest, GateEvaluatorKeepsExistingAcceptanceRules)
{
  aqua_sonar_loc::GateOptions options;
  options.max_fitness_score = 1.0;
  options.max_correction_translation_m = 1.0;
  options.max_correction_rotation_rad = 0.5;
  aqua_sonar_loc::LoopGateEvaluator evaluator(options);

  aqua_sonar_loc::MatchResult accepted;
  accepted.success = true;
  accepted.converged = true;
  accepted.fitness = 0.2;
  accepted.candidate_to_current = Eigen::Isometry3d::Identity();

  const auto accepted_gate = evaluator.evaluate(Eigen::Isometry3d::Identity(), accepted);
  EXPECT_TRUE(accepted_gate.accepted);
  EXPECT_TRUE(accepted_gate.correction_pose_valid);
  EXPECT_NEAR(accepted_gate.correction.translation().norm(), 0.0, 1e-9);
  EXPECT_EQ(accepted_gate.status, "accepted");

  aqua_sonar_loc::MatchResult rejected = accepted;
  rejected.candidate_to_current.translation().x() = 2.0;

  const auto rejected_gate = evaluator.evaluate(Eigen::Isometry3d::Identity(), rejected);
  EXPECT_FALSE(rejected_gate.accepted);
  EXPECT_TRUE(rejected_gate.correction_pose_valid);
  EXPECT_NEAR(rejected_gate.correction.translation().x(), 2.0, 1e-9);
  EXPECT_EQ(rejected_gate.status, "translation correction exceeds gate");
}

TEST(MbesLoopClosureFrontendTest, GateEvaluatorRejectsHighRotationOnShortPlanViewEdges)
{
  aqua_sonar_loc::GateOptions options;
  options.max_fitness_score = 1.0;
  options.max_correction_translation_m = 1.0;
  options.max_correction_rotation_rad = 0.5;
  options.min_plan_view_separation_m = 1.0;
  options.max_short_plan_view_rotation_rad = 0.2;
  aqua_sonar_loc::LoopGateEvaluator evaluator(options);

  Eigen::Isometry3d short_guess = Eigen::Isometry3d::Identity();
  short_guess.translation().x() = 0.5;
  aqua_sonar_loc::MatchResult short_rotated;
  short_rotated.success = true;
  short_rotated.converged = true;
  short_rotated.fitness = 0.2;
  short_rotated.candidate_to_current = short_guess;
  short_rotated.candidate_to_current.linear() =
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const auto short_gate = evaluator.evaluate(short_guess, short_rotated);

  EXPECT_FALSE(short_gate.accepted);
  EXPECT_TRUE(short_gate.correction_pose_valid);
  EXPECT_NEAR(short_gate.correction_rotation_rad, 0.3, 1.0e-9);
  EXPECT_EQ(short_gate.status, "short plan-view rotation gate rejected");

  Eigen::Isometry3d long_guess = Eigen::Isometry3d::Identity();
  long_guess.translation().x() = 2.0;
  auto long_rotated = short_rotated;
  long_rotated.candidate_to_current = long_guess;
  long_rotated.candidate_to_current.linear() =
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const auto long_gate = evaluator.evaluate(long_guess, long_rotated);

  EXPECT_TRUE(long_gate.accepted);
  EXPECT_EQ(long_gate.status, "accepted");
}

TEST(MbesLoopClosureFrontendTest, DescribeCloudComputesCentroidExtentAndPointCount)
{
  aqua_sonar_loc::PointCloud cloud;
  cloud.push_back(pcl::PointXYZ(-1.0F, 0.0F, 2.0F));
  cloud.push_back(pcl::PointXYZ(3.0F, 2.0F, -1.0F));
  cloud.push_back(pcl::PointXYZ(1.0F, 4.0F, 5.0F));

  const auto descriptor = aqua_sonar_loc::describe_cloud(cloud);

  EXPECT_TRUE(descriptor.valid);
  EXPECT_EQ(descriptor.point_count, 3U);
  EXPECT_NEAR(descriptor.centroid.x(), 1.0, 1e-9);
  EXPECT_NEAR(descriptor.centroid.y(), 2.0, 1e-9);
  EXPECT_NEAR(descriptor.centroid.z(), 2.0, 1e-9);
  EXPECT_NEAR(descriptor.extent.x(), 4.0, 1e-9);
  EXPECT_NEAR(descriptor.extent.y(), 4.0, 1e-9);
  EXPECT_NEAR(descriptor.extent.z(), 6.0, 1e-9);
}

TEST(MbesLoopClosureFrontendTest, DescriptorGateRejectsMismatchedSubmapShapes)
{
  aqua_sonar_loc::DescriptorGateOptions options;
  options.max_centroid_distance_m = 1.0;
  options.max_extent_ratio = 2.0;
  options.min_point_count_ratio = 0.5;
  aqua_sonar_loc::DescriptorGateEvaluator evaluator(options);

  auto candidate = make_described_submap(1, make_cloud(10));
  auto current = make_described_submap(2, make_cloud(10));

  auto gate = evaluator.evaluate(candidate, current);
  EXPECT_TRUE(gate.accepted);

  current.descriptor.centroid.x() = 3.0;
  gate = evaluator.evaluate(candidate, current);
  EXPECT_FALSE(gate.accepted);
  EXPECT_EQ(gate.status, "descriptor gate rejected");

  current = make_described_submap(2, make_cloud(3));
  gate = evaluator.evaluate(candidate, current);
  EXPECT_FALSE(gate.accepted);
  EXPECT_EQ(gate.status, "descriptor gate rejected");

  current = candidate;
  current.descriptor.extent.x() = 30.0;
  gate = evaluator.evaluate(candidate, current);
  EXPECT_FALSE(gate.accepted);
  EXPECT_EQ(gate.status, "descriptor gate rejected");
}

TEST(MbesLoopClosureFrontendTest, DescriptorGateCanBeDisabled)
{
  aqua_sonar_loc::DescriptorGateEvaluator evaluator(aqua_sonar_loc::DescriptorGateOptions{});

  auto candidate = make_described_submap(1, make_cloud(10));
  auto current = make_described_submap(2, make_cloud(1));
  current.descriptor.centroid.x() = 100.0;
  current.descriptor.extent.x() = 100.0;

  const auto gate = evaluator.evaluate(candidate, current);

  EXPECT_TRUE(gate.accepted);
  EXPECT_EQ(gate.status, "descriptor gate accepted");
  EXPECT_TRUE(std::isfinite(gate.descriptor_centroid_distance_m));
  EXPECT_TRUE(std::isfinite(gate.descriptor_extent_ratio));
  EXPECT_TRUE(std::isfinite(gate.descriptor_point_count_ratio));
}

TEST(MbesLoopClosureFrontendTest, SubmapManagerCapsPointsAndHistory)
{
  aqua_sonar_loc::SubmapManagerOptions options;
  options.max_submaps = 1;
  options.min_points_per_submap = 2;
  options.max_points_per_submap = 3;
  options.voxel_leaf_m = 0.0;
  aqua_sonar_loc::SubmapManager manager(options);

  manager.start_submap(1, rclcpp::Time(1, 0, RCL_ROS_TIME), Eigen::Isometry3d::Identity());
  manager.append_points(make_cloud(5));
  auto result = manager.finalize_current();
  ASSERT_EQ(result.status, aqua_sonar_loc::FinalizeSubmapStatus::Ready);
  EXPECT_EQ(result.raw_points, 3U);
  EXPECT_EQ(result.final_points, 3U);
  manager.add_finalized_submap(result.submap);

  manager.start_submap(2, rclcpp::Time(2, 0, RCL_ROS_TIME), Eigen::Isometry3d::Identity());
  manager.append_points(make_cloud(2));
  result = manager.finalize_current();
  ASSERT_EQ(result.status, aqua_sonar_loc::FinalizeSubmapStatus::Ready);
  manager.add_finalized_submap(result.submap);

  ASSERT_EQ(manager.submaps().size(), 1U);
  EXPECT_EQ(manager.submaps().front().id, 2U);
}

TEST(MbesLoopClosureFrontendTest, AcceptedLoopTrackerSuppressesNearbyAcceptedPairs)
{
  aqua_sonar_loc::LoopSuppressionOptions options;
  options.min_repeat_keyframe_gap = 3;
  aqua_sonar_loc::AcceptedLoopTracker tracker(options);

  EXPECT_FALSE(tracker.is_suppressed(10, 30));
  tracker.record(10, 30);

  EXPECT_TRUE(tracker.is_suppressed(12, 32));
  EXPECT_TRUE(tracker.is_suppressed(7, 27));
  EXPECT_FALSE(tracker.is_suppressed(14, 32));
  EXPECT_FALSE(tracker.is_suppressed(12, 34));
  ASSERT_EQ(tracker.accepted_loops().size(), 1U);
}

TEST(MbesLoopClosureFrontendTest, AcceptedLoopTrackerCanBeDisabled)
{
  aqua_sonar_loc::LoopSuppressionOptions options;
  options.min_repeat_keyframe_gap = 0;
  aqua_sonar_loc::AcceptedLoopTracker tracker(options);

  tracker.record(10, 30);

  EXPECT_FALSE(tracker.is_suppressed(10, 30));
  EXPECT_FALSE(tracker.is_suppressed(11, 31));
}

TEST(MbesLoopClosureFrontendTest, AcceptedLoopTrackerRejectsInconsistentCorrections)
{
  aqua_sonar_loc::LoopSuppressionOptions options;
  options.max_consistency_translation_delta_m = 1.0;
  options.max_consistency_rotation_delta_rad = 0.25;
  aqua_sonar_loc::AcceptedLoopTracker tracker(options);

  tracker.record(10, 30, Eigen::Isometry3d::Identity());

  Eigen::Isometry3d consistent = Eigen::Isometry3d::Identity();
  consistent.translation().x() = 0.5;
  EXPECT_TRUE(tracker.is_consistent(consistent));

  Eigen::Isometry3d inconsistent_translation = Eigen::Isometry3d::Identity();
  inconsistent_translation.translation().x() = 2.0;
  EXPECT_FALSE(tracker.is_consistent(inconsistent_translation));

  Eigen::Isometry3d inconsistent_rotation = Eigen::Isometry3d::Identity();
  inconsistent_rotation.linear() =
    Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_FALSE(tracker.is_consistent(inconsistent_rotation));
}

TEST(MbesLoopClosureFrontendTest, AcceptedLoopTrackerCanRequireMultipleConsistencySupports)
{
  aqua_sonar_loc::LoopSuppressionOptions options;
  options.max_consistency_translation_delta_m = 0.5;
  options.max_consistency_rotation_delta_rad = 0.25;
  options.min_consistency_support_count = 2;
  aqua_sonar_loc::AcceptedLoopTracker tracker(options);

  Eigen::Isometry3d first = Eigen::Isometry3d::Identity();
  tracker.record(10, 30, first);

  Eigen::Isometry3d bootstrap = Eigen::Isometry3d::Identity();
  bootstrap.translation().x() = 0.25;
  EXPECT_TRUE(tracker.is_consistent(bootstrap));

  Eigen::Isometry3d second = Eigen::Isometry3d::Identity();
  second.translation().x() = 0.4;
  tracker.record(20, 40, second);

  Eigen::Isometry3d supported_by_one = Eigen::Isometry3d::Identity();
  supported_by_one.translation().x() = 0.9;
  EXPECT_FALSE(tracker.is_consistent(supported_by_one));
  const auto supported_by_one_check = tracker.check_consistency(supported_by_one);
  EXPECT_FALSE(supported_by_one_check.consistent);
  EXPECT_EQ(supported_by_one_check.support_count, 1U);
  EXPECT_EQ(supported_by_one_check.required_support_count, 2U);
  EXPECT_NEAR(supported_by_one_check.nearest_translation_delta_m, 0.5, 1.0e-9);

  Eigen::Isometry3d supported_by_two = Eigen::Isometry3d::Identity();
  supported_by_two.translation().x() = 0.2;
  EXPECT_TRUE(tracker.is_consistent(supported_by_two));
  const auto supported_by_two_check = tracker.check_consistency(supported_by_two);
  EXPECT_TRUE(supported_by_two_check.consistent);
  EXPECT_EQ(supported_by_two_check.support_count, 2U);
  EXPECT_EQ(supported_by_two_check.required_support_count, 2U);
  EXPECT_NEAR(supported_by_two_check.nearest_translation_delta_m, 0.2, 1.0e-9);
}
