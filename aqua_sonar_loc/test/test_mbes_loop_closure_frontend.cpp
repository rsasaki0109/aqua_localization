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

TEST(PoseInterpolator, InterpolatesTranslationAndRotation)
{
  aqua_sonar_loc::PoseInterpolator history(100.0, 0.5);
  Eigen::Isometry3d a = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d b = Eigen::Isometry3d::Identity();
  b.translation() = Eigen::Vector3d(2.0, -4.0, 1.0);
  b.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  history.add(10.0, a);
  history.add(10.2, b);
  history.add(10.1, b);  // out of order: ignored

  const auto mid = history.at(10.05);
  ASSERT_TRUE(mid.has_value());
  EXPECT_TRUE(mid->translation().isApprox(Eigen::Vector3d(0.5, -1.0, 0.25), 1.0e-12));
  EXPECT_NEAR(Eigen::AngleAxisd(mid->linear()).angle(), 0.1, 1.0e-9);
  ASSERT_TRUE(history.at(10.2).has_value());
  EXPECT_FALSE(history.at(9.99).has_value());
  EXPECT_FALSE(history.at(10.21).has_value());
}

TEST(PoseInterpolator, RefusesGapsAndDropsOldHistory)
{
  aqua_sonar_loc::PoseInterpolator history(1.0, 0.5);
  history.add(0.0, Eigen::Isometry3d::Identity());
  history.add(2.0, Eigen::Isometry3d::Identity());  // evicts t=0 (older than 1 s)
  EXPECT_EQ(history.size(), 1U);
  history.add(2.4, Eigen::Isometry3d::Identity());
  EXPECT_TRUE(history.at(2.2).has_value());
  history.add(3.4, Eigen::Isometry3d::Identity());  // evicts t=2.0
  EXPECT_FALSE(history.at(2.2).has_value());
  EXPECT_FALSE(history.at(3.0).has_value());  // 1 s gap > max_gap_s
}

TEST(TransformCloud, MovesPointsAndDropsInvalidReturns)
{
  aqua_sonar_loc::PointCloud cloud;
  cloud.push_back(pcl::PointXYZ(0.0F, 3.0F, 8.0F));
  cloud.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));  // invalid beam
  cloud.push_back(pcl::PointXYZ(std::nanf(""), 1.0F, 1.0F));
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  const auto out = aqua_sonar_loc::transform_cloud(cloud, T, 0.5);
  ASSERT_EQ(out->size(), 1U);
  EXPECT_NEAR(out->at(0).x, 1.0F, 1.0e-5F);
  EXPECT_NEAR(out->at(0).y, -3.0F, 1.0e-5F);
  EXPECT_NEAR(out->at(0).z, -8.0F, 1.0e-5F);
}

TEST(TransformCloud, DropsOuterBeamsBeyondAcrossTrackRatio)
{
  aqua_sonar_loc::PointCloud cloud;
  cloud.push_back(pcl::PointXYZ(0.0F, 5.0F, 10.0F));   // 26.6 deg: kept
  cloud.push_back(pcl::PointXYZ(0.0F, -15.0F, 10.0F));  // 56.3 deg: dropped
  const auto out = aqua_sonar_loc::transform_cloud(cloud, Eigen::Isometry3d::Identity(), 0.5, 1.0);
  ASSERT_EQ(out->size(), 1U);
  EXPECT_NEAR(out->at(0).y, 5.0F, 1.0e-6F);
  EXPECT_EQ(aqua_sonar_loc::transform_cloud(cloud, Eigen::Isometry3d::Identity(), 0.5)->size(), 2U);
}

namespace
{

double seafloor(double x, double y, bool flat)
{
  if (flat) {
    return -20.0 + 0.02 * x;  // a plane: no horizontal information
  }
  return -20.0 + 2.0 * std::sin(x / 7.0) * std::cos(y / 5.0) + 0.7 * std::sin(x / 3.1 + y / 4.3) +
         3.0 * std::exp(-((x - 4.0) * (x - 4.0) + (y + 3.0) * (y + 3.0)) / 30.0);
}

// Points of a swath patch sampled in the world frame, returned in `frame`.
aqua_sonar_loc::PointCloud swath(
  double x0, double x1, double y0, double y1, const Eigen::Isometry3d & frame, bool flat)
{
  aqua_sonar_loc::PointCloud cloud;
  const Eigen::Isometry3d world_to_frame = frame.inverse();
  for (double x = x0; x <= x1; x += 0.25) {
    for (double y = y0; y <= y1; y += 0.25) {
      const Eigen::Vector3d p = world_to_frame * Eigen::Vector3d(x, y, seafloor(x, y, flat));
      cloud.push_back(pcl::PointXYZ(
          static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())));
    }
  }
  return cloud;
}

Eigen::Isometry3d planar_pose(double x, double y, double yaw)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  pose.translation() << x, y, 0.0;
  return pose;
}

}  // namespace

TEST(MbesLoopClosureFrontendTest, CoarseAlignRecoversCrossingFromDriftedGuess)
{
  // Candidate pass along x, current pass crossing it along y.
  const Eigen::Isometry3d candidate_pose = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d current_pose = planar_pose(5.0, 3.0, 0.3);
  const auto candidate = swath(-30.0, 30.0, -10.0, 10.0, candidate_pose, false);
  const auto current = swath(-10.0, 10.0, -30.0, 30.0, current_pose, false);
  const Eigen::Isometry3d truth = candidate_pose.inverse() * current_pose;
  // Dead-reckoned guess: 0.2 rad and ~10 m off.
  const Eigen::Isometry3d guess = planar_pose(8.0, -6.0, -0.2) * truth;

  aqua_sonar_loc::CoarseAlignOptions options;
  options.enabled = true;
  // A perpendicular crossing covers a third of each swath.
  options.min_overlap_fraction = 0.2;
  const auto result = aqua_sonar_loc::coarse_align_heightmaps(
    candidate, current, candidate_pose, guess, options);
  ASSERT_TRUE(result.accepted) << result.status;
  const Eigen::Isometry3d error = truth.inverse() * result.candidate_to_current;
  EXPECT_LT(Eigen::AngleAxisd(error.linear()).angle(), 0.05);
  EXPECT_LT(error.translation().head<2>().norm(), 2.0);
  EXPECT_LT(std::abs(error.translation().z()), 0.3);
  EXPECT_GT(result.relief_m, 0.5);
  EXPECT_GT(result.distinctiveness, 1.2);
}

TEST(MbesLoopClosureFrontendTest, CoarseAlignRejectsFlatSeafloor)
{
  const Eigen::Isometry3d candidate_pose = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d current_pose = planar_pose(5.0, 3.0, 0.3);
  const auto candidate = swath(-30.0, 30.0, -10.0, 10.0, candidate_pose, true);
  const auto current = swath(-10.0, 10.0, -30.0, 30.0, current_pose, true);
  const Eigen::Isometry3d guess = candidate_pose.inverse() * current_pose;

  aqua_sonar_loc::CoarseAlignOptions options;
  options.enabled = true;
  const auto result = aqua_sonar_loc::coarse_align_heightmaps(
    candidate, current, candidate_pose, guess, options);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, "coarse: insufficient relief");
}

TEST(MbesLoopClosureFrontendTest, RegistrationUsesPoseOfCurrentInCandidateFrame)
{
  // Two overlapping passes over the same terrain; the guess is the
  // dead-reckoned pose of the current submap in the candidate frame, 1 m and
  // 0.03 rad off. The result must be that same relative pose, corrected.
  aqua_sonar_loc::Submap candidate;
  candidate.id = 1;
  candidate.pose = Eigen::Isometry3d::Identity();
  aqua_sonar_loc::Submap current;
  current.id = 2;
  current.pose = planar_pose(6.0, 2.0, 0.15);
  candidate.cloud = std::make_shared<aqua_sonar_loc::PointCloud>(
    swath(-20.0, 20.0, -10.0, 10.0, candidate.pose, false));
  current.cloud = std::make_shared<aqua_sonar_loc::PointCloud>(
    swath(-14.0, 26.0, -8.0, 12.0, current.pose, false));
  const Eigen::Isometry3d truth = candidate.pose.inverse() * current.pose;
  const Eigen::Isometry3d guess = planar_pose(0.8, -0.6, 0.03) * truth;

  aqua_sonar_loc::RegistrationOptions options;
  const aqua_sonar_loc::RegistrationPipeline pipeline(options);
  const auto result = pipeline.match(candidate, current, guess);
  ASSERT_TRUE(result.success) << result.status;
  const Eigen::Isometry3d error = truth.inverse() * result.candidate_to_current;
  EXPECT_LT(error.translation().norm(), 0.3);
  EXPECT_LT(Eigen::AngleAxisd(error.linear()).angle(), 0.01);

  aqua_sonar_loc::GateOptions gate_options;
  gate_options.max_fitness_score = 0.0;
  const aqua_sonar_loc::LoopGateEvaluator gate(gate_options);
  const auto verdict = gate.evaluate(guess, result);
  // The correction undoes the injected guess error.
  EXPECT_NEAR(verdict.correction_translation_m, (guess.inverse() * truth).translation().norm(), 0.3);
}
