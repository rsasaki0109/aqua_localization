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
  EXPECT_EQ(accepted_gate.status, "accepted");

  aqua_sonar_loc::MatchResult rejected = accepted;
  rejected.candidate_to_current.translation().x() = 2.0;

  const auto rejected_gate = evaluator.evaluate(Eigen::Isometry3d::Identity(), rejected);
  EXPECT_FALSE(rejected_gate.accepted);
  EXPECT_EQ(rejected_gate.status, "translation correction exceeds gate");
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
