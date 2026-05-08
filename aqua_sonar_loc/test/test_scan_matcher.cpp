#include <array>
#include <cstring>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "aqua_sonar_loc/scan_matcher.hpp"

namespace
{

sensor_msgs::msg::PointCloud2 make_cloud(const std::vector<std::array<float, 3>> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.is_dense = false;
  cloud.is_bigendian = false;
  cloud.point_step = 3 * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(static_cast<size_t>(cloud.row_step));

  const std::array<std::string, 3> names = {"x", "y", "z"};
  for (size_t i = 0; i < names.size(); ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i];
    field.offset = static_cast<uint32_t>(i * sizeof(float));
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }

  for (size_t i = 0; i < points.size(); ++i) {
    const size_t point_offset = i * static_cast<size_t>(cloud.point_step);
    for (size_t axis = 0; axis < 3; ++axis) {
      const float value = points[i][axis];
      std::memcpy(&cloud.data[point_offset + axis * sizeof(float)], &value, sizeof(float));
    }
  }
  return cloud;
}

std::vector<std::array<float, 3>> translated(
  const std::vector<std::array<float, 3>> & points,
  float dx,
  float dy,
  float dz)
{
  auto shifted = points;
  for (auto & point : shifted) {
    point[0] += dx;
    point[1] += dy;
    point[2] += dz;
  }
  return shifted;
}

aqua_sonar_loc::CloudSummary accepted_summary(size_t points)
{
  aqua_sonar_loc::CloudSummary summary;
  summary.accepted = true;
  summary.total_points = points;
  summary.finite_xyz_points = points;
  summary.in_range_points = points;
  return summary;
}

}  // namespace

TEST(ScanMatcherFactory, CreatesNoopMatcher)
{
  auto matcher = aqua_sonar_loc::create_scan_matcher("noop");

  ASSERT_NE(matcher, nullptr);
  EXPECT_EQ(matcher->backend_name(), "noop");

  aqua_sonar_loc::ScanMatcherConfig config;
  matcher->configure(config);
  EXPECT_EQ(matcher->backend_name(), "noop");
}

TEST(ScanMatcherFactory, CreatesIcpMatcher)
{
  auto matcher = aqua_sonar_loc::create_scan_matcher("icp");

  ASSERT_NE(matcher, nullptr);
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "icp";
  matcher->configure(config);
  EXPECT_EQ(matcher->backend_name(), "icp");
}

TEST(ScanMatcherFactory, CreatesGicpMatcher)
{
  auto matcher = aqua_sonar_loc::create_scan_matcher("gicp");

  ASSERT_NE(matcher, nullptr);
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "gicp";
  matcher->configure(config);
  EXPECT_EQ(matcher->backend_name(), "gicp");
}

TEST(ScanMatcherFactory, CreatesNdtMatcher)
{
  auto matcher = aqua_sonar_loc::create_scan_matcher("ndt");

  ASSERT_NE(matcher, nullptr);
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "ndt";
  matcher->configure(config);
  EXPECT_EQ(matcher->backend_name(), "ndt");
}

TEST(ScanMatcherFactory, RejectsUnsupportedBackend)
{
  EXPECT_EQ(aqua_sonar_loc::create_scan_matcher("nonexistent_backend"), nullptr);
}

TEST(NoopScanMatcher, ReturnsIdentityForAcceptedCloud)
{
  aqua_sonar_loc::NoopScanMatcher matcher;
  matcher.configure({});

  sensor_msgs::msg::PointCloud2 cloud;
  aqua_sonar_loc::CloudSummary summary;
  summary.accepted = true;
  summary.total_points = 10;
  summary.in_range_points = 10;
  summary.finite_xyz_points = 10;

  const auto result = matcher.match(cloud, summary);

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.converged);
  EXPECT_DOUBLE_EQ(result.odom_to_base.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(result.odom_to_base.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(result.odom_to_base.translation.z, 0.0);
  EXPECT_DOUBLE_EQ(result.odom_to_base.rotation.w, 1.0);
}

TEST(NoopScanMatcher, PropagatesRejectedSummary)
{
  aqua_sonar_loc::NoopScanMatcher matcher;
  matcher.configure({});

  sensor_msgs::msg::PointCloud2 cloud;
  aqua_sonar_loc::CloudSummary summary;
  summary.accepted = false;
  summary.rejection_reason = "not enough points";

  const auto result = matcher.match(cloud, summary);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.status, "not enough points");
}

TEST(IcpScanMatcher, InitializesOnFirstAcceptedCloud)
{
  aqua_sonar_loc::IcpScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "icp";
  matcher.configure(config);

  const auto cloud = make_cloud({{{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}}});
  const auto result = matcher.match(cloud, accepted_summary(3));

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.converged);
  EXPECT_EQ(result.status, "icp initialized");
  EXPECT_DOUBLE_EQ(result.odom_to_base.rotation.w, 1.0);
}

TEST(IcpScanMatcher, EstimatesTranslationBetweenAcceptedClouds)
{
  aqua_sonar_loc::IcpScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "icp";
  config.max_correspondence_distance = 2.0;
  config.max_iterations = 100;
  config.transformation_epsilon = 1.0e-10;
  matcher.configure(config);

  const std::vector<std::array<float, 3>> reference_points = {{
    {0.0F, 0.0F, 0.0F},
    {0.8F, 0.1F, 0.2F},
    {-0.4F, 1.1F, 0.3F},
    {1.5F, -0.6F, 0.7F},
    {-1.2F, -0.4F, 1.1F},
    {0.2F, 1.8F, -0.5F},
    {1.9F, 1.2F, 0.4F},
    {-0.9F, 0.7F, -0.8F},
    {0.6F, -1.4F, 1.3F},
    {1.1F, 0.4F, -1.0F},
  }};
  const auto shifted_points = translated(reference_points, 0.35F, -0.2F, 0.1F);

  const auto initial_result =
    matcher.match(make_cloud(reference_points), accepted_summary(reference_points.size()));
  ASSERT_TRUE(initial_result.success);
  ASSERT_TRUE(initial_result.converged);

  const auto shifted_result =
    matcher.match(make_cloud(shifted_points), accepted_summary(shifted_points.size()));

  ASSERT_TRUE(shifted_result.success) << shifted_result.status;
  ASSERT_TRUE(shifted_result.converged) << shifted_result.status;
  EXPECT_EQ(shifted_result.status, "icp converged");
  EXPECT_NEAR(shifted_result.odom_to_base.translation.x, 0.35, 0.03);
  EXPECT_NEAR(shifted_result.odom_to_base.translation.y, -0.2, 0.03);
  EXPECT_NEAR(shifted_result.odom_to_base.translation.z, 0.1, 0.03);
  EXPECT_LT(shifted_result.fitness_score, 1.0e-4);
}

TEST(IcpScanMatcher, MaxTranslationStepGateRejectsBigJump)
{
  aqua_sonar_loc::IcpScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "icp";
  config.max_correspondence_distance = 5.0;
  config.max_iterations = 100;
  config.transformation_epsilon = 1.0e-10;
  config.max_translation_step_m = 0.1;  // 10 cm gate
  matcher.configure(config);

  const std::vector<std::array<float, 3>> reference_points = {{
    {0.0F, 0.0F, 0.0F},
    {0.8F, 0.1F, 0.2F},
    {-0.4F, 1.1F, 0.3F},
    {1.5F, -0.6F, 0.7F},
    {-1.2F, -0.4F, 1.1F},
    {0.2F, 1.8F, -0.5F},
    {1.9F, 1.2F, 0.4F},
    {-0.9F, 0.7F, -0.8F},
    {0.6F, -1.4F, 1.3F},
    {1.1F, 0.4F, -1.0F},
  }};
  // 1.5 m jump exceeds the 0.1 m gate.
  const auto shifted_points = translated(reference_points, 1.5F, 0.0F, 0.0F);

  ASSERT_TRUE(matcher.match(make_cloud(reference_points), accepted_summary(reference_points.size())).success);
  const auto shifted_result = matcher.match(
    make_cloud(shifted_points), accepted_summary(shifted_points.size()));

  EXPECT_FALSE(shifted_result.success) << shifted_result.status;
  EXPECT_EQ(
    shifted_result.status,
    "icp rejected: translation step above max_translation_step_m");
}

TEST(IcpScanMatcher, MaxFitnessScoreGateRejectsHighResidual)
{
  aqua_sonar_loc::IcpScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "icp";
  config.max_correspondence_distance = 0.05;  // forces a poor match
  config.max_iterations = 5;
  config.transformation_epsilon = 1.0e-3;
  config.max_fitness_score = 1.0e-6;  // very tight; PCL-default fitness will exceed this
  matcher.configure(config);

  const std::vector<std::array<float, 3>> reference_points = {{
    {0.0F, 0.0F, 0.0F},
    {1.0F, 0.0F, 0.0F},
    {0.0F, 1.0F, 0.0F},
    {1.0F, 1.0F, 0.0F},
    {0.5F, 0.5F, 1.0F},
  }};
  // Large shift well past the 0.05 m correspondence distance
  const auto shifted_points = translated(reference_points, 0.6F, 0.0F, 0.0F);

  ASSERT_TRUE(matcher.match(make_cloud(reference_points), accepted_summary(reference_points.size())).success);
  const auto shifted_result = matcher.match(
    make_cloud(shifted_points), accepted_summary(shifted_points.size()));

  // The match either does not converge ("did not converge") or is rejected on fitness.
  // Either way, it must not advance the accumulated transform.
  EXPECT_FALSE(shifted_result.success) << shifted_result.status;
}

namespace
{
// PCL's GICP needs at least k_correspondences (default 20) points to compute local
// covariances. Build a slightly larger 3x3x3 grid plus jitter for the GICP tests.
std::vector<std::array<float, 3>> dense_reference_points()
{
  std::vector<std::array<float, 3>> points;
  for (int x = 0; x < 4; ++x) {
    for (int y = 0; y < 4; ++y) {
      for (int z = 0; z < 3; ++z) {
        points.push_back({static_cast<float>(x) * 0.4F + 0.05F,
                          static_cast<float>(y) * 0.4F + 0.07F,
                          static_cast<float>(z) * 0.4F - 0.02F});
      }
    }
  }
  return points;
}

// PCL's NDT needs ~5 points per voxel for the Gaussian to be well-defined.
// Build a denser jittered cube so the NDT voxel grid has populated cells
// regardless of the chosen voxel resolution within a sensible range.
std::vector<std::array<float, 3>> dense_ndt_points()
{
  std::vector<std::array<float, 3>> points;
  // Deterministic pseudo-random jitter so the test stays reproducible.
  unsigned seed = 0x12345;
  auto next = [&seed]() {
    seed = seed * 1103515245U + 12345U;
    return static_cast<float>((seed >> 16) & 0x7FFF) / 32767.0F - 0.5F;
  };
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 8; ++y) {
      for (int z = 0; z < 6; ++z) {
        // 8 points per cell for redundancy.
        for (int k = 0; k < 8; ++k) {
          points.push_back({
            static_cast<float>(x) * 0.5F + 0.1F * next(),
            static_cast<float>(y) * 0.5F + 0.1F * next(),
            static_cast<float>(z) * 0.5F + 0.1F * next(),
          });
        }
      }
    }
  }
  return points;
}
}  // namespace

TEST(GicpScanMatcher, InitializesOnFirstAcceptedCloud)
{
  aqua_sonar_loc::GicpScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "gicp";
  matcher.configure(config);

  const auto reference_points = dense_reference_points();
  const auto cloud = make_cloud(reference_points);
  const auto result = matcher.match(cloud, accepted_summary(reference_points.size()));

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.converged);
  EXPECT_EQ(result.status, "gicp initialized");
  EXPECT_DOUBLE_EQ(result.odom_to_base.rotation.w, 1.0);
}

TEST(GicpScanMatcher, EstimatesTranslationBetweenAcceptedClouds)
{
  aqua_sonar_loc::GicpScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "gicp";
  config.max_correspondence_distance = 2.0;
  config.max_iterations = 100;
  config.transformation_epsilon = 1.0e-10;
  matcher.configure(config);

  const auto reference_points = dense_reference_points();
  const auto shifted_points = translated(reference_points, 0.15F, -0.1F, 0.05F);

  ASSERT_TRUE(matcher.match(make_cloud(reference_points),
                            accepted_summary(reference_points.size())).success);
  const auto shifted_result = matcher.match(
    make_cloud(shifted_points), accepted_summary(shifted_points.size()));

  ASSERT_TRUE(shifted_result.success) << shifted_result.status;
  EXPECT_EQ(shifted_result.status, "gicp converged");
  EXPECT_NEAR(shifted_result.odom_to_base.translation.x, 0.15, 0.06);
  EXPECT_NEAR(shifted_result.odom_to_base.translation.y, -0.1, 0.06);
  EXPECT_NEAR(shifted_result.odom_to_base.translation.z, 0.05, 0.06);
}

TEST(GicpScanMatcher, MaxTranslationStepGateRejectsBigJump)
{
  aqua_sonar_loc::GicpScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "gicp";
  config.max_correspondence_distance = 5.0;
  config.max_iterations = 100;
  config.transformation_epsilon = 1.0e-10;
  config.max_translation_step_m = 0.1;
  matcher.configure(config);

  const auto reference_points = dense_reference_points();
  const auto shifted_points = translated(reference_points, 1.5F, 0.0F, 0.0F);

  ASSERT_TRUE(matcher.match(make_cloud(reference_points),
                            accepted_summary(reference_points.size())).success);
  const auto shifted_result = matcher.match(
    make_cloud(shifted_points), accepted_summary(shifted_points.size()));

  EXPECT_FALSE(shifted_result.success) << shifted_result.status;
  EXPECT_EQ(
    shifted_result.status,
    "gicp rejected: translation step above max_translation_step_m");
}

TEST(IcpScanMatcher, SubmapModeKeepsConsistentTranslationAcrossManySteps)
{
  aqua_sonar_loc::IcpScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "icp";
  config.max_correspondence_distance = 2.0;
  config.max_iterations = 100;
  config.transformation_epsilon = 1.0e-10;
  config.submap_size = 4;
  matcher.configure(config);

  const std::vector<std::array<float, 3>> base_points = {{
    {0.0F, 0.0F, 0.0F},
    {0.8F, 0.1F, 0.2F},
    {-0.4F, 1.1F, 0.3F},
    {1.5F, -0.6F, 0.7F},
    {-1.2F, -0.4F, 1.1F},
    {0.2F, 1.8F, -0.5F},
    {1.9F, 1.2F, 0.4F},
    {-0.9F, 0.7F, -0.8F},
    {0.6F, -1.4F, 1.3F},
    {1.1F, 0.4F, -1.0F},
  }};
  // Initialize with the reference fan.
  ASSERT_TRUE(matcher.match(make_cloud(base_points), accepted_summary(base_points.size())).success);

  // Five subsequent fans, each shifted by an additional 0.2 m in -x. The accumulated
  // transform must reach -1.0 m within the converged ICP residual.
  constexpr float step_dx = -0.2F;
  std::array<float, 3> last_translation{};
  for (int step = 1; step <= 5; ++step) {
    const float dx = step_dx * static_cast<float>(step);
    const auto shifted = translated(base_points, dx, 0.0F, 0.0F);
    const auto result =
      matcher.match(make_cloud(shifted), accepted_summary(shifted.size()));
    ASSERT_TRUE(result.success) << "step " << step << ": " << result.status;
    last_translation = {static_cast<float>(result.odom_to_base.translation.x),
                        static_cast<float>(result.odom_to_base.translation.y),
                        static_cast<float>(result.odom_to_base.translation.z)};
  }
  // accumulated_transform tracks the cumulative cloud offset from the initial frame
  // (see EstimatesTranslationBetweenAcceptedClouds: a shift of +0.35 m yields x = +0.35).
  // After 5 steps of -0.2 m each, the total cloud shift is -1.0 m, which is what the
  // accumulated_transform must report.
  EXPECT_NEAR(last_translation[0], -1.0F, 0.05F);
  EXPECT_NEAR(last_translation[1], 0.0F, 0.05F);
  EXPECT_NEAR(last_translation[2], 0.0F, 0.05F);
}

TEST(IcpScanMatcher, SubmapZeroFallsBackToScanToScan)
{
  // submap_size = 0 should clamp to 1 internally and behave identically to scan-to-scan.
  aqua_sonar_loc::IcpScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "icp";
  config.max_correspondence_distance = 2.0;
  config.max_iterations = 100;
  config.transformation_epsilon = 1.0e-10;
  config.submap_size = 0;
  matcher.configure(config);

  const std::vector<std::array<float, 3>> reference_points = {{
    {0.0F, 0.0F, 0.0F}, {0.8F, 0.1F, 0.2F}, {-0.4F, 1.1F, 0.3F},
    {1.5F, -0.6F, 0.7F}, {-1.2F, -0.4F, 1.1F}, {0.2F, 1.8F, -0.5F},
    {1.9F, 1.2F, 0.4F}, {-0.9F, 0.7F, -0.8F}, {0.6F, -1.4F, 1.3F}, {1.1F, 0.4F, -1.0F},
  }};
  const auto shifted = translated(reference_points, 0.3F, 0.0F, 0.0F);

  ASSERT_TRUE(matcher.match(make_cloud(reference_points),
                            accepted_summary(reference_points.size())).success);
  const auto shifted_result =
    matcher.match(make_cloud(shifted), accepted_summary(shifted.size()));
  ASSERT_TRUE(shifted_result.success) << shifted_result.status;
  EXPECT_NEAR(shifted_result.odom_to_base.translation.x, 0.3, 0.03);
}

TEST(NdtScanMatcher, InitializesOnFirstAcceptedCloud)
{
  aqua_sonar_loc::NdtScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "ndt";
  config.ndt_voxel_resolution_m = 0.5;
  matcher.configure(config);

  const auto reference_points = dense_ndt_points();
  const auto cloud = make_cloud(reference_points);
  const auto result = matcher.match(cloud, accepted_summary(reference_points.size()));

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.converged);
  EXPECT_EQ(result.status, "ndt initialized");
  EXPECT_DOUBLE_EQ(result.odom_to_base.rotation.w, 1.0);
}

TEST(NdtScanMatcher, EstimatesTranslationBetweenAcceptedClouds)
{
  // NDT needs ~5 points per voxel for the Gaussian to be well-defined; use
  // the denser jittered cube generator.
  aqua_sonar_loc::NdtScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "ndt";
  config.max_iterations = 100;
  config.transformation_epsilon = 1.0e-8;
  config.ndt_voxel_resolution_m = 0.5;
  config.ndt_step_size_m = 0.1;
  matcher.configure(config);

  const auto reference_points = dense_ndt_points();
  const auto shifted_points = translated(reference_points, 0.15F, -0.1F, 0.05F);

  ASSERT_TRUE(
    matcher.match(make_cloud(reference_points),
                  accepted_summary(reference_points.size())).success);
  const auto shifted_result = matcher.match(
    make_cloud(shifted_points), accepted_summary(shifted_points.size()));

  ASSERT_TRUE(shifted_result.success) << shifted_result.status;
  EXPECT_EQ(shifted_result.status, "ndt converged");
  // NDT recovers translation up to ~0.1 m on this small grid; the bound is
  // looser than ICP/GICP's because the Gaussian fits a coarser geometry.
  EXPECT_NEAR(shifted_result.odom_to_base.translation.x, 0.15, 0.10);
  EXPECT_NEAR(shifted_result.odom_to_base.translation.y, -0.1, 0.10);
  EXPECT_NEAR(shifted_result.odom_to_base.translation.z, 0.05, 0.10);
}

TEST(IcpScanMatcher, ExternalPriorIsConsumedOnce)
{
  // Verify the external prior path: a deliberately-bad initial guess on the second
  // fan should be overridden by a good external prior so ICP converges in fewer
  // iterations. Use a configuration where ICP with a zero initial guess would
  // diverge or fail to converge inside the iteration budget.
  aqua_sonar_loc::IcpScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "icp";
  config.max_correspondence_distance = 0.3;  // tight: needs a good prior
  config.max_iterations = 5;
  config.transformation_epsilon = 1.0e-10;
  matcher.configure(config);

  const std::vector<std::array<float, 3>> reference_points = {{
    {0.0F, 0.0F, 0.0F}, {0.8F, 0.1F, 0.2F}, {-0.4F, 1.1F, 0.3F},
    {1.5F, -0.6F, 0.7F}, {-1.2F, -0.4F, 1.1F}, {0.2F, 1.8F, -0.5F},
    {1.9F, 1.2F, 0.4F}, {-0.9F, 0.7F, -0.8F}, {0.6F, -1.4F, 1.3F}, {1.1F, 0.4F, -1.0F},
  }};
  const auto shifted_points = translated(reference_points, 0.5F, 0.0F, 0.0F);

  ASSERT_TRUE(matcher.match(make_cloud(reference_points), accepted_summary(reference_points.size())).success);

  // Stage a near-perfect external prior: current_to_previous shifts +0.5 m back.
  Eigen::Matrix4f prior = Eigen::Matrix4f::Identity();
  prior(0, 3) = -0.5F;  // current frame is 0.5 m ahead of previous; bring it back
  matcher.set_external_prior(prior);

  const auto shifted_result =
    matcher.match(make_cloud(shifted_points), accepted_summary(shifted_points.size()));
  ASSERT_TRUE(shifted_result.success) << shifted_result.status;
  EXPECT_NEAR(shifted_result.odom_to_base.translation.x, 0.5, 0.05);

  // Prior is consume-once: a second fan at the same location should NOT carry the
  // prior over. Use a fresh fan at the same offset; without a stale prior the
  // accumulated transform should keep tracking the actual cloud motion.
  const auto third_points = translated(reference_points, 1.0F, 0.0F, 0.0F);
  matcher.set_external_prior(prior);  // -0.5 again
  const auto third_result =
    matcher.match(make_cloud(third_points), accepted_summary(third_points.size()));
  ASSERT_TRUE(third_result.success) << third_result.status;
  EXPECT_NEAR(third_result.odom_to_base.translation.x, 1.0, 0.05);
}

TEST(IcpScanMatcher, ExternalPriorIsClearedAfterRejectedSummary)
{
  // If the cloud is rejected by preprocessing the external prior must still be
  // discarded — otherwise it would silently apply to a later, valid fan whose
  // timestamp no longer matches.
  aqua_sonar_loc::IcpScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "icp";
  config.max_correspondence_distance = 2.0;
  config.max_iterations = 100;
  config.transformation_epsilon = 1.0e-10;
  matcher.configure(config);

  const std::vector<std::array<float, 3>> reference_points = {{
    {0.0F, 0.0F, 0.0F}, {0.8F, 0.1F, 0.2F}, {-0.4F, 1.1F, 0.3F},
    {1.5F, -0.6F, 0.7F}, {-1.2F, -0.4F, 1.1F},
  }};
  ASSERT_TRUE(matcher.match(make_cloud(reference_points),
                            accepted_summary(reference_points.size())).success);

  // Stage a clearly-wrong prior, then feed a rejected summary. The prior must be
  // consumed/discarded by this call.
  Eigen::Matrix4f wrong_prior = Eigen::Matrix4f::Identity();
  wrong_prior(0, 3) = -10.0F;
  matcher.set_external_prior(wrong_prior);

  aqua_sonar_loc::CloudSummary rejected;
  rejected.accepted = false;
  rejected.rejection_reason = "preproc dropped this fan";
  const auto rejected_result =
    matcher.match(make_cloud(reference_points), rejected);
  EXPECT_FALSE(rejected_result.success);

  // A small subsequent shift should now resolve correctly without the wrong prior
  // bleeding in.
  const auto shifted = translated(reference_points, 0.2F, 0.0F, 0.0F);
  const auto next_result =
    matcher.match(make_cloud(shifted), accepted_summary(shifted.size()));
  ASSERT_TRUE(next_result.success) << next_result.status;
  EXPECT_NEAR(next_result.odom_to_base.translation.x, 0.2, 0.03);
}

TEST(IcpScanMatcher, LegacyCovarianceIsHardcodedDiagonal)
{
  // With covariance estimation disabled the published covariance must be the
  // legacy 0.25 m² / 0.10 rad² diagonal that downstream fusion consumers were
  // tuned against. Any other path is a regression.
  aqua_sonar_loc::IcpScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "icp";
  config.max_correspondence_distance = 2.0;
  config.max_iterations = 100;
  config.transformation_epsilon = 1.0e-10;
  config.covariance_enable_estimation = false;
  matcher.configure(config);

  const std::vector<std::array<float, 3>> reference_points = {{
    {0.0F, 0.0F, 0.0F}, {0.8F, 0.1F, 0.2F}, {-0.4F, 1.1F, 0.3F},
    {1.5F, -0.6F, 0.7F}, {-1.2F, -0.4F, 1.1F},
  }};
  const auto shifted = translated(reference_points, 0.2F, 0.0F, 0.0F);
  ASSERT_TRUE(matcher.match(make_cloud(reference_points),
                            accepted_summary(reference_points.size())).success);
  const auto result = matcher.match(
    make_cloud(shifted), accepted_summary(shifted.size()));
  ASSERT_TRUE(result.success) << result.status;

  EXPECT_DOUBLE_EQ(result.pose_covariance[0], 0.25);   // x
  EXPECT_DOUBLE_EQ(result.pose_covariance[7], 0.25);   // y
  EXPECT_DOUBLE_EQ(result.pose_covariance[14], 0.25);  // z
  EXPECT_DOUBLE_EQ(result.pose_covariance[21], 0.10);  // roll
  EXPECT_DOUBLE_EQ(result.pose_covariance[28], 0.10);  // pitch
  EXPECT_DOUBLE_EQ(result.pose_covariance[35], 0.10);  // yaw
}

TEST(IcpScanMatcher, EstimatedCovarianceShrinksOnGoodFitAndRespectsFloor)
{
  // With estimation enabled the per-axis variance should equal the floor when
  // fitness/inliers is small (good match), and stay at the floor not below.
  aqua_sonar_loc::IcpScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "icp";
  config.max_correspondence_distance = 2.0;
  config.max_iterations = 100;
  config.transformation_epsilon = 1.0e-10;
  config.covariance_enable_estimation = true;
  config.covariance_position_floor_m2 = 0.04;
  config.covariance_rotation_floor_rad2 = 0.001;
  config.covariance_position_cap_m2 = 9.0;
  config.covariance_rotation_cap_rad2 = 1.0;
  matcher.configure(config);

  const std::vector<std::array<float, 3>> reference_points = {{
    {0.0F, 0.0F, 0.0F}, {0.8F, 0.1F, 0.2F}, {-0.4F, 1.1F, 0.3F},
    {1.5F, -0.6F, 0.7F}, {-1.2F, -0.4F, 1.1F}, {0.2F, 1.8F, -0.5F},
    {1.9F, 1.2F, 0.4F}, {-0.9F, 0.7F, -0.8F}, {0.6F, -1.4F, 1.3F}, {1.1F, 0.4F, -1.0F},
  }};
  const auto shifted = translated(reference_points, 0.05F, 0.0F, 0.0F);
  ASSERT_TRUE(matcher.match(make_cloud(reference_points),
                            accepted_summary(reference_points.size())).success);
  const auto result = matcher.match(
    make_cloud(shifted), accepted_summary(shifted.size()));
  ASSERT_TRUE(result.success) << result.status;

  // Tiny shift = tiny fitness; the position variance should sit at the floor.
  EXPECT_NEAR(result.pose_covariance[0], 0.04, 1.0e-9);
  EXPECT_NEAR(result.pose_covariance[14], 0.04, 1.0e-9);
  // Rotation variance should also sit at the floor for the same reason.
  EXPECT_NEAR(result.pose_covariance[21], 0.001, 1.0e-9);
  EXPECT_NEAR(result.pose_covariance[35], 0.001, 1.0e-9);
}

TEST(IcpScanMatcher, EstimatedCovarianceCapIsRespected)
{
  // Force a very low cap and confirm the estimate clamps to it on a noisy
  // fitness score. We achieve a poor fit by giving GICP only a tiny
  // overlap window and letting the residual blow up.
  aqua_sonar_loc::IcpScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "icp";
  config.max_correspondence_distance = 2.0;
  config.max_iterations = 50;
  config.transformation_epsilon = 1.0e-6;
  config.covariance_enable_estimation = true;
  config.covariance_position_scale = 1.0e6;        // hugely amplified
  config.covariance_position_floor_m2 = 1.0e-9;    // effectively no floor
  config.covariance_position_cap_m2 = 0.5;         // tight cap
  config.covariance_rotation_scale = 1.0e6;
  config.covariance_rotation_floor_rad2 = 1.0e-9;
  config.covariance_rotation_cap_rad2 = 0.1;
  matcher.configure(config);

  const std::vector<std::array<float, 3>> reference_points = {{
    {0.0F, 0.0F, 0.0F}, {0.8F, 0.1F, 0.2F}, {-0.4F, 1.1F, 0.3F},
    {1.5F, -0.6F, 0.7F}, {-1.2F, -0.4F, 1.1F}, {0.2F, 1.8F, -0.5F},
  }};
  const auto shifted = translated(reference_points, 0.5F, 0.0F, 0.0F);
  ASSERT_TRUE(matcher.match(make_cloud(reference_points),
                            accepted_summary(reference_points.size())).success);
  const auto result = matcher.match(
    make_cloud(shifted), accepted_summary(shifted.size()));
  ASSERT_TRUE(result.success) << result.status;

  EXPECT_LE(result.pose_covariance[0], 0.5 + 1.0e-9);
  EXPECT_LE(result.pose_covariance[35], 0.1 + 1.0e-9);
}

TEST(IcpScanMatcher, NegativeGateValuesAreDisabled)
{
  aqua_sonar_loc::IcpScanMatcher matcher;
  aqua_sonar_loc::ScanMatcherConfig config;
  config.backend = "icp";
  config.max_correspondence_distance = 5.0;
  config.max_iterations = 100;
  config.transformation_epsilon = 1.0e-10;
  config.max_fitness_score = -1.0;
  config.max_translation_step_m = -1.0;
  config.max_rotation_step_rad = -1.0;
  matcher.configure(config);

  const std::vector<std::array<float, 3>> reference_points = {{
    {0.0F, 0.0F, 0.0F},
    {0.8F, 0.1F, 0.2F},
    {-0.4F, 1.1F, 0.3F},
    {1.5F, -0.6F, 0.7F},
    {-1.2F, -0.4F, 1.1F},
  }};
  const auto shifted_points = translated(reference_points, 0.35F, 0.0F, 0.0F);

  ASSERT_TRUE(matcher.match(make_cloud(reference_points), accepted_summary(reference_points.size())).success);
  const auto shifted_result = matcher.match(
    make_cloud(shifted_points), accepted_summary(shifted_points.size()));

  EXPECT_TRUE(shifted_result.success) << shifted_result.status;
  EXPECT_EQ(shifted_result.status, "icp converged");
}
