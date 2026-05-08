// SPDX-License-Identifier: Apache-2.0
// SE(3) pose graph backed by g2o. Tracks keyframes from upstream odometry
// and exposes hooks for loop closure constraints. The graph runs without
// loop closures (chain only) and starts to *do something* once external
// constraints are inserted via add_loop_constraint().
#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <memory>
#include <vector>

namespace g2o {
class SparseOptimizer;
}  // namespace g2o

namespace aqua_pose_graph {

struct PoseGraphConfig
{
  // Minimum translation between successive keyframes (m). Smaller spacings
  // produce a denser graph and slow optimization down.
  double keyframe_translation_m{0.5};
  // Minimum rotation between keyframes (rad). 10° default keeps rotations
  // well-resolved without bloating the graph.
  double keyframe_rotation_rad{10.0 * M_PI / 180.0};
  // Default information matrix diagonal for odometry edges when the
  // upstream odometry message does not carry a usable covariance. Larger
  // values mean tighter edge constraint.
  double default_translation_information{100.0};   // 1 / (0.1 m)^2
  double default_rotation_information{1000.0};     // 1 / (0.0316 rad)^2
  // How many g2o iterations to run when optimize() is called.
  int optimization_iterations{20};
  // Number of keyframes added between automatic optimize() calls. 0 disables
  // automatic optimization (callers must trigger it manually).
  int optimize_every_n_keyframes{20};
};

struct Keyframe
{
  std::size_t id;
  double stamp_seconds;
  Eigen::Isometry3d pose;
};

struct LoopConstraint
{
  std::size_t from_id;
  std::size_t to_id;
  Eigen::Isometry3d relative_pose;
  Eigen::Matrix<double, 6, 6> information;
};

class PoseGraph
{
public:
  explicit PoseGraph(const PoseGraphConfig & config);
  ~PoseGraph();

  PoseGraph(const PoseGraph &) = delete;
  PoseGraph & operator=(const PoseGraph &) = delete;

  // Add an upstream odometry sample. If it is far enough from the current
  // last keyframe (per config thresholds) a new keyframe is appended and
  // an edge is created from the previous keyframe with the relative SE(3)
  // computed from the two odometry poses. Returns true iff a new keyframe
  // was added.
  bool add_odometry_sample(
    double stamp_seconds,
    const Eigen::Isometry3d & odometry_pose,
    const Eigen::Matrix<double, 6, 6> & covariance);

  // Insert a loop closure / external constraint between two existing
  // keyframes. Returns false when either id does not exist.
  bool add_loop_constraint(const LoopConstraint & constraint);

  // Run g2o optimization for `optimization_iterations`. Returns the final
  // active edge chi^2 (post-optimization). Safe to call with no edges.
  double optimize();

  // Reset everything: clears keyframes, edges, and the underlying g2o graph.
  void reset();

  // Read-only views.
  const std::vector<Keyframe> & keyframes() const { return keyframes_; }
  std::size_t edge_count() const { return edges_; }
  std::size_t loop_constraint_count() const { return loop_edges_; }

  // Pull the current optimized pose for a given keyframe id (returns false
  // if the id does not exist).
  bool keyframe_pose(std::size_t id, Eigen::Isometry3d * pose) const;

private:
  void seed_first_keyframe(double stamp_seconds, const Eigen::Isometry3d & pose);
  void append_keyframe_with_edge(
    double stamp_seconds,
    const Eigen::Isometry3d & odometry_pose,
    const Eigen::Matrix<double, 6, 6> & covariance);

  PoseGraphConfig config_;
  std::unique_ptr<g2o::SparseOptimizer> optimizer_;
  std::vector<Keyframe> keyframes_;
  Eigen::Isometry3d last_odometry_pose_{Eigen::Isometry3d::Identity()};
  bool has_odometry_{false};
  std::size_t edges_{0};
  std::size_t loop_edges_{0};
  int keyframes_since_last_optimize_{0};
};

}  // namespace aqua_pose_graph
