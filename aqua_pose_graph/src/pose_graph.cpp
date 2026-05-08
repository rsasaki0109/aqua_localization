// SPDX-License-Identifier: Apache-2.0
#include "aqua_pose_graph/pose_graph.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <utility>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

namespace aqua_pose_graph {

namespace {

// Map a 6x6 ROS pose covariance (translation x/y/z then rotation) to a g2o
// SE(3) information matrix. We only keep the 3+3 diagonal because the
// scan matcher publishes a diagonal-only covariance and any cross terms
// from the IMU UKF are tiny on the path-edge scale.
Eigen::Matrix<double, 6, 6> covariance_to_information(
  const Eigen::Matrix<double, 6, 6> & cov,
  const PoseGraphConfig & config)
{
  Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Zero();
  for (int i = 0; i < 6; ++i) {
    const double v = cov(i, i);
    if (std::isfinite(v) && v > 1e-9) {
      info(i, i) = 1.0 / v;
    } else {
      info(i, i) = (i < 3)
        ? config.default_translation_information
        : config.default_rotation_information;
    }
  }
  return info;
}

bool exceeds_keyframe_threshold(
  const Eigen::Isometry3d & last,
  const Eigen::Isometry3d & current,
  const PoseGraphConfig & config)
{
  const Eigen::Vector3d dt = current.translation() - last.translation();
  if (dt.norm() >= config.keyframe_translation_m) {
    return true;
  }
  const Eigen::Matrix3d rel = last.linear().transpose() * current.linear();
  const Eigen::AngleAxisd aa(rel);
  return std::abs(aa.angle()) >= config.keyframe_rotation_rad;
}

}  // namespace

PoseGraph::PoseGraph(const PoseGraphConfig & config)
: config_(config)
{
  reset();
}

PoseGraph::~PoseGraph() = default;

void PoseGraph::reset()
{
  using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>;
  using LinearSolverType =
    g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

  optimizer_ = std::make_unique<g2o::SparseOptimizer>();
  auto linear_solver = std::make_unique<LinearSolverType>();
  auto block_solver =
    std::make_unique<BlockSolverType>(std::move(linear_solver));
  auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
  optimizer_->setAlgorithm(algorithm);
  optimizer_->setVerbose(false);

  keyframes_.clear();
  has_odometry_ = false;
  last_odometry_pose_.setIdentity();
  edges_ = 0;
  loop_edges_ = 0;
  keyframes_since_last_optimize_ = 0;
}

void PoseGraph::seed_first_keyframe(
  double stamp_seconds,
  const Eigen::Isometry3d & pose)
{
  Keyframe kf{0, stamp_seconds, pose};
  keyframes_.push_back(kf);

  auto * v = new g2o::VertexSE3();
  v->setId(0);
  v->setEstimate(pose);
  v->setFixed(true);
  optimizer_->addVertex(v);
}

void PoseGraph::append_keyframe_with_edge(
  double stamp_seconds,
  const Eigen::Isometry3d & odometry_pose,
  const Eigen::Matrix<double, 6, 6> & covariance)
{
  const std::size_t prev_id = keyframes_.back().id;
  const Eigen::Isometry3d & prev_kf_pose = keyframes_.back().pose;

  // Relative motion between odometry samples since the last keyframe.
  const Eigen::Isometry3d delta = last_odometry_pose_.inverse() * odometry_pose;
  const Eigen::Isometry3d new_kf_pose = prev_kf_pose * delta;

  Keyframe kf{prev_id + 1, stamp_seconds, new_kf_pose};
  keyframes_.push_back(kf);

  auto * v = new g2o::VertexSE3();
  v->setId(static_cast<int>(kf.id));
  v->setEstimate(new_kf_pose);
  optimizer_->addVertex(v);

  auto * e = new g2o::EdgeSE3();
  e->setVertex(0, optimizer_->vertex(static_cast<int>(prev_id)));
  e->setVertex(1, optimizer_->vertex(static_cast<int>(kf.id)));
  e->setMeasurement(delta);
  e->setInformation(covariance_to_information(covariance, config_));
  optimizer_->addEdge(e);

  edges_ += 1;
  keyframes_since_last_optimize_ += 1;
}

bool PoseGraph::add_odometry_sample(
  double stamp_seconds,
  const Eigen::Isometry3d & odometry_pose,
  const Eigen::Matrix<double, 6, 6> & covariance)
{
  if (!has_odometry_) {
    seed_first_keyframe(stamp_seconds, odometry_pose);
    last_odometry_pose_ = odometry_pose;
    has_odometry_ = true;
    return true;
  }

  if (!exceeds_keyframe_threshold(last_odometry_pose_, odometry_pose, config_)) {
    return false;
  }

  append_keyframe_with_edge(stamp_seconds, odometry_pose, covariance);
  last_odometry_pose_ = odometry_pose;

  if (config_.optimize_every_n_keyframes > 0
      && keyframes_since_last_optimize_ >= config_.optimize_every_n_keyframes)
  {
    optimize();
  }
  return true;
}

bool PoseGraph::add_loop_constraint(const LoopConstraint & constraint)
{
  auto * v_from = optimizer_->vertex(static_cast<int>(constraint.from_id));
  auto * v_to = optimizer_->vertex(static_cast<int>(constraint.to_id));
  if (v_from == nullptr || v_to == nullptr) {
    return false;
  }
  auto * e = new g2o::EdgeSE3();
  e->setVertex(0, v_from);
  e->setVertex(1, v_to);
  e->setMeasurement(constraint.relative_pose);
  e->setInformation(constraint.information);
  optimizer_->addEdge(e);
  edges_ += 1;
  loop_edges_ += 1;
  return true;
}

double PoseGraph::optimize()
{
  if (keyframes_.size() < 2 || edges_ == 0) {
    return 0.0;
  }
  optimizer_->initializeOptimization();
  optimizer_->optimize(config_.optimization_iterations);
  keyframes_since_last_optimize_ = 0;

  // Pull optimized poses back into the keyframe cache.
  for (auto & kf : keyframes_) {
    auto * v = dynamic_cast<g2o::VertexSE3 *>(
      optimizer_->vertex(static_cast<int>(kf.id)));
    if (v != nullptr) {
      kf.pose = v->estimate();
    }
  }
  optimizer_->computeActiveErrors();
  return optimizer_->activeChi2();
}

bool PoseGraph::keyframe_pose(std::size_t id, Eigen::Isometry3d * pose) const
{
  if (id >= keyframes_.size()) {
    return false;
  }
  if (pose != nullptr) {
    *pose = keyframes_[id].pose;
  }
  return true;
}

}  // namespace aqua_pose_graph
