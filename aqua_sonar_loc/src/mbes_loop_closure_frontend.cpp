#include "aqua_sonar_loc/mbes_loop_closure_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iterator>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

namespace aqua_sonar_loc
{

namespace
{

std::uint32_t id_distance(std::uint32_t a, std::uint32_t b)
{
  return a > b ? a - b : b - a;
}

double ratio_or_infinity(double a, double b)
{
  constexpr double kEpsilon = 1.0e-9;
  const double lo = std::min(std::abs(a), std::abs(b));
  const double hi = std::max(std::abs(a), std::abs(b));
  if (hi < kEpsilon) {
    return 1.0;
  }
  if (lo < kEpsilon) {
    return std::numeric_limits<double>::infinity();
  }
  return hi / lo;
}

double rotation_distance_rad(const Eigen::Isometry3d & delta)
{
  const Eigen::AngleAxisd aa(delta.linear());
  return std::abs(aa.angle());
}

double plan_view_distance_m(const Eigen::Isometry3d & transform)
{
  const Eigen::Vector3d translation = transform.translation();
  return std::hypot(translation.x(), translation.y());
}

double positive_or_infinity(double value)
{
  return value > 0.0 ? value : std::numeric_limits<double>::infinity();
}

double descriptor_mismatch_score(
  const SubmapDescriptor & candidate,
  const SubmapDescriptor & current,
  const CandidateSelectionOptions & options)
{
  if (!candidate.valid || !current.valid) {
    return std::numeric_limits<double>::infinity();
  }
  const auto min_points = std::min(candidate.point_count, current.point_count);
  const auto max_points = std::max(candidate.point_count, current.point_count);
  if (max_points == 0U) {
    return std::numeric_limits<double>::infinity();
  }

  const double centroid_score =
    (candidate.centroid - current.centroid).norm() /
    positive_or_infinity(options.descriptor_centroid_scale_m);
  const double extent_ratio = std::max({
    ratio_or_infinity(candidate.extent.x(), current.extent.x()),
    ratio_or_infinity(candidate.extent.y(), current.extent.y()),
    ratio_or_infinity(candidate.extent.z(), current.extent.z())});
  const double extent_score =
    std::max(0.0, extent_ratio - 1.0) /
    positive_or_infinity(options.descriptor_extent_scale);
  const double point_count_ratio =
    static_cast<double>(min_points) / static_cast<double>(max_points);
  const double point_count_score =
    std::max(0.0, 1.0 - point_count_ratio) /
    positive_or_infinity(options.descriptor_point_count_ratio_scale);

  return centroid_score + extent_score + point_count_score;
}

double candidate_distance_score(double distance_m, const CandidateSelectionOptions & options)
{
  if (options.max_distance_m > 0.0) {
    return distance_m / options.max_distance_m;
  }
  return distance_m;
}

Eigen::Isometry3d loop_correction(
  const Eigen::Isometry3d & guess,
  const Eigen::Isometry3d & candidate_to_current)
{
  return guess.inverse() * candidate_to_current;
}

template<typename RegistrationT>
MatchResult run_registration(
  RegistrationT & registration,
  const PointCloud::Ptr & candidate,
  const PointCloud::Ptr & current,
  const Eigen::Isometry3d & candidate_to_current_guess,
  const RegistrationOptions & options)
{
  MatchResult result;
  registration.setInputSource(current);
  registration.setInputTarget(candidate);
  registration.setMaximumIterations(options.max_iterations);
  registration.setMaxCorrespondenceDistance(options.max_correspondence_distance_m);
  registration.setTransformationEpsilon(options.transformation_epsilon);

  PointCloud aligned;
  // Source = current, target = candidate: the transform maps current-submap
  // points into the candidate frame, i.e. it is the pose of the current
  // submap in the candidate frame (candidate.pose^-1 * current.pose).
  registration.align(aligned, candidate_to_current_guess.matrix().cast<float>());
  result.converged = registration.hasConverged();
  result.fitness = registration.getFitnessScore();
  if (!result.converged || !std::isfinite(result.fitness)) {
    result.status = "registration did not converge";
    return result;
  }

  const Eigen::Matrix4f final_f = registration.getFinalTransformation();
  result.candidate_to_current = Eigen::Isometry3d(final_f.cast<double>());
  result.success = true;
  result.status = "registration converged";
  return result;
}

}  // namespace

namespace
{

struct HeightCell
{
  double x{0.0};
  double y{0.0};
  float h{0.0F};
};

// Median height per cell of points already in the levelled frame.
std::vector<HeightCell> grid_heights(const std::vector<Eigen::Vector3d> & pts, double cell)
{
  std::unordered_map<std::int64_t, std::vector<float>> cells;
  for (const auto & p : pts) {
    const auto ix = static_cast<std::int64_t>(std::floor(p.x() / cell));
    const auto iy = static_cast<std::int64_t>(std::floor(p.y() / cell));
    cells[(ix << 32) ^ (iy & 0xffffffffLL)].push_back(static_cast<float>(p.z()));
  }
  std::vector<HeightCell> out;
  out.reserve(cells.size());
  for (auto & kv : cells) {
    auto & v = kv.second;
    if (v.size() < 2) {
      continue;
    }
    std::nth_element(v.begin(), v.begin() + static_cast<long>(v.size() / 2), v.end());
    const auto ix = kv.first >> 32;
    const auto iy = static_cast<std::int32_t>(kv.first & 0xffffffffLL);
    out.push_back(HeightCell{(static_cast<double>(ix) + 0.5) * cell,
        (static_cast<double>(iy) + 0.5) * cell, v[v.size() / 2]});
  }
  // Deterministic order regardless of hash iteration.
  std::sort(out.begin(), out.end(), [](const HeightCell & a, const HeightCell & b) {
      return a.x != b.x ? a.x < b.x : a.y < b.y;
    });
  return out;
}

// Std of heights about their least-squares plane.
double detrended_std(const std::vector<HeightCell> & cells)
{
  if (cells.size() < 4) {
    return 0.0;
  }
  Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();
  for (const auto & c : cells) {
    const Eigen::Vector3d row(c.x, c.y, 1.0);
    A += row * row.transpose();
    b += row * static_cast<double>(c.h);
  }
  const Eigen::Vector3d coef = A.ldlt().solve(b);
  double sq = 0.0;
  for (const auto & c : cells) {
    const double r = static_cast<double>(c.h) - (coef.x() * c.x + coef.y() * c.y + coef.z());
    sq += r * r;
  }
  return std::sqrt(sq / static_cast<double>(cells.size()));
}

}  // namespace

std::string CoarseAlignResult::detail() const
{
  char buf[128];
  std::snprintf(
    buf, sizeof(buf), "relief %.2f m, distinct %.2f, rms %.3f m, cells %d", relief_m,
    distinctiveness, residual_m, overlap_cells);
  return buf;
}

CoarseAlignResult coarse_align_heightmaps(
  const PointCloud & candidate, const PointCloud & current,
  const Eigen::Isometry3d & candidate_pose,
  const Eigen::Isometry3d & guess, const CoarseAlignOptions & o)
{
  CoarseAlignResult result;
  // Level the candidate frame: keep its heading, drop its roll and pitch.
  const Eigen::Matrix3d R_wc = candidate_pose.linear();
  const double yaw_c = std::atan2(R_wc(1, 0), R_wc(0, 0));
  const Eigen::Matrix3d R_level =
    Eigen::AngleAxisd(-yaw_c, Eigen::Vector3d::UnitZ()).toRotationMatrix() * R_wc;

  std::vector<Eigen::Vector3d> cand_pts;
  cand_pts.reserve(candidate.size());
  for (const auto & p : candidate.points) {
    cand_pts.push_back(R_level * Eigen::Vector3d(p.x, p.y, p.z));
  }
  std::vector<Eigen::Vector3d> cur_pts;
  cur_pts.reserve(current.size());
  for (const auto & p : current.points) {
    cur_pts.push_back(R_level * (guess * Eigen::Vector3d(p.x, p.y, p.z)));
  }
  const double cell = o.cell_m;
  const auto cand_cells = grid_heights(cand_pts, cell);
  const auto cur_cells = grid_heights(cur_pts, cell);
  if (static_cast<int>(cand_cells.size()) < o.min_overlap_cells ||
    static_cast<int>(cur_cells.size()) < o.min_overlap_cells)
  {
    result.status = "coarse: too few height cells";
    return result;
  }

  // Dense candidate grid with a margin for the search range.
  double min_x = cand_cells.front().x, max_x = min_x, min_y = cand_cells.front().y, max_y = min_y;
  for (const auto & c : cand_cells) {
    min_x = std::min(min_x, c.x); max_x = std::max(max_x, c.x);
    min_y = std::min(min_y, c.y); max_y = std::max(max_y, c.y);
  }
  const int nx = static_cast<int>(std::lround((max_x - min_x) / cell)) + 1;
  const int ny = static_cast<int>(std::lround((max_y - min_y) / cell)) + 1;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::vector<float> dense(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny), nan);
  for (const auto & c : cand_cells) {
    const int ix = static_cast<int>(std::lround((c.x - min_x) / cell));
    const int iy = static_cast<int>(std::lround((c.y - min_y) / cell));
    dense[static_cast<std::size_t>(ix) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(iy)] = c.h;
  }
  const int min_overlap = std::max(
    o.min_overlap_cells,
    static_cast<int>(std::ceil(
      o.min_overlap_fraction * static_cast<double>(std::min(cand_cells.size(), cur_cells.size())))));
  Eigen::Vector2d pivot = Eigen::Vector2d::Zero();
  for (const auto & c : cur_cells) {
    pivot += Eigen::Vector2d(c.x, c.y);
  }
  pivot /= static_cast<double>(cur_cells.size());

  struct Score
  {
    double yaw, dx, dy, residual, dz;
    int n;
  };
  std::vector<Score> scores;
  std::vector<float> diffs;
  std::vector<double> rx(cur_cells.size()), ry(cur_cells.size());
  const int n_yaw = static_cast<int>(std::floor(o.yaw_range_rad / o.yaw_step_rad + 1e-9));
  const int n_xy = static_cast<int>(std::floor(o.xy_range_m / o.xy_step_m + 1e-9));
  for (int iyaw = -n_yaw; iyaw <= n_yaw; ++iyaw) {
    const double yaw = iyaw * o.yaw_step_rad;
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    for (std::size_t k = 0; k < cur_cells.size(); ++k) {
      const double px = cur_cells[k].x - pivot.x(), py = cur_cells[k].y - pivot.y();
      rx[k] = cy * px - sy * py + pivot.x() - min_x;
      ry[k] = sy * px + cy * py + pivot.y() - min_y;
    }
    for (int ix = -n_xy; ix <= n_xy; ++ix) {
      for (int iy = -n_xy; iy <= n_xy; ++iy) {
        const double dx = ix * o.xy_step_m, dy = iy * o.xy_step_m;
        diffs.clear();
        for (std::size_t k = 0; k < cur_cells.size(); ++k) {
          const int gx = static_cast<int>(std::lround((rx[k] + dx) / cell));
          const int gy = static_cast<int>(std::lround((ry[k] + dy) / cell));
          if (gx < 0 || gy < 0 || gx >= nx || gy >= ny) {
            continue;
          }
          const float hc =
            dense[static_cast<std::size_t>(gx) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(gy)];
          if (!std::isnan(hc)) {
            diffs.push_back(hc - cur_cells[k].h);
          }
        }
        const int n = static_cast<int>(diffs.size());
        if (n < min_overlap) {
          continue;
        }
        auto mid = diffs.begin() + n / 2;
        std::nth_element(diffs.begin(), mid, diffs.end());
        const double med = *mid;
        double sq = 0.0;
        for (const float d : diffs) {
          const double e = std::min(std::abs(d - med), o.residual_clip_m);
          sq += e * e;
        }
        scores.push_back(Score{yaw, dx, dy, std::sqrt(sq / n), med, n});
      }
    }
  }
  if (scores.empty()) {
    result.status = "coarse: no overlap in search range";
    return result;
  }
  const auto best = *std::min_element(scores.begin(), scores.end(),
      [](const Score & a, const Score & b) {return a.residual < b.residual;});
  double second = std::numeric_limits<double>::infinity();
  for (const auto & s : scores) {
    if (std::hypot(s.dx - best.dx, s.dy - best.dy) > o.distinct_radius_m ||
      std::abs(s.yaw - best.yaw) > o.distinct_yaw_rad)
    {
      second = std::min(second, s.residual);
    }
  }
  result.residual_m = best.residual;
  result.overlap_cells = best.n;
  result.distinctiveness = second / std::max(best.residual, 1e-6);

  // Relief of the candidate cells under the best placement.
  {
    const double cy = std::cos(best.yaw), sy = std::sin(best.yaw);
    std::vector<HeightCell> overlap;
    for (const auto & c : cur_cells) {
      const double px = c.x - pivot.x(), py = c.y - pivot.y();
      const int gx = static_cast<int>(std::lround((cy * px - sy * py + pivot.x() - min_x + best.dx) / cell));
      const int gy = static_cast<int>(std::lround((sy * px + cy * py + pivot.y() - min_y + best.dy) / cell));
      if (gx < 0 || gy < 0 || gx >= nx || gy >= ny) {
        continue;
      }
      const float hc =
        dense[static_cast<std::size_t>(gx) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(gy)];
      if (!std::isnan(hc)) {
        overlap.push_back(HeightCell{gx * cell, gy * cell, hc});
      }
    }
    result.relief_m = detrended_std(overlap);
  }
  if (result.relief_m < o.min_relief_m) {
    result.status = "coarse: insufficient relief";
    return result;
  }
  if (result.distinctiveness < o.min_distinctiveness) {
    result.status = "coarse: ambiguous";
    return result;
  }

  // Delta in the levelled frame, then back into the candidate frame.
  Eigen::Isometry3d delta_level = Eigen::Isometry3d::Identity();
  delta_level.linear() = Eigen::AngleAxisd(best.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d pivot3(pivot.x(), pivot.y(), 0.0);
  delta_level.translation() = pivot3 - delta_level.linear() * pivot3 +
    Eigen::Vector3d(best.dx, best.dy, best.dz);
  Eigen::Isometry3d level = Eigen::Isometry3d::Identity();
  level.linear() = R_level;
  result.candidate_to_current = level.inverse() * delta_level * level * guess;
  result.accepted = true;
  result.status = "coarse: aligned";
  return result;
}

Eigen::Isometry3d pose_to_isometry(const geometry_msgs::msg::Pose & msg)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() << msg.position.x, msg.position.y, msg.position.z;
  Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  if (q.norm() < 1.0e-9) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  pose.linear() = q.toRotationMatrix();
  return pose;
}

geometry_msgs::msg::Pose isometry_to_pose(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::Pose msg;
  const Eigen::Vector3d t = pose.translation();
  const Eigen::Quaterniond q(pose.linear());
  msg.position.x = t.x();
  msg.position.y = t.y();
  msg.position.z = t.z();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  return msg;
}

PointCloud::Ptr convert_cloud(const sensor_msgs::msg::PointCloud2 & msg)
{
  auto cloud = std::make_shared<PointCloud>();
  pcl::fromROSMsg(msg, *cloud);
  std::vector<int> finite_indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, finite_indices);
  return cloud;
}

PoseInterpolator::PoseInterpolator(double max_history_s, double max_gap_s)
: max_history_s_(max_history_s), max_gap_s_(max_gap_s)
{
}

void PoseInterpolator::add(double stamp_s, const Eigen::Isometry3d & pose)
{
  if (!std::isfinite(stamp_s) || (!samples_.empty() && stamp_s <= samples_.back().stamp_s)) {
    return;
  }
  samples_.push_back(Sample{stamp_s, pose});
  while (!samples_.empty() && samples_.front().stamp_s < stamp_s - max_history_s_) {
    samples_.pop_front();
  }
}

std::optional<Eigen::Isometry3d> PoseInterpolator::at(double stamp_s) const
{
  if (samples_.empty() || stamp_s < samples_.front().stamp_s ||
    stamp_s > samples_.back().stamp_s)
  {
    return std::nullopt;
  }
  const auto upper = std::lower_bound(
    samples_.begin(), samples_.end(), stamp_s,
    [](const Sample & s, double t) {return s.stamp_s < t;});
  if (upper->stamp_s == stamp_s) {
    return upper->pose;
  }
  const auto lower = std::prev(upper);
  const double span = upper->stamp_s - lower->stamp_s;
  if (span > max_gap_s_) {
    return std::nullopt;
  }
  const double a = (stamp_s - lower->stamp_s) / span;
  const Eigen::Quaterniond q0(lower->pose.linear());
  const Eigen::Quaterniond q1(upper->pose.linear());
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = q0.slerp(a, q1).normalized().toRotationMatrix();
  out.translation() = (1.0 - a) * lower->pose.translation() + a * upper->pose.translation();
  return out;
}

PointCloud::Ptr transform_cloud(
  const PointCloud & cloud, const Eigen::Isometry3d & T, double min_range_m,
  double max_across_track_ratio)
{
  auto out = std::make_shared<PointCloud>();
  out->reserve(cloud.size());
  const Eigen::Matrix3f R = T.linear().cast<float>();
  const Eigen::Vector3f t = T.translation().cast<float>();
  const float min_range_sq = static_cast<float>(min_range_m * min_range_m);
  for (const auto & p : cloud) {
    const Eigen::Vector3f v(p.x, p.y, p.z);
    if (!v.allFinite() || v.squaredNorm() < min_range_sq) {
      continue;
    }
    if (max_across_track_ratio > 0.0 &&
      std::abs(v.y()) > static_cast<float>(max_across_track_ratio) * std::abs(v.z()))
    {
      continue;
    }
    const Eigen::Vector3f w = R * v + t;
    out->push_back(pcl::PointXYZ(w.x(), w.y(), w.z()));
  }
  return out;
}

PointCloud::Ptr downsample(const PointCloud & cloud, double voxel_leaf_m)
{
  auto out = std::make_shared<PointCloud>();
  if (voxel_leaf_m <= 0.0 || cloud.empty()) {
    *out = cloud;
    return out;
  }
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(
    static_cast<float>(voxel_leaf_m),
    static_cast<float>(voxel_leaf_m),
    static_cast<float>(voxel_leaf_m));
  voxel.setInputCloud(cloud.makeShared());
  voxel.filter(*out);
  return out;
}

SubmapDescriptor describe_cloud(const PointCloud & cloud)
{
  SubmapDescriptor descriptor;
  descriptor.point_count = cloud.size();
  if (cloud.empty()) {
    return descriptor;
  }

  Eigen::Vector3d min_point(
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity());
  Eigen::Vector3d max_point(
    -std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity());
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto & point : cloud) {
    const Eigen::Vector3d p(point.x, point.y, point.z);
    min_point = min_point.cwiseMin(p);
    max_point = max_point.cwiseMax(p);
    sum += p;
  }

  descriptor.valid = true;
  descriptor.centroid = sum / static_cast<double>(cloud.size());
  descriptor.extent = max_point - min_point;
  return descriptor;
}

std::array<double, 36> diagonal_information(
  double translation_sigma_m,
  double rotation_sigma_rad)
{
  std::array<double, 36> info{};
  const double translation_info = 1.0 / (translation_sigma_m * translation_sigma_m);
  const double rotation_info = 1.0 / (rotation_sigma_rad * rotation_sigma_rad);
  for (int i = 0; i < 3; ++i) {
    info[static_cast<std::size_t>(i * 6 + i)] = translation_info;
  }
  for (int i = 3; i < 6; ++i) {
    info[static_cast<std::size_t>(i * 6 + i)] = rotation_info;
  }
  return info;
}

SubmapManager::SubmapManager(SubmapManagerOptions options)
: options_(std::move(options))
{
}

void SubmapManager::start_submap(
  std::uint32_t id,
  const rclcpp::Time & stamp,
  const Eigen::Isometry3d & pose)
{
  current_submap_ = Submap{};
  current_submap_.id = id;
  current_submap_.stamp = stamp;
  current_submap_.pose = pose;
  current_submap_.cloud = std::make_shared<PointCloud>();
}

bool SubmapManager::has_active_points() const
{
  return current_submap_.cloud && !current_submap_.cloud->empty();
}

void SubmapManager::append_points(const PointCloud & cloud)
{
  if (!current_submap_.cloud) {
    return;
  }
  if (static_cast<int>(current_submap_.cloud->size()) >= options_.max_points_per_submap) {
    return;
  }

  const int remaining = options_.max_points_per_submap -
    static_cast<int>(current_submap_.cloud->size());
  const int count = std::min<int>(remaining, static_cast<int>(cloud.size()));
  current_submap_.cloud->insert(
    current_submap_.cloud->end(), cloud.begin(), cloud.begin() + count);
}

FinalizeSubmapResult SubmapManager::finalize_current()
{
  FinalizeSubmapResult result;
  result.submap = current_submap_;
  if (!current_submap_.cloud || current_submap_.cloud->empty()) {
    result.status = FinalizeSubmapStatus::Empty;
    return result;
  }

  result.raw_points = current_submap_.cloud->size();
  if (static_cast<int>(result.raw_points) < options_.min_points_per_submap) {
    result.status = FinalizeSubmapStatus::TooFewPoints;
    return result;
  }

  current_submap_.cloud = downsample(*current_submap_.cloud, options_.voxel_leaf_m);
  result.submap = current_submap_;
  result.final_points = current_submap_.cloud->size();
  if (static_cast<int>(result.final_points) < options_.min_points_per_submap) {
    result.status = FinalizeSubmapStatus::TooFewPointsAfterDownsample;
    return result;
  }

  result.status = FinalizeSubmapStatus::Ready;
  result.submap.descriptor = describe_cloud(*result.submap.cloud);
  current_submap_.descriptor = result.submap.descriptor;
  return result;
}

void SubmapManager::add_finalized_submap(const Submap & submap)
{
  submaps_.push_back(submap);
  while (static_cast<int>(submaps_.size()) > options_.max_submaps) {
    submaps_.pop_front();
  }
}

const std::deque<Submap> & SubmapManager::submaps() const
{
  return submaps_;
}

LoopCandidateSelector::LoopCandidateSelector(CandidateSelectionOptions options)
: options_(std::move(options))
{
}

std::vector<Submap> LoopCandidateSelector::ranked_candidates(
  const std::deque<Submap> & submaps,
  const Submap & current) const
{
  struct RankedCandidate
  {
    Submap submap;
    double distance_m{0.0};
    double score{0.0};
  };

  std::vector<RankedCandidate> ranked;
  std::vector<Submap> candidates;
  for (const auto & candidate : submaps) {
    if (current.id <= candidate.id + static_cast<std::uint32_t>(options_.min_keyframe_separation)) {
      continue;
    }
    const double distance =
      (current.pose.translation() - candidate.pose.translation()).norm();
    if (options_.max_distance_m > 0.0 && distance > options_.max_distance_m) {
      continue;
    }
    double score = candidate_distance_score(distance, options_);
    if (options_.descriptor_weight > 0.0) {
      score += options_.descriptor_weight *
        descriptor_mismatch_score(candidate.descriptor, current.descriptor, options_);
    }
    ranked.push_back(RankedCandidate{candidate, distance, score});
  }
  std::sort(ranked.begin(), ranked.end(), [](const RankedCandidate & a, const RankedCandidate & b) {
    if (std::abs(a.score - b.score) > 1.0e-12) {
      return a.score < b.score;
    }
    if (std::abs(a.distance_m - b.distance_m) > 1.0e-12) {
      return a.distance_m < b.distance_m;
    }
    return a.submap.id < b.submap.id;
  });

  candidates.reserve(ranked.size());
  for (const auto & candidate : ranked) {
    candidates.push_back(candidate.submap);
  }
  return candidates;
}

RegistrationPipeline::RegistrationPipeline(RegistrationOptions options)
: options_(std::move(options))
{
}

MatchResult RegistrationPipeline::match(
  const Submap & candidate,
  const Submap & current,
  const Eigen::Isometry3d & candidate_to_current_input) const
{
  Eigen::Isometry3d guess = candidate_to_current_input;
  std::string coarse_detail;
  auto with_detail = [&coarse_detail](MatchResult r) {
      r.coarse_detail = coarse_detail;
      return r;
    };
  if (options_.coarse.enabled) {
    const CoarseAlignResult coarse = coarse_align_heightmaps(
      *candidate.cloud, *current.cloud, candidate.pose, candidate_to_current_input,
      options_.coarse);
    if (!coarse.accepted) {
      MatchResult rejected;
      rejected.fitness = std::numeric_limits<double>::quiet_NaN();
      rejected.status = coarse.status;
      rejected.coarse_detail = coarse.detail();
      return rejected;
    }
    guess = coarse.candidate_to_current;
    coarse_detail = coarse.detail();
  }
  if (options_.backend == "icp") {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    return with_detail(run_registration(
      icp, candidate.cloud, current.cloud, guess, options_));
  }
  if (options_.backend == "ndt") {
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setResolution(static_cast<float>(options_.ndt_resolution_m));
    ndt.setStepSize(options_.ndt_step_size_m);
    ndt.setOulierRatio(options_.ndt_outlier_ratio);
    return with_detail(run_registration(
      ndt, candidate.cloud, current.cloud, guess, options_));
  }

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  return with_detail(run_registration(
    gicp, candidate.cloud, current.cloud, guess, options_));
}

DescriptorGateEvaluator::DescriptorGateEvaluator(DescriptorGateOptions options)
: options_(std::move(options))
{
}

GateResult DescriptorGateEvaluator::evaluate(const Submap & candidate, const Submap & current) const
{
  GateResult gate;
  if (!candidate.descriptor.valid || !current.descriptor.valid) {
    gate.accepted = true;
    gate.status = "descriptor gate skipped";
    return gate;
  }

  const auto min_points =
    std::min(candidate.descriptor.point_count, current.descriptor.point_count);
  const auto max_points =
    std::max(candidate.descriptor.point_count, current.descriptor.point_count);
  gate.descriptor_point_count_ratio =
    static_cast<double>(min_points) / static_cast<double>(max_points);

  gate.descriptor_centroid_distance_m =
    (candidate.descriptor.centroid - current.descriptor.centroid).norm();

  const Eigen::Vector3d candidate_extent = candidate.descriptor.extent;
  const Eigen::Vector3d current_extent = current.descriptor.extent;
  gate.descriptor_extent_ratio = std::max({
    ratio_or_infinity(candidate_extent.x(), current_extent.x()),
    ratio_or_infinity(candidate_extent.y(), current_extent.y()),
    ratio_or_infinity(candidate_extent.z(), current_extent.z())});

  if (options_.min_point_count_ratio > 0.0 &&
    gate.descriptor_point_count_ratio < options_.min_point_count_ratio)
  {
    gate.status = "descriptor gate rejected";
    return gate;
  }

  if (options_.max_centroid_distance_m > 0.0 &&
    gate.descriptor_centroid_distance_m > options_.max_centroid_distance_m)
  {
    gate.status = "descriptor gate rejected";
    return gate;
  }

  if (options_.max_extent_ratio > 0.0 &&
    gate.descriptor_extent_ratio > options_.max_extent_ratio)
  {
    gate.status = "descriptor gate rejected";
    return gate;
  }

  gate.accepted = true;
  gate.status = "descriptor gate accepted";
  return gate;
}

LoopGateEvaluator::LoopGateEvaluator(GateOptions options)
: options_(std::move(options))
{
}

GateResult LoopGateEvaluator::evaluate(
  const Eigen::Isometry3d & guess,
  const MatchResult & result) const
{
  GateResult gate;
  if (!result.success) {
    gate.status = result.status.empty() ? "registration failed" : result.status;
    return gate;
  }
  const Eigen::Isometry3d correction = loop_correction(guess, result.candidate_to_current);
  gate.correction_pose_valid = true;
  gate.correction = correction;
  gate.correction_translation_m = correction.translation().norm();
  gate.correction_rotation_rad = rotation_distance_rad(correction);

  if (options_.max_fitness_score > 0.0 && result.fitness > options_.max_fitness_score) {
    gate.status = "fitness score exceeds gate";
    return gate;
  }
  if (options_.max_correction_translation_m > 0.0 &&
    gate.correction_translation_m > options_.max_correction_translation_m)
  {
    gate.status = "translation correction exceeds gate";
    return gate;
  }
  if (options_.max_correction_rotation_rad > 0.0 &&
    gate.correction_rotation_rad > options_.max_correction_rotation_rad)
  {
    gate.status = "rotation correction exceeds gate";
    return gate;
  }
  if (options_.min_plan_view_separation_m > 0.0 &&
    options_.max_short_plan_view_rotation_rad > 0.0 &&
    plan_view_distance_m(guess) < options_.min_plan_view_separation_m &&
    gate.correction_rotation_rad > options_.max_short_plan_view_rotation_rad)
  {
    gate.status = "short plan-view rotation gate rejected";
    return gate;
  }
  gate.accepted = true;
  gate.status = "accepted";
  return gate;
}

AcceptedLoopTracker::AcceptedLoopTracker(LoopSuppressionOptions options)
: options_(std::move(options))
{
}

bool AcceptedLoopTracker::is_suppressed(std::uint32_t from_id, std::uint32_t to_id) const
{
  if (options_.min_repeat_keyframe_gap <= 0) {
    return false;
  }

  const auto gap = static_cast<std::uint32_t>(options_.min_repeat_keyframe_gap);
  return std::any_of(
    accepted_loops_.begin(), accepted_loops_.end(),
    [from_id, to_id, gap](const AcceptedLoop & loop) {
      return id_distance(loop.from_id, from_id) <= gap &&
             id_distance(loop.to_id, to_id) <= gap;
    });
}

ConsistencyCheckResult AcceptedLoopTracker::check_consistency(
  const Eigen::Isometry3d & correction) const
{
  ConsistencyCheckResult result;
  if (accepted_loops_.empty()) {
    return result;
  }
  const bool check_translation =
    options_.max_consistency_translation_delta_m > 0.0;
  const bool check_rotation =
    options_.max_consistency_rotation_delta_rad > 0.0;
  if (!check_translation && !check_rotation) {
    return result;
  }

  const auto configured_support =
    static_cast<std::size_t>(std::max(1, options_.min_consistency_support_count));
  result.required_support_count = std::min(configured_support, accepted_loops_.size());
  double nearest_score = std::numeric_limits<double>::infinity();
  std::size_t support_count = 0;
  for (const auto & loop : accepted_loops_) {
    const Eigen::Isometry3d delta = loop.correction.inverse() * correction;
    const double translation_delta_m = delta.translation().norm();
    const double rotation_delta_rad = rotation_distance_rad(delta);
    double score = 0.0;
    if (check_translation) {
      score = std::max(
        score, translation_delta_m / options_.max_consistency_translation_delta_m);
    }
    if (check_rotation) {
      score = std::max(
        score, rotation_delta_rad / options_.max_consistency_rotation_delta_rad);
    }
    if (score < nearest_score) {
      nearest_score = score;
      result.nearest_translation_delta_m = translation_delta_m;
      result.nearest_rotation_delta_rad = rotation_delta_rad;
    }
    const bool translation_supported =
      !check_translation ||
      translation_delta_m <= options_.max_consistency_translation_delta_m;
    const bool rotation_supported =
      !check_rotation ||
      rotation_delta_rad <= options_.max_consistency_rotation_delta_rad;
    if (translation_supported && rotation_supported) {
      ++support_count;
    }
  }
  result.support_count = support_count;
  result.consistent = support_count >= result.required_support_count;
  return result;
}

bool AcceptedLoopTracker::is_consistent(const Eigen::Isometry3d & correction) const
{
  return check_consistency(correction).consistent;
}

void AcceptedLoopTracker::record(std::uint32_t from_id, std::uint32_t to_id)
{
  record(from_id, to_id, Eigen::Isometry3d::Identity());
}

void AcceptedLoopTracker::record(
  std::uint32_t from_id,
  std::uint32_t to_id,
  const Eigen::Isometry3d & correction)
{
  AcceptedLoop loop;
  loop.from_id = from_id;
  loop.to_id = to_id;
  loop.correction = correction;
  accepted_loops_.push_back(loop);
}

const std::vector<AcceptedLoop> & AcceptedLoopTracker::accepted_loops() const
{
  return accepted_loops_;
}

}  // namespace aqua_sonar_loc
