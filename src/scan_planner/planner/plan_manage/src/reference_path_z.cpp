#include <plan_manage/reference_path_z.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace scan_planner
{
  namespace
  {
    constexpr double kProgressEpsilon = 1e-6;

    double clampProgress(const double value, const double lower, const double upper)
    {
      return std::max(lower, std::min(value, upper));
    }
  } // namespace

  bool ReferencePathZProfile::setPath(const std::vector<Eigen::Vector3d> &path)
  {
    path_ = path;
    xy_progress_.assign(path_.size(), 0.0);
    for (size_t i = 1; i < path_.size(); ++i)
    {
      xy_progress_[i] = xy_progress_[i - 1] +
                        (path_[i].head<2>() - path_[i - 1].head<2>()).norm();
    }
    return valid();
  }

  bool ReferencePathZProfile::valid() const
  {
    return path_.size() >= 2 && xy_progress_.size() == path_.size() &&
           xy_progress_.back() > kProgressEpsilon;
  }

  double ReferencePathZProfile::totalProgress() const
  {
    return xy_progress_.empty() ? 0.0 : xy_progress_.back();
  }

  double ReferencePathZProfile::sampleZ(const double progress) const
  {
    if (path_.empty())
      return 0.0;
    if (!valid())
      return path_.back().z();

    const double clamped = clampProgress(progress, 0.0, totalProgress());
    const auto upper = std::upper_bound(xy_progress_.begin(), xy_progress_.end(),
                                        clamped + kProgressEpsilon);
    size_t begin_index = upper == xy_progress_.begin() ? 0 :
        static_cast<size_t>(upper - xy_progress_.begin() - 1);
    begin_index = std::min(begin_index, path_.size() - 1);

    size_t end_index = begin_index + 1;
    while (end_index < path_.size() &&
           xy_progress_[end_index] <= xy_progress_[begin_index] + kProgressEpsilon)
      ++end_index;
    if (end_index >= path_.size())
      return path_[begin_index].z();

    const double length = xy_progress_[end_index] - xy_progress_[begin_index];
    const double ratio = clampProgress((clamped - xy_progress_[begin_index]) / length, 0.0, 1.0);
    return path_[begin_index].z() + ratio * (path_[end_index].z() - path_[begin_index].z());
  }

  size_t ReferencePathZProfile::segmentIndex(const double progress) const
  {
    if (path_.size() < 2)
      return 0;

    const double clamped = clampProgress(progress, 0.0, totalProgress());
    const auto upper = std::upper_bound(xy_progress_.begin(), xy_progress_.end(),
                                        clamped + kProgressEpsilon);
    const size_t index = upper == xy_progress_.begin() ? 0 :
        static_cast<size_t>(upper - xy_progress_.begin() - 1);
    return std::min(index, path_.size() - 2);
  }

  Eigen::Vector2d ReferencePathZProfile::sampleXY(const double progress) const
  {
    if (path_.empty())
      return Eigen::Vector2d::Zero();
    if (!valid())
      return path_.back().head<2>();

    const double clamped = clampProgress(progress, 0.0, totalProgress());
    const size_t index = segmentIndex(clamped);
    size_t end_index = index + 1;
    while (end_index < path_.size() &&
           xy_progress_[end_index] <= xy_progress_[index] + kProgressEpsilon)
      ++end_index;
    if (end_index >= path_.size())
      return path_[index].head<2>();

    const double length = xy_progress_[end_index] - xy_progress_[index];
    const double ratio = clampProgress(
        (clamped - xy_progress_[index]) / length, 0.0, 1.0);
    return path_[index].head<2>() + ratio *
        (path_[end_index].head<2>() - path_[index].head<2>());
  }

  double ReferencePathZProfile::projectProgress(const Eigen::Vector3d &point,
                                                 const double min_progress,
                                                 const double max_progress) const
  {
    if (!valid())
      return 0.0;

    const double lower = clampProgress(min_progress, 0.0, totalProgress());
    const double upper = clampProgress(std::max(lower, max_progress), lower,
                                       totalProgress());
    const Eigen::Vector2d query = point.head<2>();
    double best_distance_sq = std::numeric_limits<double>::infinity();
    double best_progress = lower;
    for (size_t i = 1; i < path_.size(); ++i)
    {
      const Eigen::Vector2d begin = path_[i - 1].head<2>();
      const Eigen::Vector2d segment = path_[i].head<2>() - begin;
      const double segment_length = segment.norm();
      if (segment_length <= kProgressEpsilon)
        continue;

      const double segment_lower = std::max(lower, xy_progress_[i - 1]);
      const double segment_upper = std::min(upper, xy_progress_[i]);
      if (segment_lower > segment_upper + kProgressEpsilon)
        continue;

      const double min_ratio = (segment_lower - xy_progress_[i - 1]) / segment_length;
      const double max_ratio = (segment_upper - xy_progress_[i - 1]) / segment_length;
      const double ratio = clampProgress(
          (query - begin).dot(segment) / segment.squaredNorm(), min_ratio, max_ratio);
      const double progress = xy_progress_[i - 1] + ratio * segment_length;
      const double distance_sq = (query - (begin + ratio * segment)).squaredNorm();
      if (distance_sq < best_distance_sq - kProgressEpsilon ||
          (std::fabs(distance_sq - best_distance_sq) <= kProgressEpsilon &&
           progress < best_progress))
      {
        best_distance_sq = distance_sq;
        best_progress = progress;
      }
    }
    return best_progress;
  }

  ReferencePathZApplyResult ReferencePathZProfile::applyToInitialPath(
      std::vector<Eigen::Vector3d> &points, double start_progress,
      const double projection_tolerance, const double start_z,
      const double target_z) const
  {
    ReferencePathZApplyResult result;
    if (points.empty() || !valid())
      return result;

    start_progress = clampProgress(start_progress, 0.0, totalProgress());
    const double tolerance = std::max(0.0, projection_tolerance);
    double local_progress = 0.0;
    double previous_progress = start_progress;
    points.front().z() = start_z;
    for (size_t i = 1; i < points.size(); ++i)
    {
      local_progress += (points[i].head<2>() - points[i - 1].head<2>()).norm();
      const double max_progress = std::min(
          totalProgress(), start_progress + local_progress + tolerance);
      previous_progress = projectProgress(points[i], previous_progress, max_progress);
      points[i].z() = sampleZ(previous_progress);
    }

    result.final_progress = previous_progress;
    result.final_profile_z = sampleZ(previous_progress);
    result.final_segment_index = segmentIndex(previous_progress);
    points.back().z() = target_z;
    return result;
  }
} // namespace scan_planner
