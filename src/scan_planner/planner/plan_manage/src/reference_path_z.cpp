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

  double ReferencePathZProfile::projectProgress(const Eigen::Vector3d &point,
                                                 const double min_progress,
                                                 const double max_progress) const
  {
    if (!valid())
      return 0.0;

    const double lower = clampProgress(min_progress, 0.0, totalProgress());
    const double upper = clampProgress(std::max(min_progress, max_progress), lower, totalProgress());
    const Eigen::Vector2d query = point.head<2>();
    double best_distance_sq = std::numeric_limits<double>::infinity();
    double best_progress = lower;

    for (size_t i = 1; i < path_.size(); ++i)
    {
      const Eigen::Vector2d begin = path_[i - 1].head<2>();
      const Eigen::Vector2d segment = path_[i].head<2>() - begin;
      const double length = segment.norm();
      if (length <= kProgressEpsilon)
        continue;

      const double segment_lower = std::max(lower, xy_progress_[i - 1]);
      const double segment_upper = std::min(upper, xy_progress_[i]);
      if (segment_lower > segment_upper + kProgressEpsilon)
        continue;

      const double unconstrained_ratio = (query - begin).dot(segment) / segment.squaredNorm();
      const double progress = clampProgress(xy_progress_[i - 1] +
                                                 clampProgress(unconstrained_ratio, 0.0, 1.0) * length,
                                             segment_lower, segment_upper);
      const double ratio = (progress - xy_progress_[i - 1]) / length;
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

  void ReferencePathZProfile::applyToInitialPath(std::vector<Eigen::Vector3d> &points,
                                                   double start_progress, double target_progress,
                                                   const double start_z, const double target_z) const
  {
    if (points.empty() || !valid())
      return;

    start_progress = clampProgress(start_progress, 0.0, totalProgress());
    target_progress = clampProgress(std::max(start_progress, target_progress),
                                    start_progress, totalProgress());

    double previous_progress = start_progress;
    points.front().z() = start_z;
    for (size_t i = 1; i < points.size(); ++i)
    {
      previous_progress = projectProgress(points[i], previous_progress, target_progress);
      points[i].z() = sampleZ(previous_progress);
    }
    points.back().z() = target_z;
  }
} // namespace scan_planner
