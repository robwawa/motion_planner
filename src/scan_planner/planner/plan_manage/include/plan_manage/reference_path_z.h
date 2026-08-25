#ifndef _REFERENCE_PATH_Z_H_
#define _REFERENCE_PATH_Z_H_

#include <Eigen/Eigen>

#include <vector>

namespace scan_planner
{
  struct ReferencePathZApplyResult
  {
    double final_progress{0.0};
    double final_profile_z{0.0};
    size_t final_segment_index{0};
  };

  class ReferencePathZProfile
  {
  public:
    bool setPath(const std::vector<Eigen::Vector3d> &path);
    bool valid() const;

    double totalProgress() const;
    double sampleZ(double progress) const;
    Eigen::Vector2d sampleXY(double progress) const;
    double projectProgress(const Eigen::Vector3d &point, double min_progress,
                           double max_progress) const;

    ReferencePathZApplyResult applyToInitialPath(
        std::vector<Eigen::Vector3d> &points, double start_progress,
        double projection_tolerance, double start_z, double target_z) const;

  private:
    std::vector<Eigen::Vector3d> path_;
    std::vector<double> xy_progress_;

    size_t segmentIndex(double progress) const;
  };
} // namespace scan_planner

#endif
