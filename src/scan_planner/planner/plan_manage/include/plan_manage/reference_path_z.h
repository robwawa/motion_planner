#ifndef _REFERENCE_PATH_Z_H_
#define _REFERENCE_PATH_Z_H_

#include <Eigen/Eigen>

#include <vector>

namespace scan_planner
{
  class ReferencePathZProfile
  {
  public:
    bool setPath(const std::vector<Eigen::Vector3d> &path);
    bool valid() const;

    double totalProgress() const;
    double sampleZ(double progress) const;
    double projectProgress(const Eigen::Vector3d &point, double min_progress,
                           double max_progress) const;

    void applyToInitialPath(std::vector<Eigen::Vector3d> &points,
                            double start_progress, double target_progress,
                            double start_z, double target_z) const;

  private:
    std::vector<Eigen::Vector3d> path_;
    std::vector<double> xy_progress_;
  };
} // namespace scan_planner

#endif
