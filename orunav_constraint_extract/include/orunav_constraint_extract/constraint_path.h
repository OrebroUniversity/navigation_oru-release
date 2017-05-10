#pragma once

#include <orunav_generic/types.h>
#include <orunav_generic/path_utils.h>

namespace constraint_extract {

  class ConstraintPath : public orunav_generic::PathInterface  {
  public:
    ConstraintPath(const orunav_generic::PathInterface &p);
    //! Computes a sumbsampled path.
    const orunav_generic::Path& extractPath(double minDist, double maxDist, double maxRotation);
    const orunav_generic::Path& getOrigPath() const { return orig_path; }
    const std::vector<unsigned int> getSubsampledIdx() const { return subsampled_idx; }
    bool addStep(const orunav_generic::Pose2d &p1, const orunav_generic::Pose2d &p2, double maxDist, double maxRotation) const;

  private:
    orunav_generic::Path orig_path;
    orunav_generic::Path path;
    std::vector<unsigned int> subsampled_idx;
  };
  
} // namespace
