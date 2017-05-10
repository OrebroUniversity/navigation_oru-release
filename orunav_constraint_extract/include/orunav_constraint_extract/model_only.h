#pragma once

#include <orunav_constraint_extract/constraints.h>
#include <orunav_generic/subsample_path.h>
#include <orunav_geometry/robot_model_2d.h>

namespace constraint_extract {


//! Provides a method to extract contraint only relying on the provided path and the model - ant NOT any map. The provided path is instead assumed to be collision free.
class ConstraintExtractorModelOnly : public ConstraintExtractorInterface {
 public:
  class Params {
  public:
    Params() { 
      minDist = 0.1; 
      maxDist = 0.2; 
      maxRotation = 0.7; 
      forceOuterConvex = false;
      forceConstraintsPerPathPoint = false; // Currently needed in the scheduling?
      innerOffset = orunav_generic::Pose2d(0.1, 0.1, 0.1);
      forceParkingPolygons = true;
    }

    double minDist;
    double maxDist;
    double maxRotation;
    bool forceOuterConvex;
    bool forceConstraintsPerPathPoint;
    orunav_generic::Pose2d innerOffset;
    bool forceParkingPolygons;
  };

  ConstraintExtractorModelOnly(const orunav_generic::PathInterface &p, const orunav_geometry::RobotModel2dInterface &m, orunav_generic::RobotInternalState2d::LoadType loadType);
  
  void compute();

  const std::vector<size_t>& getSubSampledIdx() const;
  const orunav_geometry::Polygons& getOuterConstraints() const;
  const orunav_geometry::Polygons& getInnerConstraints() const;
  
  Params params;
  orunav_geometry::Polygons outerConstraints;
  orunav_geometry::Polygons innerConstraints;
  std::vector<size_t> sub_idx;
  
  const orunav_generic::PathInterface &path;
  const orunav_geometry::RobotModel2dInterface &model;
  
  private:
  orunav_geometry::Polygons upSample(const orunav_geometry::Polygons &polygons, const std::vector<size_t>& subsample_idx);
  
  bool computed_;
  orunav_generic::RobotInternalState2d::LoadType loadType_;
};
  
} // namespace
