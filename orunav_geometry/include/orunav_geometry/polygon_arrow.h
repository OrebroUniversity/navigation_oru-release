#pragma once

#include <orunav_generic/types.h>
#include <orunav_geometry/polygon.h>

namespace orunav_geometry {

  //! Represents an arrow as a polygon - this is currently used in the projector class.  

  // Each type could be formed as a singleton or another clever way. Another question would be to store this as parameters, but this is the simplest way for now. When entering the coordinates - do it counter clock-wise.
  
  class PolygonArrow {
  public:
    PolygonArrow() {
      poly_.points.push_back(Eigen::Vector2d(-0.8, -0.4)); // (x,y) - y represents the bottom width of the arrow
      poly_.points.push_back(Eigen::Vector2d(-0.8, 0.4));
      poly_.points.push_back(Eigen::Vector2d(0.6, 0.0)); // x represents the length of the arrow.
    }

    void update(const orunav_generic::Pose2d &pose)
    {
      posePolygon_ = Polygon(poly_);
      movePoint2dContainer(posePolygon_, pose);
    }

    const Polygon& getPosePolygon() const {
      return posePolygon_;
    }

  private:
    Polygon poly_;
    Polygon posePolygon_;
  };


  
} // namespace

