#pragma once

#include <orunav_generic/types.h>

namespace orunav_generic {

  inline Eigen::Vector2d getClosestPoint2dToOrigin(const Point2dContainerInterface &pts) {
    assert(pts.sizePoint2d() > 0);
    size_t min_idx = 0;
    double min_dist = pts.getPoint2d(0).squaredNorm();
    for (size_t i = 1; i < pts.sizePoint2d(); i++) {
      double tmp = pts.getPoint2d(i).squaredNorm();
      if (tmp < min_dist) {
	min_idx = i;
	tmp = min_dist;
      }
    }
    return pts.getPoint2d(min_idx);
  }

  inline double getTotalTimeBetween(const DeltaTInterface &deltaT, size_t start, size_t end) {
    double ret = 0.;
    assert(start <= end);
    assert(end <= deltaT.sizeDeltaTVec());
    
    for (size_t i = start; i < end; i++) {
      ret += deltaT.getDeltaT(i);
    }
    return ret;
  }

  inline double getTotalTime(const DeltaTInterface &deltaT) {
    return getTotalTimeBetween(deltaT, 0, deltaT.sizeDeltaTVec());
  }

  inline std::vector<double> getDoubleTimeVecFromDeltaTInterface(const DeltaTInterface &deltaT) {
    std::vector<double> ret(deltaT.sizeDeltaTVec());
    for (size_t i = 0; i < deltaT.sizeDeltaTVec(); i++) {
      ret[i] = deltaT.getDeltaT(i);
    }
    return ret;
  }

  inline Point2dVec createPoint2dVecFromPose2dContainerInterface(const Pose2dContainerInterface &p) {
    Point2dVec ret;
    for (size_t i = 0; i < p.sizePose2d(); i++) {
      ret.push_back(Eigen::Vector2d(p.getPose2d(i)[0],p.getPose2d(i)[1]));
    }
    return ret;
  }

inline orunav_generic::CoordinatedTimes computeCoordinatedTimesFromDeltaTs(const std::vector<double> &times) {
  orunav_generic::CoordinatedTimes ret;
  if (times.empty())
    return ret;
  
  ret.push_back(0.);
  for (size_t i = 0; i < times.size()-1; i++) { // The last DT is not valid - just to keep the sizes equal(!).
    ret.push_back(ret.back() + times[i]);
  }
  return ret;
}

//  inline PositionVec createPositionVecFromPose2dContainerInterface(const Pose2dContainerInterface &p) { 
//      PositionVec ret;
//      for (size_t i = 0; i < p.sizePose2d(); i++) {
//	ret.push_back(Eigen::Vector3d(p.getPose2d(i)[0], p.getPose2d(i)[1], 0.));
//      }
//      return ret;
//    }
    
inline State2d addState2d(const State2d &origin, const State2d &incr) 
{
  State2d ret;
  ret.pose = addPose2d(origin.getPose2d(), incr.getPose2d());
  ret.steeringAngle = origin.getSteeringAngle() + incr.getSteeringAngle();
  return ret;
}

//! Return the relative state between the origin and the 'state'.
inline State2d subState2d(const State2d &origin, const State2d &state)
{
  State2d ret;
  ret.pose = subPose2d(origin.getPose2d(), state.getPose2d());
  ret.steeringAngle = state.getSteeringAngle() - origin.getSteeringAngle();
  return ret;
}




} // namespace


