#pragma once

#include <orunav_conversions/conversions.h>

namespace orunav_conversions
{

constraint_extract::PolygonConstraint createPolygonConstraintFromPolygonConstraintMsg(const orunav_msgs::PolygonConstraint &msg) {
  constraint_extract::PolygonConstraint ret;
  // The polygon (xy)...
  ret.poly_ =  createConvexPolygonFromPolygonConstraintMsg(msg);
  // ... and the orientation th.
  ret.thBounds_[0] = msg.theta_min;
  ret.thBounds_[1] = msg.theta_max;
  // ... feasibility point
  ret.feasiblePose_[0] = msg.feasible_x;
  ret.feasiblePose_[1] = msg.feasible_y;
  ret.feasiblePose_[2] = msg.feasible_th;

  return ret;
}

constraint_extract::PolygonConstraintsVec createPolygonConstraintsVecFromRobotConstraintsMsg(const orunav_msgs::RobotConstraints &msg) {
  constraint_extract::PolygonConstraintsVec ret;

  for (unsigned int i = 0; i < msg.constraints.size(); i++) {
    constraint_extract::PolygonConstraint p = createPolygonConstraintFromPolygonConstraintMsg(msg.constraints[i]);
    if (i <  msg.constraints_outer.size())
      p.outerPoly_ = createConvexPolygonFromPolygonConstraintMsg(msg.constraints_outer[i]);

    ret.push_back(p);
  }
  return ret;
}

orunav_msgs::RobotConstraints createRobotConstraintsFromPolygonConstraintsVec(const constraint_extract::PolygonConstraintsVec &pcv) {
  orunav_msgs::RobotConstraints msg;
  for (size_t i = 0; i <  pcv.size(); i++) {
    orunav_msgs::PolygonConstraint poly = createPolygonConstraintMsgFromConvexPolygon(pcv[i].getInnerConstraint());
    poly.constraint_id = i;
    poly.theta_min = pcv[i].getThBounds()[0];
    poly.theta_max = pcv[i].getThBounds()[1];
    poly.feasible_x = pcv[i].feasiblePose_[0];
    poly.feasible_y = pcv[i].feasiblePose_[1];
    poly.feasible_th = pcv[i].feasiblePose_[2];
    msg.constraints.push_back(poly);
    //msg.points.push_back(i);
    
    orunav_msgs::PolygonConstraint poly_outer = createPolygonConstraintMsgFromConvexPolygon(pcv[i].getOuterConstraint());
    poly_outer.constraint_id = i;
    poly_outer.theta_min = pcv[i].getThBounds()[0];
    poly_outer.theta_max = pcv[i].getThBounds()[1];
    poly_outer.feasible_x = pcv[i].feasiblePose_[0];
    poly_outer.feasible_y = pcv[i].feasiblePose_[1];
    poly_outer.feasible_th = pcv[i].feasiblePose_[2];
    poly_outer.start_point_idx = pcv[i].start_point_idx;
    poly_outer.end_point_idx = pcv[i].end_point_idx;  
    poly_outer.outer_constraint_points = pcv[i].outer_constraint_points;
    msg.constraints_outer.push_back(poly_outer);
  }
  return msg;
}

} // namespace
