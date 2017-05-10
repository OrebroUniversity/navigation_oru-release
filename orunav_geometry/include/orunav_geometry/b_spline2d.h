#pragma once

#include <orunav_geometry/b_spline.h>
#include <orunav_generic/pose2d.h>
#include <orunav_generic/types.h>

//! Class to handle splines in two dimensions positions (only x and y) and to interact with Pose2d.
/*!
 * This is also used by the spline following driver. Unless the t-parameter is specified it will
 * be selected to be -1 for the first knot (non drivable) and then incement by 1 at ever new knot.
 */
class BSpline2d
{
public:
  class Evaluation
  {
  public:
    Evaluation()
      {
	max_abs_steering_angle = 0.;
	sum_of_steering_angle_change = 0.;
	sum_of_rotation_change = 0.;
	max_abs_heading_change = 0.;
      }
    void setToWorst()
    {
      max_abs_steering_angle = std::numeric_limits<double>::max();
      sum_of_steering_angle_change = std::numeric_limits<double>::max();
      sum_of_rotation_change = std::numeric_limits<double>::max();
      max_abs_heading_change = std::numeric_limits<double>::max();
    }
    bool valid()
    {
      if (max_abs_steering_angle > 1.05)
      	return false;
      return true;
    }
    bool betterThan(const Evaluation &eval)
    {
      if (this->sum_of_steering_angle_change < eval.sum_of_steering_angle_change)
      	return true;
      /* if (this->sum_of_rotation_change < eval.sum_of_rotation_change) */
      /* 	return true; */
      
      return false;
    }
    double max_abs_steering_angle;
    double sum_of_steering_angle_change;
    double sum_of_rotation_change;
    double max_abs_heading_change;
    friend std::ostream& operator<<(std::ostream &os, const BSpline2d::Evaluation &obj)
      {
	os << "\nmax_abs_Phi    : " << obj.max_abs_steering_angle;
	os << "\tsum_diff_Phi   : " << obj.sum_of_steering_angle_change;
	os << "\tsum_diff_Th    : " << obj.sum_of_rotation_change;
	os << "\tmax_abs_diff_Th: " << obj.max_abs_heading_change; 
	return os;
      }
   
  };

     //! Constructor
     BSpline2d();

     //! Destructor
     ~BSpline2d();

     //! Return the number of knots.
     unsigned int getNbKnots() const;

     //! Return if the spline is valid > 3 knots.
     bool isValid() const { return (this->getNbKnots() > 3); }

     //! Set a spline giving a set of poses, the spline will pass through the provided poses
     void setPoses(const orunav_generic::Pose2dContainerInterface &poses, double speed, double knotOffset = 1.);
     void setPosesAsKnots(const orunav_generic::Pose2dContainerInterface &poses, double speed, double knotOffset = 1.);

     // Add a single pose creating 3 knots.
     void addPose(const orunav_generic::Pose2d &pose, double speed, double knotOffset);
     // Add a single pose extracting only the position (x,y) to the knot.
     void addKnot(const orunav_generic::Pose2d &pose, double speed);

     //! Set the knots directly.
     void setKnots(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &speed);

     //! Set a spline giving a set of poses with speed.
//     void setPoses(const std::vector<lsl::lslPose3d> &poses, std::vector<double> speeds);

     double getKnotX(unsigned int idx) const { return _x.getValue(idx); }
     double getKnotY(unsigned int idx) const { return _y.getValue(idx); }
     double getKnotSpeed(unsigned int idx) const { return _speed.getValue(idx); }
     double getT(unsigned int idx) const;
     void getKnots(orunav_generic::PositionVec &pos) const;

     //! Return a pose at the value \param t.
     orunav_generic::Pose2d getPose(double t) const;
     double getSpeed(double t) const;
     double getSteeringAngle(double t, double L) const;
     //! Return the curvature at the value \param t.
     bool getCurvature(double &curvature, double t) const;

     //! Calculate a set of waypoint using \param resolution, which refers to the t value in the spline.
     void calcWaypoints(orunav_generic::Pose2dVec &points, double resolution) const;

     void calcPath(orunav_generic::Path &path, double L, double resolution) const;

     orunav_generic::Pose2d getGoal() const;

     //! Return if the robot will drive forward along this spline (not reversing).
     bool getForwardHeading() const;

     //! Clears all knots in the spline.
     void clear();

     bool collision(const orunav_generic::CollisionCheckInterface &collisionCheck, double resolution) const;
     bool evaluate(BSpline2d::Evaluation &eval, double L, double minSteeringAngle, double maxSteeringAngle, double resolution) const;


private:
     void calcKnots(const orunav_generic::Pose2d &pose, std::vector<double> &x, std::vector<double> &y, double offset);
     BSpline _x;
     BSpline _y;
     BSpline _speed;
     double _poseKnotOffset;

};
