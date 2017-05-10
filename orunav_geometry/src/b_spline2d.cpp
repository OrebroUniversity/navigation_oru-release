#include <orunav_geometry/b_spline2d.h>


BSpline2d::BSpline2d()
{

}

BSpline2d::~BSpline2d()
{

}

unsigned int
BSpline2d::getNbKnots() const
{
     assert(_x.size() == _y.size());
     assert(_x.size() == _speed.size());

     return _x.size();
}

void
BSpline2d::setPoses(const orunav_generic::Pose2dContainerInterface &poses, double speed, double knotOffset)
{
     clear();

     double t = -1.;

     if (speed < 0)
       knotOffset = -knotOffset;

     for (unsigned int i = 0; i < poses.sizePose2d(); i++)
     {
	  std::vector<double> x;
	  std::vector<double> y;

	  calcKnots(poses.getPose2d(i), x, y, knotOffset);

	  assert(x.size() == y.size());
	  
	  for (unsigned int i = 0; i < x.size(); i++)
	  {
	       _x.addPoint(x[i], t);
	       _y.addPoint(y[i], t);
	       _speed.addPoint(speed, t);
	       t++;
	  }
     }
}

void 
BSpline2d::setPosesAsKnots(const orunav_generic::Pose2dContainerInterface &poses, double speed, double knotOffset)
{
  clear();
  
  unsigned int size = poses.sizePose2d();
  assert(size >= 2);
  // Need to add the first and the last path pose as a 3 knots
  addPose(poses.getPose2d(0), speed, knotOffset);
  for (unsigned int i = 1; i < size - 1; i++)
    {
      addKnot(poses.getPose2d(i), speed);
    }
  addPose(poses.getPose2d(size - 1), speed, knotOffset);
}

void
BSpline2d::addPose(const orunav_generic::Pose2d &pose, double speed, double knotOffset)
{
     std::vector<double> x;
     std::vector<double> y;
     
     if (speed < 0)
       knotOffset = -knotOffset;

     calcKnots(pose, x, y, knotOffset);
     
     assert(x.size() == y.size());
     
     double t = -1.;
     if (this->getNbKnots() > 0)
	  t = getT(getNbKnots()-1)+1;
     for (unsigned int i = 0; i < x.size(); i++)
     {
	  _x.addPoint(x[i], t);
	  _y.addPoint(y[i], t);
	  _speed.addPoint(speed, t);
	  t++;
     }
}

void
BSpline2d::addKnot(const orunav_generic::Pose2d &pose, double speed)
{
  assert(_x.size() == _y.size());
  assert(_x.size() == _speed.size());
  
  double t = -1.;
  if (this->getNbKnots() > 0)
    t = getT(getNbKnots()-1)+1;
  
  
  _x.addPoint(pose(0), t);
  _y.addPoint(pose(1), t);
  _speed.addPoint(speed, t);
}

void 
BSpline2d::setKnots(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &speed)
{
     clear();
     assert(x.size() == y.size());
     assert(x.size() == speed.size());
     for (unsigned int i = 0; i < x.size(); i++)
     {
	  _x.addPoint(x[i], i - 1);
	  _y.addPoint(y[i], i - 1);
	  _speed.addPoint(speed[i], i - 1);
     }
}


void 
BSpline2d::calcKnots(const orunav_generic::Pose2d &pose, std::vector<double> &x, std::vector<double> &y, double offset)
{
  double mx = pose(0);
  double my = pose(1);
  double mth = pose(2);

     x.resize(3);
     y.resize(3);

     x[1] = mx;
     y[1] = my;

     x[2] = mx + offset*cos(mth);
     x[0] = mx - offset*cos(mth);

     y[2] = my + offset*sin(mth);
     y[0] = my - offset*sin(mth);
}

void
BSpline2d::clear()
{
     _x.clear();
     _y.clear();
     _speed.clear();
}

double
BSpline2d::getT(unsigned int idx) const
{
     double t = _x.getValueT(idx);
     assert(t == _y.getValueT(idx));
     assert(t == _speed.getValueT(idx));

     return t;
}

void 
BSpline2d::getKnots(orunav_generic::PositionVec &pos) const
{
  pos.resize(this->getNbKnots());
  for (unsigned int i = 0; i < this->getNbKnots(); i++)
    {
      pos[i] << _x.getValue(i), _y.getValue(i), 0.;
    }
}

orunav_generic::Pose2d
BSpline2d::getPose(double t) const
{
     double x, y, dx, dy;
     if (_x.getValue(x, t) && _y.getValue(y, t) && _x.getValueFirstDer(dx, t) && _y.getValueFirstDer(dy, t))
       //     if (_x.getValue(x, t) && _y.getValue(y, t) && _x.getValueFirstDerApprox(dx, t) && _y.getValueFirstDerApprox(dy, t))
     {
	  // Ok.
     }
     else
     {
	  assert(false);
     }
//     std::cout << "x : " << x << " y : " << y << " dx : " << dx << " dy : " << dy << std::endl;
     if (getSpeed(t) >= 0)
       return orunav_generic::Pose2d(x, y, atan2(dy, dx));
     else
       return orunav_generic::Pose2d(x, y, angles::normalize_angle(atan2(dy, dx)+M_PI));
}

double
BSpline2d::getSteeringAngle(double t, double L) const
{
  double x, xdot, xdotdot;
  double y, ydot, ydotdot;
  if (!_x.getAllValues(x, xdot, xdotdot, t))
    assert(false);
  if (!_y.getAllValues(y, ydot, ydotdot, t))
    assert(false);
  double direction = 1.;
  if (getSpeed(t) < 0)
    direction = -1.;
  return atan(direction*L*(xdot*ydotdot - ydot*xdotdot)/pow(xdot*xdot + ydot*ydot, 1.5));
}

double
BSpline2d::getSpeed(double t) const
{
     return _speed.getValue(t);
}

bool
BSpline2d::getCurvature(double &curvature, double t) const
{
  double x, xdot, xdotdot;
  double y, ydot, ydotdot;
  if (!_x.getAllValues(x, xdot, xdotdot, t))
    return false;
  if (_y.getAllValues(y, ydot, ydotdot, t))
    return false;
  curvature =  (xdot*ydotdot - ydot*xdotdot)/pow(1 + ydot*ydot, 1.5);
  return true;
}


bool 
BSpline2d::collision(const orunav_generic::CollisionCheckInterface &collisionCheck, double resolution) const
{
     double min_T = getT(1);
     double max_T = getT(_x.size() - 2);

     if (collisionCheck.collision(this->getPose(min_T)))
       return true;
     double tmp_T = min_T+resolution;
     while (tmp_T < max_T)
     {
       if (collisionCheck.collision(this->getPose(tmp_T)))
	 return true;
       tmp_T += resolution;
     }
     if (collisionCheck.collision(this->getPose(max_T)))
	 return true;
     return false;
}


bool
BSpline2d::evaluate(BSpline2d::Evaluation &eval, double L, double minSteeringAngle, double maxSteeringAngle, double resolution) const
{
     // Only calc from the second to the second last (the end knots shouldn't be part of any trajectory).
     double min_T = getT(1);
     double max_T = getT(_x.size() - 2);

     double last_angle = getSteeringAngle(min_T, L);
     eval.max_abs_steering_angle = fabs(last_angle);
     eval.sum_of_steering_angle_change = 0.;
     eval.max_abs_heading_change = 0.;
     eval.sum_of_rotation_change = 0.;

     double last_heading = getPose(min_T)(2);
     double heading_change = 0.;
     double tmp_T = min_T+resolution;
     
     double tmp;

     while (tmp_T < max_T)
     {
       tmp = getSteeringAngle(tmp_T, L);
       if (tmp < minSteeringAngle || tmp > maxSteeringAngle) {
	 std::cout << "steering angle failure : " << tmp << std::endl;
	 return false;
       }
       //std::cout << eval.max_abs_steering_angle << ", " << fabs(tmp) << ", " << std::flush;
       eval.max_abs_steering_angle = std::max(fabs(tmp), eval.max_abs_steering_angle);
              
       //std::cout << eval.max_abs_steering_angle << std::endl;
  
       eval.sum_of_steering_angle_change += fabs(angles::normalize_angle(tmp - last_angle));
       last_angle = tmp;

       tmp = getPose(tmp_T)(2);
       heading_change = fabs(angles::normalize_angle(tmp - last_heading));
       if (heading_change > 1.) { // TODO - Add as a param. 
	 std::cout << "heading change failure : " << heading_change << std::endl;
	 return false;
       }
       eval.sum_of_rotation_change += heading_change;
       last_heading = tmp;
       eval.max_abs_heading_change = std::max(heading_change, eval.max_abs_heading_change);

       tmp_T += resolution;
     }
     tmp = getSteeringAngle(max_T, L);
     eval.max_abs_steering_angle = std::max(fabs(tmp), eval.max_abs_steering_angle);
     eval.sum_of_steering_angle_change += fabs(angles::normalize_angle(tmp - last_angle));
     tmp = getPose(max_T)(2);
     heading_change = fabs(angles::normalize_angle(tmp - last_heading));
     eval.sum_of_rotation_change += heading_change;
     eval.max_abs_heading_change = std::max(heading_change, eval.max_abs_heading_change);
     return true;
}

void 
BSpline2d::calcWaypoints(orunav_generic::Pose2dVec &points, double resolution) const
{
     points.clear();

     assert(_x.size() > 3);
     // Only calc from the second to the second last (the end knots shouldn't be part of any trajectory).
     double min_T = getT(1);
     double max_T = getT(_x.size() - 2);

     //     points.push_back(WayPoint(this->getPose(min_T), this->getSpeed(min_T)));
     points.push_back(this->getPose(min_T));
     double tmp_T = min_T+resolution;
     while (tmp_T < max_T)
     {
       //	  points.push_back(WayPoint(this->getPose(tmp_T), this->getSpeed(tmp_T)));
       points.push_back(this->getPose(tmp_T));
	  tmp_T += resolution;
     }
     // Add the final (goal) waypoint.
     //     points.push_back(WayPoint(this->getPose(max_T), this->getSpeed(max_T)));
     points.push_back(this->getPose(max_T));
}

void 
BSpline2d::calcPath(orunav_generic::Path &path, double L, double resolution) const
{
  path.clear();

     assert(_x.size() > 3);
     // Only calc from the second to the second last (the end knots shouldn't be part of any trajectory).
     double min_T = getT(1);
     double max_T = getT(_x.size() - 2);

     path.poses.push_back(this->getPose(min_T));
     path.steeringAngles.push_back(this->getSteeringAngle(min_T, L));
     
     double tmp_T = min_T+resolution;
     while (tmp_T < max_T)
     {
       path.poses.push_back(this->getPose(tmp_T));
       path.steeringAngles.push_back(this->getSteeringAngle(tmp_T, L));
       tmp_T += resolution;
     }
     // Add the final (goal) waypoint.
     path.poses.push_back(this->getPose(max_T));
     path.steeringAngles.push_back(this->getSteeringAngle(max_T, L));
}



orunav_generic::Pose2d
BSpline2d::getGoal() const
{
     return getPose(getT(getNbKnots() - 2));
}

bool 
BSpline2d::getForwardHeading() const
{
     if (getSpeed(0.) >= 0)
	  return true;
     return false;
}
