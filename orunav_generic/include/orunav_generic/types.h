#pragma once

#include <orunav_generic/interfaces.h>

namespace orunav_generic {

  class RobotInternalState2d {
  public:
    enum LoadType { NO_LOAD = 0, EUR_PALLET, HALF_PALLET, UNKNOWN };
    RobotInternalState2d() { loadType = RobotInternalState2d::NO_LOAD; steeringAngle = 0.; speed = 0.; }
    LoadType loadType;
    double steeringAngle;
    double speed; // This would only be used to consider the increased safety zone while driving faster.
  };

  //! Container of positions (x,y,z) (Eigen::Vector3d)
  class PositionVec : public std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >, public PositionContainerInterface
  {
  public:
    PositionVec() { }
    PositionVec(const PositionContainerInterface &pts) {
      for (size_t i = 0; i < pts.sizePos(); i++) {
	this->push_back(pts.getPos(i));
      }
    }
    Eigen::Vector3d getPos(size_t idx) const {
      return (*this)[idx];
    }
    void setPos(const Eigen::Vector3d &pos, size_t idx) {
      (*this)[idx] = pos;
    }
    size_t sizePos() const { return this->size(); }
  };
  
  
  //! Container class of Point2d.
  class Point2dVec : public std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >, public Point2dContainerInterface
  {
  public:
    Point2dVec() { }
    Point2dVec(const Point2dContainerInterface &pts) {
      for (size_t i = 0; i < pts.sizePoint2d(); i++) {
	this->push_back(pts.getPoint2d(i));
      }
    }
    Eigen::Vector2d getPoint2d(size_t idx) const { return (*this)[idx]; }
    void setPoint2d(const Eigen::Vector2d& pt, size_t idx) { (*this)[idx] = pt; }
    size_t sizePoint2d() const { return this->size(); }
  };

  //! Container class of pose2d.
  class Pose2dVec : public std::vector<Pose2d, Eigen::aligned_allocator<Pose2d> >, public Pose2dContainerInterface
  {
  public:
    Pose2dVec() { }
    Pose2dVec(const Pose2dContainerInterface &p) {
      for (size_t i = 0; i < p.sizePose2d(); i++) {
	this->push_back(p.getPose2d(i));
      }
    }
    Pose2d getPose2d(size_t idx) const {
      return (*this)[idx];
    }
    void setPose2d(const Pose2d& p, size_t idx) {
      (*this)[idx] = p;
    }

    size_t sizePose2d() const { return this->size(); }
  };

  //! Control type
  class Control : public ControlInterface
  {
  public:
  Control() : v(0.), w(0.) { }
  Control(double fwdVel, double rotVel) : v(fwdVel), w(rotVel) {}
    virtual double getFwdVel() const { return v; }
    virtual void setFwdVel(double fwdVel) { this->v = fwdVel; }
    virtual double getRotVel() const { return w; }
    virtual void setRotVel(double rotVel)  { this->w = rotVel; }
    Control scale(double s) const { 
      Control ret; 
      ret.v = s*this->v; ret.w = s*this->w; 
      return ret; 
    }
    double v;
    double w;
  };
  
  //! State2d implementation
  class State2d : public State2dInterface
  {
  public:
    
    State2d() { steeringAngle = 0.; }
    State2d(double x, double y, double th, double phi) { pose[0] = x; pose[1] = y; pose[2] = th; steeringAngle = phi; }
    State2d(const Pose2d &p, const double &s) { pose = p; steeringAngle = s; }
    State2d(const State2dInterface &s) { pose = s.getPose2d(); steeringAngle = s.getSteeringAngle(); }
    State2d(const PathInterface &path, size_t idx) { pose = path.getPose2d(idx); steeringAngle = path.getSteeringAngle(idx); }
    
    virtual Pose2d getPose2d() const { return pose; }
    virtual void setPose2d(const Pose2d& p) { this->pose = p; }
    virtual double getSteeringAngle() const { return steeringAngle; }
    virtual void setSteeringAngle(double s) { this->steeringAngle = s; }
    // l is the distance between the fixed axis and the steering wheel.
    virtual double getCurvature(double l) { return (tan(this->steeringAngle)/l); }
    virtual void setCurvature(double curvature, double l) { this->steeringAngle = atan(curvature*l); }

    void addControlStep(const ControlInterface &control, double len, double dt)
    {
      pose(0) += cos(pose(2)) * control.getFwdVel() * dt;
      pose(1) += sin(pose(2)) * control.getFwdVel() * dt;
      pose(2) += (tan(steeringAngle)) * control.getFwdVel() * dt / len;
      steeringAngle += control.getRotVel() * dt;
    } 
    
    Control getControlStep(const State2dInterface &nextState, double dt)
    {
      Control ret;
      double dist = getDistBetween(this->getPose2d(), nextState.getPose2d());
      double fwd_vel = dist / dt * getDirection(this->getPose2d(), nextState.getPose2d());
      double rot_vel = /*angles::normalize_angle*/(nextState.getSteeringAngle() - this->getSteeringAngle()) / dt;// * getDirection(this->getPose2d(), nextState.getPose2d());
      ret.setFwdVel(fwd_vel);
      ret.setRotVel(rot_vel);
      return ret;
    }
    
    Control getStateIncrControlStep(double dt)
    {
      Control ret;
      double dist = getDist(this->getPose2d());
      double fwd_vel = dist / dt * getDirectionIncr(this->getPose2d());
      double rot_vel = angles::normalize_angle(this->getSteeringAngle()) / dt;
      ret.setFwdVel(fwd_vel);
      ret.setRotVel(rot_vel);
      return ret;
    }
    
    State2d scale(double s) const {
      State2d ret;
      ret.pose = s*this->pose; ret.steeringAngle = s*this->steeringAngle; 
    return ret;
    }
    
    orunav_generic::Pose2d pose;
    double steeringAngle;
  
    friend std::ostream& operator<<(std::ostream &os, const State2d &obj)
      {
	os << "(" << obj.pose(0) << " " << obj.pose(1) << " " << obj.pose(2) << ")[" << obj.steeringAngle << "]";
	return os;
      }
    

};
  
  
  class State2dControl : public State2d, public Control
  {
    
  };

  //! Path
  class Path : public PathInterface
  {
  public:
    Path() { }
    Path(const PathInterface &path) {
      for (size_t i = 0; i < path.sizePath(); i++) {
	this->addPathPoint(path.getPose2d(i), path.getSteeringAngle(i));
      }
    }
    Pose2dVec poses;
    std::vector<double> steeringAngles;
    
    void clear() { poses.clear(); steeringAngles.clear(); }
    void addPathPoint(const Pose2d &pose, const double &steeringAngle) { poses.push_back(pose); steeringAngles.push_back(steeringAngle); }
    void addPathPointInterface(const Pose2dInterface &pose, const SteeringAngleInterface& steeringAngle) { this->addPathPoint(pose.getPose2d(), steeringAngle.getSteeringAngle()); }
    void addState2dInterface(const State2dInterface &state) { this->addPathPointInterface(state, state); }

    // Interfaces
    virtual Pose2d getPose2d(size_t idx) const { return poses.getPose2d(idx); }
    virtual void setPose2d(const Pose2d& p, size_t idx) { poses[idx] = p; }
    virtual size_t sizePose2d() const { return poses.sizePose2d(); }
    
    virtual double getSteeringAngle(size_t idx) const { return steeringAngles[idx]; }
    virtual void setSteeringAngle(double s, size_t idx) { steeringAngles[idx] = s; }
    virtual size_t sizeSteeringAngle() const { return steeringAngles.size(); }
    

    State2d getState2d(size_t idx) const { return State2d(*this, idx); }
    void setState2d(const State2dInterface &s, size_t idx) { poses[idx] = s.getPose2d(); steeringAngles[idx] = s.getSteeringAngle(); }
  };
  
  class Paths : public std::vector<Path>, public PathsInterface {
  public:
    size_t sizePaths() const { return this->size(); }
    PathInterface& getPath(size_t idx) { return (*this)[idx]; }
    const PathInterface& getPath(size_t idx) const { return (*this)[idx]; }
    void addPaths(const Paths &p) { this->insert(this->end(), p.begin(), p.end()); }
    void addPath(const Path &p) { this->push_back(p); }
  };

  class Trajectory : public TrajectoryInterface
  {
  public:
    Trajectory() { }
    Trajectory(const TrajectoryInterface &traj) {
      for (size_t i = 0; i < traj.sizeTrajectory(); i++) {
	this->addTrajectoryPoint(traj.getPose2d(i), traj.getSteeringAngle(i), traj.getDriveVel(i), traj.getSteeringVel(i));
      }
    }
    
    Pose2dVec poses;
    std::vector<double> steeringAngles;
    std::vector<double> driveVels;
    std::vector<double> steeringVels;
    
    void clear() { poses.clear(); steeringAngles.clear(); driveVels.clear(); steeringVels.clear();  }
    void addTrajectoryPoint(const Pose2d &pose, const double &steeringAngle, const double &fwdVel, const double &rotVel) {
      poses.push_back(pose); steeringAngles.push_back(steeringAngle); driveVels.push_back(fwdVel); steeringVels.push_back(rotVel);
    }
    void add(const State2dInterface& state, const ControlInterface &control) {
      poses.push_back(state.getPose2d());
      steeringAngles.push_back(state.getSteeringAngle());
      driveVels.push_back(control.getFwdVel());
      steeringVels.push_back(control.getRotVel());
    }

    // Interfaces
    virtual Pose2d getPose2d(size_t idx) const { return poses.getPose2d(idx); }
    virtual void setPose2d(const Pose2d& p, size_t idx) { poses.setPose2d(p,idx); }
    virtual size_t sizePose2d() const { return poses.sizePose2d(); }
    
    virtual double getSteeringAngle(size_t idx) const { return steeringAngles[idx]; }
    virtual void setSteeringAngle(double s, size_t idx)  { steeringAngles[idx] = s; }
    virtual size_t sizeSteeringAngle() const { return steeringAngles.size(); }

    virtual double getDriveVel(size_t idx) const { return driveVels[idx]; }
    virtual void setDriveVel(double v, size_t idx) { driveVels[idx] = v; }

    virtual double getSteeringVel(size_t idx) const { return steeringVels[idx]; }
    virtual void setSteeringVel(double w, size_t idx) { steeringVels[idx] = w; }
    
    
    State2d getState2d(size_t idx) const { State2d s(getPose2d(idx), getSteeringAngle(idx)); return s; }
    Control getControl(size_t idx) const { Control c(getDriveVel(idx), getSteeringVel(idx)); return c; }
  };

  class TrajectoryChunks : public std::vector<Trajectory>, public TrajectoryChunksInterface
  {
  public:
    TrajectoryInterface& getChunk(size_t idx) { return (*this)[idx]; }
    const TrajectoryInterface& getChunk(size_t idx) const { return (*this)[idx]; }
    size_t sizeChunks() const { return this->size(); }
    size_t getSequenceStartNum() const { return sequenceStartNum; }
    size_t sequenceStartNum;
  };


class CollisionCheckDummy : public CollisionCheckInterface
{
  bool collision(const orunav_generic::Pose2d &pose) const
  {
    return false;
  }
  bool collision(const orunav_generic::Pose2d &pose, double &minDist) const
  {
    return false;
  }
};


class CoordinatedTimes : public std::vector<double> {
 public:
  CoordinatedTimes() { }
  CoordinatedTimes(size_t size) { this->resize(size); std::fill(begin(), end(), -1.); }
  CoordinatedTimes(const std::vector<double> &v) { std::copy(v.begin(), v.end(), std::back_inserter(*this)); }
  std::vector<size_t> getTimeIdx() const {
    std::vector<size_t> idx;
    for (size_t i = 0; i < this->size(); i++) {
      if ((*this)[i] >= 0)
	idx.push_back(i);
    }
    return idx;
  }
  std::vector<double> getDeltaTs() const {
    std::vector<double> dts;
    std::vector<size_t> idx = this->getTimeIdx();
    for (size_t i = 0; i < idx.size()-1; i++) {
      dts.push_back((*this)[idx[i+1]] - (*this)[idx[i]]);
      assert(dts.back() < 0);
    }
    assert(dts.size() == idx.size() - 1);
    return dts;
  }

  void addOffset(double offset) {
    for (size_t i = 0; i < this->size(); i++) {
      if ((*this)[i] >= 0.)
	(*this)[i] += offset;
    }
  }

  void removeTimesBefore(double time) {
    for (size_t i = 0; i < this->size(); i++) {
      if ((*this)[i] >= 0. && (*this)[i] < time)
	(*this)[i] = -1.;
    }
  }

  std::vector<size_t> getTimeIdxBeforeStartIdx(size_t startIdx) const {
    std::vector<size_t> ret;
    std::vector<size_t> ct_idx = this->getTimeIdx();
    for (size_t i = 0; i < ct_idx.size(); i++) {
      if (ct_idx[i] < startIdx) {
	ret.push_back(ct_idx[i]);
      }
    }
    return ret;
  }

  int getTimeIdxAfterStartIdx(size_t startIdx) const {
    std::vector<size_t> ct_idx = this->getTimeIdx();
    for (size_t i = 0; i < ct_idx.size(); i++) {
      if (ct_idx[i] > startIdx) {
	return ct_idx[i];
      }
    }
    return -1;
  }
  
  bool isStartIdxInTimeIdx(size_t startIdx) const {
    std::vector<size_t> ct_idx = this->getTimeIdx();
    for (size_t i = 0; i < ct_idx.size(); i++) {
      if (ct_idx[i] == startIdx) {
	return true;
      }
    } 
    return false;
  }

  // Check if the idx is ahead of the cts.
  bool isAhead(size_t idx, double time, double &aheadTime) {
    
    aheadTime = -1;
    std::vector<size_t> ct_idx = this->getTimeIdx();
    for (size_t i = 0; i < ct_idx.size(); i++) {
      if (ct_idx[i] > idx) {
        return false;
      }
      if ((*this)[ct_idx[i]] > time) {
        aheadTime = (*this)[ct_idx[i]] - time;
        return true;
      }
    }
    return false;
  }

  void clearAllEntries() {
    for (size_t i = 0; i < this->size(); i++) {
      (*this)[i] = -1.;
    }
  }

  void clearFirstEntryInPairs() {
    if (this->size() < 1)
      return;
    for (size_t i = 0; i < this->size()-1; i++) {
      if ((*this)[i] >= 0 && (*this)[i+1] >= 0)
      {
        (*this)[i] = -1.;
      }
    }
  }

  // Only for debugging
  void createHoles(size_t holeWidth) {
    size_t i = 0;
    size_t hole_counter = holeWidth+1; // Make sure the ct[0] is not changed.
    while (i < this->size()-1) { // Make sure that ct[size-1] is not changed.
      hole_counter++;
      if (hole_counter > holeWidth)
	hole_counter = 0;
      else
	(*this)[i] = -1.;
      i++;
    }
  }

  // Only for debugging
  void createStep(size_t stepIdx, double stepTime) {
    std::vector<size_t> ct_idx = this->getTimeIdx();
    for (size_t i = 0; i < ct_idx.size(); i++) {
      if (ct_idx[i] > stepIdx) {
	(*this)[ct_idx[i]] += stepTime;
      }
    }
  }
};

inline CoordinatedTimes truncateCts(const CoordinatedTimes &cts, size_t idx) {
  CoordinatedTimes ret;
  for (size_t i = idx; i < cts.size(); i++) {
    ret.push_back(cts[i]);
  }
  return ret;
}

inline CoordinatedTimes selectCtsInterval(const CoordinatedTimes &cts, 
                                          size_t startIdx,
                                          size_t stopIdx) {
  CoordinatedTimes ret;
  assert(stopIdx <= cts.size());
  assert(startIdx < stopIdx);
  for (size_t i = startIdx; i < stopIdx; i++) {
    ret.push_back(cts[i]);
  }
  return ret;
}

} // namespace
