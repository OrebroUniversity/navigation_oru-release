#pragma once


#include <orunav_generic/types.h>
#include <cassert>
#include <fstream>


class ControlConstraint
{
 public:
 ControlConstraint() : valid(false) { }
  void assignControl(const orunav_generic::Control &control) { c = control; valid = true; }
  orunav_generic::Control c;
  bool isValid() const { return valid; }
 private:
  bool valid;
};

//! Contains a single trajectory point/step/delta value.
/*!
  Notation stuff. There is three types of variables here.
  1). *_point, this reflects a unique instance - for example a state point
  2). d*, a delta function of something, dt is the change in time between two time points (ds is the difference between to state points). 
  3). *_step, this instead is an instance that is the average over some time (c_step is the average control value over time dt)
*/
// Currently the acceleration constraints are operated in vehicle state corrdinates whereas the velocity contraints are applied to the drive wheel (in case of an SD vehicle). (Again here is an assumption that abs(phi) < PI/2 and not "close" to PI/2.)
class TrajectoryStepNaive
{
 public:
  void updateDt(double new_dt) { if (new_dt > dt) { dt = new_dt; } }
  
  void updateControlStep() { 
    c_step = ds.getStateIncrControlStep(dt);
  }
  
  orunav_generic::Control getControlStep() { return ds.getStateIncrControlStep(dt); }
  
  //! Return driving direction, 1 = forward, -1 = backward
  double getDir() const { 
    return dir;
    //    assert(fabs(s_point.getSteeringAngle()) < M_PI/2.);
    //    return orunav_generic::getDirectionIncr(this->ds.getPose2d());
  }

  //! Return the average fwd speed.
  //  double getVStep() const { return this->getDir()*orunav_generic::getDist(ds.getPose2d()) / dt; }
  //! Return the average steering angle change fwd.
  //  double getWStep() const { return this->getDir()*ds.getSteeringAngle() / dt; }
  double getPhiStep() const { return s_point.getSteeringAngle() + 0.5*ds.getSteeringAngle(); }
  //! Return the average fwd speed of the SD wheel.
  //  double getVDriveStep() const { return getVStep()/cos(getPhiStep()); }
  
  double getPositionDiff() const { return orunav_generic::getDist(ds.getPose2d()); }
  double getHeadingDiff() const { return orunav_generic::getHeading(ds.getPose2d()); }
  double getSteeringDiff() const { return ds.getSteeringAngle(); }
  
  //! State value - never changes
  orunav_generic::State2d s_point;
  //! State value - never changes
  orunav_generic::State2d ds; // -"-
  double dt; // This is what we need to compute, all other variables here could be deduced using the TrajectoryStepNaiveVec
  
  // Constraints
  double ct_point; // Time constraint
  ControlConstraint cc_point; // Control constraints (e.g. at start, velocities are set to start velocities, typically 0).
  
  
  // Used for storage of velocities, this 
  orunav_generic::Control c_point; 
  orunav_generic::Control c_step;
  
  // TODO, need probably another type here later on, to cover more types of accelerations.
  double acc;
  
  double t_point; // The total time
  double dir; // The direction

  double ct_factor;
  
  friend std::ostream& operator<<(std::ostream &os, const TrajectoryStepNaive &obj)
    {
      os << obj.dt << " " << obj.ds.getPose2d()[0]; // << " " << obj.ds.y() << " " << obj.ds.theta() << " " << obj.ds.phi() << " " << obj.v << " " << obj.w << " " << obj.acc << " " << obj.v2 << " " << obj.dt2 << " " << obj.acc2 << " " << obj.dv << " " << obj.dv2 << " " << obj.vPoint << " " << obj.wPoint << " " << obj.ct << " " << obj.dt_ct << " " << obj.t;
      return os;
    }
  
};



//! Container class for the TrajectoryStepNavie
class TrajectoryStepNaiveVec : public std::vector<TrajectoryStepNaive>, public orunav_generic::DeltaTInterface
{
 public:
  double & getDeltaT(size_t idx) { return (*this)[idx].dt; }
  const double &getDeltaT(size_t idx) const { return (*this)[idx].dt; }
  size_t sizeDeltaTVec() const { return this->size(); }

  void printDebug() const {
    for (size_t i = 0; i < this->size(); i++) {
      const TrajectoryStepNaive& s = (*this)[i];
      std::cout << "dt:" << s.dt << "\tt:" << s.t_point << "\tct:" << s.ct_point << std::endl;
    }
  }
  
  void add(const TrajectoryStepNaiveVec &add)
  {
    for (size_t i = 0; i < add.size(); i++)
      {
	this->push_back(add[i]);
      }
  }
  
  orunav_generic::Trajectory convertNaiveStepsToTrajectory() const
    {
      orunav_generic::Trajectory ret;
      for (size_t i = 0; i < this->size(); i++)
	{
	  ret.add((*this)[i].s_point, (*this)[i].c_step);
	}
      return ret;
    }
  
  std::vector<size_t> getConstraintTimesPointIdx() const
    {
      std::vector<size_t> idx;
      for (size_t i = 0; i < this->size(); i++) {
	if ((*this)[i].ct_point >= 0)
	  idx.push_back(i);
      }
      return idx;
    }
  
  void saveGnuplotFile(const std::string &fileName) const
  {
    std::ofstream ofs(fileName.c_str());
    double time = 0.;
    for (size_t i = 0; i < this->size(); i++)
      {
	//os << obj.dt << " " << obj.ds.getPose2d()[0]; // << " " << obj.ds.y() << " " << obj.ds.theta() << " " << obj.ds.phi() << " " << obj.v << " " << obj.w << " " << obj.acc << " " << obj.v2 << " " << obj.dt2 << " " << obj.acc2 << " " << obj.dv << " " << obj.dv2 << " " << obj.vPoint << " " << obj.wPoint << " " << obj.ct << " " << obj.dt_ct << " " << obj.t;
	
	const TrajectoryStepNaive &s = (*this)[i];
	const orunav_generic::Pose2d &ds = s.ds.getPose2d();
	ofs << time << " " << time + s.dt*0.5 << " " << s.dt << " " << s.c_step.v << " " << s.c_step.w << " " << orunav_generic::getDist(ds) << " " << s.ds.getSteeringAngle() << " " << s.cc_point.isValid() << " " << s.cc_point.c.v << " " << s.cc_point.c.w << " " << s.c_point.v << " " << s.c_point.w << " " << s.acc << " " << s.dir << " " << s.s_point.getSteeringAngle() << " " << s.ds.getSteeringAngle()/s.dt << " " << s.s_point.getPose2d()[0] << " " << ds[0]/s.dt << " " << s.ct_point << " " << (*this)[0].ct_point << std::endl;
	time += (*this)[i].dt;
      }
    ofs.close();
  }
  
  std::vector<size_t> getDirectionChangeIdx() const
    {
      std::vector<size_t> ret;
      if (this->size() < 3)
	return ret;
      bool last_dir = ((*this)[0].getDir() > 0);
      for (size_t i = 0; i < this->size(); i++)
	{
	  bool curr_dir = ((*this)[i].getDir() > 0);
	  if (last_dir != curr_dir) {
	    last_dir = !last_dir;
	    ret.push_back(i);
	  }
	}
      return ret;
    }
  std::vector<size_t> getControlConstraintPointIdx() const
    {
      std::vector<size_t> ret;
      for (size_t i = 0; i < this->size(); i++)
	{
	  if ((*this)[i].cc_point.isValid()) {
	    //std::cout << "[" << i << "] : cc_point.valid " << std::endl;
	    ret.push_back(i);
	  }
	}
      return ret;
    }

  //! Return the velocity given a step index.
  orunav_generic::Control getDeltaControl(size_t idx, bool fwd)  const
  {
    assert(idx < this->size());
    if (fwd) {
      return orunav_generic::Control((*this)[idx+1].c_point.v - (*this)[idx].c_point.v,
				    (*this)[idx+1].c_point.w - (*this)[idx].c_point.w);
    }
    else {
      return orunav_generic::Control((*this)[idx].c_point.v - (*this)[idx+1].c_point.v,
				    (*this)[idx].c_point.w - (*this)[idx+1].c_point.w);
    }
  }
};

