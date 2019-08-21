#pragma once

#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <orunav_generic/interfaces.h>
#include <orunav_generic/io.h>
#include <orunav_generic/serialization.h>

/* class CoordinatedTimesInterface { */
/*  public: */
/*   std::vector<size_t> getCoordinatedTimesIdx() const = 0; */
/*   std::vector<double> getDeltaTs() const = 0; */
/* }; */



class TrajectoryProcessor
{
 public:
  class Params
  {
  public:
    Params() {
      maxVel = 1.;
      maxVelRev = maxVel;
      useSteerDriveVel = true; // false;
      maxRotationalVel = 1.;
      maxRotationalVelRev = maxRotationalVel;
      maxSteeringAngleVel = 1.;
      initVel = 0.;
      endVel = 0.;
      initSteeringAngleVel = 0.;
      endSteeringAngleVel = 0.;
      maxAcc = 1.;
      maxRotationalAcc = 1.;
      maxSteeringAngleAcc = 1.;
      timeStep = 0.06; // 60 ms, make sure this is the same as in the controller used.
      wheelBaseX = 0.68; // Snowwhite
      wheelBaseY = 0.;
      useInitialState = true;
      nbZeroVelControlCommands = 5;
      minDist = 0.00001; // Minimum step size that are needed. Used to avoid to have multiple if the state entries.
      useCoordTimeAccConstraints = true;
      useCoordTimeContraintPoints = false;
      debug = true; // Will generate a set of gnuplot files if enabled, check the gnuplot folder.
      debugPrefix = std::string("");
      creepSpeed = 0.;
      creepDistance = 0.;
      setCreepSpeedAsEndConstraint = false;
      citiTruckNbClearSpeedCommands = 0;
    }

    double maxVel;
    double maxVelRev;
    bool useSteerDriveVel;
    double maxRotationalVel;
    double maxRotationalVelRev;
    double maxSteeringAngleVel;
    double initVel;
    double endVel;
    double initSteeringAngleVel;
    double endSteeringAngleVel;
    double maxAcc;
    double maxRotationalAcc;
    double maxSteeringAngleAcc;
    double timeStep;
    double wheelBaseX;
    double wheelBaseY;
    bool useInitialState;
    int nbZeroVelControlCommands;
    double minDist;
    bool useCoordTimeAccConstraints;
    bool useCoordTimeContraintPoints;
    bool debug;
    std::string debugPrefix;
    double creepSpeed;
    double creepDistance;
    bool setCreepSpeedAsEndConstraint;
    int citiTruckNbClearSpeedCommands;

    friend std::ostream& operator<<(std::ostream &os, const TrajectoryProcessor::Params &obj)
      {
	os << "\nmaxVel                   : " << obj.maxVel;
	os << "\nmaxVelRev                : " << obj.maxVelRev;
	os << "\nuseSteerDriveVel         : " << obj.useSteerDriveVel;
	os << "\nmaxRotationalVel         : " << obj.maxRotationalVel;
	os << "\nmaxRotationalVelRev      : " << obj.maxRotationalVelRev;
	os << "\nmaxSteeringAngleVel      : " << obj.maxSteeringAngleVel;
	os << "\ninitVel                  : " << obj.initVel;
	os << "\nendVel                   : " << obj.endVel;
	os << "\ninitSteeringAngleVel     : " << obj.initSteeringAngleVel;
	os << "\nendSteeringAngleVel      : " << obj.endSteeringAngleVel;
	os << "\nmaxAcc                   : " << obj.maxAcc;
	os << "\nmaxRotationalAcc         : " << obj.maxRotationalAcc;
	os << "\nmaxSteeringAngleAcc      : " << obj.maxSteeringAngleAcc;
	os << "\ntimeStep                 : " << obj.timeStep;
	os << "\nwheelBaseX               : " << obj.wheelBaseX;
	os << "\nwheelBaseY               : " << obj.wheelBaseY;
	os << "\nuseInitialState          : " << obj.useInitialState;
	os << "\nnbZeroVelControlCommands : " << obj.nbZeroVelControlCommands;
	os << "\nminDist                  : " << obj.minDist;
	os << "\nuseCoordTimeAccConst...  : " << obj.useCoordTimeAccConstraints;
	os << "\nuseCoordTimeContrain...  : " << obj.useCoordTimeContraintPoints;
	os << "\ndebug                    : " << obj.debug;
	os << "\ndebugPrefix              : " << obj.debugPrefix;
	os << "\ncreepSpeed               : " << obj.creepSpeed;
	os << "\ncreepDistance            : " << obj.creepDistance;
	os << "\nsetCreepSpeedAsEndCo...  : " << obj.setCreepSpeedAsEndConstraint;
	os << "\ncitiTruckNbClearSpee...  : " << obj.citiTruckNbClearSpeedCommands;
	return os;
      }
    private:
    friend class boost::serialization::access;
    
    template<typename Archive>
      void serialize(Archive& ar, const unsigned version) {
      ar & maxVel & maxVelRev & useSteerDriveVel & maxRotationalVel & maxRotationalVelRev & maxSteeringAngleVel & initVel & endVel & initSteeringAngleVel & endSteeringAngleVel & maxAcc & maxRotationalAcc & maxSteeringAngleAcc & timeStep & wheelBaseX & wheelBaseY & useInitialState & nbZeroVelControlCommands & minDist & useCoordTimeAccConstraints & useCoordTimeContraintPoints & debug & debugPrefix & creepSpeed & creepDistance & setCreepSpeedAsEndConstraint & citiTruckNbClearSpeedCommands;
    }
  };

  void setParams(const TrajectoryProcessor::Params &params) { _params = params; }
  virtual orunav_generic::Trajectory getTrajectory() = 0;
  virtual void addPathInterface(const orunav_generic::PathInterface &path) = 0;
  virtual void addCoordinatedTimes(const orunav_generic::CoordinatedTimes &times) = 0;

 protected:
  TrajectoryProcessor::Params _params;

 private:
  friend class boost::serialization::access;
  
  template<typename Archive>
    void serialize(Archive& ar, const unsigned version) {
    ar & _params;
  }

};

#if 0
double calcTrajectoryError(const std::vector<std::pair<State, Control> > &trajectory, const TrajectoryProcessor::Params & params)
{
  // Return the sum of (square distance + the angular distance)
  if (trajectory.empty())
    return -1.;
  State sim = trajectory[0].first;
  double dt = params.timeStep;
  double len = params.wheelBaseX;
  
  double sum_diff = 0.;
  for (size_t i = 0; i < trajectory.size(); i++)
    {
      const State &s = trajectory[i].first;
      sum_diff += absDiffState(s, sim);
      const Control &c = trajectory[i].second;
      State ds;
      ds.x() = cos(sim.theta()) * c.v() * dt;
      ds.y() = sin(sim.theta()) * c.v() * dt;
      ds.theta() = (tan(sim.phi()) * c.v() * dt) / len;
      ds.phi() = c.w() * dt;

      sim += ds;
    }
  return sum_diff / (trajectory.size()*1.);
}
#endif


