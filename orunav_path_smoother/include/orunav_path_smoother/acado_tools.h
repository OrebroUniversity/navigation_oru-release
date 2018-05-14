#pragma once

#include <acado_toolkit.hpp>
#include <orunav_generic/types.h>

//! Very important to call this after / before a new optimzation problem is posted.
inline void ACADO_clearStaticCounters() {
  ACADO::AlgebraicState().clearStaticCounters();
  ACADO::Control().clearStaticCounters();
  ACADO::DifferentialState().clearStaticCounters();
  ACADO::DifferentialStateDerivative().clearStaticCounters();
  ACADO::Disturbance().clearStaticCounters();
  ACADO::IntegerControl().clearStaticCounters();
  ACADO::IntegerParameter().clearStaticCounters();
  ACADO::IntermediateState().clearStaticCounters();
  ACADO::Parameter().clearStaticCounters();
}

inline ACADO::VariablesGrid convertTrajectoryToACADOVariablesGrid(const orunav_generic::TrajectoryInterface &traj, double start, double deltaT)
{
  double end = start + (traj.sizeTrajectory() -1 )* deltaT; 
  ACADO::VariablesGrid grid(6, start, end, traj.sizeTrajectory());

  for (unsigned int i = 0; i < traj.sizeTrajectory(); i++) {
    ACADO::DVector v( 6 );
    v(0) = traj.getPose2d(i)(0);
    v(1) = traj.getPose2d(i)(1);
    v(2) = traj.getPose2d(i)(2);
    v(3) = traj.getSteeringAngle(i);
    v(4) = traj.getDriveVel(i);
    v(5) = traj.getSteeringVel(i);

    grid.setVector( i,v );
  }
  return grid;
}

inline ACADO::VariablesGrid convertTrajectoryToACADOControlVariablesGrid(const orunav_generic::TrajectoryInterface &traj, double start, double deltaT)
{
  double end = start + (traj.sizeTrajectory() -1 )* deltaT; 
  ACADO::VariablesGrid grid(2, start, end, traj.sizeTrajectory());

  for (unsigned int i = 0; i < traj.sizeTrajectory(); i++) {
    ACADO::DVector v( 2 );
    v(0) = traj.getDriveVel(i);
    v(1) = traj.getSteeringVel(i);

    grid.setVector( i,v );
  }
  return grid;
}

inline void setFixedACADOControlVariablesGrid(ACADO::VariablesGrid &grid, double v, double w) {
  for (unsigned int i = 0; i < grid.getLastIndex(); i++) {
    ACADO::DVector tmp( 2 );
    tmp(0) = v;
    tmp(1) = w;
    grid.setVector(i, tmp);
  }
}

inline ACADO::VariablesGrid convertPathToACADOStateVariableGrid(const orunav_generic::PathInterface &path, double start, double deltaT)
{
  double end = start + (path.sizePath() -1 )* deltaT; 
  ACADO::VariablesGrid grid(4, start, end, path.sizePath());

  for (unsigned int i = 0; i < path.sizePath(); i++) {
    ACADO::DVector v( 4 );
    v(0) = path.getPose2d(i)(0);
    v(1) = path.getPose2d(i)(1);
    v(2) = path.getPose2d(i)(2);
    v(3) = path.getSteeringAngle(i);

    grid.setVector( i,v );
  }
  return grid;
}

inline orunav_generic::Path convertACADOStateVariableGridToPath(const ACADO::VariablesGrid &states, int nbStates = 4)
{
  orunav_generic::Path path;
  nbStates = states.getNumRows(); // Why isn't this always used?!?

  for (unsigned int i = 0; i < states.getDim()/nbStates; i++) {
    double x = states(i,0);
    double y = states(i,1);
    double th = states(i,2);
    double phi = states(i,3);
    path.addPathPoint(orunav_generic::Pose2d(x,y,th), phi);
  }
  return path;
}

inline orunav_generic::Trajectory convertACADOStateControlVariableGridToTrajectory(const ACADO::VariablesGrid &states, const ACADO::VariablesGrid &controls) {
  orunav_generic::Trajectory traj;
  int nbStates = states.getNumRows();

  for (unsigned int i = 0; i < states.getDim()/nbStates; i++) {
    double x = states(i,0);
    double y = states(i,1);
    double th = states(i,2);
    double phi = states(i,3);
    double v = controls(i,0);
    double w = controls(i,1);
    traj.addTrajectoryPoint(orunav_generic::Pose2d(x,y,th), phi, v, w);
  }
  return traj;
} 

inline ACADO::DVector convertACADOStateVariableGridToVector(const ACADO::VariablesGrid &states, int idx)
{
  ACADO::DVector vec(states.getNumRows());
  for (unsigned int i = 0; i < states.getNumRows(); i++) {
    vec(i) = states(i, idx);
  }
  return vec;
}

