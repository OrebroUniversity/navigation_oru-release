#pragma once

#include <orunav_trajectory_processor/step_naive.h>
#include <orunav_trajectory_processor/trajectory_processor.h>
#include <cassert>
#include <orunav_generic/io.h>
#include <orunav_generic/path_utils.h>

class TrajectoryProcessorNaive : public TrajectoryProcessor, public orunav_generic::DeltaTInterface
{
 public:

 TrajectoryProcessorNaive() : _startIdx(0) {  }

  double& getDeltaT(size_t idx) { return _steps[idx].dt; }
  const double &getDeltaT(size_t idx) const { return _steps[idx].dt; }
  size_t sizeDeltaTVec() const { return _steps.size(); }

  size_t getPathStartIdx() const { return _startIdx; }

  std::vector<double> getGlobalPathTimes(double offset) const {
    std::vector<double> ret;
    ret.resize(this->getPathStartIdx() + this->sizeDeltaTVec());
    std::fill(ret.begin(), ret.end(), -1.);
    
    double global_time = offset;
    ret[_startIdx] = global_time;

    for (size_t i = 0; i < this->sizeDeltaTVec(); i++) {
      global_time += this->getDeltaT(i);
      ret[_startIdx + i] = global_time;
    }
    return ret;
  }

  void computeDts()
    {
      setupSteps(_steps);
      assignDs(_steps);
      assignDir(_steps);
      assignMinDtValidSpeed(_steps); // Add the min/max velocity constraints.
      assignCreepSpeed(_steps);
      assignControlSteps(_steps);    // Compute the current speed, not including acceleration constraint nor control point constraints, note this is only using ds and dt.
      assignControlConstraintPoints(_steps); // Add start / end speeds constraints including directional changes speeds to be zero.
      assignControlPoints(_steps);  // Use the control constraint points. Fill the rest with control steps.
      // The onlything that are missing now is the acceleration constraints.
      assignAccelerationConstraints(_steps);
      //      assignLastDtNonZero(_steps);

      // Update the control steps, only used for visualizing the correct steps.
      assignControlSteps(_steps);

      if (_params.debug) {
	_steps.saveGnuplotFile(_params.debugPrefix + std::string("naive_trajectory.txt"));
	orunav_generic::saveTrajectoryTextFile(_steps.convertNaiveStepsToTrajectory(), _params.debugPrefix + "naive_trajectoryXY.txt");
      }
    }
  
  orunav_generic::Trajectory getTrajectory() {
    computeDts();

    // Need the global time for the interpolation
    assignTotalTime(_steps);
    if (this->useCoordinatedTimes()) {
      //Assigning coordination times --- START ----
      assignCoordinationTimes(_steps); // This will modify the dt's.
      if (_params.useCoordTimeAccConstraints)
      { // Recompute the points based on the new dt's, this is just to make sure that the control point is updated.
      	assignControlSteps(_steps);    // Compute the current speed, not including acceleration constraint nor control point constraints, note this is only using ds and dt.
      	assignControlConstraintPoints(_steps); // Add start / end speeds constraints including directional changes speeds to be zero.
      	assignControlPoints(_steps);  // Use the control constraint points. Fill the rest with control steps.
	
	// This sometimes works a bit better (but not always).
	if (_params.useCoordTimeContraintPoints) {
	  assignControlConstraintPointsFromCT(_steps);
	}
	
	// Reassign points.
      	assignControlConstraintPoints(_steps); // Add start / end speeds constraints including directional changes speeds to be zero.
      	assignControlPoints(_steps);  // Use the control constraint points. Fill the rest with control steps.
      	assignAccelerationConstraints(_steps);
      	// Update the control steps, only used for visualizing the correct steps.
      	assignControlSteps(_steps);
	assignTotalTime(_steps);

	if (_params.debug) {
	  _steps.saveGnuplotFile(_params.debugPrefix + std::string("naive_trajectory.txt"));
	  orunav_generic::saveTrajectoryTextFile(_steps.convertNaiveStepsToTrajectory(), _params.debugPrefix + "naive_trajectoryXY.txt");
	}
	assignCoordinationTimes(_steps); // This will modify the dt's, do ths assigning one more time, this now is using the mean velocities around the coordinated points.
      }
      assignTotalTime(_steps);
      if (_params.debug) {
	// Save all path points that have coordination times.
	orunav_generic::savePathTextFile(this->coordinationPathPoints(), _params.debugPrefix + "naive_trajectoryXY_coordination_times.txt");
      }
    }
    
    TrajectoryStepNaiveVec output = createNaiveTrajectoryFixedDt(_steps, _params.timeStep);
    
    if (_params.citiTruckNbClearSpeedCommands > 0) {
      // Clear the velocity of the last trajectory points...
      int start_clear_idx = output.size() - _params.citiTruckNbClearSpeedCommands - _params.nbZeroVelControlCommands;
      if (start_clear_idx > 0) {
	for (size_t i = start_clear_idx; i < output.size(); i++) {
	  output[i].c_step.v = 0.;
	}
      }
    }
    
    if (_params.debug) {
      std::cout << "DBG: output.size(): " << output.size() << std::endl;
      assignDs(output);
      output.saveGnuplotFile(_params.debugPrefix + std::string("naive_trajectory_fixed_dt.txt"));
      orunav_generic::saveTrajectoryTextFile(output.convertNaiveStepsToTrajectory(), _params.debugPrefix + "naive_trajectory_fixed_dtXY.txt");
      orunav_generic::Path fwd_sim = orunav_generic::forwardSimulation(output.convertNaiveStepsToTrajectory(), _params.wheelBaseX, _params.timeStep);
      orunav_generic::savePathTextFile(fwd_sim, _params.debugPrefix + "naive_trajectory_fixed_dt_fwd_simXY.txt");
    }
    return output.convertNaiveStepsToTrajectory();
  }

  //! Return a function to be sent to the controller. Note that c_step's is used(!)
  TrajectoryStepNaiveVec createNaiveTrajectoryFixedDt(TrajectoryStepNaiveVec &steps, double dt)
    {
      TrajectoryStepNaiveVec ret;
      // Simply do linear interpolation between the states - yes this is the naive approach.
      double time = 0;
      //      double v_prev = 0.;
      orunav_generic::Control control_step;
      
      int prev_idx = getPrevPointIdx(steps, time);
      int next_idx = getNextPointIdx(steps, time + dt); 
      //      std::cout << "prev_idx : " << prev_idx << " next_idx : " << next_idx << std::endl;
      while (prev_idx >= 0 && next_idx > 0)
	{
	  double used_dt;
	  
	  if (!interpolateControlStep(steps, time, time + dt, control_step, used_dt)) {
	    std::cerr << "control_step.v : " << control_step.v << ", control_step.w : " << control_step.w << std::endl;
	      assert(false);
	  }
	  
	  TrajectoryStepNaive s;
	  s.c_step = control_step;
	  s.dt = dt;
	  //	  s.acc = (v - v_prev)/dt; // NOT ALLOWED!
	  //	  v_prev = v; // NOT ALLOWED!

	  s.s_point = interpolateStatePoint(steps, time);
	  //std::cout << "steps[0].s.x() : " << steps[0].s.x() << " steps[prev_idx].s.x() : " << steps[prev_idx].s.x() << " steps[next_idx].s.x() : " << steps[next_idx].s.x() << " s.s.x() : " << s.s.x() << std::endl; 
	  //	  std::cout << "prev_idx : " << prev_idx << " next_idx : " << next_idx << " time : " << time << " steps[prev_idx].t : " << steps[prev_idx].t << " steps[next_idx].t : " << steps[next_idx].t << " steps[prev_idx].s.x() : " << steps[prev_idx].s.x() << " s.s.x() : " << s.s.x() << " steps[prev_idx].v : " << steps[prev_idx].v << " s.v : " << s.v << std::endl;

	  ret.push_back(s);
	  time += dt;
	  prev_idx = getPrevPointIdx(steps, time);
	  next_idx = getNextPointIdx(steps, time + dt); 
	}
      // Ok, we are now about to pass the time (time + dt) is done.
      if (prev_idx >= 0)
      {
	double used_dt;
	TrajectoryStepNaive s;
	//std::cout << "===== At final part... ====" << std::endl;
	// Get the final 'control...'
	//std::cout << "==== time : " << time << std::endl;
	interpolateControlStep(steps, time, steps.back().t_point, control_step, used_dt); // Here the used_dt is (most likely) != dt
	s.c_step = control_step.scale(used_dt / dt);
	s.dt = dt; // Must be fixed.
	s.s_point = interpolateStatePoint(steps, steps.back().t_point);
	//std::cout << "s.c_step.v : " << s.c_step.v << " s.c_step.w : " << s.c_step.w << std::endl;
	//	std::cout << "s.s_point  : " << s.s_point << std::endl;
	//std::cout << "used_dt : " << used_dt << std::endl;
	//std::cout << "control_step.v : " << control_step.v << " control_step.w :" << control_step.w << std::endl;
	//std::cout << "==============================================" << std::endl;
	ret.push_back(s);

	
	// Add empty values, same dt, no speed
	s.c_step.v = 0;
	s.c_step.w = 0;
	for (int i = 0; i < _params.nbZeroVelControlCommands; i++)
	  ret.push_back(s);
      }
      return ret;
    }

  void addPathInterface(const orunav_generic::PathInterface &path)
  {
    if (!orunav_generic::validPathForTrajectoryProcessing(path)) {
      std::cerr << "trajectory_processor : invalid path(!) - should never happen" << std::endl;
    }
    _path = orunav_generic::Path(path);
  }

  void addCoordinatedTimes(const orunav_generic::CoordinatedTimes &times) {
    _coordinatedTimes = times;
  }

  void addControlConstraintPoint(size_t stepIdx, const orunav_generic::Control &control) {
    _controlConstraintPoints.push_back(std::pair<size_t, orunav_generic::Control>(stepIdx, control));
  }

  void addControlConstraintPoints(const std::vector<std::pair<size_t, orunav_generic::Control> > &controlConstraintPoints) {
    for (size_t i = 0; i < controlConstraintPoints.size(); i++) {
      _controlConstraintPoints.push_back(controlConstraintPoints[i]);
    }
  }

  void addControlConstraintPointAsStart(size_t startIdx, const orunav_generic::Control &control) {
    _controlConstraintPoints.push_back(std::pair<size_t, orunav_generic::Control>(0, control));
    _startIdx = startIdx;
  }

  double getCoordinationTimeFactor(size_t idx) const {
    return _steps[idx].ct_factor;
  }

  void setCoordinationTimeAtStartIdx(double factor) {
    assert(this->_steps.size() > 0); // Need to compute the steps before calling this function (the dt is used here).
    int next_ct_idx = _coordinatedTimes.getTimeIdxAfterStartIdx(_startIdx);
    assert(this->useCoordinatedTimes());
    assert(next_ct_idx >= 0);
    assert(next_ct_idx < (int)this->_coordinatedTimes.size());
    double time_diff = orunav_generic::getTotalTimeBetween(*this, 0, next_ct_idx - _startIdx);
    _coordinatedTimes[_startIdx] = _coordinatedTimes[next_ct_idx] - time_diff*factor;
  }

 protected:
  orunav_generic::Path _path; // Original path
  orunav_generic::CoordinatedTimes _coordinatedTimes; // Coordinated times (if available -> must have same size as _path)

  TrajectoryStepNaiveVec _steps; // For the delta t interface.
  std::vector<std::pair<size_t, orunav_generic::Control> > _controlConstraintPoints; // To add constraints on control values on specific step points.
  size_t _startIdx; // If the computation should be done using another start (if the updates should be one on the fly...)

  bool useCoordinatedTimes() const {
    return (_path.sizePath() == _coordinatedTimes.size());

  }

  // For debuggin purposes only.
  orunav_generic::Path coordinationPathPoints() const {
    orunav_generic::Path ret;
    assert(useCoordinatedTimes());
    for (size_t i = _startIdx; i < _path.sizePath(); i++) {
      if (_coordinatedTimes[i] >= 0.) {
	ret.addPathPoint(_path.getPose2d(i), _path.getSteeringAngle(i));
      }
    }
    return ret;
  }
  
  //! Function that combines the inputs (Path and CoordinatedTimes) to create the initial state of the TrajectoryStepNaiveVec.
  void setupSteps(TrajectoryStepNaiveVec &steps)
  {
    assert(_path.sizePath() > 1);
    if (!_coordinatedTimes.empty()) {
      assert(_path.sizePath() == _coordinatedTimes.size());
    }
    
    bool use_ct = this->useCoordinatedTimes();
    for (size_t i = _startIdx; i < _path.sizePath(); i++)
      {
	TrajectoryStepNaive s;
	s.s_point = orunav_generic::State2d(_path, i);
	if (use_ct)
	  s.ct_point = _coordinatedTimes[i];

	if (i == _startIdx)
	  {
	    // Just add it
	    steps.push_back(s);
	    continue;
	  }

	double dist = orunav_generic::getDistBetween(steps.back().s_point.getPose2d(), _path.getPose2d(i));
	if (dist < _params.minDist)
	  {
	    std::cerr << "The states are not separated enough. This is a core requirement for this function. (use orunav_generic::minIncrementalDistancePath(), to fix your path), this path is saved as [traj_proc_fail.path]." << std::endl;
	    orunav_generic::savePathTextFile(_path, "traj_proc_fail.path");
	    assert(false);
	  }
	else
	  {
	    steps.push_back(s);
	  }
      }
    // Clean the last bit. Since the delta and steps (only points) can be in the very end of the vector. Clean these here.
    steps.back().ds = orunav_generic::State2d(orunav_generic::Pose2d(0.,0.,0.), 0.);
    steps.back().dt = 0.001; // Avoid having any zero times.
    steps.back().c_step = orunav_generic::Control(0., 0.);
    steps.back().acc = 0.;
    
    if (_params.debug)
      std::cout << "DBG: setupSteps, number of entries : " << steps.size() << std::endl;
  }
  
  void assignDs(TrajectoryStepNaiveVec &steps)
  {
    for (size_t i = 0; i < steps.size()-1; i++)
      {
	steps[i].ds = orunav_generic::subState2d(steps[i].s_point, steps[i+1].s_point);
      }
  }

  void assignDir(TrajectoryStepNaiveVec &steps) 
  {
    assert(steps.size() > 1);
    // Assumes that we have access to s_point and ds.
    for (size_t i = 0; i < steps.size()-1; i++)
      {
    	// Compute the direction here, for the last step this cannot be determined otherwise
	assert(fabs(steps[i].s_point.getSteeringAngle()) < M_PI/2.);
	double dir  = orunav_generic::getDirectionIncr(steps[i].ds.getPose2d());
	steps[i].dir = dir;
      }
    steps.back().dir = steps[steps.size()-2].dir;
  }

  void assignFixedDt(TrajectoryStepNaiveVec &steps, double dt)
  {
    for (size_t i = 0; i < steps.size()-1; i++)
      {
  	steps[i].dt = dt;
      }
  }

  void assignMinDtValidSpeed(TrajectoryStepNaiveVec &steps)
  {
    for (size_t i = 0; i < steps.size()-1; i++)
      {
	steps[i].dt = minDtValidSpeed(steps[i]);
      }
  }

  int getCreepStartIdx(TrajectoryStepNaiveVec &steps) {
    int i = steps.size()-1;
    double acc_dist = 0.;
    while (i >= 0)
      {
	double position_diff = steps[i].getPositionDiff();
	acc_dist += position_diff; 
	if (acc_dist > _params.creepDistance) {
	  return i;
	}
	i--;
      }
    return i;
  }

  void assignCreepSpeed(TrajectoryStepNaiveVec &steps)
  {
    if (_params.creepDistance <= 0)
      return;
    
    int creep_start_idx = getCreepStartIdx(steps);
    if (creep_start_idx < 0) {
      std::cerr << "Creep distance is longer than the path." << std::endl;
      creep_start_idx = 0;
    }
    for (size_t i = creep_start_idx; i < steps.size(); i++) {
      double position_diff = steps[i].getPositionDiff();
      double dt_creep = position_diff / _params.creepSpeed;
      if (dt_creep > steps[i].dt) {
	steps[i].dt = dt_creep;
      }
    }
    // Note: this utilizes the step velocities and turns them into a point velocity.
    // Set the creep_start_idx to be a control point to get the accelerations correct.
    this->addControlConstraintPoint(creep_start_idx, steps[creep_start_idx].getControlStep());
    // Set also the "second to last" point to get correct accelerations in the end.
    int idx = steps.size() - 2;
    this->addControlConstraintPoint(idx, steps[idx].getControlStep());
  }

  void assignLastDtNonZero(TrajectoryStepNaiveVec &steps) 
  {
    if (steps.back().dt <= 0.001)
      steps.back().dt = 0.001;
  }

  void assignControlSteps(TrajectoryStepNaiveVec &steps)
  {
    for (size_t i = 0; i < steps.size()-1; i++)
      {
	steps[i].updateControlStep();
      }
  }

  void assignControlConstraintPoints(TrajectoryStepNaiveVec &steps)
  {
    assert(steps.size() > 2);
    if (steps.size() < 2)
      return;

    // Assign start and end velocities
    orunav_generic::Control c;
    if (_params.creepSpeed != 0 && _params.setCreepSpeedAsEndConstraint) {
      // Update the final velocities based on the creep speed.
      // Note that when the vehicle changes direction - a velocity of zero is still used ->
      // the creep distance can only be until a directional change occur.
      
      // Don't use the last one to check the direction - the one before the back.
      c.w = 0.;
      if (steps[steps.size()-2].c_step.v >= 0)
    	c.v = _params.creepSpeed;
      else
    	c.v = -_params.creepSpeed;
    }
    else
    {
      c.v = _params.endVel;
      c.w = _params.endSteeringAngleVel;
    }    
    steps.back().cc_point.assignControl(c);

    c.v = _params.initVel;
    c.w = _params.initSteeringAngleVel;
    steps.front().cc_point.assignControl(c);
    
    // For all turning points, when the direction changes, we need to set constraints on these points to have velocities = 0.
    std::vector<size_t> dir_changes = steps.getDirectionChangeIdx();
    c.v = 0.;
    c.w = 0.;
    for (size_t i = 0; i < dir_changes.size(); i++) {
      steps[dir_changes[i]].cc_point.assignControl(c);
    }

    // This is all extra control constraint points...
    for (size_t i = 0; i < _controlConstraintPoints.size(); i++) {
      c = _controlConstraintPoints[i].second;
      steps[_controlConstraintPoints[i].first].cc_point.assignControl(c);
    }

  }

  void assignControlPoints(TrajectoryStepNaiveVec &steps)  // Use the control constraint points. Fill the rest with control steps.
  {
    for (size_t i = 0; i < steps.size(); i++) {
      if (steps[i].cc_point.isValid()) {
	steps[i].c_point = steps[i].cc_point.c;
      }
      else {
	steps[i].c_point = steps[i].getControlStep();
      }
    }
  }

  void assignAccelerationConstraints(TrajectoryStepNaiveVec &steps)
  {
    // Here is the deal. We need to do forward and backward steps between the control constraint points.
    std::vector<size_t> idx = steps.getControlConstraintPointIdx(); // Note point idx.
    if (_params.debug)
      std::cout << "DBG: number of control constraint points : " << idx.size() << std::endl;
    for (size_t i = 0; i < idx.size()-1; i++) {
      addAccelerationConstraints(steps, idx[i], idx[i+1]);
    }
  }

  // Note start and end idx are point ids
  void addAccelerationConstraints(TrajectoryStepNaiveVec &steps, size_t startIdx, size_t endIdx) {
    // Fwd
    for (size_t i = startIdx; i < endIdx-1; i++) {
      addAccelerationConstraint(steps, i, true);
    }
    // Backward
    for (size_t i = endIdx; i > startIdx+1; i--) {
      addAccelerationConstraint(steps, i, false);
    }
  }

  // The acceleration constraints operates on control points (not steps)
  bool addAccelerationConstraint(TrajectoryStepNaiveVec &steps, int idx, bool fwd)
  {
    int curr_point_idx = idx; // OK.
    int step_idx = idx; // Depends on if we're going backward or forward.
    if (!fwd) {
      step_idx = idx-1;
    }
    int next_point_idx = curr_point_idx+1; // Depends if we're going backward or forward.
    if (!fwd) {
      next_point_idx = curr_point_idx-1;
    }

    assert(curr_point_idx >= 0);
    assert(curr_point_idx < (int)steps.size());
    assert(next_point_idx >= 0);
    assert(next_point_idx < (int)steps.size());
      
    double dt = steps[step_idx].dt;
    // Simply work with the fwd velocity for now.
    // Note, the dv (delta) for this step is defined to be this: (need to have points to compute a delta).
    orunav_generic::Control dc = steps.getDeltaControl(step_idx, fwd);
    /* std::cout << "dc : [" << step_idx << "], v : " << dc.v << ", w : " << dc.w << std::endl; */
    /* std::cout << "curr_point_idx, w: " << steps[curr_point_idx].c_point.w << std::endl;  */
    /* std::cout << "next_point_idx, w: " << steps[next_point_idx].c_point.w << std::endl;  */

    double dv = dc.v;
    double acc = dv / dt; // Acceleration is a step.
    
    bool updated = false;
    if (acc > _params.maxAcc || acc < -_params.maxAcc)
      {
	if (acc > 0)
	  acc = _params.maxAcc;
	else
	  acc = -_params.maxAcc;

	// Find the time dt to traverse ds.
	// Use this to update the next velocity.
	// s = v_0*t+at²/2
	// t = -v_0/a +/- sqrt(2s/a + (v_0 / a)^2)
	double v_0 = steps[curr_point_idx].c_point.v;
	double s = steps[step_idx].getPositionDiff()*steps[step_idx].getDir();
	double new_dt_solution1 = -v_0/acc + sqrt(2*s/acc + pow(v_0 / acc, 2));
	double new_dt_solution2 = -v_0/acc - sqrt(2*s/acc + pow(v_0 / acc, 2));
	double new_dt = new_dt_solution1;
	if (new_dt_solution2 > 0)
	  new_dt = new_dt_solution2;
	  
	if (steps[step_idx].dt < new_dt) {
	  steps[step_idx].dt = new_dt;
	  // Sanity check...
	  if (steps[next_point_idx].cc_point.isValid()) {
	    std::cout << "ERROR: overwriting cc_point(!!!) at [" << next_point_idx << "]" << std::endl;
	  }
	}
	updated = true;
      }

    steps[next_point_idx].c_point.v = steps[curr_point_idx].c_point.v+steps[step_idx].dt*acc;
    //steps[next_point_idx].c_point.w - not updated
    //    double dw = 2*(dc.w / steps[step_idx].dt - steps[curr_point_idx].c_point.w);
    //    std::cout << "steps[" << curr_point_idx << "]- dw:" << dw << " dc.w : " << dc.w << " (dc.v) : " << dc.v << std::endl;
    //steps[next_point_idx].c_point.w = steps[curr_point_idx].c_point.w + dw;
    //    steps[next_point_idx].c_point.w = dc.w / steps[step_idx].dt;
    
    // .acc is only for visualization
    if (fwd)
      steps[step_idx].acc = acc;
    else
      steps[step_idx].acc = -acc;
    return updated;
  }
  
  //! Return the minimum dt time step between two states keeping the velocity constraints. 
  double minDtValidSpeed(const TrajectoryStepNaive &step) const
  {
    double position_diff = step.getPositionDiff();

    double max_vel = _params.maxVel;
    double max_rotational_vel = _params.maxRotationalVel;
    
    if (step.dir < 0) { // reversing
      max_vel = _params.maxVelRev;
      max_rotational_vel = _params.maxRotationalVelRev;
    }
    
    double dt_position = position_diff / max_vel;
    double dt_rotation = fabs(step.getHeadingDiff()) / max_rotational_vel;
    double dt_steering = fabs(step.getSteeringDiff()) / _params.maxSteeringAngleVel;
    double dt = dt_position;
    if (dt < dt_rotation)
      dt = dt_rotation;
    if (dt < dt_steering)
      dt = dt_steering;

    if (_params.useSteerDriveVel)
      {
	double phi = step.getPhiStep();
	double drivewheel_diff = position_diff / (cos(phi)*cos(phi)); // This will make it go even slower when turning, should be only cos (and not cos^2).
	double dt_drivewheel = drivewheel_diff / max_vel;
	if (dt < dt_drivewheel)
	  dt = dt_drivewheel;
      }
    return dt;
  }

  void assignTotalTime(TrajectoryStepNaiveVec &steps)
  {
    double time = 0.;
    for (size_t i = 0; i < steps.size(); i++)
      {
	steps[i].t_point = time;
	time += steps[i].dt;
      }
  }

 //! Fix the coordinated times. This relies on that the assignTotalTime has been done.
 void assignCoordinationTimes(TrajectoryStepNaiveVec &steps) {
   
   std::vector<size_t> idx = steps.getConstraintTimesPointIdx();
   if (idx.empty()) {
     std::cerr << "WARNING! CTS is empty (no constraints has been set)" << std::endl;
     return;
   }
   for (size_t i = 0; i < idx.size()-1; i++)
     {
       size_t current_idx = idx[i];
       size_t next_idx = idx[i+1];
       double dt = (steps[next_idx].t_point - steps[current_idx].t_point); // Need points to compute delta
       double cdt = (steps[next_idx].ct_point - steps[current_idx].ct_point); // -"-
       
       double factor = cdt / dt;
       if (factor < 1.)
       	 std::cerr << "WARNING: trying to run faster than the orignal assigned speed" << std::endl; // This should ideally not happen...

       // Take this factor and evenly distribute that over the dt in the interval.
       for (size_t j = current_idx; j < next_idx; j++) {
	 steps[j].dt *= factor;
	 steps[j].ct_factor = factor;
       }
     }
 }

 void assignControlConstraintPointsFromCT(TrajectoryStepNaiveVec &steps) {

   std::vector<size_t> idx = steps.getConstraintTimesPointIdx();
   if (idx.size() <= 2)
     return;

   for (size_t i = 1; i < idx.size()-1; i++) {

     // Compute the average speeds.
     orunav_generic::Control c;
     size_t prev_idx = idx[i]-1;
     size_t current_idx = idx[i];

     c.v = 0.5*(steps[prev_idx].c_step.v + steps[current_idx].c_step.v);
     c.w = 0.5*(steps[prev_idx].c_step.w + steps[current_idx].c_step.w);

     std::cout << "[TPN:] : c.v : " << c.v << ", c.w : " << c.w << std::endl;
     // steps[prev_idx].cc_point.assignControl(c);
     steps[current_idx].cc_point.assignControl(c);
   }
 }
 
 int getPrevPointIdx(const TrajectoryStepNaiveVec &steps, double time) const
  {
    for (int i = steps.size() -1; i >= 0; i--)
      {
       	if (steps[i].t_point <= time)
	  {
	    return i;
	  }
      }
    return -1;
  }

  int getNextPointIdx(const TrajectoryStepNaiveVec &steps, double time) const
  {
    for (size_t i = 0; i < steps.size(); i++)
      {
	if (steps[i].t_point > time)
	  {
	    return i;
	  }
      }
    return -1;
  }

  bool interpolateStatePoint(const TrajectoryStepNaiveVec &steps, const double &time, orunav_generic::State2d &state, double &usedTime) const
  {
    // Key point here. We want to consider also the velocity change in the interpolation. Simply using the time directly will then assume a constant velocity over the whole steps[prev_idx].dt step - which especially when the step is long (happens at starts and stops) will create relatively large errors. Hence the interpolation works on the factor based on the distance traveled.
    // NOTE! This assumes that c_point have been calculated.
    bool ret = false;
    
    assert(steps.size() > 0);
    if (time < steps.front().t_point) {
      std::cerr << "time : " << time << " steps.front().t_point : " << steps.front().t_point << std::endl;
    }
    assert(time >= steps.front().t_point);
    
    int prev_idx = getPrevPointIdx(steps, time);
    int next_idx = getNextPointIdx(steps, time);
    assert(steps[prev_idx].t_point <= time);

    double factor = 0; // Interpolation factor (0 <= factor < 1)
    usedTime = time;
    if (next_idx > 0)
      {
	double acc = (steps[next_idx].c_point.v - steps[prev_idx].c_point.v)/steps[prev_idx].dt;

	// Delta distance considering direction of motion
	//	double dir_dd = orunav_generic::getDist(steps[prev_idx].ds.getPose2d()) * orunav_generic::getDirectionIncr(steps[prev_idx].ds.getPose2d());
	double total_t = (steps[next_idx].t_point - steps[prev_idx].t_point);
	double total_interp_v = steps[prev_idx].c_point.v + acc * total_t;
	double total_mean_v = (steps[prev_idx].c_point.v + total_interp_v)*0.5;
	
	double t = (time - steps[prev_idx].t_point);
	// Interpolate the velocity, the speed at the 'endpoint'
	double interp_v = steps[prev_idx].c_point.v + acc * t;
	// Need the average velocity
	double mean_v = (steps[prev_idx].c_point.v + interp_v)*0.5;
	factor = mean_v * t / (total_mean_v * total_t); //dir_dd;

	if (factor < 0. || factor >= 1.) {
	  std::cout << "ERROR!!! factor : " << factor << std::endl; 
	  std::cout << "mean_v : " << mean_v << " t : " << t << " interp_v : " << interp_v << std::endl;
	  std::cout << "total_mean_v : " << total_mean_v << " total_t : " << total_t << std::endl;
	}
	//	std::cout << "time : " << time << "steps[" << prev_idx << "].t_point : " << steps[prev_idx].t_point << "steps[" << next_idx << "].t_point : " << steps[next_idx].t_point << " acc : " << acc << " t : " << t << " interp_v : " << interp_v << " factor : " << factor << std::endl;
	ret = true;
      }
    else
      {
	//std::cerr << "---- INTERPLATION STATE POINT - at the last step  : steps[prev_idx].ds.scale(factor).getPose2d() " << steps[prev_idx].ds.scale(factor).getPose2d() << ", steering() "  << steps[prev_idx].ds.scale(factor).getSteeringAngle() << std::endl;
	// One option is simply to return false (-> this means that we're outside - aftwards of the last entry point) and set the state to steps[prev_idx].s_point... 
	usedTime = steps[prev_idx].t_point;
      }
    //    assert(factor >= 0.);
    //    assert(factor < 1.);
    state = orunav_generic::addState2d(steps[prev_idx].s_point, steps[prev_idx].ds.scale(factor)); 
    return ret;
  }

  orunav_generic::State2d interpolateStatePoint(const TrajectoryStepNaiveVec &steps, const double &time)
  {
    orunav_generic::State2d state;
    double used_time; 
    
    if (!interpolateStatePoint(steps, time, state, used_time)) {
      // This will happen in the very end...
      /* std::cerr << "--------------------------------------------------------------ERROR!!!! : FAILED TO INTERPOLATE STATE, time : " << time << " used_time : " << used_time << ", state.getPose() : " << state.getPose2d() << ", getStering() : " << state.getSteeringAngle() << std::endl; */
    }
    return state;
  }

  // TODO - rather than interpolating between these to values (the dt might contain smaller part which could be used...
  bool interpolateControlStep(const TrajectoryStepNaiveVec &steps, const double &start_time, const double &stop_time, orunav_generic::Control &controlStep, double &usedDt) const
  {
    double start_used_time, stop_used_time;
    orunav_generic::State2d start, stop;
    
    bool ret = true;
    if (!interpolateStatePoint(steps, start_time, start, start_used_time))
      {
	std::cerr << "start_time : " << start_time << " start_used_time : " << start_used_time << std::endl;
	assert(false);
	ret = false;
      }
    if (!interpolateStatePoint(steps, stop_time, stop, stop_used_time))
      {
	ret = false;
      }
    
    usedDt = stop_used_time - start_used_time;
    
    //    int prev_idx = getPrevPointIdx(steps, start_time);
    //    controlStep = orunav_generic::Control(orunav_generic::getDist(ds.getPose2d()) / usedDt * , ds.getSteeringAngle() / usedDt);
  
    controlStep = start.getControlStep(stop, usedDt);

    // Debug
    //    orunav_generic::State2d ds = orunav_generic::subState2d(start, stop); // Fine, start and stop is points
    //    std::cout << "start time : " << start_time << " ds : " << orunav_generic::getDist(ds.getPose2d()) << std::endl;
  


    return ret;
  }


  

  orunav_generic::Trajectory convertNaiveStepsToTrajectory(const TrajectoryStepNaiveVec &steps) const
    {
      return steps.convertNaiveStepsToTrajectory();
    }
};

