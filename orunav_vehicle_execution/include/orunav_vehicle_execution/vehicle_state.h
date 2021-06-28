#pragma once


//! Keep additional information to additionally keep track of the commuincation to the controller.
class VehicleState {
public:
  enum State { WAITING_FOR_TASK = 1, PERFORMING_START_OPERATION, DRIVING, PERFORMING_GOAL_OPERATION, TASK_FAILED, WAITING_FOR_TASK_INTERNAL, DRIVING_SLOWDOWN, AT_CRITICAL_POINT };
  enum ControllerState { WAITING, ACTIVE, BRAKE, FINALIZING, ERROR, UNKNOWN, WAITING_TRAJECTORY_SENT, BRAKE_SENT };
  enum OperationState { NO_OPERATION = 1, UNLOAD, LOAD, LOAD_DETECT, ACTIVATE_SUPPORT_LEGS, LOAD_DETECT_ACTIVE };
  enum ForkState { FORK_POSITION_UNKNOWN = 1, FORK_POSITION_LOW, FORK_POSITION_HIGH, FORK_POSITION_SUPPORT_LEGS, FORK_MOVING_UP, FORK_MOVING_DOWN, FORK_FAILURE };

  enum PerceptionState { PERCEPTION_INACTIVE = 1, PERCEPTION_ACTIVE = 2 };

  enum BrakeReason { SENSOR = 1, TRACKING_ERROR, SERVICE_CALL, TOPIC_CALL };
  
  VehicleState() { state_ = WAITING_FOR_TASK; controllerState_ = UNKNOWN; forkState_ = FORK_POSITION_UNKNOWN; startOperation_ = NO_OPERATION; goalOperation_ = NO_OPERATION; prev_controller_status_ = -1; controller_status_ = -1; currentTrajectoryChunkIdx_ = 0; currentTrajectoryChunkStepIdx_ = 0; currentTrajectoryChunkEstIdx_ = 0; stepIdx_ = 0; isDocking_ = false; carryingLoad_ = false; currentPathIdx_ = 0; trajectoryChunksStartTime_ = 0.; dockingFailed_ = false; receivedControllerReport_ = false; receivedForkReport_ = false; validState2d_ = false; validControl_ = false; resendTrajectory_ = false; currentTime_ = ros::Time(0); perceptionState_ = PERCEPTION_INACTIVE;  
    activeTask_.criticalPoint = -1; slowdownCounter_ = 0;
    newVelocityConstraints_ = false;
    maxLinearVelocityConstraint_ = 1.;
    maxRotationalVelocityConstraint_ = 1.;
}
  
  void update(const orunav_msgs::Task &msg) {
    startOperation_ = static_cast<OperationState>(msg.target.start_op.operation);
    goalOperation_ = static_cast<OperationState>(msg.target.goal_op.operation);
    
    if (state_ == TASK_FAILED) {
      // Ok, got new task
      state_ = WAITING_FOR_TASK;
    }
 
    task_ = msg;
    
    //    performStartOperation();

    // Clear all state flags for docking.
    //isDocking_ = false;
    dockingFailed_ = false;
  }

  void activateTask() {
    activeTask_ = task_;
  }

  bool updatedTask() const {
    if (task_.update == true)
      return true;
    return false;
  }

  int getCriticalPointIdx() const {
    return task_.criticalPoint;
  }
  
  bool hasActiveTaskCriticalPoint() const {
    if (activeTask_.criticalPoint == -1)
      return false;
    return true;
  }

  bool atCriticalPoint() const {
    if (state_ == AT_CRITICAL_POINT)
      return true;
    return false;
  }

  void setAtCriticalPoint() {
    state_ = AT_CRITICAL_POINT;
  }

  void clearGoalOperation() {
    goalOperation_ = NO_OPERATION;
  }

  void clearStartOperation() {
    startOperation_ = NO_OPERATION;
  }
  
  bool validStartOperation() {
    if (startOperation_ == ACTIVATE_SUPPORT_LEGS) {
      ROS_ERROR("ACTIVATE_SUPPORT_LEGS not allowed as start operation...");
      return false;
    }
    /*if (startOperation_ == NO_OPERATION) {
      if (forkState_ == FORK_POSITION_SUPPORT_LEGS) {
        ROS_ERROR("FORK_POSITION_SUPPORT_LEGS not allowed fork state to start operation");
        return false;
      }
      }*/
    return true;
  }

  bool performStartOperation() {
    // Is the start operation valid?
    if (!validStartOperation()) {
      state_ = TASK_FAILED;
      ROS_INFO("NOT VALID START OP!!!");
      return true;
    }
    if (state_ == PERFORMING_START_OPERATION) {
      ROS_INFO("ALREADY PERFORMING START OP!!!");
      return true;
    }
    if (state_ == WAITING_FOR_TASK || state_ == WAITING_FOR_TASK_INTERNAL) {
      state_ = PERFORMING_START_OPERATION;
      ROS_INFO("JUST SET PERFORMING START OP!!!");
      if (startOperation_ != NO_OPERATION) {
	ROS_INFO("NOT NO_OPERATION!!!");
        return true;
      }
    }
    ROS_INFO("ALL OTHER CASES!!!");
    return false;
  }

  void setResendTrajectory(bool resend) {
    resendTrajectory_ = resend;
    if (taskFailed()) {
      resendTrajectory_ = false;
    }
  }
  
  bool getResendTrajectory() const { return resendTrajectory_; }


  void handleStartOperation(bool &completedTarget, bool &moveForks, bool &load) {
    if (startOperation_ == NO_OPERATION) {
      //      controllerState_ = WAITING;
      ROS_INFO("SET STATE TO DRIVING!!!!");
      state_ = DRIVING;
      /*if (forkState_ == FORK_POSITION_SUPPORT_LEGS) {
        // Cannot start driving if the support legs is active.
        state_ = TASK_FAILED;
	}*/
      return;
    }  

    //FIXME There is some incoherence here (see goalOperation LOAD).
    // Scenarios - if operation is UNLOAD -> the forks state should be at FORK_POSITION_LOW to continue
    //           - if operation is LOAD   -> the forks state should be at FORK_POSITION_HIGH to continue
    //           - if operation is ACTIVATE_SUPPORT_LEGS   -> now allowed (if only one operation should be sent we should instead provide it as a goal operation (with an empty path).
    if (startOperation_ == LOAD) {
      if (forkState_ == FORK_POSITION_HIGH) {
        //	controllerState_ = WAITING;
        state_ = DRIVING;
	carryingLoad_ = true;
	return;
      }
      else {
        state_ = PERFORMING_START_OPERATION;
	moveForks = true;
	load = true;
	return;
      }
    }
    if (startOperation_ == UNLOAD) {
      if (forkState_ == FORK_POSITION_LOW) {
        //	controllerState_ = WAITING;
        state_ = DRIVING;
	carryingLoad_ = false;
	return;
      }
      else {
        state_ = PERFORMING_START_OPERATION;
	moveForks = true;
	load = false;
	return;
      }
    }
    if (startOperation_ == ACTIVATE_SUPPORT_LEGS) {
      ROS_ERROR("Cannot active support legs as start operation");
      //      controllerState_ = WAITING;
      state_ = TASK_FAILED;
      return;
    }
    
  }


  void handleGoalOperation(bool &completedTarget, bool &moveForks, bool &load) {
    if (goalOperation_ == NO_OPERATION) {
      state_ = WAITING_FOR_TASK;
      //      controllerState_ = WAITING;
      moveForks = false;
      completedTarget = true;
      return;
    }

    // Scenarios - if operation is UNLOAD -> the forks state should be at FORK_POSITION_LOW to continue
    //           - if operation is LOAD   -> the forks state should be at FORK_POSITION_HIGH to continue
    //           - if operation is ACTIVATE_SUPPORT_LEGS   -> the forks state should be at FORK_POSITION_SUPPORT_LEGS to continue
    if (goalOperation_ == LOAD) {
      if (!isWaitingTrajectoryEmpty()) {
	//The robot hasn't reached the picking pose
	state_ = DRIVING;
	moveForks = false;
	completedTarget = false;
	return;
      }
      if (forkState_ == FORK_POSITION_HIGH) {
        state_ = WAITING_FOR_TASK;
        //	controllerState_ = WAITING;
	completedTarget = true;
	carryingLoad_ = true;
	return;
      }
      state_ = PERFORMING_GOAL_OPERATION;
      moveForks = true;
      completedTarget = false;
      load = true;
      return;
    }
    if (goalOperation_ == UNLOAD) {
      if (forkState_ == FORK_POSITION_LOW || !this->isCarryingLoad()) {
        state_ = WAITING_FOR_TASK;
        //        controllerState_ = WAITING;
        completedTarget = true;
	carryingLoad_ = false;
	return;
      }
      else {
        state_ = PERFORMING_GOAL_OPERATION;
	moveForks = true;
	load = false;
	return;
      }
    }
    if (goalOperation_ == ACTIVATE_SUPPORT_LEGS) {
      if (forkState_ == FORK_POSITION_SUPPORT_LEGS) {
        state_ = WAITING_FOR_TASK;
        //        controllerState_ = WAITING;
        completedTarget = true;
        carryingLoad_ = true;
        return;
      }
      else {
        state_ = PERFORMING_GOAL_OPERATION;
        moveForks = true;
        load = true;
        return;
      }
    }
    //Move the forks down to load the pallet (if necessary)
    if (goalOperation_ = LOAD_DETECT) {
      if (forkState_ != FORK_POSITION_LOW) moveForks = true;
      return;
    }
  }

  void update(const orunav_msgs::ForkReportConstPtr &msg, bool &completedTarget, bool &moveForks, bool &load, OperationState &operation) {
    
    moveForks = false;
    load = true;
    completedTarget = false;
    
    forkState_ = static_cast<ForkState>(msg->status);

    // Is the task ok to continue?
    if (taskFailed()) {
      return;
    }

    // Process the start operation (if any)
    if (state_ == PERFORMING_START_OPERATION) {
      handleStartOperation(completedTarget, moveForks, load);
      operation = startOperation_;
    }
    
    // Process the goal operation if (any)
    if (state_ == PERFORMING_GOAL_OPERATION) { 
      handleGoalOperation(completedTarget, moveForks, load);
      operation = goalOperation_;
    } 
  }


  void update(const orunav_msgs::ControllerReportConstPtr &msg, bool &completedTarget, bool useForks = false) {
    
    receivedControllerReport_ = true;
    controller_status_ = msg->status;
    currentTime_ = msg->stamp;

    // Current state and control
    orunav_generic::State2d state = orunav_conversions::createState2dFromControllerStateMsg(msg->state);
    if (validCurrentState2d()) {
      currentControl_ = currentState2d_.getControlStep(state, this->timeStep_);
      validControl_ = true;
    }
    currentState2d_ = state;
    validState2d_ = true;
    
    // Keep the chunk idx up to date
    if (!trajectoryChunks_.empty()) {
      // Update the current chunk idx when it's driving. 
      currentTrajectoryChunkEstIdx_ = orunav_generic::getCurrentChunkIdx(trajectoryChunks_, state.getPose2d(), currentTrajectoryChunkEstIdx_);
      // The controller will report zero as the sequence number whenever the controller is done tracking but before the complete_target flag is switched.
      if (currentTrajectoryChunkIdx_ < msg->traj_chunk_sequence_num) {
        currentTrajectoryChunkIdx_ = msg->traj_chunk_sequence_num;
      }
      currentTrajectoryChunkStepIdx_ = msg->traj_step_sequence_num;
      // if (abs(currentTrajectoryChunkEstIdx_ - currentTrajectoryChunkIdx_) > 0) {
      //   ROS_INFO_STREAM("ChunkIdx difference : " << currentTrajectoryChunkEstIdx_ - currentTrajectoryChunkIdx_);
      // }
    }

    // Keep the current path idx up to date.
    if (path_.sizePath() > 0) {
      orunav_generic::State2d state = orunav_conversions::createState2dFromControllerStateMsg(msg->state);
      currentPathIdx_ = orunav_generic::getCurrentIdx(path_, state.getPose2d(), currentPathIdx_);
    }

    completedTarget = false;
    bool completed_target = false;
    // Find out the transitions between the states in the controller. This is used to determine if the system completed the target or not.
    if (prev_controller_status_ != -1) {
      if (controller_status_ != prev_controller_status_) {
	// State has changed.
	// FINALIZING -> WAIT -> target completed
	// ACTIVE -> WAIT -> target completed
	if (controller_status_ == msg->CONTROLLER_STATUS_WAIT && (prev_controller_status_ == msg->CONTROLLER_STATUS_ACTIVE || prev_controller_status_ == msg->CONTROLLER_STATUS_FINALIZE)) {
	  if (!this->hasActiveTaskCriticalPoint()) {
	    if (currentPathIdx_ == path_.sizePath()-1 || goalOperation_ == LOAD) {
		completed_target = true;
		if (!useForks) {
		  completedTarget = completed_target;
		  state_ = WAITING_FOR_TASK;
		  ROS_INFO_STREAM("Detected active/finalize->wait transition (no forks). Mission is completed.");
		}
		else ROS_INFO_STREAM("Detected active/finalize->wait transition (use forks). Navigation completed ... going to perform goal operation.");
		this->clearTrajectoryChunks();
		this->clearCurrentPath();
	    }
	    else ROS_INFO_STREAM("Detected active/finalize->wait transition but mission is not completed yet. Keeping the same state.");
	  }
	  else {
	    state_ = AT_CRITICAL_POINT;
	    ROS_INFO_STREAM("Detected active/finalize->wait transition with active critical points.");
	  }
	}
      }
      
      // Update the internal state.
      if (completed_target && useForks) { // This state can only be altered by the fork report callback.
	controllerState_ = WAITING;
	state_ = PERFORMING_GOAL_OPERATION;
      }
      else if (state_ == PERFORMING_START_OPERATION && useForks) { // This state can only be altered by the fork report callback.
      }
      else if (state_ == PERFORMING_START_OPERATION && useForks == false) { 
	state_ = DRIVING;
      }
      else {
	if (controller_status_ == msg->CONTROLLER_STATUS_WAIT) {
	  // Only if no trajectory has been sent and the vehicle is not yet in active state.
	  if (controllerState_ != WAITING_TRAJECTORY_SENT) {
	    controllerState_ = WAITING;
	  }
	}
	if (controller_status_ == msg->CONTROLLER_STATUS_ACTIVE) {
	  if (controllerState_ != BRAKE_SENT)  // Shouldn't really be any difference ACTIVE / BRAKE_SENT. (remove?)
	    controllerState_ = ACTIVE;
	}
	if (controller_status_ == msg->CONTROLLER_STATUS_FAIL) {
	  controllerState_ = BRAKE;
	  // Need to clear the trajectory chunk
	}
	if (controller_status_ == msg->CONTROLLER_STATUS_FINALIZE) {
	  controllerState_ = FINALIZING;
	}
	if (controller_status_ == msg->CONTROLLER_STATUS_TERMINATE) {
	  controllerState_ = ERROR;
	}
      }
      if (state_ == AT_CRITICAL_POINT) {
	if (controllerState_ == ACTIVE) {
	  state_ = DRIVING;
	}
      }
    }

    prev_controller_status_ = controller_status_;

    // Update driven trajectory
    if (controller_status_ != msg->CONTROLLER_STATUS_WAIT) {
      drivenTrajectory_.add(currentState2d_, currentControl_);
      drivenTrajectoryTimes_.push_back(msg->stamp.toSec());
    }
    if (completed_target) {
      drivenTrajectory_.clear();
      drivenTrajectoryTimes_.clear();
    }
  }

  bool canSendTrajectory() const {
    // Can only send it if the vehicle is in waiting state or in active state
    if (controllerState_ == WAITING || controllerState_ == ACTIVE)
      return true;

    if (brakeSentUsingServiceCall())
      return true;
    
    return false;
  }

  bool isDriving() const { 
    if (state_ == DRIVING || state_ == DRIVING_SLOWDOWN)
      return true;
    return false;
  }

  bool isDrivingSlowdown() const {
    if (state_ == DRIVING_SLOWDOWN)
      return true;
    return false;
  }

  bool setDrivingSlowdown(bool slowdown) {
    if (slowdown && state_ == DRIVING) {
      state_ = DRIVING_SLOWDOWN;
      slowdownCounter_ = 0;
      return true;
    }
    if (!slowdown && state_ == DRIVING_SLOWDOWN) {
      slowdownCounter_++;
      if (slowdownCounter_ > 3) { // The counter is to avoid influence of "jitter" in the readings
        state_ = DRIVING;
        return true;
      }
    }
    return false;
  }

  bool taskFailed() const {
    if (state_ == TASK_FAILED)
      return true;
    return false;
  }

  void trajectorySent() {
    assert(canSendTrajectory());
    controllerState_ = WAITING_TRAJECTORY_SENT;
  }

  void brakeSent(BrakeReason reason) {
    active_brake_reasons_.insert(reason);
  }

  void brakeClear(BrakeReason reason) {
    auto it=active_brake_reasons_.find(reason);
    if (it != active_brake_reasons_.end()) {
      active_brake_reasons_.erase(it);
    }
  }
  
  bool allBrakeReasonsCleared() const {
    return (active_brake_reasons_.empty());
  }

  bool brakeSentUsingServiceCall() const {
    return (active_brake_reasons_.count(BrakeReason::SERVICE_CALL) != 0);
  }
  
  ControllerState getControllerState() const { 
    return controllerState_; 
  }

  orunav_generic::TrajectoryChunks getTrajectoryChunks() const {
    return trajectoryChunks_;
  }

  bool isChunksEmpty() const {
    return trajectoryChunks_.empty();
  }

  bool isChunkIdxValid(unsigned int chunkIdx) const {
    return (chunkIdx < trajectoryChunks_.size());
  }


  bool isCurrentChunkIdxValid() const {
    return (isChunkIdxValid(currentTrajectoryChunkIdx_));
  }

  bool isCurrentPathIdxValid() const {
    if (currentPathIdx_ < path_.sizePath()) {
      return true;
    }
    return false;
  }

  const orunav_generic::TrajectoryChunks& getTrajectoryChunksRef() const {
    return trajectoryChunks_;
  }
  
  void appendTrajectoryChunks(unsigned int chunkIdx, const orunav_generic::TrajectoryChunks &add) {
    this->trajectoryChunks_ = orunav_generic::appendChunks(this->getTrajectoryChunks(), chunkIdx, add);
  }

  void setTrajectoryChunksStartTime(double time) { 
    //if (canSendTrajectory()) 
    {
      this->trajectoryChunksStartTime_ = time; 
    }
  }
  
  unsigned int getCurrentTrajectoryChunkIdxUsingTime(double time) {
    // 0.06 is the dt (10 dt in each chunk = 0.6 secs)... we have the starting time (trajectoryChunksStartTime_).
    if (trajectoryChunksStartTime_ == 0.)
      return 0;

    if (time < trajectoryChunksStartTime_)
      return 0;
 
    double diff = time - trajectoryChunksStartTime_;
    unsigned int idx = static_cast<unsigned int>(diff / (this->timeStep_*10)); // 0.6

    if (idx >= trajectoryChunks_.size()) {
      return trajectoryChunks_.size() - 1;
    }
    return idx;
  }

  double timeWhenChunkIdxIsReached(unsigned int chunkIdx) const {

    int diff = chunkIdx - currentTrajectoryChunkIdx_;
    if (diff < 0)
      return -1.;
    
    double time = currentTime_.toSec();
    time += diff * this->timeStep_*10;
    time -= currentTrajectoryChunkStepIdx_*this->timeStep_;
    return time;
  }

  //  void setTrajectoryChunks(const orunav_generic::TrajectoryChunks &trajChunks) {
  //    trajectoryChunks_ = trajChunks;
  //  }

  unsigned int getCurrentTrajectoryChunkIdx() const {
    return currentTrajectoryChunkIdx_;
  }
  
  unsigned int getCurrentTrajectoryStepIdx() const {
    return currentTrajectoryChunkStepIdx_;
  }

  void clearTrajectoryChunkIdx() {
    currentTrajectoryChunkIdx_ = 0;
    currentTrajectoryChunkEstIdx_ = 0;
    stepIdx_ = 0;
    
  }
  
  void clearTrajectoryChunks() {
    // Only allowed to clear the chunks if the hole trajectory has been driven.
    // The size of trajectoryChunks_ is used to determine if the controller
    // was interupted during execution(!), e.g. using a brake.
    trajectoryChunks_.clear();
    clearTrajectoryChunkIdx();
  }

  void setStepIdx(unsigned int idx) {
    stepIdx_ = idx;
  }

  unsigned int getStepIdx() const {
    return stepIdx_;
  }

  // -- Possible states in the loop --
  bool vehicleStoppedAndTrajectoryNotCompleted() const {
    if (controllerState_ == WAITING && !trajectoryChunks_.empty())
      return true;
    return false;
  }

  bool isActive() const  {
    if (controllerState_ == ACTIVE) {
      return true;
    }
    return false;
  }

  bool isBraking() const {
    if (controllerState_ == BRAKE) {
      return true;
    }
    return false;
  }

  bool isWaitingTrajectoryEmpty() const {
    if (controllerState_ == WAITING && trajectoryChunks_.empty()) {
      return true;
    }
    return false;
  }

  // -- Possible states in the loop --
  
  bool isWaiting() const {
    if (controllerState_ == WAITING) {
      return true;
    }
    return false;
  }

  bool canSendActivate() const {
    const orunav_msgs::ControllerReport msg;
    if (controller_status_ == msg.CONTROLLER_STATUS_WAIT)
      return true;
    return false;
  }
  
  std::string getControllerStatusStr(int status) const {
    const orunav_msgs::ControllerReport msg;
    std::string ret = std::string("<null>");
    if (status == msg.CONTROLLER_STATUS_FAIL) {
      ret = std::string("FAIL");
    }
    if (status == msg.CONTROLLER_STATUS_ACTIVE) {
      ret = std::string("ACTIVE");
    }
    if (status == msg.CONTROLLER_STATUS_WAIT) {
      ret = std::string("WAIT");
    }
    if (status == msg.CONTROLLER_STATUS_FINALIZE) {
      ret = std::string("FINALIZE");
    }
    if (status == msg.CONTROLLER_STATUS_TERMINATE) {
      ret = std::string("TERMINATE");
    }
    return ret;
  }

  std::string getDebugStringExtended() const {
    return getDebugString() + std::string("\n[ControlStatus] : ") + getControllerStatusStr(controller_status_) + std::string("\n[StartOperation] : ") + getStrOperation(startOperation_)  + std::string("\n[GoalOperation] : ") + getStrOperation(goalOperation_);
  }
  
  std::string getDebugString() const {
    return std::string("[VehicleState] : ") + getStr() + std::string("\n[ControllerState] : ") + getStrController() + std::string("\n[ForkState] : ") + getStrFork();
  }

  std::string getStrOperation(OperationState state) const {
    switch(state) {
      case NO_OPERATION:
        return std::string("NO_OPERATION");
      case UNLOAD:
        return std::string("UNLOAD");
      case LOAD:
        return std::string("LOAD");
      case LOAD_DETECT:
        return std::string("LOAD_DETECT");
      case ACTIVATE_SUPPORT_LEGS:
        return std::string("ACTIVATE_SUPPORT_LEGS");
      default:
        assert(false);
        break;
        return std::string("unknown(!)");
    }
  }

  

  bool isDocking() const {
    return isDocking_;
  }

  void setDocking(bool docking) {
    isDocking_ = docking;
  }

  void setDockingFailed(bool failed) {
    dockingFailed_ = failed;
  }

  bool getDockingFailed() const {
    return dockingFailed_;
  }

  void setDockingPose(const orunav_generic::Pose2d &pose) {
    dockingPose_ = pose;
  }

  orunav_generic::Pose2d getDockingPose() const {
    return dockingPose_;
  }

  bool goalOperationLoad() const {
    if (goalOperation_ == LOAD)
      return true;
    return false;
  }

  bool goalOperationLoadDetect() const {
    if (goalOperation_ == LOAD_DETECT)
      return true;
    return false;
  }

  bool isCarryingLoad() const {
    return carryingLoad_;
  }

  void setPath(const orunav_generic::Path &path) {
    path_ = path;
  }

  void setCoordinatedTimes(const orunav_generic::CoordinatedTimes &cts) {
    cts_ = cts;
  }

  bool updatePath(const orunav_generic::Path &path) {
    if (path_.sizePath() == 0) {
      path_ = path;
      return true;
    }
    orunav_generic::Pose2d current_start = path_.getPose2d(0);
    orunav_generic::Pose2d current_goal = path_.getPose2d(path_.sizePath()-1);
    double dist1 = orunav_generic::getDistBetween(current_start, path.getPose2d(0));
    double dist2 = orunav_generic::getDistBetween(current_goal, path.getPose2d(0));
    if (dist1 > 0.1 && dist2 > 0.2)
      return false;
    if (dist1 < dist2) {
      path_ = path;
      return true;
    }
    // Need to connect them... // TODO - handle the constraints + coordinated times.
    orunav_generic::addPathToPath(path_, path);
    orunav_generic::makeValidPathForTrajectoryProcessing(path_);
  }
  
  //! Return the path, if the criticalIdx is set the path up to this index will be returned.
  orunav_generic::Path getPath() const {
    if (this->getCriticalPointIdx() >= 0) {
      if (this->getCriticalPointIdx()+1 < path_.sizePath()) {
        return orunav_generic::selectPathIntervall(path_, 0, this->getCriticalPointIdx()+1);
      }
      else {
        // bad critical point index. return the full path
        if (path_.sizePath() != 0)
          ROS_WARN_STREAM("--- critical point index is outside the path(!) : " << path_.sizePath() << " " << this->getCriticalPointIdx());
      }
    }
    return path_;
  }
  
  orunav_generic::Path getPathFromLastAssignedTask() const { 
    return orunav_conversions::createPathFromPathMsgUsingTargetsAsFirstLast(task_.path);
  }

  //! Return the cts, if the criticalIdx is set the path up to this index will be returned.
  orunav_generic::CoordinatedTimes getCoordinatedTimes() const {
    if (this->getCriticalPointIdx() >= 0) {
      if (this->getCriticalPointIdx()+1 < cts_.size()) {
        return orunav_generic::selectCtsInterval(cts_, 0, this->getCriticalPointIdx()+1);
      }
      else {
        // bad critical point index. return the full cts
        ROS_WARN_STREAM("--- critical point index is outside the cts(!) : " << this->getCriticalPointIdx());
      }
    }

    return cts_;
  }

  bool validCoordinatedTimes() const {
    if (cts_.empty()) {
      return false;
    }
    if (cts_.size() != path_.sizePath()) {
      return false;
    }
    return true;
  }
  double getCoordinatedStartTime() const {
    
    if (validCoordinatedTimes())
      return cts_[0];
    return -1.;
  }

  void clearCoordinatedTimes() {
    cts_.clearAllEntries();
  }

  unsigned int getCurrentPathIdx() const {
    return currentPathIdx_;
  }

  void setCurrentPathIdx(unsigned int pathIdx) {
    currentPathIdx_ = pathIdx;
  }

  unsigned int getEarliestConnectChunkIdx() const {
    return currentTrajectoryChunkIdx_ + 3;
  }

  void clearCurrentPath() {
    path_.clear();
    currentPathIdx_ = 0;
  }

  bool validCurrentState2d() {
    return validState2d_;
  }
  
  orunav_generic::State2d getCurrentState2d() const {
    return currentState2d_;
  }

  bool validCurrentControl() {
    return validControl_;
  }

  orunav_generic::Control getCurrentControl() const {
    return currentControl_;
  }

  std::string getStr() const {
    switch(state_) {
      case WAITING_FOR_TASK:
        if (trajectoryChunks_.empty())
          return std::string("WAITING_FOR_TASK");
        else
          return std::string("WAITING_FOR_TASK INTERM");
      case PERFORMING_START_OPERATION:
        return std::string("PERFORMING_START_OPERATION");
      case PERFORMING_GOAL_OPERATION:
        return std::string("PERFORMING_GOAL_OPERATION");
      case DRIVING:
        return std::string("DRIVING");
      case TASK_FAILED:
        return std::string("TASK_FAILED");
      case WAITING_FOR_TASK_INTERNAL:
        return std::string("WAITING_FOR_TASK_INTERNAL");
      case DRIVING_SLOWDOWN:
        return std::string("DRIVING_SLOWDOWN");
      case AT_CRITICAL_POINT:
        return std::string("AT_CRITICAL_POINT");
      default:
        break;
    }
    return std::string("unknown(!)");
  }

  std::string getStrController() const {
    switch(controllerState_) {
      case WAITING:
        return std::string("WAITING");
      case ACTIVE:
        return std::string("ACTIVE");
      case BRAKE:
        return std::string("BRAKE");
      case FINALIZING:
        return std::string("FINALIZING");
      case ERROR:
        return std::string("ERROR");
      case UNKNOWN:
        return std::string("UNKNOWN");
      case WAITING_TRAJECTORY_SENT:
        return std::string("WAITING_TRAJECTORY_SENT");
      case BRAKE_SENT:
        return std::string("BRAKE_SENT");
      default:
        break;
    }
    return std::string("unknown(!)");
  }
  
  std::string getStrFork() const {
    switch (forkState_) {
      case FORK_POSITION_UNKNOWN:
        return std::string("FORK_POSITION_UNKNOWN");
      case FORK_POSITION_LOW:
        return std::string("FORK_POSITION_LOW");
      case FORK_POSITION_HIGH:
        return std::string("FORK_POSITION_HIGH");
      case FORK_POSITION_SUPPORT_LEGS:
        return std::string("FORK_POSITION_SUPPORT_LEGS");
      case FORK_MOVING_UP:
        return std::string("FORK_MOVING_UP");
      case FORK_MOVING_DOWN:
        return std::string("FORK_MOVING_DOWN");
      case FORK_FAILURE:
        return std::string("FORK_FAILURE");
      default:
        break;
    }
    return std::string("unknown(!)");
  }

  int getEarliestPathIdxToConnect() {
    if (this->getCurrentTrajectoryChunkIdx() < 0)
      return -1;
    orunav_generic::TrajectoryChunks chunks = this->getTrajectoryChunks();
    unsigned int chunk_idx = this->getCurrentTrajectoryChunkIdx()+3; // Approx. 1.5 sec ahead.
    if (chunk_idx >= chunks.size())
      return -1;
    orunav_generic::Path path = this->getPath();
    double distance;
    int idx = getPathIdxUsingFirstPointInChunk3(path, chunks, chunk_idx, distance);
    return idx;
  }

  orunav_generic::Trajectory getDrivenTrajectory() const { return drivenTrajectory_; }
  orunav_generic::CoordinatedTimes getDrivenTrajectoryTimes() const { return drivenTrajectoryTimes_; }

  orunav_msgs::Task getTask() const { return task_; }

  orunav_generic::State2d getCurrentGoalState() const {
    if (path_.sizePath() == 0) {
      return currentState2d_;
    }
    return orunav_generic::State2d(path_, path_.sizePath()-1);
  }

  void saveCurrentTrajectoryChunks(const std::string &fileName) const {
    orunav_generic::saveTrajectoryChunksTextFile(trajectoryChunks_, fileName);
  }

  orunav_generic::RobotInternalState2d getInternalState2d() const {
    return internalState2d_;
  }
  
  orunav_msgs::RobotReport getReport() const {
    orunav_msgs::RobotReport msg;
    msg.stamp = currentTime_;
    msg.status = state_;
    msg.state = orunav_conversions::createPoseSteeringMsgFromState2d(currentState2d_);
    msg.sequence_num = currentPathIdx_;
    msg.nb_active_brake_reasons = active_brake_reasons_.count(BrakeReason::SERVICE_CALL);
    return msg;
  }

  orunav_geometry::PalletModel2dWithState getPalletModelLoadDetect() const {
    return getPalletModelFromRobotTarget(task_.target);
  }

  bool isPerceptionRequired() const {
    if (goalOperation_ == LOAD_DETECT) {
      if (task_.target.goal_load.status != task_.target.goal_load.EMPTY) {
        if (state_ == PERFORMING_GOAL_OPERATION) {
          return true;
        }
      }
    }
    
    if (goalOperation_ == LOAD_DETECT_ACTIVE) {
      if (task_.target.goal_load.status != task_.target.goal_load.EMPTY) {
        if (state_ == PERFORMING_GOAL_OPERATION || state_ == DRIVING) {
          return true;
        }
      }
    }
    return false;
  }

  bool activatePerception() {
    if (isPerceptionRequired()) {
      if (perceptionState_ == PERCEPTION_ACTIVE)
        return false;
      perceptionState_ = PERCEPTION_ACTIVE;
      return true;
    }
    return false;
  }
  
  bool inactivatePerception() {
    if (!isPerceptionRequired()) {
      if (perceptionState_ == PERCEPTION_INACTIVE)
        return false;
      perceptionState_ = PERCEPTION_INACTIVE;
      return true;
    }
    return false;
  }

  bool setPerceptionReceived() {
    if (state_ == PERFORMING_GOAL_OPERATION && 
        controllerState_ == WAITING &&
        perceptionState_ == PERCEPTION_ACTIVE)
    {
      state_ = WAITING_FOR_TASK_INTERNAL;
      return true;
    }
    return false;
  }

  void setTimeStep(double ts) {
    timeStep_ = ts;
  }

  bool newVelocityConstraints() const {
    return newVelocityConstraints_;
  }

  double getMaxLinearVelocityConstraint() const {
    return maxLinearVelocityConstraint_;
  }

  double getMaxLinearVelocityConstraintRev() const {
    return maxLinearVelocityConstraintRev_;
  }

  double getMaxRotationalVelocityConstraint() const {
    return maxRotationalVelocityConstraint_;
  }

  double getMaxRotationalVelocityConstraintRev() const {
    return maxRotationalVelocityConstraintRev_;
  }

  void setNewVelocityConstraints(double max_linear_vel,
				 double max_rotational_vel,
				 double max_linear_vel_rev,
				 double max_rotational_vel_rev) {
    maxLinearVelocityConstraint_ = max_linear_vel;
    maxRotationalVelocityConstraint_ = max_rotational_vel;
    maxLinearVelocityConstraintRev_ = max_linear_vel_rev;
    maxRotationalVelocityConstraintRev_ = max_rotational_vel_rev;
    newVelocityConstraints_ = true;
  }

  void resetNewVelocityConstraint() {
    newVelocityConstraints_ = false;
  }
  
  bool abortTask() {
    if (isActive()) {
      return false;
    }
    this->clearTrajectoryChunks();
    this->clearCurrentPath();
    state_ = WAITING_FOR_TASK;
    return true;
  }

private:

  State state_;
  ControllerState controllerState_;
  ForkState forkState_;
  OperationState startOperation_;
  OperationState goalOperation_;
  PerceptionState perceptionState_;
  int controller_status_; // Current controller state / status.
  int prev_controller_status_; // Previous controller state / status. Used to trigger state transitions.
  orunav_generic::TrajectoryChunks trajectoryChunks_;
  unsigned int currentTrajectoryChunkIdx_; // From the controller reports.
  unsigned int currentTrajectoryChunkStepIdx_; // More fine graded index than ChunkIdx.
  unsigned int currentTrajectoryChunkEstIdx_; // This is estimated based on the current pose.
  unsigned int stepIdx_; // This relates the path index... - TODO. -> update this internally.
  bool isDocking_;
  bool carryingLoad_; // TODO - should be replaced by internal state 2d.
  orunav_generic::Path path_;
  orunav_generic::CoordinatedTimes cts_;
  unsigned int currentPathIdx_;
  double trajectoryChunksStartTime_;
  bool dockingFailed_;
  orunav_generic::Pose2d dockingPose_;
  orunav_generic::State2d currentState2d_; // Current state from the controller report.
  orunav_generic::Control currentControl_;
  ros::Time currentTime_;

  bool receivedControllerReport_;
  bool receivedForkReport_;
  bool validState2d_;
  bool validControl_;
  bool resendTrajectory_;

  orunav_generic::Trajectory drivenTrajectory_;
  orunav_generic::CoordinatedTimes drivenTrajectoryTimes_; // Global time when each trajectory_ point was reached.

  orunav_msgs::Task task_; // Last task requested (by the update function)
  orunav_msgs::Task activeTask_; // The task that is currently executed.

  orunav_generic::RobotInternalState2d internalState2d_;

  double timeStep_;
  int slowdownCounter_;

  double maxLinearVelocityConstraint_;
  double maxRotationalVelocityConstraint_;
  double maxLinearVelocityConstraintRev_;
  double maxRotationalVelocityConstraintRev_;
  bool newVelocityConstraints_;

  std::set<BrakeReason> active_brake_reasons_;
};
