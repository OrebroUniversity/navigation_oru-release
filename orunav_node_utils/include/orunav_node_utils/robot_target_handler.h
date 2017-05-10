#pragma once

#include <orunav_generic/utils.h>
#include <orunav_msgs/RobotTarget.h>

namespace orunav_node_utils {

inline bool isOldTarget(const ros::Time &currentTime, const orunav_msgs::RobotTarget& target) {
  /* if (currentTime > target.goal_deadline) */
  /*   return true; */
  return false;
}

inline bool isActiveTarget(const ros::Time &currentTime, const orunav_msgs::RobotTarget& target) {
  /* if (target.start_earliest <= currentTime && currentTime <= target.goal_deadline) */
  /*   return true; */
  /* return false; */
  return true;
}

inline bool isFutureTarget(const ros::Time &currentTime, const orunav_msgs::RobotTarget& target) {
  /* if (currentTime <= target.start_earliest) */
  /*   return true; */
  return false;
}

const int MAX_ROBOT_ID = 10000;

inline long getUniqueID_(int robotID, int goalID) {
  //  return (long)robotID * (long)std::numeric_limits<int>::max() + (long)goalID;
  assert(robotID < MAX_ROBOT_ID);
  return (long)goalID * MAX_ROBOT_ID + (long)robotID;
}

inline long getUniqueID(const orunav_msgs::RobotTarget& target) {
  return getUniqueID_(target.robot_id, target.goal_id);
}


inline void getIDsFromUnique(const long &uniqueID, int &robotID, int &goalID) {
  //  goalID = static_cast<int>(uniqueID % std::numeric_limits<int>::max());
  //  robotID = static_cast<int>(uniqueID / std::numeric_limits<int>::max());
   goalID = static_cast<int>(uniqueID / MAX_ROBOT_ID);
   robotID = static_cast<int>(uniqueID % MAX_ROBOT_ID);
}

 inline std::string getIDsString(long ID) {
   int robot_id, goal_id;
   getIDsFromUnique(ID, robot_id, goal_id);
   std::string ret = std::string("RID:") + orunav_generic::toString(robot_id) + std::string(" GID:") + orunav_generic::toString(goal_id);
   return ret;
 }

 inline std::string getIDsFileString(long ID) {
   int robot_id, goal_id;
   getIDsFromUnique(ID, robot_id, goal_id);
   std::string ret = std::string("RID_") + orunav_generic::toString(robot_id) + std::string("_GID_") + orunav_generic::toString(goal_id);
   return ret;
 }


//! Helper class for the RobotTargetHandler
class SingleRobotTargetHandler {
 public:
 SingleRobotTargetHandler(int robotID) : robotID_(robotID) { } 
  void assignTarget(const orunav_msgs::RobotTarget &target) {
    if (target.robot_id != robotID_)
      return;
    if (targets_.count(target.goal_id) <= 0) {
      targets_[target.goal_id] = target;
    }
  }
  
  void cleanOldTargets(const ros::Time &currentTime) {
    std::vector<int> old_goals;
    for (std::map<int, orunav_msgs::RobotTarget>::iterator it=targets_.begin(); it!=targets_.end(); ++it) {
      if (isOldTarget(currentTime, it->second))
	old_goals.push_back(it->first);
    }
    for (unsigned int i = 0; i < old_goals.size(); i++) {
      targets_.erase(old_goals[i]);
    }
  }
  
  bool validProcessedGoal(int goalID) const {
    return (processedGoals_.count(goalID) != 0);
  }
  
  void setProcessedGoal(int goalID) { 
    if (targets_.count(goalID) == 0) {
      ROS_WARN("Unknown goalID");
      return;
    }
    if (processedGoals_.count(goalID) == 0) {
      processedGoals_.insert(goalID);
    }
    if (!validProcessedGoal(goalID)) {
      ROS_ERROR("setProcessedGoal not updated(!)");
    }
  }

  void clearProcessedGoal(int goalID) {
    if (targets_.count(goalID) == 0) {
      ROS_WARN("Unknown goalID");
      return;
    }
    if (processedGoals_.count(goalID) == 0) {
      // Note - this can happen.
      ROS_WARN("Trying to clear a non-processed goalID");
    }
    else {
      // Remove it
      std::set<int>::iterator it=processedGoals_.find(goalID);
      processedGoals_.erase(it); 
    }
    if (validProcessedGoal(goalID)) {
      ROS_ERROR("clearProcessedGoal not updated(!)");
    }
  }

  void setCompletedGoal(int goalID) { 
    if (targets_.count(goalID) == 0) {
      ROS_WARN("Unknown goalID");
      return;
    }
    if (completedGoals_.count(goalID) == 0) {
      completedGoals_.insert(goalID);
    } else {
      ROS_ERROR("setCompletedGoal already set completed for GOAL:%d", goalID);
    }
    // Remove the target?
    //    targets_.erase(goalID);
  }

  bool isCompletedGoal(int goalID) const {
    if (completedGoals_.count(goalID) == 0)
      return false;
    return true;
  }

  std::vector<int> getActiveProcessedGoals(const ros::Time &currentTime) const {
    // This really should be only a single target(!)
    std::vector<int> ret;
    for (std::map<int, orunav_msgs::RobotTarget>::const_iterator it=targets_.begin(); it!=targets_.end(); ++it) {
      if (validProcessedGoal(it->second.goal_id) && !isCompletedGoal(it->second.goal_id)) {
	if (isActiveTarget(currentTime, it->second))
	  ret.push_back(it->first);
      }
    }
    //    assert(ret.size() < 2); // Could be empty or max 1 in size.
    // This happens easily when you stress the system, however, this should not happen during proper execution.
    if (ret.size() > 1) { 
      //ROS_WARN("There is more than one active target for one robot(!)"); 
      //this->printDebug();
    }
    return ret;
  }

  std::vector<int> getFutureProcessedGoals(const ros::Time &currentTime) const {
    std::vector<int> ret;
    for (std::map<int, orunav_msgs::RobotTarget>::const_iterator it=targets_.begin(); it!=targets_.end(); ++it) {
      if (validProcessedGoal(it->second.goal_id) && !isCompletedGoal(it->second.goal_id)) {
	if (isFutureTarget(currentTime, it->second))
	  ret.push_back(it->first);
      }
    }
    return ret;
  }

  // Find out if there is anything that needs to be processed
  std::vector<int> getGoalsToProcess() const {
    std::vector<int> ret;
    for (std::map<int, orunav_msgs::RobotTarget>::const_iterator it=targets_.begin(); it!=targets_.end(); ++it) {
      
      if (processedGoals_.count(it->first) == 0 && completedGoals_.count(it->first) == 0) {
	ret.push_back(it->first);
      }
    }
    return ret;
  }

  // Return the processed goals
  std::vector<int> getProcessedGoals() const {
    return std::vector<int>(processedGoals_.begin(), processedGoals_.end());
  }

  std::vector<int> getCompletedGoals() const {
    return std::vector<int>(completedGoals_.begin(), completedGoals_.end());
  }

  double getStartDeadline(int goalID) const {
    return targets_.find(goalID)->second.start_deadline.toSec();
  }

  int getNextGoalToProcess() const {
    std::vector<int> goals = getGoalsToProcess();
    if (goals.empty())
      return -1;
    if (goals.size() == 1)
      return goals[0];

    // Check the most important one (the one with the earliest start deadline)
    unsigned int min_idx = 0;
    double min_start_time = goals[0];
    for (unsigned i = 1; i < goals.size(); i++) {
      double st = getStartDeadline(goals[i]);
      if (st < min_start_time) {
    	min_idx = i;
    	min_start_time = st;
      }
    }

    return goals[min_idx];
  }
  
  void printDebug() const {
    for (std::map<int, orunav_msgs::RobotTarget>::const_iterator it=targets_.begin(); it!=targets_.end(); ++it) {
      std::cout << "-- robot_id : " << it->second.robot_id << " goal_id : " << it->second.goal_id << std::endl;
      std::cout << "   current time   : " << ros::Time::now() << std::endl;
      std::cout << "   isOldTarget    : " << isOldTarget(ros::Time::now(), it->second) << std::endl;
      std::cout << "   isActiveTarget : " << isActiveTarget(ros::Time::now(), it->second) << std::endl;
      std::cout << "   start_deadline : " << it->second.start_deadline << std::endl;

    }
    std::cout << "    processed goals sizes : " << processedGoals_.size() << std::endl;
    for (std::set<int>::const_iterator it=processedGoals_.begin(); it!=processedGoals_.end(); ++it) {
      std::cout << "-- processed goal_id : " << *it << std::endl;
    }
    std::cout << "    completed goals sizes : " << completedGoals_.size() << std::endl;
    for (std::set<int>::const_iterator it=completedGoals_.begin(); it!=completedGoals_.end(); ++it) {
      std::cout << "-- completed goal_id : " << *it << std::endl;
    }

  }

 private:
  int robotID_;
  std::map<int, orunav_msgs::RobotTarget> targets_; // Here the index is the goal_id
  std::set<int> processedGoals_;
  std::set<int> completedGoals_;
};


class RobotTargetHandler {
 public:
  RobotTargetHandler() { }
  //! If the handler should only respond to a specific set of robotID's. (If this is not set (or an empty vector is set), all robotID's will be used).
  void setRobotIDsToCompute(const std::vector<int> &robotIDs) {
    robotIDs_ = robotIDs;
  }
  void assignTarget(const orunav_msgs::RobotTarget &target) {
    // Should we use this target?
    if (!robotIDs_.empty()) {
      std::vector<int>::const_iterator result = std::find(robotIDs_.begin(), robotIDs_.end(), target.robot_id);
      if (result == robotIDs_.end()) {
	// Not in the vector - skip this one.
	return;
      }
    }
    // Do we have the robot_id?
    if (targets_.count(target.robot_id) == 0) {
      // targets_[target.robot_id] = SingleRobotTargetHandler(target.robot_id);
      targets_.insert(std::pair<int, SingleRobotTargetHandler>(target.robot_id, SingleRobotTargetHandler(target.robot_id)));
    }
    targets_.find(target.robot_id)->second.assignTarget(target);
  }

  std::vector<long> getAllIDsToProcess() const {
    // Sort them based on some criteria?
    std::vector<long> ret;
    for (std::map<int, SingleRobotTargetHandler>::const_iterator it=targets_.begin(); it!=targets_.end(); ++it) {
      int robot_id = it->first;
      int goal_id = it->second.getNextGoalToProcess();
      if (robot_id >= 0 && goal_id >= 0) {
	ret.push_back(getUniqueID_(robot_id, goal_id));
      }
    }
    return ret;
  }

  long getNextIDToProcess() const {
    int min_time_robot_id = -1;
    int min_time_goal_id = -1;
    double min_time = std::numeric_limits<double>::max();
    for (std::map<int, SingleRobotTargetHandler>::const_iterator it=targets_.begin(); it!=targets_.end(); ++it) {
      int robot_id = it->first;
      int goal_id = it->second.getNextGoalToProcess();
      
      if (robot_id >= 0 && goal_id >= 0) {
	double st = it->second.getStartDeadline(goal_id);
	if (st < min_time) {
	  min_time_robot_id = robot_id;
	  min_time_goal_id = goal_id;
	  min_time = st;
	}
      }
    }
    if (min_time_robot_id < 0 || min_time_goal_id < 0)
      return -1;
    return getUniqueID_(min_time_robot_id, min_time_goal_id);
  }

  bool validProcessedID(long ID) {
    int robot_id, goal_id;
    getIDsFromUnique(ID, robot_id, goal_id);
    // Check that the robot id exist
    if (targets_.count(robot_id) == 0) {
      ROS_WARN("Failed to find robot_id : %d", robot_id);
      return false;
    }
    return (targets_.find(robot_id)->second.validProcessedGoal(goal_id));
  }

  long getLastProcessedID() const {
    return lastProcessedID_;
  }

  void setProcessedID(long ID) {
    lastProcessedID_ = ID;
    int robot_id, goal_id;
    getIDsFromUnique(ID, robot_id, goal_id);
    // Check that the robot id exist
    if (targets_.count(robot_id) == 0) {
      ROS_WARN("Failed to find robot_id : %d", robot_id);
      return;
    }
    // OK fine.
    targets_.find(robot_id)->second.setProcessedGoal(goal_id);
  }

  void clearProcessedID(long ID) {
    int robot_id, goal_id;
    getIDsFromUnique(ID, robot_id, goal_id);
    // Check that the robot id exist
    if (targets_.count(robot_id) == 0) {
      ROS_WARN("Failed to find robot_id : %d", robot_id);
      return;
    }
    // OK fine.
    targets_.find(robot_id)->second.clearProcessedGoal(goal_id);
  }

  bool isCompletedTarget(long ID) {
    int robot_id, goal_id;
    getIDsFromUnique(ID, robot_id, goal_id);
    // Check that the robot id exist
    if (targets_.count(robot_id) == 0) {
      ROS_WARN("Failed to find robot_id : %d", robot_id);
      return false;
    }
    // OK fine.
    return targets_.find(robot_id)->second.isCompletedGoal(goal_id);
  }

  void setCompletedTarget(long ID) {
    int robot_id, goal_id;
    getIDsFromUnique(ID, robot_id, goal_id);
    // Check that the robot id exist
    if (targets_.count(robot_id) == 0) {
      ROS_WARN("Failed to find robot_id : %d", robot_id);
      return;
    }
    // OK fine.
    targets_.find(robot_id)->second.setCompletedGoal(goal_id);
  }

  void printDebug() const {
    for (std::map<int, SingleRobotTargetHandler>::const_iterator it=targets_.begin(); it!=targets_.end(); ++it) {
      std::cout << "++ robot_id : " << it->first << std::endl;
      std::cout << "   set size : " << targets_.size() << std::endl;
      it->second.printDebug();
    }
  }

  std::vector<long> getActiveProcessedIDs(const ros::Time &currentTime) const {
    std::vector<long> ret;
    for (std::map<int, SingleRobotTargetHandler>::const_iterator it=targets_.begin(); it!=targets_.end(); ++it) {
      std::vector<int> goals = it->second.getActiveProcessedGoals(currentTime);
      for (unsigned int i = 0; i < goals.size(); i++) {
	ret.push_back(getUniqueID_(it->first, goals[i]));
      }
    }
    return ret;
  }
  
  std::vector<long> getFutureProcessedIDs(const ros::Time &currentTime) const {
    std::vector<long> ret;
    for (std::map<int, SingleRobotTargetHandler>::const_iterator it=targets_.begin(); it!=targets_.end(); ++it) {
      std::vector<int> goals = it->second.getFutureProcessedGoals(currentTime);
      for (unsigned int i = 0; i < goals.size(); i++) {
	ret.push_back(getUniqueID_(it->first, goals[i]));
      }
    }
    return ret;
  }

  std::vector<long> getActiveAndFutureProcessedIDs(const ros::Time &currentTime) const {
    std::vector<long> ret = getActiveProcessedIDs(currentTime);
    std::vector<long> f = getFutureProcessedIDs(currentTime);
    ret.insert( ret.end(), f.begin(), f.end() );
    return ret;
  }
  
  std::vector<long> getCompletedIDs() const {
    std::vector<long> ret;
    for (std::map<int, SingleRobotTargetHandler>::const_iterator it=targets_.begin(); it!=targets_.end(); ++it) {
      std::vector<int> completed = it->second.getCompletedGoals();
      for (unsigned int i = 0; i < completed.size(); i++) {
	ret.push_back(getUniqueID_(it->first, completed[i]));
      }
    }
    return ret;
  }

  bool validRobotID(int robotID) {
    if (robotIDs_.empty()) 
      return true;
    std::vector<int>::const_iterator it = std::find(robotIDs_.begin(), robotIDs_.end(), robotID);
    if (it != robotIDs_.end()) {
      return true;
    }
    return false;
  }

  bool addedRobotID(int robotID) {
    if (addedRobotIDs_.empty()) {
      addedRobotIDs_.push_back(robotID);
      return false;
    }
    std::vector<int>::const_iterator it = std::find(addedRobotIDs_.begin(), addedRobotIDs_.end(), robotID);
    if (it != addedRobotIDs_.end()) {
      return true;
    }
    addedRobotIDs_.push_back(robotID);
    return false;
  }
  
  const std::vector<int>& getRobotIDs() const { return robotIDs_; }

  std::vector<int> getProcessedGoals(int robotID) {
    std::vector<int> ret;
    if (!validRobotID(robotID)) {
      return ret;
    }
    return targets_.find(robotID)->second.getProcessedGoals();
  }

 private:
  std::map<int, SingleRobotTargetHandler> targets_; // Here the index is the robot_id
  std::vector<int> robotIDs_;
  std::vector<int> addedRobotIDs_;
  long lastProcessedID_;
};

} // namespace
