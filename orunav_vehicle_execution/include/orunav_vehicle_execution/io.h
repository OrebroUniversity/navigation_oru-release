#pragma once

#include <orunav_conversions/conversions.h>
#include <fstream>

std::vector<orunav_msgs::RobotTarget> loadRobotTargets(const std::string &fileName) {
  
  std::vector<orunav_msgs::RobotTarget> vec;

  // One target per line.
  std::ifstream ifs;
  ifs.open(fileName.c_str());
  if (!ifs.is_open()) {
    std::cerr << __FILE__ << ":" << __LINE__ << " cannot open file : " << fileName << std::endl;
    return vec;
  }
  

  while (!ifs.eof())
  {
    orunav_msgs::RobotTarget t;
    std::string line;
    getline(ifs, line);
    
    orunav_generic::State2d start, goal;
    
    if (sscanf(line.c_str(), "%d %d (%lf %lf %lf)[%lf] (%lf %lf %lf)[%lf] CL:%d GL:%d SO:%d GO:%d",
        &t.task_id, &t.robot_id, &start.pose(0), &start.pose(1), &start.pose(2), &start.steeringAngle,
               &goal.pose(0), &goal.pose(1), &goal.pose(2), &goal.steeringAngle, &t.current_load.status, &t.goal_load.status, &t.start_op.operation, &t.goal_op.operation) == 14)
    {
      t.start = orunav_conversions::createPoseSteeringMsgFromState2d(start);
      t.goal = orunav_conversions::createPoseSteeringMsgFromState2d(goal);
      vec.push_back(t);
    }
  }
  return vec;
  
}

std::vector<orunav_msgs::RobotTarget> filterTargetsOnRobotId(const std::vector<orunav_msgs::RobotTarget> &targets, int robot_id) {
  std::vector<orunav_msgs::RobotTarget> ret;
  for (size_t i = 0; i < targets.size(); i++) {
    if (targets[i].robot_id == robot_id) {
      ret.push_back(targets[i]);
    }
  } 
  return ret;
}

std::vector<orunav_msgs::RobotTarget> loadRobotTargets(const std::string &fileName, int robot_id) {
  std::vector<orunav_msgs::RobotTarget> ret = loadRobotTargets(fileName);
  ret = filterTargetsOnRobotId(ret, robot_id);
  return ret;
}

//! Only saves a subsets of targets...
void saveRobotTargets(const std::vector<orunav_msgs::RobotTarget> &targets, const std::string &fileName) {
 
   std::ofstream ofs(fileName.c_str());
   for (size_t i = 0; i < targets.size(); i++) {
     
     const orunav_msgs::RobotTarget& t = targets[i];
     orunav_generic::State2d start = orunav_conversions::createState2dFromPoseSteeringMsg(t.start);
     orunav_generic::State2d goal = orunav_conversions::createState2dFromPoseSteeringMsg(t.goal);

     
     ofs << t.task_id << " " << t.robot_id << " " << start << " " << goal << " " << "CL:" << t.current_load.status << " GL:" << t.goal_load.status << " SO:" << t.start_op.operation << " GO:" << t.goal_op.operation << std::endl;
   }
}

void saveRobotTarget(const orunav_msgs::RobotTarget &target, const std::string &fileName) {
  std::vector<orunav_msgs::RobotTarget> targets;
  targets.push_back(target);
  saveRobotTargets(targets, fileName);
}

