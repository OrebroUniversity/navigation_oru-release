#include <ros/ros.h>
#include <orunav_msgs/RobotTarget.h>
#include <orunav_node_utils/robot_target_handler.h>
#include <iostream>


using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_model2d_test");
  ros::NodeHandle nh;

  //  orunav_node_utils::SingleRobotTargetHandler<int> handler(1);
  orunav_node_utils::SingleRobotTargetHandler handler(1);

  orunav_msgs::RobotTarget target;
  target.robot_id = 1;
  target.goal_id = 0;
  target.start_earliest = ros::Time::now();
  target.start_deadline = ros::Time::now()+ros::Duration(10);
  target.goal_earliest = ros::Time::now()+ros::Duration(10);
  target.goal_deadline = ros::Time::now()+ros::Duration(20);
  handler.assignTarget(target);

  target.goal_id = 1;
  target.start_earliest += ros::Duration(30);
  target.start_deadline += ros::Duration(30);
  target.goal_earliest += ros::Duration(30);
  target.goal_deadline += ros::Duration(30);
  handler.assignTarget(target);

  target.goal_id = 2;
  target.start_earliest += ros::Duration(30);
  target.start_deadline += ros::Duration(30);
  target.goal_earliest += ros::Duration(30);
  target.goal_deadline += ros::Duration(30);
  handler.assignTarget(target);
  
  // Loop through the time and plot some output.

  vector<int> goals_to_process = handler.getGoalsToProcess();
  
  for (unsigned int i = 0; i < goals_to_process.size(); i++) {
    std::cout << "unprocessed goal[" << i << "] : " << goals_to_process[i] << std::endl;

    std::cout << "valid processed goal : " << handler.validProcessedGoal(goals_to_process[i]) << std::endl;
  }

  handler.setProcessedGoal(-1); // Should return a warning
  std::cout << "handler.getNextGoalToProcess() : " << handler.getNextGoalToProcess() << std::endl;
  handler.setProcessedGoal(0);
  std::cout << "handler.getNextGoalToProcess() : " << handler.getNextGoalToProcess() << std::endl;
  handler.setProcessedGoal(1);
  std::cout << "handler.getNextGoalToProcess() : " << handler.getNextGoalToProcess() << std::endl;
  handler.clearProcessedGoal(1);
  std::cout << "handler.getNextGoalToProcess() : " << handler.getNextGoalToProcess() << std::endl;
  handler.setProcessedGoal(1);
  std::cout << "handler.getNextGoalToProcess() : " << handler.getNextGoalToProcess() << std::endl;
  
  ros::Rate r(0.25);
  
  while (ros::ok()) {
    std::cout << "==========================================================" << std::endl;
    handler.printDebug();
    handler.cleanOldTargets(ros::Time::now());
    r.sleep();
  }

}

