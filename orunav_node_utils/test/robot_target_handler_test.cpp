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
  orunav_node_utils::RobotTargetHandler handler;

  std::vector<int> robot_ids_to_use;
  robot_ids_to_use.push_back(1);
  robot_ids_to_use.push_back(3);
  handler.setRobotIDsToCompute(robot_ids_to_use);
  
  orunav_msgs::RobotTarget target;
  const int nb_robots = 4;
  const int nb_targets = 4;

  for (int i = 0; i < nb_robots; i++) {
    std::cout << "RID: " << i << " valid : " << handler.validRobotID(i) << std::endl;
  }
  


  target.start_earliest = ros::Time::now()+ros::Duration(5);
  target.start_deadline = ros::Time::now()+ros::Duration(10);
  target.goal_earliest = ros::Time::now()+ros::Duration(10);
  target.goal_deadline = ros::Time::now()+ros::Duration(15);
  
  for (int i = 0; i < nb_targets; i++) {
    for (int j = 0; j < nb_robots; j++) {
      target.goal_id = i;
      target.robot_id = j;
      
      target.start_earliest += ros::Duration(3);
      target.start_deadline += ros::Duration(3);
      target.goal_earliest += ros::Duration(3);
      target.goal_deadline += ros::Duration(3);
      handler.assignTarget(target);
      std::cout << "Assigned RID: " << target.robot_id << " GID: " << target.goal_id << std::endl;
    }
  }

  {
    // UniqueID test
    int robot_id = 0;
    int goal_id = 0;
    std::cout << "std::numeric_limits<int>::max() : " << std::numeric_limits<int>::max() << std::endl;
    for (int i = 0; i < 10; i++) {
      long id = orunav_node_utils::getUniqueID_(robot_id, goal_id);
      std::cout << "--Fwd : " << id << " <- [RID:" << robot_id << " GID:" << goal_id << "]" << std::endl;
      orunav_node_utils::getIDsFromUnique(id, robot_id, goal_id);
      std::cout << "--Rev : " << id << " -> [RID:" << robot_id << " GID:" << goal_id << "]" << std::endl;
      robot_id++; goal_id++;
    }

  }
  

  handler.printDebug();

  ros::Rate r(1);
  while (ros::ok()) {
    std::cout << "==========================================================" << std::endl;
    long id = handler.getNextIDToProcess();
    if (id >= 0) {
      int robot_id, goal_id;
      orunav_node_utils::getIDsFromUnique(id, robot_id, goal_id);
      std::cout << "--Next ID to process : " << id << " -> [RID:" << robot_id << " GID:" << goal_id << "]" << std::endl;
      handler.setProcessedID(id);
      
    }
    else {
      std::cout << "nothing to process..." << std::endl;
    }
    //    handler.printDebug();

    std::vector<long> active_ids = handler.getActiveProcessedIDs(ros::Time::now());
    std::vector<long> future_ids = handler.getFutureProcessedIDs(ros::Time::now());
    for (unsigned int i = 0; i < active_ids.size(); i++) {
      int robot_id, goal_id;
      orunav_node_utils::getIDsFromUnique(active_ids[i], robot_id, goal_id);
      std::cout << " - ACTIVE : " << active_ids[i] << " -> [RID:" << robot_id << " GID:" << goal_id << "]" << std::endl;
    }

    for (unsigned int i = 0; i < future_ids.size(); i++) {
      int robot_id, goal_id;
      orunav_node_utils::getIDsFromUnique(future_ids[i], robot_id, goal_id);
      std::cout << " - FUTURE : " << future_ids[i] << " -> [RID:" << robot_id << " GID:" << goal_id << "]" << std::endl;
    }

    r.sleep();
  }

}

