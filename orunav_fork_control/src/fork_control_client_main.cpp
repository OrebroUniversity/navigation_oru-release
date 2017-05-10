#include <ros/ros.h>
#include <orunav_msgs/ForkCommand.h>
#include <boost/program_options.hpp>
#include <orunav_generic/utils.h>

namespace po = boost::program_options;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fork_control_client");

  po::options_description desc("Allowed options");
  
  int robot_id;
  int command;
  desc.add_options()
      ("help", "produce help message")
      ("robot_id", po::value<int>(&robot_id)->default_value(1), "robot ID (if < 0 the fork command topic will not have any /robotX added")
    ("command", po::value<int>(&command)->default_value(0), "0: move the forks up, 1: move the forks down, 2: activate support legs - move forks all the way down");
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  
  if (vm.count("help")) {
      cout << desc << "\n";
      
      return 1;
  }
  
  po::notify(vm);    
  
  ros::NodeHandle nh;
  ros::Publisher forkcmd_pub;
  std::string fork_command_topic; 
  if (robot_id >=0)
      fork_command_topic = orunav_generic::getRobotTopicName(robot_id, "/fork/command");
  else
      fork_command_topic = std::string("/fork/command");
  
  
  forkcmd_pub = nh.advertise<orunav_msgs::ForkCommand>(fork_command_topic, 1);
  ROS_INFO("Sending ForkCommand to : %s", fork_command_topic.c_str());

  ros::Rate r(1);
  
  orunav_msgs::ForkCommand cmd;
  cmd.robot_id = robot_id;
  switch (command) {
    case 0:
      ROS_INFO("moving forks UP");
      cmd.state.position_z = 0.1;
      break;
    case 1:
      cmd.state.position_z = 0.0;
      ROS_INFO("moving forks DOWN");
      break;
    case 2:
      cmd.state.position_z = -0.1;
      ROS_INFO("activating support legs - moving all the way DOWN");
      break;
    default:
      ROS_ERROR("Not a valid command param...");
      exit(-1);
      break;
  }

  while (ros::ok()) {
  ROS_INFO("Sending fork command : %d, position z : %f", command, cmd.state.position_z);
      forkcmd_pub.publish(cmd);
      ros::spinOnce();
      r.sleep();
  }
  return 0;
}

