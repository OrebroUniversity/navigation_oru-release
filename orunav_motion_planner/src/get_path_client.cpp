#include <ros/ros.h>
#include <boost/program_options.hpp>
#include <orunav_generic/interfaces.h>
#include <orunav_generic/utils.h>
#include <orunav_conversions/conversions.h>
#include <orunav_msgs/GetPath.h>
#include <orunav_msgs/RobotTarget.h>

namespace po = boost::program_options;

using namespace std;

nav_msgs::OccupancyGrid current_map;
bool got_map = false;

void process_map(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  current_map = *msg;
  got_map = true;
  std::cout << "... got a map!" << std::endl;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "get_path_client");

  int robot_id;
  double brake_cycle_time;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  
  if (vm.count("help")) {
    cout << desc << "\n";
    return 1;
  }

  po::notify(vm);    



  ros::NodeHandle nh;
  ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,&process_map);

  ros::Rate r(2);
  r.sleep();
  while (ros::ok() && got_map == false) {
    ros::spinOnce();
    r.sleep();
    ROS_INFO("Waiting for a map... (on topic /map)");
  }
  
  //  orunav_generic::State2d start(5.197849,12.020535,0.019037, 0.000000);
  //  orunav_generic::State2d goal(5.192985,12.122826,0.102908, 0.000000);
  orunav_generic::State2d start(7.035001,13.537113,1.504957, -0.000063);
  orunav_generic::State2d goal(5.208815,11.934435,0.085662, 0.000000);

  orunav_msgs::RobotTarget target;
  target.robot_id = 1;
  target.goal_id = 1;
  target.start = orunav_conversions::createPoseSteeringMsgFromState2d(start);
  target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(goal);


  orunav_generic::Path path;
  { // Get path service call related stuff goes here...
    orunav_msgs::GetPath srv;
    srv.request.map = current_map;
    srv.request.target = target;
    // Update the target goal pose and map based on the load operations
    
    srv.request.max_planning_time = 20.; // TODO param
    // Need to package the target + the map and ask the motion planner.
    ros::ServiceClient client = nh.serviceClient<orunav_msgs::GetPath>("get_path");
    
    if (client.call(srv)) {
      ROS_INFO("[get_path_client] - get_path sucessfull");
    }
    else
    {
      ROS_ERROR("[get_path_client] - Failed to call service: GetPath");
      return false;
    }
    
    if (!srv.response.valid) {
      ROS_WARN("[get_path_client] - no path found(!), cannot computeTask");
      return false;
    }
    path = orunav_conversions::createPathFromPathMsgUsingTargetsAsFirstLast(srv.response.path);
  }

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
    ROS_INFO_STREAM(".");
  }
}


