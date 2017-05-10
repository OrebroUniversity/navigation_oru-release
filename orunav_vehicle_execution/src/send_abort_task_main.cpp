#include <ros/ros.h>
#include <orunav_msgs/ExecuteTask.h>
#include <boost/program_options.hpp>
#include <orunav_generic/interfaces.h>
#include <orunav_generic/utils.h>
#include <orunav_conversions/conversions.h>

namespace po = boost::program_options;

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "send_abort_task");

  int robot_id;
  double brake_cycle_time;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("robot_id", po::value<int>(&robot_id)->required(), "the robot to break")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  
  if (vm.count("help")) {
    cout << desc << "\n";
    return 1;
  }

  ros::NodeHandle nh;

  // Send the task to the coordinator
  ros::ServiceClient client = nh.serviceClient<orunav_msgs::ExecuteTask>("execute_task");
  orunav_msgs::ExecuteTask srv;
  srv.request.task.abort = true;

  if (client.call(srv)) {
    ROS_INFO("[VehicleExecutionClientNode] - execute_task sucessfull");
  }
  else
  {
    ROS_ERROR("[VehicleExecutionClientNode] - Failed to call service: execute_task");
    return -1;
  }
  ROS_INFO_STREAM("[VehicleExecutionClientNode] - set_task return value : " << srv.response.result);

  return 1;
}
