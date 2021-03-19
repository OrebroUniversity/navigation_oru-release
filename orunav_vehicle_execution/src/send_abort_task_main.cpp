#include <ros/ros.h>
#include <orunav_msgs/ExecuteTask.h>
#include <boost/program_options.hpp>
#include <orunav_generic/interfaces.h>
#include <orunav_generic/utils.h>
#include <orunav_conversions/conversions.h>
#include <sstream>

namespace po = boost::program_options;

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "send_abort_task");

  int robot_id=4;
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
  std::stringstream out;
  out << robot_id;
  std::string service_name = out.str();
  service_name  = "/robot" +  service_name  + "/execute_task";

  ros::ServiceClient client;


  
  if (ros::service::waitForService(service_name, 100)) {
    client = nh.serviceClient<orunav_msgs::ExecuteTask>(service_name);
  } else {
    ROS_ERROR_STREAM("[VehicleExecutionClientNode] - service [" << service_name << "] not available");
    return -1;
  }

  orunav_msgs::ExecuteTask srv;
  srv.request.task.abort = true;
  srv.request.task.target.robot_id = robot_id;

  if (client.call(srv)) {
    ROS_INFO_STREAM("[VehicleExecutionClientNode] - Call [" << service_name << "] sucessfull");
  }
  else
  {
    ROS_ERROR_STREAM("[VehicleExecutionClientNode] - Failed to call service: [" << service_name << "]");
    return -1;
  }
  ROS_INFO_STREAM("[VehicleExecutionClientNode] - set_task return value : " << srv.response.result);

  return 1;
}
