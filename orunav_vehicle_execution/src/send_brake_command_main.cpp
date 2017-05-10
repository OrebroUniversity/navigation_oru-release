#include <ros/ros.h>
#include <orunav_msgs/ControllerCommand.h>
#include <boost/program_options.hpp>
#include <orunav_generic/interfaces.h>
#include <orunav_generic/utils.h>
#include <orunav_conversions/conversions.h>

namespace po = boost::program_options;

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_model2d_test");

  int robot_id;
  double brake_cycle_time;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("robot_id", po::value<int>(&robot_id)->required(), "the robot to break")
    ("recover", "send recover command instead of break command")
    ("activate", "send activate command instead of break command")
    ("brake_cycle_time", po::value<double>(&brake_cycle_time)->default_value(-1.), "if a complete brake cycle should be sent - BRAKE, delay brake_cycle_time, RECOVER")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  
  if (vm.count("help")) {
    cout << desc << "\n";
    return 1;
  }

  bool recover = (vm.count("recover"));
  bool activate = (vm.count("activate"));
  po::notify(vm);    

  std::string command_topic = orunav_generic::getRobotTopicName(robot_id, std::string("/controller/commands"));

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<orunav_msgs::ControllerCommand>(command_topic, 10);
  
  ros::Rate r(2);

  orunav_msgs::ControllerCommand cmd;
  cmd.robot_id = robot_id;
  cmd.command = cmd.COMMAND_BRAKE;
  if (recover) {
    cmd.command = cmd.COMMAND_RECOVER;
  }
  if (activate) {
    cmd.command = cmd.COMMAND_ACTIVATE;
  }
  
  r.sleep();

  if (brake_cycle_time > 0.) {
    pub.publish(cmd);
    cout << "Sending BRAKE" << std::endl;
    cout << "waiting " << brake_cycle_time << " secs." << std::endl;
    ros::Duration(brake_cycle_time).sleep();
    cmd.command = cmd.COMMAND_RECOVER;
    pub.publish(cmd);
    cout << "Sending RECOVER" << std::endl;
  }
  else {
    while (ros::ok()) {
      pub.publish(cmd);
      r.sleep();
    }
  }
}
