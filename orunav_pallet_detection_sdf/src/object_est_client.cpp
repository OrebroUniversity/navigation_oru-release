#include <ros/ros.h>
#include <boost/program_options.hpp>
#include <orunav_generic/interfaces.h>
#include <orunav_generic/utils.h>
#include <orunav_conversions/conversions.h>
#include <orunav_msgs/GetPath.h>
#include <orunav_msgs/RobotTarget.h>
#include <orunav_msgs/ObjectPoseEstimation.h>

namespace po = boost::program_options;

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "get_path_client");

  int robot_id;
  double px, py, pth;
  int object_type;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("px", po::value<double>(&px)->default_value(-1.), "init pallet pose [x] in /world")
    ("py", po::value<double>(&py)->default_value(0.), "init pallet pose [y] in /world")
    ("pth", po::value<double>(&pth)->default_value(0.), "init pallet pose [theta] in /world")
    ("robot_id", po::value<int>(&robot_id)->default_value(1), "robot id to be used to select the service /robot{robot_id}/pallet_estimation_service")
    ("object_type", po::value<int>(&object_type)->default_value(1), "object type, EUR pallet = 1, half pallet = 2")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  
  if (vm.count("help")) {
    cout << desc << "\n";
    return 1;
  }

  po::notify(vm);    

  ros::NodeHandle nh;
 
  orunav_generic::Pose2d pallet_pose(px, py, pth);
  
  orunav_msgs::ObjectPoseEstimation srv;
  srv.request.active = true;
  srv.request.pose = orunav_conversions::createMsgFromPose2d(pallet_pose);
  srv.request.object.type = object_type;
  ros::ServiceClient client = nh.serviceClient<orunav_msgs::ObjectPoseEstimation>(orunav_generic::getRobotTopicName(robot_id, "/pallet_estimation_service"));
  if (client.call(srv)) {
    ROS_INFO("[ObjectEstClient] - pallet_estimation_service (started) - successfull");
  }
  else
  {
    ROS_ERROR("[KMOVehicleExecutionNode] - Failed to call service: pallet_estimation_service");
  }
  
  // ros::Rate r(2);
  // while (ros::ok()) {
  //   ros::spinOnce();
  //   r.sleep();
  //   ROS_INFO_STREAM(".");
  // }
}
