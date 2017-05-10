#include <ros/ros.h>
#include <orunav_rviz/orunav_rviz.h>
#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>
#include <orunav_generic/interfaces.h>
#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>
#include <orunav_path_pool/wef_utils.h>


namespace po = boost::program_options;
using namespace std;



int main(int argc, char **argv)
{
  ros::init(argc, argv, "wef2path"); // Only to use for visualization...

  po::options_description desc("Allowed options");
  std::string wef_file_name;
  double resolution, length;
  desc.add_options()
    ("help", "produce help message")
    ("wef_file_name", po::value<std::string>(&wef_file_name)->default_value(std::string("")), "input .wef file (obtained from the Kollmorgen's CWay tool)")
    ("resolution", po::value<double>(&resolution)->default_value(0.01), "resolution of the returned spline 0.01-> gives 100 points.")
    ("length", po::value<double>(&length)->default_value(0.68), "wheel base length, needed to get the steering angle values.")
    ("save_goals", "save goal files (add .goal) to <filename>.wef")
    ("visualize", "send the path to rviz.")
    ("save_paths", "save paths (to wef<x>.path)")
    ("quaternion", "save the files using 2 values quaterions - (to be more compatible with the target_recorder / orunav_task_allocator)")
    ;
  
     po::variables_map vm;
     po::store(po::parse_command_line(argc, argv, desc), vm);
     
     if (vm.count("help")) {
	  cout << desc << "\n";
	  
	  return 1;
     }
     bool visualize = vm.count("visualize");
     bool save_goals = vm.count("save_goals");
     bool quaternion = vm.count("quaternion");
     bool save_paths = vm.count("save_paths");
     po::notify(vm);    


     WefPathParser weffer(wef_file_name);
     std::vector<orunav_generic::Path> paths = weffer.getPaths(length, resolution);
     const std::vector<ControlPoints>& all_ctrl_points = weffer.getAllControlPoints();
     const std::vector<orunav_generic::Pose2dVec>& all_start_goal_poses = weffer.getStartAndGoalPoses();


     orunav_generic::Path goal_states = orunav_generic::getGoalStatesFromPaths(paths);
     orunav_generic::Path start_states = orunav_generic::getStartStatesFromPaths(paths);
     if (save_goals) {
       orunav_generic::savePathTextFile(goal_states, wef_file_name + std::string(".goals"));
       orunav_generic::savePathTextFile(start_states, wef_file_name + std::string(".starts"));
     }
     if (quaternion) {
       orunav_generic::saveQuaternionPathTextFile(goal_states, wef_file_name + std::string(".goals.txt"));
       orunav_generic::saveQuaternionPathTextFile(start_states, wef_file_name + std::string(".starts.txt"));
     }
     if (save_paths) {
         for (int i = 0; i < paths.size(); i++) {
             orunav_generic::savePathTextFile(paths[i], std::string("wef") + orunav_generic::toString(i) + std::string(".path")); 
         }
     }
     cout << "Number of paths loaded : " << paths.size() << std::endl;
     // Load the wef file and extract a vector of paths.
     // Extract the paths.
     // Save each path as a separate file.
     if (visualize) {
	ros::NodeHandle nh;
        ros::Publisher markers_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
	ros::Rate r(1.);

	while(ros::ok()) {
	  for (unsigned int i = 0; i < paths.size(); i++) {
	    std::cout << " Path : " << i << " has length : " << paths[i].sizePath() << ", nb control points : " << all_ctrl_points[i].size() << std::endl;
	    orunav_rviz::drawPathInterface(paths[i], std::string("wefpath") + orunav_generic::toString(i), 1, length, markers_pub);
	    orunav_rviz::drawPointsVec( all_ctrl_points[i].getControlPointsVec(), std::string("knots_pts") + orunav_generic::toString(i), 2, markers_pub);
	    r.sleep();
	    orunav_rviz::drawPose2dContainer(goal_states, std::string("goals"), 2, markers_pub);
	    orunav_rviz::drawPose2dContainer(start_states, std::string("starts"), 0, markers_pub);
	    orunav_rviz::drawPose2dContainer(all_start_goal_poses[i], std::string("wef_start_goal") + orunav_generic::toString(i), 3, markers_pub);
	  }
	}
     }
     cout << "Done." << endl;
}
