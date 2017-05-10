#include <ros/ros.h>
#include <orunav_generic/path_utils.h>
#include <orunav_rviz/orunav_rviz.h>
#include <orunav_generic/interfaces.h>
#include <orunav_generic/io.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "view_path");

  string path_file_name;
  int debug_nb;
  double steering_wheel_dist_offset;

     po::options_description desc("Allowed options");
     desc.add_options()
       ("help", "produce help message")
       ("debug_nb", po::value<int>(&debug_nb)->default_value(0), "perform debugging")
       ("fileName", po::value<string>(&path_file_name)->required(), "path file to be used")
       ("steering_wheel_dist_offset", po::value<double>(&steering_wheel_dist_offset)->default_value(1.19), "default stering wheel pose offset (in x)")
       ;
     
     po::variables_map vm;
     po::store(po::parse_command_line(argc, argv, desc), vm);
     
     if (vm.count("help")) {
	  cout << desc << "\n";
	  return 1;
     }

     
     po::notify(vm);    
     
     ros::NodeHandle nh;
     ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

     ros::Rate r(1);


     orunav_generic::Pose2d p1, p2;
     p1 << 1, 0, 0;
     p2 << 2, 0, 0;

     std::cout << "direction : " << orunav_generic::getDirection(p1, p2) << std::endl;
     std::cout << "direction : " << orunav_generic::getDirection(p2, p1) << std::endl;


     cout << "loading : " << path_file_name << endl;
     orunav_generic::Path path = orunav_generic::loadPathTextFile(path_file_name);
     cout << "number of path steps : " << path.sizePath() << endl;
     orunav_generic::Path path_min_dist = orunav_generic::minIncrementalDistancePath(path, 0.001);
     cout << "number of path_min_dist steps : " << path_min_dist.sizePath() << endl;
     
     std::vector<size_t> req_idx = calculateRequiredPathPointsIdx(path_min_dist);
     for (size_t i = 0; i < req_idx.size(); i++) {
       std::cout << "req_idx[" << i << "] : " << req_idx[i] << std::endl;
     }
     orunav_generic::Path required_path_points = orunav_generic::calculateRequiredPathPoints(path_min_dist);
     
     int iter = 0;

     while (iter < 20 && ros::ok())
       {
	 orunav_rviz::drawPathInterface(path, "path", 0, steering_wheel_dist_offset, pub);
	 orunav_rviz::drawPathInterface(required_path_points, "required_path_points", 1, steering_wheel_dist_offset, pub);
         orunav_rviz::drawPathInterfaceIncZ(path, "path_incz", 2, 2, 0.05, pub);

	 r.sleep();
	 iter++;
       }

     return 1;
}
