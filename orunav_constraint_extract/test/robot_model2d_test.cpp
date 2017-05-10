#include <ros/ros.h>
#include <orunav_msgs/RobotConstraints.h>
#include <orunav_rviz/orunav_rviz.h>
#include <orunav_geometry/robot_model_2d.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_model2d_test");
  int model_type;
  int load_type;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("load_type", po::value<int>(&load_type)->default_value(0), "if the model should carry load - will change the boundaries for some vehicles 0: no load, 2: load type 1 - EUR pallet, 3: load type 2 - half pallet")
    ("model_type", po::value<int>(&model_type)->default_value(1), "model type 1: snowwhite, 2: cititruck")
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
  
  ros::Rate r(2);
  
  orunav_geometry::RobotModel2dInterface* model = 0x0;
  switch (model_type) {
  case 1:
    model = new orunav_geometry::RobotModel2dSnowWhite();
    break;
  case 2:
    model = new orunav_geometry::RobotModel2dCiTiTruck();
    break;
  default:
    model = new orunav_geometry::RobotModel2dSnowWhite();
    break;
  };

  orunav_geometry::RobotModel2dWithState robot(*model);
  orunav_generic::RobotInternalState2d s;

  //  orunav_generic::RobotInternalState2d::LoadType load;
  switch (load_type) {
  case 1:
    s.loadType = orunav_generic::RobotInternalState2d::NO_LOAD;
    break;
  case 2:
    s.loadType = orunav_generic::RobotInternalState2d::EUR_PALLET;
    break;
  case 3:
    s.loadType = orunav_generic::RobotInternalState2d::HALF_PALLET;
    break;
  };

  orunav_generic::Pose2d pose(0,0,0);
  orunav_generic::Pose2d inc_pose(0.3, 0.0, 0.03);
  
  orunav_geometry::Polygon sweep_area;

  while (ros::ok()) {

    orunav_rviz::drawPoint2dContainerAsConnectedLine(model->getBoundingRegion(s), "model", 0, 0, pub);
    robot.update(pose, s);
    orunav_rviz::drawPoint2dContainerAsConnectedLine(robot.getPosePolygon(), "pose_model", 0, 0, pub);
    sweep_area.addPolygon(robot.getPosePolygon());
    orunav_rviz::drawPoint2dContainerAsConnectedLine(sweep_area, "sweep", 2, 2, pub);

    // Inside check
    std::vector<Eigen::Vector3d> inside;
    std::vector<Eigen::Vector3d> outside;
    for (int x = 0; x < 21; x++) {
      for (int y = 0; y < 21; y++) {
	Eigen::Vector2d p(x,y);
	if (robot.collisionPoint2d(p))
	  inside.push_back(Eigen::Vector3d(x,y,0));
	else
	  outside.push_back(Eigen::Vector3d(x,y,0));
      }
    }
    orunav_rviz::drawPointsVec(inside, "inside", 0, pub); // Red
    orunav_rviz::drawPointsVec(outside, "outside", 1, pub); // Green
    pose = orunav_generic::addPose2d(pose, inc_pose);
    
    r.sleep();
  }
}
