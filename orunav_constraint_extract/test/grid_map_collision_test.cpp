#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <orunav_msgs/RobotConstraints.h>
#include <orunav_rviz/orunav_rviz.h>

#include <orunav_generic/interfaces.h>
#include <orunav_generic/io.h>
#include <orunav_generic/subsample_path.h>
#include <orunav_geometry/robot_model_2d.h>
#include <orunav_constraint_extract/grid_map.h>
#include <orunav_constraint_extract/polygon_constraint.h>

#include <boost/program_options.hpp>

#include <iostream>
#include <fstream>

namespace po = boost::program_options;
using namespace std;

nav_msgs::OccupancyGrid occ_map;
bool valid_map = false;
orunav_generic::Pose2d pose;
bool valid_pose = false;


void process_map(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    occ_map = *msg;
    valid_map = true;
    std::cout << "... got a map!" << std::endl;
}

void process_goal(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    pose = orunav_conversions::createPose2dFromMsg(msg->pose);
    valid_pose = true;
    std::cout << "... got a goal!" << std::endl;
}

int main(int argc, char **argv)
{
    
  ros::init(argc, argv, "grid_map_collision_test");
  int model_type;
  int load_type;
  int debug_nb;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("load_type", po::value<int>(&load_type)->default_value(0), "if the model should carry load - will change the boundaries for some vehicles 1 : no load, 2 : EUR pallet, 3: half EUR pallet")
    ("model_type", po::value<int>(&model_type)->default_value(1), "model type 1: snowwhite, 2: cititruck")
    ("debug_nb", po::value<int>(&debug_nb)->default_value(2), "debug number value - check code")
    ("no_motion", "if the vehicle should not move")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  
  if (vm.count("help")) {
    cout << desc << "\n";
    return 1;
  }

  bool motion = !vm.count("no_motion");
  
  po::notify(vm);    


  //  valid_pose = true;
  //  pose = orunav_generic::Pose2d(5., 5., 0.);

  ros::NodeHandle nh;
  ros::Subscriber sub_map = nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,process_map);
  ros::Subscriber sub_goal = nh.subscribe<geometry_msgs::PoseStamped>("/goal",10,process_goal);
  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Publisher pub_map = nh.advertise<nav_msgs::OccupancyGrid>("/map_output", 10);
  
  ros::Rate r(10);

  std::cout << "debug_nb : " << debug_nb << std::endl;
  
  orunav_geometry::RobotModel2dInterface* model = 0x0;
  switch (model_type) {

  case 1:
    model = new orunav_geometry::RobotModel2dSnowWhite();
    break;
  case 2:
    model = new orunav_geometry::RobotModel2dCiTiTruck();
    break;
  case 3:
    model = new orunav_geometry::RobotModel2dOneSquareMeter();
    break;
  default:
    model = new orunav_geometry::RobotModel2dSnowWhite();
    break;
  };

  orunav_geometry::RobotModel2dWithState robot(*model);
  orunav_generic::RobotInternalState2d s;

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
  
  // Wait here until we have recived a map...
  bool print_once = false;
  while (ros::ok() && valid_map == false) {
    if (!print_once)
      std::cout << "Waiting for a map (/map topic)" << std::endl;
    print_once = true;
      ros::spinOnce();
      r.sleep();
  }

  // We need a pose aswell...
  print_once = false;
  while (ros::ok() && valid_pose == false) {
    if (!print_once)
      std::cout << "Waiting for a pose (/goal topic)..." << std::endl;
    print_once = true;
    ros::spinOnce();
    r.sleep();
  }

  unsigned int theta_idx = 0;

  while (ros::ok()) {

      orunav_geometry::RobotModel2dWithState robot(*model);
      robot.update(pose, s);

      int color = 0;
      if (!constraint_extract::isOccupiedRobotModel2dWithState(occ_map, robot)) {
          color = 1;
      }
      orunav_rviz::drawPoint2dContainerAsConnectedLine(robot.getPosePolygon(), "pose_model", 0, color, pub);

      
      if (debug_nb == 1)
      {
          nav_msgs::OccupancyGrid occ_map_out = occ_map;
          constraint_extract::GridPatchIdx tmp = constraint_extract::getRadiusGridPatchIdx(occ_map_out, 0.5);
          Eigen::Vector2i offset;
          constraint_extract::metricToPixelOccupancyGrid(occ_map, orunav_generic::getPosition(pose), offset);
          constraint_extract::addOffsetToGridPatchIdx(occ_map, tmp, offset);
          constraint_extract::drawGridPatchIdxOnOccupancyMap(tmp, occ_map_out);
          pub_map.publish(occ_map_out);
      }

      if (debug_nb == 2)
      {
          nav_msgs::OccupancyGrid occ_map_out = occ_map;
          Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> allowedStates;
          Eigen::Matrix<constraint_extract::StateInterval,Eigen::Dynamic,Eigen::Dynamic> stateIntervals;
          constraint_extract::precomputeAllowedPositions(occ_map,
                                     *model,
                                     s,
                                     allowedStates, 
                                     stateIntervals);
          constraint_extract::drawAllowedStatesOnOccupancyMap(allowedStates, occ_map_out);
          pub_map.publish(occ_map_out);
      }

      if (debug_nb == 3)
      {
          nav_msgs::OccupancyGrid occ_map_out = occ_map;
          Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> allowedStates;
          Eigen::Matrix<constraint_extract::StateInterval,Eigen::Dynamic,Eigen::Dynamic> stateIntervals;
          constraint_extract::precomputeAllowedPositions(occ_map,
                                     *model,
                                     s,
                                     allowedStates, 
                                     stateIntervals);
          
          unsigned int t = (theta_idx++ % N_THETA_INCREMENTS);
          constraint_extract::drawStateIntervalsOnOccupancyMap(stateIntervals, allowedStates, t, 0, occ_map_out);
          std::cout << "t : " << t << std::endl;
          pub_map.publish(occ_map_out);

          motion = false;
          pose(2) = t * 2*M_PI /(double) N_THETA_INCREMENTS;


      }

      if (debug_nb == 4)
      {
          nav_msgs::OccupancyGrid occ_map_out = occ_map;
          constraint_extract::GridPatchIdx tmp = constraint_extract::getRobotModel2dWithStateGridPatchIdx(occ_map, robot);

          Eigen::Vector2i offset;
          constraint_extract::drawGridPatchIdxOnOccupancyMap(tmp, occ_map_out);
          pub_map.publish(occ_map_out);
      }

      ros::spinOnce();
      r.sleep();
      
      if (motion) {
          orunav_generic::Pose2d inc_pose(0.03, 0.0, 0.003);
          pose = orunav_generic::addPose2d(pose, inc_pose);
      }

  }
  
  
  delete model;
}
