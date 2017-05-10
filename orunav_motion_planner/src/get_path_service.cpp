#include <ros/ros.h>

#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>

#include <orunav_msgs/GetPath.h>

#include <orunav_motion_planner/PathFinder.h>
#include <orunav_motion_planner/VehicleMission.h>

#include <iostream>
#include <string>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <visualization_msgs/Marker.h>
#include <orunav_rviz/orunav_rviz.h>

#include <orunav_motion_planner/nav_msgs_occupancy_grid_to_planner_map.h>

/**
 * Responds to GetPath requests
 */
class GetPathService {

private:
  std::string motion_prim_dir_;
  std::string lookup_tables_dir_;
  std::string maps_dir_;
  CarModel* car_model_;
  
  bool visualize_;
  
  ros::Publisher marker_pub_;
  
  double min_incr_path_dist_;
  bool save_paths_;

  ros::NodeHandle nh_;
  ros::ServiceServer service_;

public:
  GetPathService(ros::NodeHandle param_nh, std::string name) 
  {
      // read parameters
      param_nh.param<std::string>("motion_primitives_directory", motion_prim_dir_, "./Primitives/");
      param_nh.param<std::string>("lookup_tables_directory", lookup_tables_dir_, "./LookupTables/");
      param_nh.param<std::string>("maps_directory", maps_dir_, "./");
      std::string model;
      param_nh.param<std::string>("model", model, "");
      param_nh.param<double>("min_incr_path_dist", min_incr_path_dist_, 0.001);
      param_nh.param<bool>("save_paths", save_paths_, false);

      WP::setPrimitivesDir(motion_prim_dir_);
      WP::setTablesDir(lookup_tables_dir_);
      WP::setMapsDir(maps_dir_);
      car_model_ = new CarModel(model);

      ROS_INFO_STREAM("[GetPathService] - Using model : " << model << "\n");

      

      param_nh.param<bool>("visualize",visualize_,false);
      if (visualize_)
	{
	  ROS_INFO("[GetPathService] -  The output is visualized using /visualization_markers (in rviz).");
	  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
	}

      service_ = nh_.advertiseService("get_path", &GetPathService::getPathCB, this);


  }


  ~GetPathService()
    {
      delete car_model_;
      ROS_INFO_STREAM("[GetPathService] - shutting down\n");
    }

  
  bool getPathCB(orunav_msgs::GetPath::Request &req,
		 orunav_msgs::GetPath::Response &res)
  {
    orunav_msgs::RobotTarget tgt = req.target;
    ROS_INFO("[GetPathService] - start : [%f,%f,%f](%f)", tgt.start.pose.position.x, tgt.start.pose.position.y,tf::getYaw(tgt.start.pose.orientation), tgt.start.steering);
    ROS_INFO("[GetPathService] - goal :  [%f,%f,%f](%f)", tgt.goal.pose.position.x, tgt.goal.pose.position.y,tf::getYaw(tgt.goal.pose.orientation), tgt.goal.steering);
    
    WorldOccupancyMap planner_map;
    convertNavMsgsOccupancyGridToWorldOccupancyMapRef(req.map, planner_map);

    orunav_generic::Pose2d map_pose_offset = getNavMsgsOccupancyGridOffsetRef(req.map);
    double map_offset_x = map_pose_offset(0);
    double map_offset_y = map_pose_offset(1);
    double start_orientation = tf::getYaw(tgt.start.pose.orientation);
    double goal_orientation = tf::getYaw(tgt.goal.pose.orientation);
 
    assert(fabs(map_pose_offset(2) < 0.001)); // The orientation should be the same... implement this whenever needed. 
    
    if (planner_map.getMap().empty()) {
      ROS_ERROR("[GetPathService] - error in the provided map / conversion");
      return false;
    }

    PathFinder* pf;
    if (planner_map.getMap().empty())
      pf = new PathFinder(20, 20);
    else
      pf = new PathFinder(planner_map);
    
    if (req.max_planning_time > 0.) 
      pf->setTimeBound(req.max_planning_time);
    
    VehicleMission vm(car_model_,
                      tgt.start.pose.position.x-map_offset_x, tgt.start.pose.position.y-map_offset_y, start_orientation, tgt.start.steering,
                      tgt.goal.pose.position.x-map_offset_x, tgt.goal.pose.position.y-map_offset_y, goal_orientation, tgt.goal.steering);
    

    pf->addMission(&vm);
    if (req.max_planning_time > 0)
      pf->setTimeBound(req.max_planning_time);
    
    ROS_INFO("[GetPathService] - Starting to solve the path planning problem ... ");
    ros::Time start_time = ros::Time::now();
    std::vector<std::vector<Configuration*> > solution = pf->solve(false);
    ros::Time stop_time = ros::Time::now();
    ROS_INFO("[GetPathService] - Starting to solve the path planning problem - done");
    ROS_INFO("[GetPathService] - PATHPLANNER_PROCESSING_TIME: %f", (stop_time-start_time).toSec());
    
    assert(!solution.empty());
    bool solution_found = (solution[0].size() != 0);
    ROS_INFO_STREAM("[GetPathService] - solution_found : " << solution_found);
    ROS_INFO_STREAM("[GetPathService] - solution[0].size() : " << solution[0].size());
    
    orunav_generic::Path path;
    
    for (std::vector<std::vector<Configuration*> >::iterator it = solution.begin(); it != solution.end(); it++)
    {
      for (std::vector<Configuration*>::iterator confit = (*it).begin(); confit != (*it).end(); confit++) {
        std::vector<vehicleSimplePoint> path_local = (*confit)->getTrajectory();
        
        for (std::vector<vehicleSimplePoint>::iterator it2 = path_local.begin(); it2 != path_local.end(); it2++) {
          double orientation = it2->orient;  
          
          orunav_generic::State2d state(orunav_generic::Pose2d(it2->x+map_offset_x,
                                                               it2->y+map_offset_y,
                                                               orientation), it2->steering);
          path.addState2dInterface(state);
          
        }
      }
    }

    ROS_INFO_STREAM("[GetPathService] - Nb of path points : " << path.sizePath());

    // Cleanup
    delete pf;
    for (std::vector<std::vector<Configuration*> >::iterator it = solution.begin(); it != solution.end(); it++) {
      std::vector<Configuration*> confs = (*it);
      for (std::vector<Configuration*>::iterator confit = confs.begin(); confit != confs.end(); confit++) {
        delete *confit;
      }
      confs.clear();
    }
    solution.clear();
    // Cleanup - end
    
    if (path.sizePath() == 0)
      solution_found = false;

    res.valid = solution_found;
    if (solution_found) {
      // First requirement (that the points are separated by a minimum distance).
      orunav_generic::Path path_min_dist = orunav_generic::minIncrementalDistancePath(path, min_incr_path_dist_);
      // Second requirment (path states are not allowed to change direction of motion without any intermediate points).
      orunav_generic::Path path_dir_change = orunav_generic::minIntermediateDirPathPoints(path_min_dist);

      res.path = orunav_conversions::createPathMsgFromPathInterface(path_dir_change);
      res.path.target_start = tgt.start;
      res.path.target_goal = tgt.goal;
     

      if (visualize_) {
        orunav_generic::Pose2d start_pose(tgt.start.pose.position.x,
                                          tgt.start.pose.position.y,
                                          tf::getYaw(tgt.start.pose.orientation));
        orunav_generic::Pose2d goal_pose(tgt.goal.pose.position.x,
                                         tgt.goal.pose.position.y,
                                         tf::getYaw(tgt.goal.pose.orientation));
        
        orunav_rviz::drawPose2d(start_pose, 0, 0, 1., "start_pose2d", marker_pub_);
        orunav_rviz::drawPose2d(goal_pose, 0, 2, 1., "goal_pose2d", marker_pub_);
        orunav_rviz::drawPose2dContainer(orunav_generic::minIncrementalDistancePath(path_dir_change, 0.2), "path_subsampled", 1, marker_pub_);
      }

      if (save_paths_) {
        orunav_generic::Path path = orunav_conversions::createPathFromPathMsg(res.path);
        std::stringstream st;
        st << "path_" << res.path.robot_id << "-" << res.path.goal_id << ".path";
        std::string fn = st.str();
        
        orunav_generic::savePathTextFile(path, fn);
      }
      return true;
    }
    return false;
    
  }
  
};



int main(int argc, char** argv) {

  ros::init(argc, argv, "get_path_service");
  ros::NodeHandle parameters("~");
  GetPathService gps(parameters, ros::this_node::getName());

  ros::spin();
}
