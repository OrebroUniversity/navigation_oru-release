#include <ros/ros.h>

#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>

#include <orunav_msgs/GetPath.h>

#include <orunav_path_pool/path_pool.h>
#include <orunav_path_pool/path_search_astar.h>

#include <iostream>
#include <string>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <visualization_msgs/Marker.h>
#include <orunav_rviz/orunav_rviz.h>

/**
 * Responds to GetPath requests
 */
class GetPathService {

private:
  bool visualize_;
  
  ros::Publisher marker_pub_;
  
  bool use_astar_;

  PathPool pool_;
  double max_dist_offset, max_angular_offset;
  double min_incr_path_dist_;
  bool force_zero_steering_;
  bool save_paths_;

  ros::NodeHandle nh_;
  ros::ServiceServer service_;

public:
  GetPathService(ros::NodeHandle param_nh, std::string name) 
  {
      // read parameters
    std::string path_dir, wef_file_name;
    double vehicle_length, path_resolution;
    param_nh.param<std::string>("path_dir", path_dir, "");
    param_nh.param<std::string>("wef_file_name", wef_file_name, "");
    param_nh.param<double>("vehicle_length", vehicle_length, 0.68); // Only needed for wef.
    param_nh.param<double>("path_resolution", path_resolution, 0.01); // Only needed for wef.
    param_nh.param<double>("max_dist_offset", max_dist_offset, 0.1);
    param_nh.param<double>("max_angular_offset", max_angular_offset, 0.1);
    param_nh.param<bool>("use_astar", use_astar_, true);
    param_nh.param<double>("min_incr_path_dist", min_incr_path_dist_, 0.01);
    param_nh.param<bool>("save_paths", save_paths_, false);
    param_nh.param<bool>("force_zero_steering", force_zero_steering_, true);

    //Load the paths... favour wef files.
    if (path_dir != std::string("") && wef_file_name != std::string(""))
    {
      ROS_WARN("Both .path directory and .wef file name specified - will go with .wef");
    }
    if (wef_file_name != std::string(""))
    {
      ROS_INFO_STREAM("Loading wef file : " << wef_file_name);
      pool_.loadWefFile(wef_file_name, vehicle_length, path_resolution);
    }
    if (path_dir != std::string("") && wef_file_name == std::string(""))
    {
      ROS_INFO_STREAM("Loading paths directory : " << path_dir);
      pool_.loadPathDir(path_dir);
    }
    ROS_INFO_STREAM("# of paths loaded : " << pool_.getPaths().size());    
    if (pool_.getPaths().empty()) {
        ROS_ERROR("No path loaded... check the wef_file_name / path_dir(!)");
    }
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
      ROS_INFO_STREAM("[GetPathService] - shutting down\n");
    }

  
  bool getPathCB(orunav_msgs::GetPath::Request &req,
		 orunav_msgs::GetPath::Response &res)
  {
    orunav_msgs::RobotTarget tgt = req.target;
    ROS_INFO("[GetPathService] - start : [%f,%f,%f](%f)", tgt.start.pose.position.x, tgt.start.pose.position.y,tf::getYaw(tgt.start.pose.orientation), tgt.start.steering);
    ROS_INFO("[GetPathService] - goal :  [%f,%f,%f](%f)", tgt.goal.pose.position.x, tgt.goal.pose.position.y,tf::getYaw(tgt.goal.pose.orientation), tgt.goal.steering);
    
    orunav_generic::Path path;
    bool solution_found = false;
    if (!use_astar_) {
      solution_found = pool_.findPath(path, 
                                      orunav_conversions::createPose2dFromMsg(tgt.start.pose),
                                      orunav_conversions::createPose2dFromMsg(tgt.goal.pose),
                                      max_dist_offset, max_angular_offset);
    }
    else {
      PathSearchASTAR astar;
      astar.assignPaths(pool_.getPaths());
      astar.printDebug();
      solution_found = astar.findPath(path, 
                                      orunav_conversions::createPose2dFromMsg(tgt.start.pose),
                                      orunav_conversions::createPose2dFromMsg(tgt.goal.pose),
                                      max_dist_offset, max_angular_offset);
    }
    
    res.valid = solution_found;
    if (!solution_found || path.sizePath() == 0) {
      ROS_WARN("[GetPathService] - couldn't find a path...(!)");
      return false;
    }

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
  
};



int main(int argc, char** argv) {

  ros::init(argc, argv, "get_path_service");
  ros::NodeHandle parameters("~");
  GetPathService gps(parameters, ros::this_node::getName());

  ros::spin();
}
