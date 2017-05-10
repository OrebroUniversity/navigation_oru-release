#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>

#include <orunav_conversions/conversions.h>

#include <orunav_constraint_extract/polygon_constraint.h>
#include <orunav_constraint_extract/conversions.h>
#include <orunav_constraint_extract/grid_map.h>
#include <orunav_constraint_extract/utils.h>

#include <orunav_rviz/orunav_rviz.h>

#include <orunav_msgs/ControllerReport.h>
#include <orunav_msgs/RobotTarget.h>
#include <orunav_msgs/GetPath.h>
#include <orunav_msgs/GetPolygonConstraints.h>
#include <orunav_msgs/GetSmoothedPath.h>
#include <orunav_msgs/Task.h>
#include <orunav_msgs/GetDeltaTVec.h>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <boost/program_options.hpp>


namespace po = boost::program_options;

class PathSmootherSpatialTestNode
{
private:
  ros::NodeHandle nh_;
  int robot_id_;
  int goal_id_inc_;
  bool b_shutdown_;
  
  nav_msgs::OccupancyGrid current_map_;
  ros::Publisher marker_pub_;
  bool valid_map_;
  
  ros::Subscriber map_sub_;
  
  boost::mutex input_mutex_, map_mutex_;
  boost::thread client_thread_;
  double min_incr_path_dist_;
  
  bool visualize_;
  ros::Timer heartbeat_visualization_;
  orunav_msgs::Task task_;
  
  std::vector<orunav_generic::Path> paths_;
  std::vector<orunav_msgs::RobotConstraints> constraints_;
  orunav_msgs::DeltaTVec deltatvec_;

  std::string start_goal_file_name_;
  int start_idx_;
  bool move_constraints_;

public:
  PathSmootherSpatialTestNode(ros::NodeHandle paramHandle) : goal_id_inc_(0)
  {
    paramHandle.param<double>("min_incr_path_dist", min_incr_path_dist_, 0.1);
    paramHandle.param<bool>("visualize", visualize_, true);
    paramHandle.param<std::string>("start_goal_file", start_goal_file_name_, std::string(""));
    paramHandle.param<bool>("move_constraint", move_constraints_, false);
    paramHandle.param<int>("start_idx", start_idx_, 0);

    // Subscribers
    map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map",10,&PathSmootherSpatialTestNode::process_map, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

    heartbeat_visualization_   = nh_.createTimer(ros::Duration(1.0),&PathSmootherSpatialTestNode::publish_visualization,this);
    
    //call worker thread 
    client_thread_ = boost::thread(boost::bind(&PathSmootherSpatialTestNode::run,this));

    valid_map_ = false;
    b_shutdown_ = false;
  }
  
  PathSmootherSpatialTestNode() {
    b_shutdown_ = true;
    client_thread_.join();
  }

  bool computePath(const orunav_msgs::RobotTarget &target, const nav_msgs::OccupancyGrid &map, orunav_generic::Path &path) {

    { // Get path service call related stuff goes here...
      orunav_msgs::GetPath srv;
      srv.request.map = map;
      srv.request.target = target;
      // Update the target goal pose and map based on the load operations
      
      srv.request.max_planning_time = 20.; // TODO param
      // Need to package the target + the map and ask the motion planner.
      ros::ServiceClient client = nh_.serviceClient<orunav_msgs::GetPath>("get_path");
      
      if (client.call(srv)) {
        ROS_INFO("[KMOVehicleExecutionNode] - get_path sucessfull");
      }
      else
      {
        ROS_ERROR("[KMOVehicleExecutionNode] - Failed to call service: GetPath");
        return false;
      }
      path = orunav_conversions::createPathFromPathMsgUsingTargetsAsFirstLast(srv.response.path);
    }
    return true;
  }

  bool computeConstraints(const orunav_msgs::RobotTarget &target, const nav_msgs::OccupancyGrid &map, const orunav_generic::Path &path, orunav_msgs::RobotConstraints &constraints) {
    { // Compute the constraints goes here
      orunav_msgs::GetPolygonConstraints srv;
      srv.request.map = map;
      srv.request.path = orunav_conversions::createPathMsgFromPathAndState2dInterface(path, 
                                                                                      orunav_conversions::createState2dFromPoseSteeringMsg(target.start),
                                                                                      orunav_conversions::createState2dFromPoseSteeringMsg(target.goal));
      
      ros::ServiceClient client = nh_.serviceClient<orunav_msgs::GetPolygonConstraints>("polygonconstraint_service");
      if (client.call(srv)) {
        ROS_INFO("[KMOVehicleExecutionNode] - polygonconstraint_service - successfull");
      }
      else
      {
        ROS_ERROR("[KMOVehicleExecutionNode] - Failed to call service: PolygonConstraint");
        return false;
      }
      
      // Check that the constraints are valid / add valid flag in the msg.
      constraints = srv.response.constraints;
    }
    return true;
  }

  bool computeSmoothedPath(const orunav_msgs::RobotTarget &target, const nav_msgs::OccupancyGrid &map, const orunav_generic::Path &path, const orunav_msgs::RobotConstraints &constraints, orunav_generic::Path &smoothedPath) {
    {
      // Perform optimization
      orunav_msgs::GetSmoothedPath srv;
      srv.request.path = orunav_conversions::createPathMsgFromPathAndState2dInterface(path, 
                                                                                      orunav_conversions::createState2dFromPoseSteeringMsg(target.start),
                                                                                      orunav_conversions::createState2dFromPoseSteeringMsg(target.goal));
      srv.request.map = map;
      srv.request.constraints = constraints;

      ros::ServiceClient client = nh_.serviceClient<orunav_msgs::GetSmoothedPath>("get_smoothed_path");
      if (client.call(srv)) {
        ROS_INFO("[KMOVehicleExecutionNode] - get_smoothed_path - successfull");
      }
      else
      {
        ROS_ERROR("[KMOVehicleExecutionNode] - Failed to call service: GetSmoothedPath");
        return false;
      }
      smoothedPath = orunav_conversions::createPathFromPathMsg(srv.response.path);
      
      double max_steering_angle = M_PI/2.;
      double max_dist_offset = 0.1;
      double max_heading_offset = 0.1;
      if (!orunav_generic::validSmoothedPath(smoothedPath,
                                             orunav_conversions::createState2dFromPoseSteeringMsg(target.start),
                                             orunav_conversions::createState2dFromPoseSteeringMsg(target.goal),
                                             max_steering_angle,
                                             max_dist_offset,
                                             max_heading_offset)) {
        ROS_ERROR("Invalid smoothed path(!)");

        std::cout << " smoothedPath.getPose2d(0) : " << smoothedPath.getPose2d(0) << std::endl;
        std::cout << " smoothedPath.getPose2d(smoothedPath.sizePath()-1) : " << smoothedPath.getPose2d(smoothedPath.sizePath()-1) << std::endl;
        std::cout << " orunav_conversions::createState2dFromPoseSteeringMsg(target.start).getPose2d() : " << orunav_conversions::createState2dFromPoseSteeringMsg(target.start).getPose2d() << std::endl;
        std::cout << " orunav_conversions::createState2dFromPoseSteeringMsg(target.goal).getPose2d() : " << orunav_conversions::createState2dFromPoseSteeringMsg(target.goal).getPose2d() << std::endl;
        return false;
      }                                             
    }
    return true;
  }

  bool computeDeltaTVec(const orunav_msgs::RobotTarget &target, const orunav_generic::Path &path, orunav_msgs::DeltaTVec &dts) {
    {
      orunav_msgs::GetDeltaTVec srv;
      
      srv.request.path = orunav_conversions::createPathMsgFromPathAndState2dInterface(path, 
                                                                   orunav_conversions::createState2dFromPoseSteeringMsg(target.start),
                                                                   orunav_conversions::createState2dFromPoseSteeringMsg(target.goal));
      srv.request.target = target;
      // Need to package the target + the map and ask the motion planner.
      ros::ServiceClient client = nh_.serviceClient<orunav_msgs::GetDeltaTVec>("deltatvec_service");
      
      if (client.call(srv)) {
        ROS_INFO("[KMOVehicleExecutionNode] - deltatvec_service sucessfull");
      }
      else
      {
        ROS_ERROR("[KMOVehicleExecutionNode] - Failed to call service: deltatvec_service");
        return false;
      }
      
      if (!srv.response.valid) {
        ROS_WARN("[KMOVehicleExecutionNode] RID:%d - couldn't find deltatvecs, cannot computeTask", robot_id_);
        return false;
      }
      dts = srv.response.dts;
    }
    return true;
  }

  void moveConstraints(orunav_msgs::RobotConstraints &msg) {

    std::vector<orunav_geometry::Polygon> polys = orunav_conversions::createConvexPolygonsFromRobotConstraintsMsg(msg);
    for (size_t i = 0; i < polys.size(); i++) {
      orunav_generic::Pose2d p(0., 1.5*sin(i/(1.*polys.size())*M_PI), 0.);

      orunav_geometry::movePoint2dContainer(polys[i], p);
    }
    msg = orunav_conversions::createRobotConstraintsMsgFromConvexPolygons(polys);
  }

  bool computeTestTask(const orunav_msgs::RobotTarget &target, orunav_msgs::Task &task, int test) {
    
    // Do we have a map available? TODO - this should be possible to be sent from the coordination as well(!).
    if (!valid_map_) {
      ROS_WARN("[KMOVehicleExecutionNode] RID:%d - empty map(!), cannot computeTestTask", robot_id_);
      return false;
    }

    orunav_generic::Path path;
    nav_msgs::OccupancyGrid map = current_map_;
    if (!computePath(target, map, path)) {
      return false;
    }

    // Remove duplicate points in the path.
    orunav_generic::makeValidPathForTrajectoryProcessing(path);
    // Make it less dense... important for the smoothing steps.
    path = orunav_generic::minIncrementalDistancePath(path, min_incr_path_dist_);

    paths_.push_back(path);

    // Compute constraints
    orunav_msgs::RobotConstraints constraints;
    if (!computeConstraints(target, map, path, constraints)) {
      return false;
    }
    
    //constraints_.push_back(constraints);
    if (move_constraints_) {
      moveConstraints(constraints);
    }
    constraints_.push_back(constraints);

    // Perform smoothing
    orunav_generic::Path smoothed_path;
    if (!computeSmoothedPath(target, map, path, constraints, smoothed_path)) {
      return false;
    }

    paths_.push_back(smoothed_path);
    path = smoothed_path;
    
    // Compute deltaT's.
    // Need to make some adjustments to the constraints and the path... - very important(!)
    // The only reason why the constraints is updated -> to keep the 1:1 correspondence between them.
    {
      constraint_extract::PolygonConstraintsVec c = orunav_conversions::createPolygonConstraintsVecFromRobotConstraintsMsg(constraints);
      constraint_extract::makeValidPathConstraintsForTrajectoryProcessing(path, c);
      constraints = orunav_conversions::createRobotConstraintsFromPolygonConstraintsVec(c);
    }
    // Now we're ready to get the delta T's.
    orunav_msgs::DeltaTVec dts;
    if (!computeDeltaTVec(target, path, dts)) {
      return false;
    }
    
    task.target = target;
    task.path = orunav_conversions::createPathMsgFromPathAndState2dInterface(path, 
                                                                             orunav_conversions::createState2dFromPoseSteeringMsg(target.start),
                                                                             orunav_conversions::createState2dFromPoseSteeringMsg(target.goal));
    task.constraints = constraints;
    task.dts = dts;
    deltatvec_ = dts;
    return true;
  }

  // Processing callbacks from subscriptions
  void process_map(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    map_mutex_.lock();
    current_map_ = *msg;
    valid_map_ = true;
    map_mutex_.unlock();
  }

  void publish_visualization(const ros::TimerEvent &event) {
    if (!visualize_)
      return;

    for (size_t i = 0; i < constraints_.size(); i++) {
      orunav_rviz::drawConstraints(constraints_[i], marker_pub_);
    }

    for (size_t i = 0; i < paths_.size(); i++) {
      orunav_rviz::drawPathInterface(paths_[i], "path", i, 1.19, marker_pub_);
    }
    orunav_rviz::drawDeltaTVec(paths_.back(), deltatvec_, marker_pub_);
  }

  void run() {

    bool print_once = true;
    while (!valid_map_) {
      usleep(1000000);
      if (print_once) {
        ROS_INFO("Waiting for a valid map...");
        print_once = false;
      }
    }

    // Also need to wait until the primitive etc. are loaded... simply sleep for now.
    
    for (size_t i = 0; i < 10; i++) {
      ROS_INFO_STREAM("Sleeping (waiting until all services starts up)");
      usleep(1000000);
    }
    
    
    if (start_goal_file_name_ != std::string("")) {
      // Load the start / end as a path...
      orunav_generic::Path start_goals = orunav_generic::loadPathTextFile(start_goal_file_name_);
      
      ROS_ERROR_STREAM("start_goals size : " << start_goals.sizePath());

      orunav_generic::savePathTextFile(start_goals, "loaded_start_goals.txt");
      size_t j = start_idx_; 
      for (size_t i = start_idx_; i < start_goals.sizePath(); i+= 2) {
	
	ROS_ERROR_STREAM("Computing tasks : " << j);
	orunav_msgs::RobotTarget target;
	orunav_msgs::Task task;
	orunav_generic::State2d start(start_goals, i);
	orunav_generic::State2d goal(start_goals, i+1);
	
	target.start = orunav_conversions::createPoseSteeringMsgFromState2d(start);
	target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(goal);
	target.start_op.operation = target.start_op.NO_OPERATION;
	target.goal_op.operation = target.goal_op.NO_OPERATION;
	target.start_earliest = ros::Time::now();
	
	
	computeTestTask(target, task, 1);
	
	orunav_generic::Path new_path = orunav_conversions::createPathFromPathMsg(task.path);
	
	orunav_generic::savePathTextFile(new_path, start_goal_file_name_ + orunav_generic::toString(j) + ".path");
	j++;
      }
    }
    else {
      orunav_msgs::RobotTarget target;
      orunav_generic::State2d start(1., 0., 0., 0.);
      orunav_generic::State2d goal(5., 0., 0., 0.);
      target.start = orunav_conversions::createPoseSteeringMsgFromState2d(start);
      target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(goal);
      target.start_op.operation = target.start_op.NO_OPERATION;
      target.goal_op.operation = target.goal_op.NO_OPERATION;
      target.start_earliest = ros::Time::now();
      
      computeTestTask(target, task_, 1);
    }
    std::cout << "Done." << std::endl;
  }
 };

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_smoother_spatial_test");
  ros::NodeHandle params ("~");

  PathSmootherSpatialTestNode p(params);
  
  ros::spin();
}


