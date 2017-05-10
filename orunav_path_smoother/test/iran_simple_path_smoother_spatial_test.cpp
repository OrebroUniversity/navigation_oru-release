#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>

#include <orunav_conversions/conversions.h>

#include <orunav_rviz/orunav_rviz.h>

#include <orunav_msgs/ControllerReport.h>
#include <orunav_msgs/RobotTarget.h>
#include <orunav_msgs/GetPath.h>
#include <orunav_msgs/GetPolygonConstraints.h>
#include <orunav_msgs/GetSmoothedPath.h>
#include <orunav_msgs/Task.h>

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

public:
  PathSmootherSpatialTestNode(ros::NodeHandle paramHandle) : goal_id_inc_(0)
  {
    paramHandle.param<double>("min_incr_path_dist", min_incr_path_dist_, 0.1);
    paramHandle.param<bool>("visualize", visualize_, true);
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

  void moveConstraints(orunav_msgs::RobotConstraints &msg) {

    std::vector<orunav_geometry::Polygon> polys = orunav_conversions::createConvexPolygonsFromRobotConstraintsMsg(msg);
//    for (size_t i = 0; i < polys.size(); i++) {
//      orunav_generic::Pose2d p(0., 1.5*sin(i/(1.*polys.size())*M_PI), 0.);
//
//      orunav_geometry::movePoint2dContainer(polys[i], p);
//    }

//    std::cout << "number of poly" << polys.size() << std::endl;
//    double d = 0.0;
//    for (size_t i = 0; i < polys.size(); i++) {
//    	if(i > 10 && i < 50)
//    		d = 3.0;
//    	else if(i == 50 || i == 10 ||  i == 51 || i == 9)
//    		d = 1.5;
//    	else
//    		d = 0.0;
//
//    	orunav_generic::Pose2d p(0., d, 0.);
//
//      orunav_geometry::movePoint2dContainer(polys[i], p);
//    }


    std::cout << "number of poly" << polys.size() << std::endl;
    double d = 0.0;
    for (size_t i = 0; i < polys.size(); i++) {
    	if(i < 5 || i > polys.size() - 5)
    		d = 0.0;
    	else
    		d = 2.0;

    	orunav_generic::Pose2d p(0., d, 0.);

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
    moveConstraints(constraints);
    constraints_.push_back(constraints);

    // Perform smoothing
    orunav_generic::Path smoothed_path;
    if (!computeSmoothedPath(target, map, path, constraints, smoothed_path)) {
      return false;
    }

    paths_.push_back(smoothed_path);

    task.target = target; // Simply return the original target (not the one with modified goal / start poses).
    task.path = orunav_conversions::createPathMsgFromPathAndState2dInterface(path,
                                                                             orunav_conversions::createState2dFromPoseSteeringMsg(target.start),
                                                                             orunav_conversions::createState2dFromPoseSteeringMsg(target.goal));
    task.constraints = constraints;
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


    orunav_msgs::RobotTarget target;
    orunav_generic::State2d start(1., 0., 0., 0.);
    orunav_generic::State2d goal(10., 0., 0., 0.);
    target.start = orunav_conversions::createPoseSteeringMsgFromState2d(start);
    target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(goal);
    target.start_op.operation = target.start_op.NO_OPERATION;
    target.goal_op.operation = target.goal_op.NO_OPERATION;
    target.start_earliest = ros::Time::now();


    ros::Publisher path_pub = nh_.advertise<orunav_msgs::RobotTarget>("/forIran/path", 1000);
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
    	path_pub.publish(target);
    	ros::spinOnce();
      	  loop_rate.sleep();
    }

    //computeTestTask(target, task_, 1);
    std::cout << "Done." << std::endl;

  }
 };

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_n_click_target_client");
  ros::NodeHandle params ("~");

  PathSmootherSpatialTestNode p(params);

  ros::spin();
}


