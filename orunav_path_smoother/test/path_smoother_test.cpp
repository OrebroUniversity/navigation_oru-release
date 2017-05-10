#include <ros/ros.h>


#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>

#include <orunav_conversions/conversions.h>

#include <orunav_msgs/ComputeTask.h>
#include <orunav_msgs/ExecuteTask.h>
#include <orunav_msgs/ControllerReport.h>
#include <orunav_msgs/RobotTarget.h>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <boost/program_options.hpp>


namespace po = boost::program_options;

//! Simple client to stress the vehicle execution node - by sending a large amount of set_execution service calls.

class PathSmootherTestNode
{
private:
  ros::NodeHandle nh_;
  int robot_id_;
  int goal_id_inc_;
  bool b_shutdown_;
  bool valid_map_;
  
  ros::Subscriber map_sub_;
  
  boost::mutex input_mutex_, map_mutex_;
  boost::thread client_thread_;
 
  

public:
  PathSmootherTestNode(ros::NodeHandle paramHandle) : goal_id_inc_(0)
  {
    // Subscribers
    map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map",10,&KMOVehicleExecutionNode::process_map, this);
  }
  
  VehicleExecutionDebugClientNode() {
    b_shutdown_ = true;
    client_thread_.join();
  }

  bool computeTestTask(const orunav_msgs::RobotTarget &target, orunav_msgs::Task &task, int test) {
    
    // Do we have a map available? TODO - this should be possible to be sent from the coordination as well(!).
    if (!valid_map_) {
      ROS_WARN("[KMOVehicleExecutionNode] RID:%d - empty map(!), cannot computeTestTask", robot_id_);
      res.result = 0;
      return false;
    }

    orunav_msgs::RobotTarget target = req.target;
    map_mutex_.lock();
    nav_msgs::OccupancyGrid map = current_map_;
    map_mutex_.unlock();

    //constraint_extract::addPolygonToOccupancyMap(pm.getPosePolygon(), map, 100);
    
    orunav_generic::Path path;
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
    // Remove duplicate points in the path.
    orunav_generic::makeValidPathForTrajectoryProcessing(path);
    // Make it less dense... important for the smoothing steps.
    path = orunav_generic::minIncrementalDistancePath(path, min_incr_path_dist_);
    ROS_INFO_STREAM("[KMOVehicleExecutionNode] - size of path : " << path.sizePath());
 
    // Perform smoothing
    orunav_msgs::GetPolygonConstraints srv_constraints;
    orunav_msgs::GetSmoothedPath srv_smoothedpath;
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
      srv_constraints = srv;
    }

    {
      // Perform optimization
      orunav_msgs::GetSmoothedPath srv;
      srv.request.path = srv_constraints.request.path;
      srv.request.map = srv_constraints.request.map;
      srv.request.constraints = srv_constraints.response.constraints;

      ros::ServiceClient client = nh_.serviceClient<orunav_msgs::GetSmoothedPath>("get_smoothed_path");
      if (client.call(srv)) {
        ROS_INFO("[KMOVehicleExecutionNode] - get_smoothed_path - successfull");
      }
      else
      {
        ROS_ERROR("[KMOVehicleExecutionNode] - Failed to call service: GetSmoothedPath");
        return false;
      }
      srv_smoothedpath = srv;
      path = orunav_conversions::createPathFromPathMsg(srv.response.path);
      
      double max_steering_angle = M_PI/2.;
      double max_dist_offset = 0.1;
      double max_heading_offset = 0.1;
      if (!orunav_generic::validSmoothedPath(path,
                                             orunav_conversions::createState2dFromPoseSteeringMsg(target.start),
                                             orunav_conversions::createState2dFromPoseSteeringMsg(target.goal),
                                             max_steering_angle,
                                             max_dist_offset,
                                             max_heading_offset)) {
        ROS_ERROR("Invalid smoothed path(!)");

        std::cout << " path.getPose2d(0) : " << path.getPose2d(0) << std::endl;
        std::cout << " path.getPose2d(path.sizePath()-1) : " << path.getPose2d(path.sizePath()-1) << std::endl;
        std::cout << " orunav_conversions::createState2dFromPoseSteeringMsg(target.start).getPose2d() : " << orunav_conversions::createState2dFromPoseSteeringMsg(target.start).getPose2d() << std::endl;
        std::cout << " orunav_conversions::createState2dFromPoseSteeringMsg(target.goal).getPose2d() : " << orunav_conversions::createState2dFromPoseSteeringMsg(target.goal).getPose2d() << std::endl;
        return false;
      }                                             
    }

    // Packet the message and return it.
    task.target = req.target; // Simply return the original target (not the one with modified goal / start poses).
    task.path = srv_smoothedpath.response.path;
    task.constraints = srv_constraints.response.constraints;
  }
  
  
  void run() {
    
    // Do whatever...
    //1). Get the current position.
    //2). Compute a task going 2 meter forward.
    //3). Send the task.
    //4). Send the same task over and over every 0.1 seconds for 10 seconds.
    bool print_once = true;
    while (!validState2d_) {
      usleep(1000000);
      if (print_once) {
        ROS_INFO("Waiting for a current state (controller report).");
        print_once = false;
      }
    }
    
    input_mutex_.lock();
    orunav_generic::State2d start = currentState2d_;
    input_mutex_.unlock();
    
    ROS_INFO_STREAM("Got start state : " << start);
    orunav_msgs::RobotTarget target;
    target.start = orunav_conversions::createPoseSteeringMsgFromState2d(start);
    orunav_generic::State2d goal = start;
    orunav_generic::Control step(5., 0.);
    goal.addControlStep(step, 1.19, 1.);
    target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(goal);
    target.start_op.operation = target.start_op.ACTIVATE_SUPPORT_LEGS;//LOAD;
    target.goal_op.operation = target.goal_op.UNLOAD;
    target.start_earliest = ros::Time::now();

    // Compute task
    orunav_msgs::Task task;
    if (!computeTask(target, task))
      return;
        
    executeTask(task); // First time sent(!)
    ROS_INFO("Sending task - first time.");
    task.update = true;
    
    for (int i = 0; i < 0; i++) {
      usleep(100000);
      ROS_INFO_STREAM("(Re)-sending task, amount : " << i);
      executeTask(task);
    }

    ROS_INFO("Done...");
  }

    // Processing callbacks from subscriptions
  void process_map(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    map_mutex_.lock();
    current_map_ = *msg;
    valid_map_ = true;
    map_mutex_.unlock();
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
    orunav_generic::State2d start(3., 5., 0., 0.);
    orunav_generic::State2d goal(7., 5., 0., 0.);
    target.start = orunav_conversions::createPoseSteeringMsgFromState2d(start);
    target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(goal);
    target.start_op.operation = target.start_op.NO_OPERATION;
    target.goal_op.operation = target.goal_op.NO_OPERATION;
    target.start_earliest = ros::Time::now();
    
    
    
  }
 };

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_n_click_target_client");
  ros::NodeHandle params ("~");

  VehicleExecutionDebugClientNode p(params);
  
  ros::spin();
}


