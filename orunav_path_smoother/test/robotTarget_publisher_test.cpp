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

    //heartbeat_visualization_   = nh_.createTimer(ros::Duration(1.0),&PathSmootherSpatialTestNode::publish_visualization,this);

    //call worker thread
    client_thread_ = boost::thread(boost::bind(&PathSmootherSpatialTestNode::run,this));

    valid_map_ = false;
    b_shutdown_ = false;
  }



  // Processing callbacks from subscriptions
  void process_map(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    map_mutex_.lock();
    current_map_ = *msg;
    valid_map_ = true;
    map_mutex_.unlock();
  }



  void run() {

    std::cout << "Start Publishing RobotTargets." << std::endl;

    orunav_msgs::RobotTarget target;
    orunav_generic::State2d start(3., 3., 0., 0.);
    orunav_generic::State2d goal(15., 3., 0., 0.);
    target.start = orunav_conversions::createPoseSteeringMsgFromState2d(start);
    target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(goal);
    target.start_op.operation = target.start_op.NO_OPERATION;
    target.goal_op.operation = target.goal_op.NO_OPERATION;
    target.start_earliest = ros::Time::now();
    target.robot_id = 0.0;
    target.goal_id = 0.0;

    orunav_msgs::RobotTarget target1;
    orunav_generic::State2d start1(3., 5., 0., 0.);
    orunav_generic::State2d goal1(15., 5., 0., 0.);
    target1.start = orunav_conversions::createPoseSteeringMsgFromState2d(start1);
    target1.goal = orunav_conversions::createPoseSteeringMsgFromState2d(goal1);
    target1.start_op.operation = target1.start_op.NO_OPERATION;
    target1.goal_op.operation = target1.goal_op.NO_OPERATION;
    target1.start_earliest = ros::Time::now();
    target1.robot_id = 1.0;
    target1.goal_id = 1.0;


   orunav_msgs::RobotTarget target2;
    orunav_generic::State2d start2(3., 11., 0., 0.);
    orunav_generic::State2d goal2(15., 3., 0., 0.);
    target2.start = orunav_conversions::createPoseSteeringMsgFromState2d(start2);
    target2.goal = orunav_conversions::createPoseSteeringMsgFromState2d(goal2);
    target2.start_op.operation = target2.start_op.NO_OPERATION;
    target2.goal_op.operation = target2.goal_op.NO_OPERATION;
    target2.start_earliest = ros::Time::now();
    target2.robot_id = 2.0;
    target2.goal_id = 2.0;



    ros::Publisher path_pub = nh_.advertise<orunav_msgs::RobotTarget>("/forIran/path", 1000);
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        path_pub.publish(target1);
      	//loop_rate.sleep();
      	path_pub.publish(target);
      	//path_pub.publish(target2);
      	ros::spinOnce();
    }

    //computeTestTask(target, task_, 1);


  }
 };

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_n_click_target_client");
  ros::NodeHandle params ("~");

  PathSmootherSpatialTestNode p(params);

  ros::spin();
}


