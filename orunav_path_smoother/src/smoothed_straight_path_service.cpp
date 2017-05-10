#include <iostream>
#include <fstream>

#include <acado_toolkit.hpp>
#include <acado/bindings/acado_gnuplot/gnuplot_window.hpp>
#include <orunav_path_smoother/acado_tools.h>
#include <orunav_path_smoother/path_smoother_dynamic.h>

#include <ros/ros.h>
#include <orunav_msgs/GetSmoothedStraightPath.h>
#include <orunav_generic/interfaces.h>
#include <orunav_generic/io.h>
#include <orunav_generic/path_utils.h>
#include <orunav_conversions/conversions.h>
#include <orunav_constraint_extract/conversions.h>
#include <orunav_rviz/orunav_rviz.h>
#include <visualization_msgs/Marker.h>

bool visualize;
ros::Publisher marker_pub;
PathSmootherDynamic smoother;

// Only possible to use this the last meter to do fine adjustments to the picking pose.
bool getSmoothedStraightPathCallback(orunav_msgs::GetSmoothedStraightPath::Request  &req,
                                     orunav_msgs::GetSmoothedStraightPath::Response &res )
{
     ROS_INFO("Obtained a request for a smoothed straight path");
     

    // Get the path to be optimized.
    orunav_generic::Path path = orunav_generic::createStraightPathFromStartPose(
        orunav_conversions::createPose2dFromMsg(req.start.pose),
        orunav_conversions::createPose2dFromMsg(req.goal.pose),
        req.resolution);

    // Just hard-code some reasonable inputs here...
    smoother.params.max_nb_opt_points = path.sizePath();
    smoother.params.nb_iter_steps = 100;
    smoother.params.w_min = -10; // Should be high
    smoother.params.w_max = 10;  // Should be high
    smoother.params.v_max = 0; // Only allowed to drive towards the pallet
    smoother.params.update_v_w_bounds = true;
    smoother.params.keep_w_bounds = true;
    smoother.params.get_speed = false;
    smoother.params.max_nb_opt_points = req.max_nb_opt_points;
    smoother.params.wheel_base = req.wheel_base;
    smoother.params.visualize = false;
    
    orunav_generic::State2d start = orunav_conversions::createState2dFromPoseSteeringMsg(req.start);
    orunav_generic::State2d goal = orunav_conversions::createState2dFromPoseSteeringMsg(req.goal);


    std::cout << "---------------- SMOOTHED STRAIGHT PATH SERVICE ----------------" << std::endl;
    std::cout << "start : \n" << start.getPose2d() << std::endl;
    std::cout << "goal  : \n" << goal.getPose2d()  << std::endl;
    std::cout << "resolution : " << req.resolution << std::endl;

    orunav_generic::Path smoothed_path = smoother.smooth(path, start, goal);
    std::cout << "------------- SMOOTHED STRAIGHT PATH SERVICE - END --------------" << std::endl;

    res.path = orunav_conversions::createPathMsgFromPathInterface(smoothed_path);

    if (visualize) {
        orunav_rviz::drawPose2dContainer(smoothed_path, "path_smoother_straight_new_path", 0, marker_pub);
    }
    
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_smoothed_straight_path");
  ros::NodeHandle n;
  ros::NodeHandle nh = ros::NodeHandle("~");

  nh.param<bool>("visualize",visualize,false);
  nh.param<bool>("visualize_deep", smoother.params.visualize, false);
  nh.param<int>("nb_iter_steps", smoother.params.nb_iter_steps, 100);
  nh.param<bool>("minimize_phi_and_dist", smoother.params.minimize_phi_and_dist, false);
  nh.param<bool>("init_controls", smoother.params.init_controls, false);
  nh.param<bool>("use_th_constraints", smoother.params.use_th_constraints, false);
  nh.param<bool>("use_xy_constraints", smoother.params.use_xy_constraints, false);
  nh.param<double>("kkt_tolerance", smoother.params.kkt_tolerance, 0.01);
  nh.param<double>("integrator_tolerance", smoother.params.integrator_tolerance, 0.00001 );
  nh.param<double>("weight_steering_control", smoother.params.weight_steering_control, 1.);
  //  nh.param<bool>("interp_dense_path", smoother.params.interp_dense_path, true);
  nh.param<double>("phi_min", smoother.params.phi_min, -1.1);
  nh.param<double>("phi_max", smoother.params.phi_max, 1.1);

  if (visualize)
  {
      std::cout << "The output is visualized using visualization_markers (in rviz)." << std::endl;
      marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  }
  
  ros::ServiceServer service = n.advertiseService("get_smoothed_straight_path", getSmoothedStraightPathCallback);
  ros::spin();
  return 0;
}

