#include <iostream>
#include <fstream>

#include <acado_toolkit.hpp>
#include <acado/bindings/acado_gnuplot/gnuplot_window.hpp>
#include <orunav_path_smoother/acado_tools.h>
#include <orunav_path_smoother/path_smoother_dynamic.h>

#include <ros/ros.h>
#include <orunav_msgs/GetSmoothedPath.h>
#include <orunav_generic/interfaces.h>
#include <orunav_generic/io.h>
#include <orunav_generic/path_utils.h>
#include <orunav_conversions/conversions.h>
#include <orunav_constraint_extract/conversions.h>
#include <orunav_constraint_extract/utils.h>
#include <orunav_rviz/orunav_rviz.h>
#include <visualization_msgs/Marker.h>

bool visualize;
bool do_nothing;
bool reassign_constraints;
int reassign_iters;
double reassign_min_distance_;
ros::Publisher marker_pub;
PathSmootherDynamic::Params smoother_params;

bool getSmoothedPathCallback(orunav_msgs::GetSmoothedPath::Request  &req,
                             orunav_msgs::GetSmoothedPath::Response &res )
{
    ROS_INFO("Obtained a request for a smoothed path");
    if (do_nothing) {
      ROS_WARN("[SmoothedPathService]: doing nothing.");
      res.path = req.path;
      return true;
    }
    res.segmentaion_id = req.segmentaion_id;
    // Get the path to be optimized.
    orunav_generic::Path path = orunav_conversions::createPathFromPathMsg(req.path);
    if (path.sizePath() < 3) {
        ROS_ERROR("way to short path");
        return false;
    }

    // The constraints.
    constraint_extract::PolygonConstraintsVec constraints = orunav_conversions::createPolygonConstraintsVecFromRobotConstraintsMsg(req.constraints);
    
    if (constraints.size() != path.sizePath()) {
        ROS_ERROR("constraints size and path size differs");
        return false;
    }

    std::cout << "number of path points : " << constraints.size() << std::endl;;
    std::cout << "number of constraints : " << path.sizePath() << std::endl;

    
    PathSmootherDynamic smoother;
    smoother.params = smoother_params;
    smoother.params.max_nb_opt_points = path.sizePath();
    
    // This is fixed to be the start / goal of the path.
    
    orunav_generic::State2d start(path, 0);
    orunav_generic::State2d goal(path, path.sizePath() - 1);

    std::cout << "---------------- SMOOTHED PATH SERVICE ----------------" << std::endl;
    std::cout << "start : \n" << start.getPose2d() << std::endl;
    std::cout << "goal  : \n" << goal.getPose2d()  << std::endl;
    
    orunav_generic::Path smoothed_path = smoother.smooth(path, start, goal, constraints);
    std::cout << "------------- SMOOTHED PATH SERVICE - END --------------" << std::endl;

    res.constraints = req.constraints;

    if (reassign_constraints && reassign_iters > 0) {
      // Reassign the constraints - for all active constraints check if there is another constraints that could be used.
      std::cout << "---------------- SMOOTHED PATH SERVICE - REASSIGN ----------------" << std::endl;
      constraint_extract::PolygonConstraintsVec reassigned_constraints;
      for (int i = 0; i < reassign_iters; i++) {
        if (reassign_min_distance_ > 0.) {
          std::cout << "---------------- SMOOTHED PATH SERVICE - REASSIGN MIN DISTANCE ----------------" << std::endl;
          std::cout << "reassign_min_distance_ : " << reassign_min_distance_ << std::endl;
          constraint_extract::minIncrementalDistancePathConstraints(smoothed_path, constraints, reassign_min_distance_);
          constraint_extract::makeValidPathConstraintsForTrajectoryProcessing(smoothed_path, constraints);
        }
        reassigned_constraints = 
          constraint_extract::reassignConstraints(smoothed_path, constraints);
        path = smoothed_path;
        smoothed_path = smoother.smooth(path, start, goal, reassigned_constraints);
        constraints = reassigned_constraints;
        // Make sure that the path is still valid for trajectory processing.
        constraint_extract::makeValidPathConstraintsForTrajectoryProcessing(smoothed_path, constraints);
        
      }
      res.constraints = orunav_conversions::createRobotConstraintsFromPolygonConstraintsVec(constraints);
    }
    // Return the smoothed path as it is.
    res.path = orunav_conversions::createPathMsgFromPathInterface(smoothed_path);
    if (visualize) {
        orunav_rviz::drawPose2dContainer(smoothed_path, "path_smoother_new_path", 0, marker_pub);
    }
    
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_smoothed_straight_path");
  ros::NodeHandle n;
  ros::NodeHandle nh = ros::NodeHandle("~");

  nh.param<bool>("visualize",visualize,false);
  nh.param<bool>("visualize_deep", smoother_params.visualize, false);
  nh.param<int>("nb_iter_steps", smoother_params.nb_iter_steps, 100);
  nh.param<bool>("minimize_phi_and_dist", smoother_params.minimize_phi_and_dist, true);
  nh.param<double>("wheel_base",smoother_params.wheel_base, 0.68);
  nh.param<bool>("init_controls", smoother_params.init_controls, false);
  nh.param<bool>("use_th_constraints", smoother_params.use_th_constraints, true);
  nh.param<bool>("use_xy_constraints", smoother_params.use_xy_constraints, true);
  nh.param<double>("kkt_tolerance", smoother_params.kkt_tolerance, 0.01);
  nh.param<double>("integrator_tolerance", smoother_params.integrator_tolerance, 0.00001 );
  nh.param<double>("weight_steering_control", smoother_params.weight_steering_control, 1.);
  nh.param<double>("phi_min", smoother_params.phi_min, -1.1);
  nh.param<double>("phi_max", smoother_params.phi_max, 1.1);
  nh.param<bool>("do_nothing", do_nothing, false);
  nh.param<bool>("reassign_constraints", reassign_constraints, false);
  nh.param<int>("reassign_iters", reassign_iters, 1);
  nh.param<double>("reassign_min_distance", reassign_min_distance_, -1.);
  nh.param<bool>("use_incremental", smoother_params.use_incremental, false);
  nh.param<int>("incr_max_nb_points", smoother_params.incr_max_nb_points, 20);
  nh.param<int>("incr_nb_points_discard", smoother_params.incr_nb_points_discard, 5);
  if (visualize)
  {
      std::cout << "The output is visualized using visualization_markers (in rviz)." << std::endl;
      marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  }
  
  ros::ServiceServer service = n.advertiseService("get_smoothed_path", getSmoothedPathCallback);
  ros::spin();
  return 0;
}

