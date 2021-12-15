#include <iostream>
#include <fstream>
#include <cmath>

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

double wrap_rads( double r )
{
    while ( r > M_PI ) { r -= 2 * M_PI;}
    while ( r <= -M_PI ) {r += 2 * M_PI;}
    return r;
}

 int findDirection(orunav_generic::Pose2d prev,orunav_generic::Pose2d next){
      double x0 = prev(0); double y0 = prev(1); double th = prev(2);
      double x1 = next(0); double y1 = next(1);
      th = wrap_rads(th);
      double m ,sign = 1;
      //std::cout << "th " <<th <<" " << cos(th+M_PI/2)  <<std::endl;
      if (abs(cos(th+M_PI/2)) <= 0.0001 || th == 0){ 
          m=1000;
          
          if ((y1-y0) < m*(x1-x0)) return 1;
          else return -1;
        }
      else{ m = tan(th+M_PI/2);}
      std::cout << " " << th;
      if (signbit(th) == 1){
        if ((y1-y0) < m*(x1-x0)){std::cout << " a "; return 1;}
      }
      else {
        if ((y1-y0) > m*(x1-x0)){std::cout << " b "; return 1;}
      }
      std::cout << " c ";
      return -1;
    }

int cuspidi(int incr , orunav_generic::Path path){
      int motion_old3=0, motion_old2=0, motion_old = 0, motion = 0;
      int cuspide = 0;
      int inc = 1 + incr;
      for (int i = 0; i < path.sizePath()-2; i += inc){
        motion_old3 = motion_old2;
        motion_old2 = motion_old;
        motion_old = motion;
        motion = findDirection(path.getPose2d(i) , path.getPose2d(i+inc));
        std::cout << motion << " " << std::endl;
        //std::cout << "motionOld " << motion_old << " motion "<< motion << " p " <<
        //path.getPose2d(i)(0) << " " << path.getPose2d(i)(1) <<" "<< path.getPose2d(i)(2)<<std::endl;
        //orunav_rviz::drawPose2d(path.getPose2d(i), 0, 0, 1.5, "cuspide", marker_pub_);
        if (motion_old3 != 0 && motion_old != motion_old2 && motion_old == motion && motion_old2 == motion_old3){
          //if (motion_old != motion_old2 && motion_old == motion){
            cuspide += 1;
            //std::cout << "cuspide!         -" << path.getPose2d(i)(0) << " " << path.getPose2d(i)(1) <<" "<< path.getPose2d(i)(2) << std::endl;
        }
        //getchar();
      }
      std::cout << "total cuspidi" << cuspide << std::endl;
      return cuspide;
    }
  


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

    std::cout << "number of constraints : " << constraints.size() << std::endl;;
    std::cout << "number of path points  : " << path.sizePath() << std::endl;

    
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



    std::ofstream f;
    f.open("/home/ubuntu18/catkin_ws/src/volvo_ce/hx_smooth_control/results/data.txt", std::ios::app);
    f << "smoothed length: " << smoothed_path.getLength() << std::endl;
    //f << "smoothed cuspidu: " << smoothed_path.cuspidi(0) << std::endl;
    f << "smoothed cuspidi: " << cuspidi(0, smoothed_path) << std::endl;

    f << "smoothed path: "<< std::endl;
    for (unsigned int i = 0; i < smoothed_path.sizePath(); i++)
      {
      f << smoothed_path.getPose2d(i)(0) << " "
          << smoothed_path.getPose2d(i)(1) << " "
          << smoothed_path.getPose2d(i)(2) << " "
          << smoothed_path.getSteeringAngle(i) << " "
          << smoothed_path.getSteeringAngleRear(i) << std::endl; //Cecchi_add
          }
    f.close();

    
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
  nh.param<bool>("biSteering", smoother_params.BS, false);
  if (visualize)
  {
      std::cout << "The output is visualized using visualization_markers (in rviz)." << std::endl;
      marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  }
  
  ros::ServiceServer service = n.advertiseService("get_smoothed_path", getSmoothedPathCallback);
  ros::spin();
  return 0;
}

