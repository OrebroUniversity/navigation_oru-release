#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <orunav_msgs/GetDeltaTVec.h>
#include <orunav_generic/interfaces.h>
#include <orunav_generic/io.h>
#include <orunav_generic/path_utils.h>
#include <orunav_conversions/conversions.h>
#include <orunav_rviz/orunav_rviz.h>
#include <orunav_trajectory_processor/trajectory_processor_naive.h>


bool visualize;
ros::Publisher marker_pub;
std::vector<TrajectoryProcessor::Params> params;


bool getDeltaTVecCallback(orunav_msgs::GetDeltaTVec::Request  &req,
                          orunav_msgs::GetDeltaTVec::Response &res )
{
    ROS_INFO("Obtained a request for a deltaTVec");

    // Get the path to be optimized.
    orunav_generic::Path path = orunav_conversions::createPathFromPathMsg(req.path);
    if (path.sizePath() < 3) {
        ROS_ERROR("way to short path");
        res.valid = true;
        return false;
    }

    // Compute min and max dts using different min/max acc and min/max vel.
    for (unsigned int i = 0; i < params.size(); i++)
    {
        TrajectoryProcessorNaive gen;
        gen.setParams(params[i]);
        gen.addPathInterface(path);
        gen.computeDts(); // The processing
        res.dts.dts.push_back(orunav_conversions::createDeltaTMsgFromDeltaTInterface(gen));
        std::string file_name("deltatvec" + orunav_generic::toString(i) + ".txt");
        orunav_generic::saveDeltaTInterface(gen, file_name);
    }

    if (visualize) {
      orunav_rviz::drawDeltaTVec(path, res.dts, marker_pub);
    }

    res.valid = true;
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "deltatvec_service");
  ros::NodeHandle n;
  ros::NodeHandle nh = ros::NodeHandle("~");

  TrajectoryProcessor::Params default_params;
  nh.param<bool>("use_initial_state", default_params.useInitialState, true);
  nh.param<double>("wheel_base_x", default_params.wheelBaseX, 1.190);
  nh.param<double>("time_step", default_params.timeStep, 0.06);
  
  TrajectoryProcessor::Params fast_params = default_params;
  TrajectoryProcessor::Params slow_params = default_params;
  
  nh.param<double>("fast_max_vel", fast_params.maxVel, 1.);
  nh.param<double>("fast_max_rotational_vel", fast_params.maxRotationalVel, 1.);
  nh.param<double>("fast_max_acc", fast_params.maxAcc, 1.);
  nh.param<double>("fast_max_steering_angle_vel", fast_params.maxSteeringAngleVel, 1.);
  nh.param<double>("fast_creep_speed", fast_params.creepSpeed, 0.);
  nh.param<double>("fast_creep_distance", fast_params.creepDistance, 0.);
  
  nh.param<double>("slow_max_vel", slow_params.maxVel, 0.2);
  nh.param<double>("slow_max_rotational_vel", slow_params.maxRotationalVel, 0.2);
  nh.param<double>("slow_max_acc", slow_params.maxAcc, 0.2);
  nh.param<double>("slow_max_steering_angle_vel", slow_params.maxSteeringAngleVel, 0.2);
  nh.param<double>("slow_creep_speed", slow_params.creepSpeed, 0.);
  nh.param<double>("slow_creep_distance", slow_params.creepDistance, 0.);
  params.push_back(fast_params);
  params.push_back(slow_params);


  nh.param("visualize",visualize,false);
  
  if (visualize)
  {
      std::cout << "The output is visualized using visualization_markers (in rviz)." << std::endl;
      marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  }
  
  ros::ServiceServer service = n.advertiseService("deltatvec_service", getDeltaTVecCallback);
  ros::spin();
  return 0;
}

