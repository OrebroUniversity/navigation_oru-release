#include <ros/ros.h>
#include "orunav_manipulator_control/manipulator_control.h"

int main(int argc, char **argv)
{
    if(!ros::isInitialized())
    {
	ros::init(argc,argv,"manipulator_control_node");
    }
  
    manipulatorControl controller;

    ROS_INFO_STREAM("[ILIAD] Manipulator Control: Started");

    ros::spin();

    return 0;
}
