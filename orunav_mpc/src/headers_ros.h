#pragma once

#include "commonDefines.h"

// ROS headers are sloppy, disable some of the warnings for them.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-pedantic"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <orunav_msgs/ControllerTrajectoryChunkVec.h>
#include <orunav_msgs/ControllerTrajectoryChunk.h>
#include <orunav_msgs/ControllerReport.h>
#include <orunav_msgs/ControllerCommand.h>
#include <orunav_msgs/ActiveRobots.h>

#ifdef SW_BUILD_SIMULATION
#include <geometry_msgs/Twist.h>

#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/transform_listener.h>
#endif // SW_BUILD_SIMULATION

#pragma GCC diagnostic pop
