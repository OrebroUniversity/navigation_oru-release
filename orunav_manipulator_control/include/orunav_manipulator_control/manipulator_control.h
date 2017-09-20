#ifndef MANIPULATOR_CONTROL_CITI_TRUCK_H
#define MANIPULATOR_CONTROL_CITI_TRUCK_H

#include <lwr_controllers/PoseRPY.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <map>
#include <orunav_msgs/ManipulatorReport.h>
#include <orunav_msgs/ManipulatorCommand.h>
#include <mutex>
#include <kdl/frames.hpp>

class manipulatorControl
{
public:
    manipulatorControl();
    ~manipulatorControl();

private:
    ros::NodeHandle nh;

    ros::Timer report_timer;
    std::mutex report_mutex;
    void publish_report(const ros::TimerEvent& event);
    ros::Publisher report_pub;

    ros::Publisher cmd_pub_left;
    ros::Publisher cmd_pub_right;

    ros::Subscriber command_sub;
    void process_manipulator_command(const orunav_msgs::ManipulatorCommand::ConstPtr &msg);

    tf::TransformListener listener;

    void send_target();
    
    orunav_msgs::ManipulatorCommand last_cmd;
    orunav_msgs::ManipulatorReport current_report;
    
    void from_ManipulatorCommand_to_KDLFrame(const orunav_msgs::ManipulatorCommand& in, KDL::Frame& out);
    void from_KDLFrame_to_PoseRPY(const KDL::Frame& in, lwr_controllers::PoseRPY& out);
};

#endif