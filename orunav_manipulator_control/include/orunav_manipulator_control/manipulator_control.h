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

    void perform_unload(const orunav_msgs::ManipulatorCommand::ConstPtr& cmd);
    void perform_load(const orunav_msgs::ManipulatorCommand::ConstPtr& cmd);
    void perform_unwrap(const orunav_msgs::ManipulatorCommand::ConstPtr& cmd);
    void perform_homing(const orunav_msgs::ManipulatorCommand::ConstPtr& cmd);
    void perform_idle(const orunav_msgs::ManipulatorCommand::ConstPtr& cmd);

    void update_report(const int32_t& new_status);
    
    orunav_msgs::ManipulatorReport current_report;
    
    void extract_unload_pose_from_ManipulatorCommand(const orunav_msgs::ManipulatorCommand::ConstPtr& in, KDL::Frame& out);
    void from_KDLFrame_to_PoseRPY(const KDL::Frame& in, lwr_controllers::PoseRPY& out);
    void from_KDLVector_to_PoseRPY_position(const KDL::Vector& in, lwr_controllers::PoseRPY::_position_type& out);
    void from_KDLRotation_to_PoseRPY_roation(const KDL::Rotation& in, lwr_controllers::PoseRPY::_orientation_type& out);
};

#endif