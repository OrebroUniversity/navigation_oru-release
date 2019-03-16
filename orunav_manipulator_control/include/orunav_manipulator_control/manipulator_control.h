#ifndef MANIPULATOR_CONTROL_CITI_TRUCK_H
#define MANIPULATOR_CONTROL_CITI_TRUCK_H

#include <lwr_controllers/PoseRPY.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <map>
#include <orunav_msgs/ManipulatorReport.h>
#include <orunav_msgs/ManipulatorCommand.h>
#include <orunav_msgs/IliadItemArray.h>
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

    void update_ee_transformations();
    KDL::Frame base_T_right_hand;
    KDL::Frame base_T_velvet_tray;

    void perform_pick_items(const orunav_msgs::ManipulatorCommand::ConstPtr& cmd);
    void load_item(const orunav_msgs::IliadItem& item);
    void unload_item(const orunav_msgs::IliadItem& item);
    void perform_unwrap();
    
    void go_to_homing_position();
    void go_to_emergency();

    void update_report(const int32_t& new_status, const int32_t& item_id=-1, const std::string& item_name="");
    
    orunav_msgs::ManipulatorReport current_report;
    
    void from_IliadItem_to_KDLFrame(const geometry_msgs::Point& in, const int32_t rotation_type, KDL::Frame& out);    
    void from_KDLFrame_to_PoseRPY(const KDL::Frame& in, lwr_controllers::PoseRPY& out);
    void from_KDLVector_to_PoseRPY_position(const KDL::Vector& in, lwr_controllers::PoseRPY::_position_type& out);
    void from_KDLRotation_to_PoseRPY_rotation(const KDL::Rotation& in, lwr_controllers::PoseRPY::_orientation_type& out);
};

#endif