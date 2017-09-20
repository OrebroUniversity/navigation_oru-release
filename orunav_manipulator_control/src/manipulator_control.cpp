#include "orunav_manipulator_control/manipulator_control.h"
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>
#include <string>

manipulatorControl::manipulatorControl()
{
    report_timer = nh.createTimer(ros::Duration(0.1),&manipulatorControl::publish_report,this);
    report_pub = nh.advertise<orunav_msgs::ManipulatorReport>("manipulator/report", 1000);

    cmd_pub_right = nh.advertise<lwr_controllers::PoseRPY>("/right_arm/CLIK_controller/command",1);
    cmd_pub_left  = nh.advertise<lwr_controllers::PoseRPY>("/left_arm/CLIK_controller/command",1);
    
    command_sub = nh.subscribe<orunav_msgs::ManipulatorCommand>("manipulator/command",0,&manipulatorControl::process_manipulator_command,this);
}

void manipulatorControl::publish_report(const ros::TimerEvent& event)
{
    report_mutex.lock();
    current_report.stamp = ros::Time::now();
    report_pub.publish(current_report);
    report_mutex.unlock();
}

void manipulatorControl::from_ManipulatorCommand_to_PoseRPY(const orunav_msgs::ManipulatorCommand& in, lwr_controllers::PoseRPY out)
{
    //TODO
}

void manipulatorControl::process_manipulator_command(const orunav_msgs::ManipulatorCommand::ConstPtr& msg)
{
    //TODO

    send_target();
}

void manipulatorControl::send_target()
{
    std::string base_link = "/robot1/vito_anchor";
    std::string right_hand_link = "/robot1/right_hand_palm_link";
    std::string veltet_tray_link = "/robot1/gripper_link";

    KDL::Frame pallet_T_base;
    pallet_T_base = KDL::Frame::Identity();
    pallet_T_base.p.x(1.0);
    pallet_T_base.p.z(-0.2);

    tf::StampedTransform base_TF_right_hand;
    tf::StampedTransform base_TF_velvet_tray;
    KDL::Frame base_T_right_hand;
    KDL::Frame base_T_velvet_tray;
    std::string err_msg;

    //HACK ros::Duration(ros::DURATION_MAX) since otherwise it uses simulation time...
    if(listener.waitForTransform(base_link,right_hand_link,ros::Time::now(),ros::Duration(ros::DURATION_MAX),ros::Duration(0.01),&err_msg))
    {
	listener.lookupTransform(base_link,right_hand_link,ros::Time(0),base_TF_right_hand);
	tf::transformTFToKDL(base_TF_right_hand,base_T_right_hand);
    }
    else
    {
	ROS_ERROR_STREAM("abort for error in TF for right hand: " << err_msg);
	return;
    }

    //HACK ros::Duration(ros::DURATION_MAX) since otherwise it uses simulation time...
    if(listener.waitForTransform(base_link,veltet_tray_link,ros::Time::now(),ros::Duration(ros::DURATION_MAX),ros::Duration(0.01),&err_msg))
    {
	listener.lookupTransform(base_link,veltet_tray_link,ros::Time(0),base_TF_velvet_tray);
	tf::transformTFToKDL(base_TF_velvet_tray,base_T_velvet_tray);
    }
    else
    {
	ROS_ERROR_STREAM("abort for error in TF for velvet tray: " << err_msg);
	return;
    }

    lwr_controllers::PoseRPY right_hand_pose;
    lwr_controllers::PoseRPY veltet_tray_pose;

    from_ManipulatorCommand_to_PoseRPY(last_cmd,right_hand_pose);
    from_ManipulatorCommand_to_PoseRPY(last_cmd,veltet_tray_pose);

    cmd_pub_left.publish(veltet_tray_pose);
    cmd_pub_right.publish(right_hand_pose);

    //TODO update report

    ROS_INFO_STREAM(">> " << "Sent Target: TODO");
}

manipulatorControl::~manipulatorControl()
{

}
