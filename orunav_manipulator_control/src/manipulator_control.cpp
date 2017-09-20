#include "orunav_manipulator_control/manipulator_control.h"
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

void manipulatorControl::from_KDLFrame_to_PoseRPY(const KDL::Frame& in, lwr_controllers::PoseRPY& out)
{
//TODO
}

void manipulatorControl::from_ManipulatorCommand_to_KDLFrame(const orunav_msgs::ManipulatorCommand& in, KDL::Frame& out)
{
//TODO
}

void manipulatorControl::process_manipulator_command(const orunav_msgs::ManipulatorCommand::ConstPtr& msg)
{
    switch(msg->cmd)
    {
	case orunav_msgs::ManipulatorCommand::MANIPULATOR_LOAD:
	    ROS_WARN("command to be implemented");
	    break;
	case orunav_msgs::ManipulatorCommand::MANIPULATOR_UNLOAD:
	    ROS_INFO_STREAM(">> " << "Sent Command: UNLOAD");
	    send_target();
	    break;
	case orunav_msgs::ManipulatorCommand::MANIPULATOR_UNWRAP:
	    ROS_WARN("command to be implemented");
	    break;
	case orunav_msgs::ManipulatorCommand::MANIPULATOR_GOTO_IDLE:
	    ROS_WARN("command to be implemented");
	    break;
	case orunav_msgs::ManipulatorCommand::MANIPULATOR_GOTO_HOME:
	    ROS_WARN("command to be implemented");
	    break;
	default:
	    ROS_ERROR_STREAM(msg->cmd << " is not a valid command...");
	    break;
    }
}

void manipulatorControl::send_target()
{
    std::string base_link = "/robot1/vito_anchor";
    std::string right_hand_link = "/robot1/right_hand_palm_link";
    std::string veltet_tray_link = "/robot1/gripper_link";

    KDL::Frame pallet_T_base; //NOTE supposing a fixed pallet notation
    pallet_T_base = KDL::Frame::Identity();
    pallet_T_base.p.x(1.0);
    pallet_T_base.p.z(-0.2);

    tf::StampedTransform base_TF_right_hand;
    tf::StampedTransform base_TF_velvet_tray;
    KDL::Frame base_T_right_hand;
    KDL::Frame base_T_velvet_tray;
    KDL::Frame base_T_right_hand_desired;
    KDL::Frame base_T_velvet_tray_desired;
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

    lwr_controllers::PoseRPY right_hand_cmd;
    lwr_controllers::PoseRPY veltet_tray_cmd;

    KDL::Frame pallet_T_object;
    from_ManipulatorCommand_to_KDLFrame(last_cmd,pallet_T_object);

    base_T_right_hand_desired  = (pallet_T_base.Inverse()) * pallet_T_object;
    base_T_velvet_tray_desired = (pallet_T_base.Inverse()) * pallet_T_object;;
    
    from_KDLFrame_to_PoseRPY(base_T_right_hand_desired,right_hand_cmd);
    from_KDLFrame_to_PoseRPY(base_T_velvet_tray_desired,veltet_tray_cmd);

    cmd_pub_right.publish(right_hand_cmd);
    cmd_pub_left.publish(veltet_tray_cmd);

    //TODO update report
}

manipulatorControl::~manipulatorControl()
{

}
