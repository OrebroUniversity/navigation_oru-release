#include "orunav_manipulator_control/manipulator_control.h"
#include <tf_conversions/tf_kdl.h>
#include <string>

manipulatorControl::manipulatorControl()
{
    current_report.status = orunav_msgs::ManipulatorReport::MANIPULATOR_GOTO_IDLE_DONE;

    report_timer = nh.createTimer(ros::Duration(ros::Rate(10)),&manipulatorControl::publish_report,this);
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
    from_KDLVector_to_PoseRPY_position(in.p,out.position);
    from_KDLRotation_to_PoseRPY_rotation(in.M,out.orientation);
}

void manipulatorControl::from_KDLVector_to_PoseRPY_position(const KDL::Vector& in, lwr_controllers::PoseRPY::_position_type& out)
{
    out.x = in.x();
    out.y = in.y();
    out.z = in.z();
}

void manipulatorControl::from_KDLRotation_to_PoseRPY_rotation(const KDL::Rotation& in, lwr_controllers::PoseRPY::_orientation_type& out)
{
    in.GetRPY(out.roll,out.pitch,out.yaw);
}

void manipulatorControl::extract_unload_pose_from_ManipulatorCommand(const orunav_msgs::ManipulatorCommand::ConstPtr& in, KDL::Frame& out)
{
    tf::poseMsgToKDL(in->item_pose,out);
}

void manipulatorControl::process_manipulator_command(const orunav_msgs::ManipulatorCommand::ConstPtr& msg)
{
    switch(msg->cmd)
    {
	case orunav_msgs::ManipulatorCommand::MANIPULATOR_LOAD:
	    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> Sent Command: LOAD");
	    update_report(orunav_msgs::ManipulatorReport::MANIPULATOR_LOAD_ITEM_START);
	    perform_load(msg);
	    update_report(orunav_msgs::ManipulatorReport::MANIPULATOR_LOAD_ITEM_DONE);
	    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> Done Command: LOAD");
	    break;
	case orunav_msgs::ManipulatorCommand::MANIPULATOR_UNLOAD:
	    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> Sent Command: UNLOAD");
	    update_report(orunav_msgs::ManipulatorReport::MANIPULATOR_UNLOAD_ITEM_START);
	    perform_unload(msg);
	    update_report(orunav_msgs::ManipulatorReport::MANIPULATOR_UNLOAD_ITEM_DONE);
	    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> Done Command: UNLOAD");
	    break;
	case orunav_msgs::ManipulatorCommand::MANIPULATOR_UNWRAP:
	    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> Sent Command: UNWRAP");
	    update_report(orunav_msgs::ManipulatorReport::MANIPULATOR_UNWRAP_PALLET_START);
	    perform_unwrap(msg);
	    update_report(orunav_msgs::ManipulatorReport::MANIPULATOR_UNWRAP_PALLET_DONE);
	    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> Done Command: UNWRAP");
	    break;
	case orunav_msgs::ManipulatorCommand::MANIPULATOR_GOTO_IDLE:
	    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> Sent Command: IDLE");
	    update_report(orunav_msgs::ManipulatorReport::MANIPULATOR_GOTO_IDLE_START);
	    perform_idle(msg);
	    update_report(orunav_msgs::ManipulatorReport::MANIPULATOR_GOTO_IDLE_DONE);
	    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> Done Command: IDLE");
	    break;
	case orunav_msgs::ManipulatorCommand::MANIPULATOR_GOTO_HOME:
	    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> Sent Command: HOME");
	    update_report(orunav_msgs::ManipulatorReport::MANIPULATOR_GOTO_HOME_START);
	    perform_homing(msg);
	    update_report(orunav_msgs::ManipulatorReport::MANIPULATOR_GOTO_HOME_DONE);
	    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> Done Command: HOME");
	    break;
	default:
	    ROS_ERROR_STREAM("[MANIPULATOR CONTROL] "<<msg->cmd << " is not a valid command...");
	    break;
    }
}

void manipulatorControl::update_ee_transformations(KDL::Frame& base_T_right_hand, KDL::Frame& base_T_velvet_tray)
{
    std::string base_link = "/robot1/vito_anchor";
    std::string right_hand_link = "/robot1/right_hand_palm_link";
    std::string veltet_tray_link = "/robot1/gripper_link";
    
    tf::StampedTransform base_TF_right_hand;
    tf::StampedTransform base_TF_velvet_tray;
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
}

void manipulatorControl::perform_load(const orunav_msgs::ManipulatorCommand::ConstPtr& cmd)
{
    //NOTE here we should have information from perception

    KDL::Frame pallet_T_base; //supposing a fixed pallet position
    pallet_T_base = KDL::Frame::Identity();
    pallet_T_base.p.x(1.0);
    pallet_T_base.p.z(-0.4);
    
    KDL::Frame base_T_right_hand;
    KDL::Frame base_T_velvet_tray;
    KDL::Frame base_T_right_hand_desired;
    KDL::Frame base_T_velvet_tray_desired;

    update_ee_transformations(base_T_right_hand,base_T_velvet_tray);

    lwr_controllers::PoseRPY right_hand_cmd;
    lwr_controllers::PoseRPY veltet_tray_cmd;

    KDL::Frame pallet_T_object;
    
    //supposing the loading pose come from perception
    pallet_T_object = KDL::Frame::Identity();
    pallet_T_object.p.x(0.1);

    base_T_right_hand_desired  = (pallet_T_base.Inverse()) * pallet_T_object;
    base_T_velvet_tray_desired = (pallet_T_base.Inverse()) * pallet_T_object;;
    
    // for now we maintain the same orientation as the starting pose
    from_KDLRotation_to_PoseRPY_rotation(base_T_right_hand.M,right_hand_cmd.orientation);
    from_KDLRotation_to_PoseRPY_rotation(base_T_velvet_tray.M,veltet_tray_cmd.orientation);
    
    from_KDLVector_to_PoseRPY_position(base_T_right_hand_desired.p,right_hand_cmd.position);
    right_hand_cmd.position.z = right_hand_cmd.position.z + 0.1; //to simulate object size
    from_KDLVector_to_PoseRPY_position(base_T_velvet_tray_desired.p,veltet_tray_cmd.position);

    cmd_pub_right.publish(right_hand_cmd);
    cmd_pub_left.publish(veltet_tray_cmd);
    
    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> loading item...");
    
    ros::Time start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(1,500000000))
    {
	ros::spinOnce();
	usleep(10);
    }
    
    //stopping arms
    right_hand_cmd.id=-1;
    veltet_tray_cmd.id=-1;
    cmd_pub_right.publish(right_hand_cmd);
    cmd_pub_left.publish(veltet_tray_cmd);
}

void manipulatorControl::perform_unload(const orunav_msgs::ManipulatorCommand::ConstPtr& cmd)
{
    //NOTE here the unload pose comes from the vehicle execution

    KDL::Frame pallet_T_base; //supposing a fixed pallet position
    pallet_T_base = KDL::Frame::Identity();
    pallet_T_base.p.x(1.0);
    pallet_T_base.p.z(-0.2);

    KDL::Frame base_T_right_hand;
    KDL::Frame base_T_velvet_tray;
    KDL::Frame base_T_right_hand_desired;
    KDL::Frame base_T_velvet_tray_desired;

    update_ee_transformations(base_T_right_hand,base_T_velvet_tray);

    lwr_controllers::PoseRPY right_hand_cmd;
    lwr_controllers::PoseRPY veltet_tray_cmd;

    KDL::Frame pallet_T_object;
    extract_unload_pose_from_ManipulatorCommand(cmd,pallet_T_object);

    base_T_right_hand_desired  = (pallet_T_base.Inverse()) * pallet_T_object;
    base_T_velvet_tray_desired = (pallet_T_base.Inverse()) * pallet_T_object;;
    
    // for now we maintain the same orientation as the starting pose
    from_KDLRotation_to_PoseRPY_rotation(base_T_right_hand.M,right_hand_cmd.orientation);
    from_KDLRotation_to_PoseRPY_rotation(base_T_velvet_tray.M,veltet_tray_cmd.orientation);
    
    from_KDLVector_to_PoseRPY_position(base_T_right_hand_desired.p,right_hand_cmd.position);
    right_hand_cmd.position.z = right_hand_cmd.position.z + 0.1; //to simulate object size
    from_KDLVector_to_PoseRPY_position(base_T_velvet_tray_desired.p,veltet_tray_cmd.position);

    cmd_pub_right.publish(right_hand_cmd);
    cmd_pub_left.publish(veltet_tray_cmd);
    
    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> unloading item...");
    
    ros::Time start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(2,0))
    {
	ros::spinOnce();
	usleep(10);
    }

    //stopping arms
    right_hand_cmd.id=-1;
    veltet_tray_cmd.id=-1;
    cmd_pub_right.publish(right_hand_cmd);
    cmd_pub_left.publish(veltet_tray_cmd);
}

void manipulatorControl::perform_unwrap(const orunav_msgs::ManipulatorCommand::ConstPtr& cmd)
{
    //NOTE here we should have information from perception

    KDL::Frame pallet_T_base; //supposing a fixed pallet position
    pallet_T_base = KDL::Frame::Identity();
    pallet_T_base.p.x(1.0);
    pallet_T_base.p.z(-0.4);
    
    KDL::Frame base_T_right_hand;
    KDL::Frame base_T_velvet_tray;
    KDL::Frame base_T_right_hand_desired;
    KDL::Frame base_T_velvet_tray_desired;

    update_ee_transformations(base_T_right_hand,base_T_velvet_tray);

    lwr_controllers::PoseRPY right_hand_cmd;
    lwr_controllers::PoseRPY veltet_tray_cmd;

    KDL::Frame pallet_T_object;
    
    //supposing the loading pose come from perception
    pallet_T_object = KDL::Frame::Identity();

    base_T_right_hand_desired  = (pallet_T_base.Inverse()) * pallet_T_object;
    base_T_velvet_tray_desired = (pallet_T_base.Inverse()) * pallet_T_object;;
    
    // for now we maintain the same orientation as the starting pose
    from_KDLRotation_to_PoseRPY_rotation(base_T_right_hand.M,right_hand_cmd.orientation);
    from_KDLRotation_to_PoseRPY_rotation(base_T_velvet_tray.M,veltet_tray_cmd.orientation);
    
    from_KDLVector_to_PoseRPY_position(base_T_right_hand_desired.p,right_hand_cmd.position);
    right_hand_cmd.position.z = right_hand_cmd.position.z + 0.3;
    right_hand_cmd.position.y = right_hand_cmd.position.y + 0.3;

    from_KDLVector_to_PoseRPY_position(base_T_velvet_tray_desired.p,veltet_tray_cmd.position);
    veltet_tray_cmd.position.y = veltet_tray_cmd.position.y - 0.6;

    cmd_pub_right.publish(right_hand_cmd);
    cmd_pub_left.publish(veltet_tray_cmd);
    
    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> unwrapping pallet...");
    
    ros::Time start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(3,0))
    {
	ros::spinOnce();
	usleep(10);
    }
    
    right_hand_cmd.position.z = right_hand_cmd.position.z - 0.6;

    cmd_pub_right.publish(right_hand_cmd);
    
    start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(1,500000000))
    {
	ros::spinOnce();
	usleep(10);
    }

    //stopping arms
    right_hand_cmd.id=-1;
    veltet_tray_cmd.id=-1;
    cmd_pub_right.publish(right_hand_cmd);
    cmd_pub_left.publish(veltet_tray_cmd);
}

void manipulatorControl::perform_homing(const orunav_msgs::ManipulatorCommand::ConstPtr& cmd)
{
    lwr_controllers::PoseRPY right_hand_cmd;
    lwr_controllers::PoseRPY veltet_tray_cmd;
    right_hand_cmd.id=-2;
    veltet_tray_cmd.id=-2;
    cmd_pub_right.publish(right_hand_cmd);
    cmd_pub_left.publish(veltet_tray_cmd);
    
    ros::Time start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(1,500000000))
    {
	ros::spinOnce();
	usleep(10);
    }

    //stopping arms
    right_hand_cmd.id=-1;
    veltet_tray_cmd.id=-1;
    cmd_pub_right.publish(right_hand_cmd);
    cmd_pub_left.publish(veltet_tray_cmd);
}

void manipulatorControl::perform_idle(const orunav_msgs::ManipulatorCommand::ConstPtr& cmd)
{
    lwr_controllers::PoseRPY right_hand_cmd;
    lwr_controllers::PoseRPY veltet_tray_cmd;
    right_hand_cmd.id=-1;
    veltet_tray_cmd.id=-1;
    cmd_pub_right.publish(right_hand_cmd);
    cmd_pub_left.publish(veltet_tray_cmd);
    
    ros::Time start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(1,500000000))
    {
	ros::spinOnce();
	usleep(10);
    }
}

void manipulatorControl::update_report(const int32_t& new_status)
{
    report_mutex.lock();
    current_report.status = new_status;
    report_mutex.unlock();
}

manipulatorControl::~manipulatorControl()
{

}
