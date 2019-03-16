#include "orunav_manipulator_control/manipulator_control.h"
#include <tf_conversions/tf_kdl.h>
#include <string>

manipulatorControl::manipulatorControl()
{
    current_report.status = orunav_msgs::ManipulatorReport::IDLE;

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

double cm2m(const double& cms)
{
    return cms/100.0;
}

void manipulatorControl::from_IliadItem_to_KDLFrame(const geometry_msgs::Point& in, const int32_t rotation_type, KDL::Frame& out)
{
    out.p.x(cm2m(in.x));
    out.p.y(cm2m(in.y));
    out.p.z(cm2m(in.z));

    double roll=0.0;
    double pitch=0.0;
    double yaw=0.0;

    switch(rotation_type) //TODO
    {
	case orunav_msgs::IliadItem::NONE:
	break;
	case orunav_msgs::IliadItem::X:
	break;
	case orunav_msgs::IliadItem::Y:
	break;
	case orunav_msgs::IliadItem::Z:
	break;
	case orunav_msgs::IliadItem::XZ:
	break;
	case orunav_msgs::IliadItem::ZX:
	break;
	default:
	break;
    }

    out.M = KDL::Rotation::RPY(roll,pitch,yaw);
}

void manipulatorControl::process_manipulator_command(const orunav_msgs::ManipulatorCommand::ConstPtr& msg)
{
    switch(msg->cmd)
    {
      case orunav_msgs::ManipulatorCommand::PICK_ITEMS:
	    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> Sent Command: PICK_ITEMS");
	    perform_pick_items(msg);
	    go_to_homing_position();
	    go_to_emergency(); //NOTE to ensure stop
	    update_report(orunav_msgs::ManipulatorReport::IDLE);
	    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> Done Command: PICK_ITEMS");
	    break;
	case orunav_msgs::ManipulatorCommand::UNWRAP:
	    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> Sent Command: UNWRAP");
	    perform_unwrap();
	    go_to_homing_position();
	    go_to_emergency(); //NOTE to ensure stop
	    update_report(orunav_msgs::ManipulatorReport::IDLE);
	    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> Done Command: UNWRAP");
	    break;
	case orunav_msgs::ManipulatorCommand::NO_OPERATION:
	    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> Sent Command: NO_OPERATION");
	    go_to_emergency();
	    update_report(orunav_msgs::ManipulatorReport::IDLE);
	    ROS_INFO_STREAM("[MANIPULATOR CONTROL] >> Done Command: NO_OPERATION");
	    break;
	default:
	    ROS_ERROR_STREAM("[MANIPULATOR CONTROL] "<<msg->cmd << " is not a valid command...");
	    break;
    }
}

void manipulatorControl::update_ee_transformations()
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

void manipulatorControl::perform_pick_items(const orunav_msgs::ManipulatorCommand_< std::allocator< void > >::ConstPtr& cmd)
{
    update_ee_transformations();

    int i=0;
    for(auto item:cmd->item_list.items)
    {
	update_report(orunav_msgs::ManipulatorReport::LOADING_ITEM,i,item.name);
      
	load_item(item);

	update_report(orunav_msgs::ManipulatorReport::UNLOADING_ITEM,i,item.name);

	unload_item(item);

	i++;
    }
}

void manipulatorControl::load_item(const orunav_msgs::IliadItem& item)
{
    //NOTE supposing a fixed pallet position w.r.t. the robot
    KDL::Frame pallet_T_base;
    pallet_T_base = KDL::Frame::Identity();
    pallet_T_base.p.x(1.2);
    pallet_T_base.p.z(-0.5);

    KDL::Frame base_T_right_hand_desired;
    KDL::Frame base_T_velvet_tray_desired;

    lwr_controllers::PoseRPY right_hand_cmd;
    lwr_controllers::PoseRPY veltet_tray_cmd;

    KDL::Frame pallet_T_object;
    
    //NOTE here we should have information from perception
    pallet_T_object = KDL::Frame::Identity();

    base_T_right_hand_desired  = (pallet_T_base.Inverse()) * pallet_T_object;
    base_T_velvet_tray_desired = (pallet_T_base.Inverse()) * pallet_T_object;
    
    // for now we maintain the same orientation as the starting pose
    from_KDLRotation_to_PoseRPY_rotation(base_T_right_hand.M,right_hand_cmd.orientation);
    from_KDLRotation_to_PoseRPY_rotation(base_T_velvet_tray.M,veltet_tray_cmd.orientation);
    
    from_KDLVector_to_PoseRPY_position(base_T_right_hand_desired.p,right_hand_cmd.position);
    from_KDLVector_to_PoseRPY_position(base_T_velvet_tray_desired.p,veltet_tray_cmd.position);
    
    //to simulate object size
    if(item.name=="Hallonsoppa")
    {
	right_hand_cmd.position.z = veltet_tray_cmd.position.z + 0.2;
	right_hand_cmd.position.x = veltet_tray_cmd.position.x - 0.3;
	right_hand_cmd.orientation.pitch = M_PI/4.0;
    }
    
    if(item.name=="Jacky")
    {
	right_hand_cmd.position.z = veltet_tray_cmd.position.z + 0.2;
    }
    

    cmd_pub_right.publish(right_hand_cmd);
    cmd_pub_left.publish(veltet_tray_cmd);

    ros::Time start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(2,0))
    {
	ros::spinOnce();
	usleep(10);
    }
}

void manipulatorControl::unload_item(const orunav_msgs::IliadItem& item)
{
    //NOTE here the unload pose comes from the vehicle execution

    KDL::Frame pallet_T_base; //supposing a fixed pallet position
    pallet_T_base = KDL::Frame::Identity();
    pallet_T_base.p.x(1.0);
    pallet_T_base.p.z(-0.2);

    KDL::Frame base_T_right_hand_desired;
    KDL::Frame base_T_velvet_tray_desired;

    lwr_controllers::PoseRPY right_hand_cmd;
    lwr_controllers::PoseRPY veltet_tray_cmd;

    KDL::Frame pallet_T_object;
    from_IliadItem_to_KDLFrame(item.position,item.rotation_type,pallet_T_object);

    base_T_right_hand_desired  = (pallet_T_base.Inverse()) * pallet_T_object;
    base_T_velvet_tray_desired = (pallet_T_base.Inverse()) * pallet_T_object;
    
    // for now we maintain the same orientation as the starting pose
    from_KDLRotation_to_PoseRPY_rotation(base_T_right_hand.M,right_hand_cmd.orientation);
    from_KDLRotation_to_PoseRPY_rotation(base_T_velvet_tray.M,veltet_tray_cmd.orientation);
    
    from_KDLVector_to_PoseRPY_position(base_T_right_hand_desired.p,right_hand_cmd.position);
    from_KDLVector_to_PoseRPY_position(base_T_velvet_tray_desired.p,veltet_tray_cmd.position);

    //to simulate object size
    if(item.name=="Hallonsoppa")
    {
	right_hand_cmd.position.z = veltet_tray_cmd.position.z + 0.2;
	right_hand_cmd.position.x = veltet_tray_cmd.position.x - 0.3;
	right_hand_cmd.orientation.pitch = M_PI/4.0;
    }
    
    if(item.name=="Jacky")
    {
	right_hand_cmd.position.z = veltet_tray_cmd.position.z + 0.2;
    }

    cmd_pub_right.publish(right_hand_cmd);
    cmd_pub_left.publish(veltet_tray_cmd);
    
    ros::Time start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(2,0))
    {
	ros::spinOnce();
	usleep(10);
    }
}

void manipulatorControl::perform_unwrap()
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

    update_ee_transformations();

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

    update_report(orunav_msgs::ManipulatorReport::UNWRAP_PHASE_1);
    
    cmd_pub_right.publish(right_hand_cmd);
    cmd_pub_left.publish(veltet_tray_cmd);
    
    ros::Time start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(2,0))
    {
	ros::spinOnce();
	usleep(10);
    }
    
    right_hand_cmd.position.z = right_hand_cmd.position.z - 0.6;

    update_report(orunav_msgs::ManipulatorReport::UNWRAP_PHASE_2);

    cmd_pub_right.publish(right_hand_cmd);
    
    start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(1,500000000))
    {
	ros::spinOnce();
	usleep(10);
    }
}

void manipulatorControl::go_to_homing_position()
{
    lwr_controllers::PoseRPY right_hand_cmd;
    lwr_controllers::PoseRPY veltet_tray_cmd;
    right_hand_cmd.id=-2;
    veltet_tray_cmd.id=-2;
    cmd_pub_right.publish(right_hand_cmd);
    cmd_pub_left.publish(veltet_tray_cmd);
    
    update_report(orunav_msgs::ManipulatorReport::HOMING);

    ros::Time start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(1,500000000))
    {
	ros::spinOnce();
	usleep(10);
    }
}

void manipulatorControl::go_to_emergency()
{
    lwr_controllers::PoseRPY right_hand_cmd;
    lwr_controllers::PoseRPY veltet_tray_cmd;
    right_hand_cmd.id=-1;
    veltet_tray_cmd.id=-1;
    cmd_pub_right.publish(right_hand_cmd);
    cmd_pub_left.publish(veltet_tray_cmd);
}

void manipulatorControl::update_report(const int32_t& new_status, const int32_t& item_id, const std::string& item_name)
{
    report_mutex.lock();
    current_report.status = new_status;
    current_report.item_id = item_id;
    current_report.item_name = item_name;
    report_mutex.unlock();
}

manipulatorControl::~manipulatorControl()
{

}
