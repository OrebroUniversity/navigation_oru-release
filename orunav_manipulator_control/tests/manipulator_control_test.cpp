#include <ros/ros.h>
#include "orunav_msgs/ManipulatorCommand.h"
#include "orunav_msgs/ManipulatorReport.h"
#include "orunav_msgs/IliadItemArray.h"

std::string str_status;

void report_callback(const orunav_msgs::ManipulatorReportConstPtr& report)
{
    if(report->status==orunav_msgs::ManipulatorReport::NOT_AVAILABLE) str_status="NOT_AVAILABLE";
    if(report->status==orunav_msgs::ManipulatorReport::IDLE) str_status="IDLE";
    if(report->status==orunav_msgs::ManipulatorReport::LOADING_ITEM) str_status="LOADING_ITEM (" + std::to_string(report->item_id) + ") - " + report->item_name;
    if(report->status==orunav_msgs::ManipulatorReport::UNLOADING_ITEM) str_status="UNLOADING_ITEM (" + std::to_string(report->item_id) + ") - " + report->item_name;
    if(report->status==orunav_msgs::ManipulatorReport::UNWRAP_PHASE_1) str_status="UNWRAP_PHASE_1";
    if(report->status==orunav_msgs::ManipulatorReport::UNWRAP_PHASE_2) str_status="UNWRAP_PHASE_2";
    if(report->status==orunav_msgs::ManipulatorReport::HOMING) str_status="HOMING";
    if(report->status==orunav_msgs::ManipulatorReport::FAILURE) str_status="FAILURE";

    std::cout<<"status: "<<str_status<<std::endl;
}

int main(int argc, char **argv)
{
    if(!ros::isInitialized())
    {
	ros::init(argc,argv,"manipulator_control_test");
    }
  
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<orunav_msgs::ManipulatorCommand>("/robot1/manipulator/command",10);
    ros::Subscriber sub = nh.subscribe("/robot1/manipulator/report",10,&report_callback);
    
    orunav_msgs::ManipulatorCommand cmd;
    
    cmd.robot_id = 1;
    cmd.cmd = 1;

    while(str_status!="IDLE")
    {
	ros::spinOnce();
	usleep(10);
    }

    orunav_msgs::IliadItem item;

//     item.name="Hallonsoppa";
//     item.position.x=0;
//     item.position.y=-30;
//     item.position.z=-10;
//     item.rotation_type=0;
//     cmd.item_list.items.push_back(item);
// 
//     item.name="Hallonsoppa";
//     item.position.x=0;
//     item.position.y=0;
//     item.position.z=-10;
//     item.rotation_type=0;
//     cmd.item_list.items.push_back(item);
//     
//     item.name="Hallonsoppa";
//     item.position.x=0;
//     item.position.y=30;
//     item.position.z=-10;
//     item.rotation_type=0;
//     cmd.item_list.items.push_back(item);
    
    item.name="Jacky";
    item.position.x=0;
    item.position.y=-30;
    item.position.z=10;
    item.rotation_type=0;
    cmd.item_list.items.push_back(item);

    item.name="Jacky";
    item.position.x=0;
    item.position.y=0;
    item.position.z=10;
    item.rotation_type=0;
    cmd.item_list.items.push_back(item);
    
    item.name="Jacky";
    item.position.x=0;
    item.position.y=30;
    item.position.z=10;
    item.rotation_type=0;
    cmd.item_list.items.push_back(item);
    
    pub.publish(cmd);

    while(str_status!="IDLE")
    {
	ros::spinOnce();
	usleep(10);
    }
    
    ros::spin();

    return 0;
}
