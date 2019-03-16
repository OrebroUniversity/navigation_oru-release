#include <ros/ros.h>
#include "orunav_msgs/ManipulatorReport.h"

void report_callback(const orunav_msgs::ManipulatorReportConstPtr& report)
{
    std::string str_status="";

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
	ros::init(argc,argv,"manipulator_control_status_parser");
    }
  
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/robot1/manipulator/report",10,&report_callback);

    ros::spin();

    return 0;
}
