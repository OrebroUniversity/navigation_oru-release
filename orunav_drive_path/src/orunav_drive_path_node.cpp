#include <ros/ros.h>

#include <boost/program_options.hpp>

#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>

#include <orunav_msgs/ExecuteTask.h>
#include <orunav_msgs/Task.h>
#include <orunav_msgs/ControllerReport.h>

#include <orunav_conversions/conversions.h>

#include <iostream>
#include <fstream>
#include <string>
#include <array>

namespace po = boost::program_options;
using namespace std;
/*bool GetMePath_server(orunav_msgs::GetMePath::Request &req,
		 			 orunav_msgs::GetMePath::Response &res)
{
	orunav_generic::Path path;

	std::array<std::array<int, 4>, 20> data;
	std::ifstream file("traj.txt");
	for (int i = 0; i < 20; ++i) {
    	for (int j = 0; j < 4; ++j) {
        	file >> data[i][j];
    	}
    	orunav_generic::State2d state(orunav_generic::Pose2d(data[i][0], data[i][1]
                                                             data[i][2]), data[i][3]);
       	path.addState2dInterface(state);
	}
	ROS_INFO_STREAM("[GetMePathService] - Nb of path points : " << path.sizePath());

}*/


int main(int argc, char **argv)
{
  int robot_id_;
	ros::init(argc, argv, "drive_path_node");
  ros::NodeHandle nh_;
  nh_.param<int>("robot_id", robot_id_, 1);
  //ros::Subscriber control_report_sub_;
  	string path_file_name, controller_reports_topic;
  		po::options_description desc("Allowed options");
     	desc.add_options()
       		("help", "produce help message")
       		("debug", "print debug output")
       		("fileName", po::value<string>(&path_file_name)->required(), "path file to be used")
       		("controller_reports_topic", po::value<string>(&controller_reports_topic)->required(), "controller/reports topic")
          ;

     po::variables_map vm;
     po::store(po::parse_command_line(argc, argv, desc), vm);
     po::notify(vm);
    cout << "Loading : " << path_file_name << endl;
    //orunav_generic::Path loaded_path = orunav_generic::loadPathTextFile("Path_2.txt");
    orunav_generic::Path loaded_path = orunav_generic::loadPathTextFile(path_file_name);
    cout << "Loaded path sizes : " << loaded_path.sizePath() << endl;
  	if (loaded_path.sizePath() == 0) {
        cout << "no points - exiting" << endl;
        exit(-1);
    }

    cout << "First pose : " << loaded_path.getPose2d(0) << endl;

//    boost::shared_ptr<orunav_msgs::ControllerReport const> msg;
//    orunav_msgs::ControllerReport cr;
//
//    cout << "Waiting for reports..." << std::endl;
//    msg  = ros::topic::waitForMessage<orunav_msgs::ControllerReport>(controller_reports_topic, nh_);
//    if (msg != NULL) {
//        cr = *msg;
//    }
//    cout << "Got controller reports msg" << std::endl;
//    orunav_generic::State2d state = orunav_conversions::createState2dFromControllerStateMsg(msg->state);
//    orunav_generic::Pose2d pose = orunav_conversions::createPose2dFromControllerStateMsg(msg->state);
//
//    cout<<"Pose from topic: " << pose(0)<<" "<< pose(1) << " " << pose(2) << endl;
//    //cout<<"Pose from topic: " << pose.getPose2d(0) << endl;
//    orunav_generic::setFirstPoseAsOrigin(loaded_path);
//    cout << "First pose (after origin transformation) : " << loaded_path.getPose2d(0) << endl;
//    cout << "Second pose (after origin transformation) : " << loaded_path.getPose2d(1) << endl;
//    orunav_generic::savePathTextFile(loaded_path,"path_ori.txt");
//
//    orunav_generic::addPose2dOffset(loaded_path, pose);
//    //orunav_generic::moveToOrigin(loaded_path, pose);
//
//    cout << "First pose (after offset transformation) : " << loaded_path.getPose2d(0) << endl;
//    cout << "Second pose (after offset transformation) : " << loaded_path.getPose2d(1) << endl;
//    orunav_generic::savePathTextFile(loaded_path,"path_off.txt");
        
    //control_report_sub_ = nh_.subscribe<orunav_msgs::ControllerReport>(orunav_generic::getRobotTopicName(robot_id_, "/controller/reports"), 10,drive_pathCB);
  		
  	orunav_msgs::Task task;
    task.target.start = orunav_conversions::createPoseSteeringMsgFromState2d(loaded_path.getState2d(0));
    task.target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(loaded_path.getState2d(loaded_path.sizePath()-1));
    task.target.start_op.operation = task.target.start_op.NO_OPERATION;
  	task.target.goal_op.operation = task.target.goal_op.NO_OPERATION;
    task.target.current_load.status = task.target.current_load.EMPTY;
    task.target.goal_load.status = task.target.goal_load.EMPTY;
     
    task.target.robot_id = robot_id_;
  	task.target.task_id = 1;
    task.criticalPoint = -1;  
  	task.path = orunav_conversions::createPathMsgFromPathInterface(loaded_path);
  	task.update = false;
  	task.abort = false;

    ros::ServiceClient client = nh_.serviceClient<orunav_msgs::ExecuteTask>(orunav_generic::getRobotTopicName(robot_id_,"/execute_task"));
    client.waitForExistence();
    orunav_msgs::ExecuteTask srv;
    srv.request.task = task;

    if (client.call(srv)) {
      ROS_INFO("[DrivePathExecutionClientNode] - execute_task successful");
    }
    else
    {
      ROS_ERROR("[DrivePathExecutionClientNode] - Failed to call service: execute_task");
      return 1;
    }
  ros::spin();

	return 0;
}
