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

int main(int argc, char **argv)
{
  int robot_id_1, robot_id_2; 
	ros::init(argc, argv, "drive_path_node");
  ros::NodeHandle nh_;
  nh_.param<int>("robot_id1", robot_id_1, 1);
  nh_.param<int>("robot_id2", robot_id_2, 2);
  string file_name_1, file_name_2;
      po::options_description desc("Allowed options");
      desc.add_options()
          ("help", "produce help message")
          ("debug", "print debug output")
          ("fileName1", po::value<string>(&file_name_1)->required(), "file to be used 1")
          ("fileName2", po::value<string>(&file_name_2)->required(), "file to be used 2")
          ;

     po::variables_map vm;
     po::store(po::parse_command_line(argc, argv, desc), vm);
     po::notify(vm);
    cout << "Loading : " << file_name_1 << endl;
    cout << "Loading : " << file_name_2 << endl;
    orunav_generic::Path loaded_path_1 = orunav_generic::loadPathTextFile(file_name_1);
    orunav_generic::Path loaded_path_2 = orunav_generic::loadPathTextFile(file_name_2);
    cout << "Loaded file 1 sizes : " << loaded_path_1.sizePath() << endl;
    cout << "Loaded file 2 sizes : " << loaded_path_2.sizePath() << endl;
  	if (loaded_path_1.sizePath() == 0) {
        cout << "no points in file 1 - exiting" << endl;
        exit(-1);
    }
    if (loaded_path_2.sizePath() == 0) {
        cout << "no points in file 2 - exiting" << endl;
        exit(-1);
    }

    cout << "First pose file 1: " << loaded_path_1.getPose2d(0) << endl;
    cout << "First pose file 2: " << loaded_path_2.getPose2d(0) << endl;

    boost::shared_ptr<orunav_msgs::ControllerReport const> msg, msg_2;
    orunav_msgs::ControllerReport cr, cr_2;

    msg  = ros::topic::waitForMessage<orunav_msgs::ControllerReport>(orunav_generic::getRobotTopicName(robot_id_1, "/controller/reports"), nh_);
    msg_2  = ros::topic::waitForMessage<orunav_msgs::ControllerReport>(orunav_generic::getRobotTopicName(robot_id_2, "/controller/reports"), nh_);
    if (msg != NULL) {
        cr = *msg;
    }
    if (msg_2 != NULL) {
        cr_2 = *msg_2;
    }
    orunav_generic::State2d state = orunav_conversions::createState2dFromControllerStateMsg(msg->state);
    orunav_generic::Pose2d pose = orunav_conversions::createPose2dFromControllerStateMsg(msg->state);
    
    cout<<"Pose from topic 1: " << pose(0)<<" "<< pose(1) << " " << pose(2) << endl;
    //cout<<"Pose from topic: " << pose.getPose2d(0) << endl;
    orunav_generic::setFirstPoseAsOrigin(loaded_path_1);
    cout << "First pose (after origin transformation) 1 : " << loaded_path_1.getPose2d(0) << endl;
    cout << "Second pose (after origin transformation) 1 : " << loaded_path_1.getPose2d(1) << endl;
    orunav_generic::savePathTextFile(loaded_path_1,"path_1_ori.txt");

    orunav_generic::addPose2dOffset(loaded_path_1, pose);
    //orunav_generic::moveToOrigin(loaded_path, pose);

    cout << "First pose (after offset transformation) 1 : " << loaded_path_1.getPose2d(0) << endl;
    cout << "Second pose (after offset transformation) 1 : " << loaded_path_1.getPose2d(1) << endl;
    orunav_generic::savePathTextFile(loaded_path_1,"path_1_off.txt");
        
    //control_report_sub_ = nh_.subscribe<orunav_msgs::ControllerReport>(orunav_generic::getRobotTopicName(robot_id_, "/controller/reports"), 10,drive_pathCB);
  	orunav_generic::Pose2d pose_2 = orunav_conversions::createPose2dFromControllerStateMsg(msg_2->state);	
    cout<<"Pose from topic 2: " << pose_2(0)<<" "<< pose_2(1) << " " << pose_2(2) << endl;
    orunav_generic::setFirstPoseAsOrigin(loaded_path_2);
    cout << "First pose (after origin transformation) 2 : " << loaded_path_2.getPose2d(0) << endl;
    cout << "Second pose (after origin transformation) 2 : " << loaded_path_2.getPose2d(1) << endl;
    orunav_generic::savePathTextFile(loaded_path_2,"path_2_ori.txt");

    orunav_generic::addPose2dOffset(loaded_path_2, pose_2);

    cout << "First pose (after offset transformation) 2 : " << loaded_path_2.getPose2d(0) << endl;
    cout << "Second pose (after offset transformation) 2 : " << loaded_path_2.getPose2d(1) << endl;
    orunav_generic::savePathTextFile(loaded_path_2,"path_2_off.txt");

  	orunav_msgs::Task task, task_2;
    task.target.start = orunav_conversions::createPoseSteeringMsgFromState2d(loaded_path_1.getState2d(0));
    task.target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(loaded_path_1.getState2d(loaded_path_1.sizePath()-1));
    task.target.start_op.operation = task.target.start_op.NO_OPERATION;
  	task.target.goal_op.operation = task.target.goal_op.NO_OPERATION;
    task.target.current_load.status = task.target.current_load.EMPTY;
    task.target.goal_load.status = task.target.goal_load.EMPTY;
     
    task.target.robot_id = robot_id_1;
  	task.target.task_id = 1;
    task.criticalPoint = -1;  
  	task.path = orunav_conversions::createPathMsgFromPathInterface(loaded_path_1);
  	task.update = false;
  	task.abort = false;

    task_2.target.start = orunav_conversions::createPoseSteeringMsgFromState2d(loaded_path_2.getState2d(0));
    task_2.target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(loaded_path_2.getState2d(loaded_path_2.sizePath()-1));
    task_2.target.start_op.operation = task.target.start_op.NO_OPERATION;
    task_2.target.goal_op.operation = task.target.goal_op.NO_OPERATION;
    task_2.target.current_load.status = task.target.current_load.EMPTY;
    task_2.target.goal_load.status = task.target.goal_load.EMPTY;
     
    task_2.target.robot_id = robot_id_2;
    task_2.target.task_id = 1;
    task_2.criticalPoint = -1;  
    task_2.path = orunav_conversions::createPathMsgFromPathInterface(loaded_path_2);
    task_2.update = false;
    task_2.abort = false;

    ros::ServiceClient client_1 = nh_.serviceClient<orunav_msgs::ExecuteTask>(orunav_generic::getRobotTopicName(robot_id_1,"/execute_task"));
    ros::ServiceClient client_2 = nh_.serviceClient<orunav_msgs::ExecuteTask>(orunav_generic::getRobotTopicName(robot_id_2,"/execute_task"));
    orunav_msgs::ExecuteTask srv_1, srv_2;
    srv_1.request.task = task;
    srv_2.request.task = task_2;

    if (client_1.call(srv_1)) {
      ROS_INFO("[DrivePathExecutionClientNode] - execute_task sucessfull for 1");
    }
    else
    {
      ROS_ERROR("[DrivePathExecutionClientNode] - Failed to call service: execute_task for 1");
      return 1;
    }

    if (client_2.call(srv_2)) {
      ROS_INFO("[DrivePathExecutionClientNode] - execute_task sucessfull for 2");
    }
    else
    {
      ROS_ERROR("[DrivePathExecutionClientNode] - Failed to call service: execute_task for 2");
      return 1;
    }
  ros::spin();

	return 0;
}