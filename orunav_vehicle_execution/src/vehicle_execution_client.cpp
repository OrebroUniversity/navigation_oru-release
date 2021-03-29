#include <ros/ros.h>


#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>

#include <orunav_conversions/conversions.h>

#include <std_srvs/Empty.h>

#include <orunav_msgs/ComputeTask.h>
#include <orunav_msgs/ExecuteTask.h>
#include <orunav_msgs/RobotTarget.h>
#include <orunav_msgs/RobotReport.h>

#include <orunav_vehicle_execution/io.h>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <boost/program_options.hpp>


namespace po = boost::program_options;

class VehicleExecutionClientNode
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber report_sub_;
  int robot_id_;
  int goal_id_inc_;
  std::string targets_file_name_;
  int task_id_;
  int nb_iters_;
  bool b_shutdown_;
  bool start_from_current_state_;
  
  boost::mutex input_mutex_;
  boost::thread client_thread_;

  orunav_msgs::RobotReport currentReport_;
  bool validReport_;
  bool execSent_;
  
  ros::ServiceServer service_next_task_;
  
public:
  VehicleExecutionClientNode(ros::NodeHandle paramHandle) : goal_id_inc_(0), validReport_(false), execSent_(false)
 {
   
   paramHandle.param<int>("robot_id", robot_id_, 1);
   paramHandle.param<std::string>("targets_file_name",targets_file_name_, std::string(""));
   paramHandle.param<int>("task_id", task_id_, -1);
   paramHandle.param<int>("nb_iters", nb_iters_, 0);
   paramHandle.param<bool>("start_from_current_state", start_from_current_state_, true);

   // Listen to the controll messages directly.
   report_sub_ = nh_.subscribe<orunav_msgs::RobotReport>(orunav_generic::getRobotTopicName(robot_id_, "/control/report"), 10,&VehicleExecutionClientNode::process_report, this);

   service_next_task_ = nh_.advertiseService(orunav_generic::getRobotTopicName(robot_id_, "/next_task"), &VehicleExecutionClientNode::nextTaskCB, this);
   
   //call worker thread 
   client_thread_ = boost::thread(boost::bind(&VehicleExecutionClientNode::run,this));
 }
  VehicleExecutionClientNode() {
    b_shutdown_ = true;
    client_thread_.join();
  }

  void process_report(const orunav_msgs::RobotReportConstPtr &msg) {    

    input_mutex_.lock();
    currentReport_ = *msg;    
    //ROS_INFO_STREAM(currentReport_);
    //std::cout << "<<-- currentReport_.status : " << currentReport_.status << "(" << currentReport_.WAITING_FOR_TASK << ")" << std::endl;
    
    validReport_ = true;
    if (execSent_ == true && currentReport_.status != currentReport_.WAITING_FOR_TASK) {
      execSent_ = false;
    }
    input_mutex_.unlock();
  }	    

  bool in_waiting_state() const {
    if (validReport_ == true && currentReport_.status == currentReport_.WAITING_FOR_TASK && execSent_ == false)
      return true;
    return false;
  }
  
  bool computeTask(const orunav_msgs::RobotTarget &target, orunav_msgs::Task &task) {
    
    ros::ServiceClient client = nh_.serviceClient<orunav_msgs::ComputeTask>("compute_task");
    orunav_msgs::ComputeTask srv;
    
    srv.request.target = target;
    srv.request.start_from_current_state = true;

    if (client.call(srv)) {
      ROS_INFO("[VehicleExecutionClientNode] - compute_task sucessfull");
    }
    else
    {
      ROS_ERROR_STREAM("[VehicleExecutionClientNode] - Failed to call service: " << client.getService() );
      return false;
    }
    task = srv.response.task;
    ROS_INFO_STREAM("[VehicleExecutionClientNode] - compute_task return value : " << srv.response.result);
    return true;
  }
   
  bool executeTask(const orunav_msgs::Task &task) {

    // Send the task to the coordinator
    ros::ServiceClient client = nh_.serviceClient<orunav_msgs::ExecuteTask>("execute_task");
    orunav_msgs::ExecuteTask srv;
    srv.request.task = task;
    
    if (client.call(srv)) {
      ROS_INFO("[VehicleExecutionClientNode] - execute_task sucessfull");
      execSent_ = true;
    }
    else
    {
      ROS_ERROR("[VehicleExecutionClientNode] - Failed to call service: execute_task");
      return false;
    }
    ROS_INFO_STREAM("[VehicleExecutionClientNode] - set_task return value : " << srv.response.result);
    return true;
  }	    
  
  bool execute_target(const orunav_msgs::RobotTarget &target) {
    if (!validReport_) {
      ROS_WARN("Invalid report, don't know the current location of the vehicle");
      return false;
    }

    while (!in_waiting_state()) {
      usleep(1000*1000);
      if (!ros::ok())
        return false;
      std::cout << "." << std::flush;
    } 
    
    orunav_msgs::RobotTarget _target = target;
    if (start_from_current_state_) {
      input_mutex_.lock();
      _target.start = currentReport_.state;
      input_mutex_.unlock();
    }
    _target.start_earliest = ros::Time::now();
    ROS_INFO_STREAM("target used : \n" << _target);
    
    // Compute task
    orunav_msgs::Task task;
    if (!computeTask(_target, task))
      return false;
    
    if (!executeTask(task))
      return false;

    return true;
  }

  bool execute_targets(const std::vector<orunav_msgs::RobotTarget> &targets) {
    
    for (size_t i = 0; i < targets.size(); i++) {
      if (!execute_target(targets[i])) {
        ROS_ERROR_STREAM("Failed in executing target: " << i);
        return false;
      }
    }
    return true;
  }

  // Service callback
  bool nextTaskCB(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res) {


    std::vector<orunav_msgs::RobotTarget> targets = loadRobotTargets(targets_file_name_);
    
    if (targets.empty()) {
      ROS_ERROR_STREAM("Couldn't load targets_file : " << targets_file_name_);
      ROS_INFO_STREAM("Specify the targets_file using : _targets_file_name:=");
      return false;
    }

    while (!validReport_) {
      usleep(1000*1000);
      if (!ros::ok())
        return false;
      std::cout << "." << std::flush;
    }
    
    if (task_id_ < 0)
      task_id_ = 0;

    if (task_id_ >= targets.size())
      task_id_ = 0;

    orunav_msgs::RobotTarget target = targets[task_id_];
    
    if (!execute_target(target)) {
      return false;
    }  

    while (!in_waiting_state()) {
      usleep(1000*1000);
      if (!ros::ok())
        return false;
      std::cout << "." << std::flush;
    } 

    task_id_++;
    
    return true;
  }
   
  void run() {

    if (task_id_ < 0) {
      while (ros::ok()) {
        usleep(1000*1000);
      }
      return;
    }

    std::vector<orunav_msgs::RobotTarget> targets = loadRobotTargets(targets_file_name_);

    if (targets.empty()) {
      ROS_ERROR_STREAM("Couldn't load targets_file : " << targets_file_name_);
      ROS_INFO_STREAM("Specify the targets_file using : _targets_file_name:=");
      ROS_INFO("Shutting down...");
      ros::shutdown();
      return;
    }
    ROS_INFO_STREAM("Loaded number of targets : " << targets.size());
  
    while (!validReport_) {
      usleep(1000*1000);
      std::cout << "." << std::flush;
    }

    if (nb_iters_ == 0) {
      
      if (task_id_ >= targets.size()) {
        ROS_ERROR_STREAM("Invalid task_id : " << task_id_);
        return;
      }
      ROS_INFO_STREAM("task_id : " << task_id_);
      orunav_msgs::RobotTarget target = targets[task_id_];
      
      if (!execute_target(target)) {
        ROS_ERROR_STREAM("Failed executing target (task_id : " << task_id_ << ")");
        return;
      }
      ROS_INFO("Done...");
      return;
    }
      
    for (int i = 0; i < nb_iters_; i++) {
      if (!execute_targets(targets)) {
        ROS_ERROR_STREAM("Failed in iter: " << i);
        return;
      }
    }
    ROS_INFO("Done...");
  }  
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vehicle_execution_client");
  ros::NodeHandle params ("~");

  VehicleExecutionClientNode p(params);
  
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
}


