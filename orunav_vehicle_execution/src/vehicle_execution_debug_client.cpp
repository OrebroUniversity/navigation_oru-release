#include <ros/ros.h>


#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>

#include <orunav_conversions/conversions.h>

#include <orunav_msgs/ComputeTask.h>
#include <orunav_msgs/ExecuteTask.h>
#include <orunav_msgs/ControllerReport.h>
#include <orunav_msgs/RobotTarget.h>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <boost/program_options.hpp>


namespace po = boost::program_options;

//! Simple client to stress the vehicle execution node - by sending a large amount of set_execution service calls.

class VehicleExecutionDebugClientNode
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber control_report_sub_;
  int test_id_;
  int robot_id_;
  int goal_id_inc_;
  bool load_operation_;
  orunav_generic::State2d currentState2d_; // Current state from the controller report.
  bool validState2d_;
  bool b_shutdown_;

  boost::mutex input_mutex_;
  boost::thread client_thread_;
 

public:
  VehicleExecutionDebugClientNode(ros::NodeHandle paramHandle) : goal_id_inc_(0), validState2d_(false)
 {

   paramHandle.param<int>("test_id", test_id_, 1);
   paramHandle.param<int>("robot_id", robot_id_, 1);
   paramHandle.param<bool>("load_operation",load_operation_,false);

   // Listen to the controll messages directly.
   control_report_sub_ = nh_.subscribe<orunav_msgs::ControllerReport>(orunav_generic::getRobotTopicName(robot_id_, "/controller/reports"), 10,&VehicleExecutionDebugClientNode::process_report, this);

   //call worker thread 
   client_thread_ = boost::thread(boost::bind(&VehicleExecutionDebugClientNode::run,this));
 }
  VehicleExecutionDebugClientNode() {
    b_shutdown_ = true;
    client_thread_.join();
  }

  void process_report(const orunav_msgs::ControllerReportConstPtr &msg) {    

    orunav_generic::State2d state = orunav_conversions::createState2dFromControllerStateMsg(msg->state);

    input_mutex_.lock();
    
    currentState2d_ = state;
    validState2d_ = true;
    input_mutex_.unlock();
  }	    
  
  bool computeTask(const orunav_msgs::RobotTarget &target, orunav_msgs::Task &task) {
    
    ros::ServiceClient client = nh_.serviceClient<orunav_msgs::ComputeTask>("compute_task");
    orunav_msgs::ComputeTask srv;
    
    srv.request.target = target;
    srv.request.start_from_current_state = true;
    
    if (client.call(srv)) {
      ROS_INFO("[VehicleExecutionDebugClientNode] - compute_task sucessfull");
    }
    else
    {
      ROS_ERROR("[VehicleExecutionDebugClientNode] - Failed to call service: compute_task");
      return false;
    }
    task = srv.response.task;
    ROS_INFO_STREAM("[VehicleExecutionDebugClientNode] - compute_task return value : " << srv.response.result);
    return true;
  }
   
  void executeTask(const orunav_msgs::Task &task) {

    // Send the task to the coordinator
    ros::ServiceClient client = nh_.serviceClient<orunav_msgs::ExecuteTask>("execute_task");
    orunav_msgs::ExecuteTask srv;
    srv.request.task = task;
    
    if (client.call(srv)) {
      ROS_INFO("[VehicleExecutionDebugClientNode] - set_task sucessfull");
    }
    else
    {
      ROS_ERROR("[VehicleExecutionDebugClientNode] - Failed to call service: set_task");
      return;
    }
    ROS_INFO_STREAM("[VehicleExecutionDebugClientNode] - set_task return value : " << srv.response.result);
  }	    
  
  // "Flood test" - send multiple update task requests.
  void test1() {
    //1). Get the current position.
    //2). Compute a task going x meter forward.
    //3). Send the task.
    //4). Send the same task over and over every 0.1 seconds for 10 seconds.
    bool print_once = true;
    while (!validState2d_) {
      usleep(1000000);
      if (print_once) {
        ROS_INFO("Waiting for a current state (controller report).");
        print_once = false;
      }
    }
    
    input_mutex_.lock();
    orunav_generic::State2d start = currentState2d_;
    input_mutex_.unlock();
    
    ROS_INFO_STREAM("Got start state : " << start);
    orunav_msgs::RobotTarget target;
    target.start = orunav_conversions::createPoseSteeringMsgFromState2d(start);
    orunav_generic::State2d goal = start;
    orunav_generic::Control step(5., 0.);
    goal.addControlStep(step, 1.19, 1.);
    target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(goal);
    //    target.start_op.operation = target.start_op.UNLOAD;
    //    target.goal_op.operation = target.goal_op.LOAD;
    target.start_op.operation = target.start_op.NO_OPERATION;
    target.goal_op.operation = target.goal_op.NO_OPERATION;
    target.start_earliest = ros::Time::now();

    // Compute task
    orunav_msgs::Task task;
    if (!computeTask(target, task))
      return;
        
    // Add coordination times...
    orunav_generic::CoordinatedTimes ct;

    // Check that we got dts computed
    if (task.dts.dts.size() == 0) {
      ROS_ERROR("DTS is not computed, check that use_ct param is set in the execution node.");
      return;
    }
    std::vector<double> d = orunav_conversions::getDoubleVecFromDeltaTMsg(task.dts.dts[0]); // [0] -> fastest
    ROS_INFO_STREAM("size dts : " << d.size());
    ct = orunav_generic::computeCoordinatedTimesFromDeltaTs(d);
    ct.addOffset(ros::Time::now().toSec()+4.);
    ct.createStep(20, 10.);
    task.cts.ts.push_back(orunav_conversions::getCoordinatorTimeFromDoubleVec(ct));
    executeTask(task); // First time sent(!)
    ROS_INFO("Sending task - first time.");
    task.update = true;
    
    for (int i = 0; i < 1000; i++) {
      usleep(1000000);
      ct.addOffset(0.4);
      task.cts.ts[0] = orunav_conversions::getCoordinatorTimeFromDoubleVec(ct);
      ROS_INFO_STREAM("(Re)-sending task, amount : " << i);
      executeTask(task);
      //      ROS_INFO_STREAM("task : " << task);
    }
  }

  // Critical point test...
  void test2() {
    //1). Get the current position.
    //2). Compute a task going x meter forward.
    //3). Send the task.
    //4). Send the same task with updated critical point idx.
    bool print_once = true;
    while (!validState2d_) {
      usleep(1000000);
      if (print_once) {
        ROS_INFO("Waiting for a current state (controller report).");
        print_once = false;
      }
    }
    
    input_mutex_.lock();
    orunav_generic::State2d start = currentState2d_;
    input_mutex_.unlock();
    
    ROS_INFO_STREAM("Got start state : " << start);
    orunav_msgs::RobotTarget target;
    target.start = orunav_conversions::createPoseSteeringMsgFromState2d(start);
    orunav_generic::State2d goal = start;
    orunav_generic::Control step(5., 0.);
    goal.addControlStep(step, 1.19, 1.);
    target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(goal);
    target.start_op.operation = target.start_op.NO_OPERATION;
    target.goal_op.operation = target.goal_op.NO_OPERATION;
    target.start_earliest = ros::Time::now();

    // Compute task
    orunav_msgs::Task task;
    if (!computeTask(target, task))
      return;
        
    // Add coordination times...
    orunav_generic::CoordinatedTimes ct;
    std::vector<double> d = orunav_conversions::getDoubleVecFromDeltaTMsg(task.dts.dts[0]); // [0] -> fastest
    ROS_INFO_STREAM("size dts : " << d.size());
    ct = orunav_generic::computeCoordinatedTimesFromDeltaTs(d);
    ct.addOffset(ros::Time::now().toSec()+4.);
    ct.createStep(20, 10.);
    task.cts.ts.push_back(orunav_conversions::getCoordinatorTimeFromDoubleVec(ct));


    if (test_id_ == 2) {
      task.criticalPoint = 0;
      executeTask(task); // First time sent(!)
      ROS_INFO("Sending task - first time, crit point = 0.");
      task.update = true;
      
      // Wait 15 secs and send a new critical point.
      usleep(4*1000*1000);
      task.criticalPoint = 40;
      ROS_INFO("Sending task - second time, crit point = 40.");
      executeTask(task);
      
      // Wait 15 secs and send a new critical point.
      usleep(15*1000*1000);
      task.criticalPoint = -1;
      ROS_INFO("Sending task - third time, crit point = -1.");
      executeTask(task);
    }
    if (test_id_ == 3) {
      // Flood test - run to the end...
      executeTask(task);
      task.update = true;
      for (int i = 0; i < 100; i++) {
        usleep(1000000);
        task.criticalPoint = 20+5*i;
        ROS_INFO_STREAM("(Re)-sending critical point : " << task.criticalPoint);
        executeTask(task);
      }
      
    }
    if (test_id_ == 4) {
      task.criticalPoint = 20;
      executeTask(task); // First time sent(!)
      ROS_INFO("Sending task - first time, crit point = 20.");
      task.update = true;
      
      // Wait 15 secs and send a new critical point.
      usleep(15*1000*1000);
      task.criticalPoint = 40;
      ROS_INFO("Sending task - second time, crit point = 40.");
      executeTask(task);
      
      // Wait 15 secs and send a new critical point.
      usleep(15*1000*1000);
      task.criticalPoint = -1;
      ROS_INFO("Sending task - third time, crit point = -1.");
      executeTask(task);
    }

    if (test_id_ == 5) {
      task.criticalPoint = 1;
      executeTask(task); // First time sent(!)
      ROS_INFO("Sending task - first time, crit point = 1.");
      task.update = true;
      
      // Wait 15 secs and send a new critical point.
      usleep(15*1000*1000);
      task.criticalPoint = 2;
      ROS_INFO("Sending task - second time, crit point = 2.");
      executeTask(task);
      
      // Wait 15 secs and send a new critical point.
      usleep(15*1000*1000);
      task.criticalPoint = 4; //3 -> fix this, need to have on intermediate point as it is now(!)
      ROS_INFO("Sending task - third time, crit point = 4.");
      executeTask(task);

      // Wait 15 secs and send a new critical point.
      usleep(15*1000*1000);
      task.criticalPoint = -1;
      ROS_INFO("Sending task - third time, crit point = -1.");
      executeTask(task);

    }
    
    if (test_id_ == 6) {
      task.criticalPoint = 10;
      executeTask(task); // First time sent(!)
      ROS_INFO("Sending task - first time, crit point = 1.");
      task.update = true;

      for (int i = 0; i < 10; i++) {
        task.criticalPoint = 10;
        usleep(1000*1000);
        executeTask(task); 
        ROS_INFO("Sending task - crit point = 10.");
      }
    }
  }

  void run() {
    
    
    if (test_id_ == 1) {
      test1();
    }
    if (test_id_ >= 2) {
      test2();
    }
    ROS_INFO("Done...");
  }  
 };

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_n_click_target_client");
  ros::NodeHandle params ("~");

  VehicleExecutionDebugClientNode p(params);
  
  ros::spin();
}


