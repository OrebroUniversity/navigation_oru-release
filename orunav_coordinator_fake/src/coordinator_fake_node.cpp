#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <orunav_rviz/orunav_rviz.h>

#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>

#include <orunav_msgs/SetTask.h>
#include <orunav_msgs/ComputeTask.h>
#include <orunav_msgs/UpdateTask.h>
#include <orunav_msgs/ExecuteTask.h>

#include <orunav_msgs/RobotTarget.h>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp> // for std::pair
#include <iostream>
#include <fstream>


class CoordinatorFakeNode {

private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_set_;
  ros::ServiceServer service_update_;

  ros::Publisher marker_pub_;

  boost::mutex inputs_mutex_, run_mutex_;
  boost::thread client_thread_;
  boost::condition_variable cond_;

  bool visualize_;
  ros::Timer heartbeat_slow_visualization_;
  bool b_shutdown_;
  orunav_msgs::Task task_;
  int task_id_inc_;
  double start_time_offset_;
  bool slowest_dts_;
  bool alter_fast_slow_dts_;
  int cts_hole_width_;
  bool multiple_vehicles_;
  bool use_ct_;
  std::string execute_task_name_;
  
 public:
  CoordinatorFakeNode(ros::NodeHandle &paramHandle) : task_id_inc_(0)
  {
      // Parameters
      paramHandle.param<bool>("visualize", visualize_,false);
      paramHandle.param<double>("start_time_offset", start_time_offset_, 3.);
      paramHandle.param<bool>("slowest_dts", slowest_dts_, false);
      paramHandle.param<bool>("alter_fast_slow_dts", alter_fast_slow_dts_, false);
      paramHandle.param<int>("cts_hole_width", cts_hole_width_, 0);
      paramHandle.param<bool>("multiple_vehicles", multiple_vehicles_, false);
      paramHandle.param<bool>("use_ct", use_ct_, true);
      paramHandle.param<std::string>("execute_task_name", execute_task_name_, std::string("/execute_task"));

      
      // Services
      service_set_ = nh_.advertiseService("set_task", &CoordinatorFakeNode::setTaskCB, this);
      service_update_ = nh_.advertiseService("update_task", &CoordinatorFakeNode::updateTaskCB, this);

      heartbeat_slow_visualization_   = nh_.createTimer(ros::Duration(1.0),&CoordinatorFakeNode::publish_visualization_slow,this);
      
      //call worker thread 
      client_thread_ = boost::thread(boost::bind(&CoordinatorFakeNode::run,this));
      b_shutdown_ = false;
    }

  CoordinatorFakeNode() {
    b_shutdown_ = true;
    cond_.notify_one();
    client_thread_.join();
  }

 
  void publish_visualization_slow(const ros::TimerEvent &event) {
    
    if (!visualize_)
      return;
    
  }

  orunav_msgs::CoordinatorTime computeCTSDirectlyWithoutDoingAnyCoordinationAtAll(const orunav_msgs::DeltaTVec &dts) {
    orunav_generic::CoordinatedTimes ct;
    std::vector<double> d;

    d = orunav_conversions::getDoubleVecFromDeltaTMsg(dts.dts[0]); // [0] -> fastest

    if (slowest_dts_) {
      ROS_ERROR("[USING SLOWEST DTS AS CTS INPUT]");
      d = orunav_conversions::getDoubleVecFromDeltaTMsg(dts.dts[1]);
    }
    
    if (alter_fast_slow_dts_) {
      std::vector<double> d1 = orunav_conversions::getDoubleVecFromDeltaTMsg(dts.dts[0]);
      std::vector<double> d2 = orunav_conversions::getDoubleVecFromDeltaTMsg(dts.dts[1]);
      d.resize(d1.size());
      assert (d1.size() == d2.size());
      for (size_t i = 0; i < d1.size(); i++) {
        if (i % 2 == 0) {
          d[i] = d1[i];
        }
        else {
          d[i] = d2[i];
        }
        ROS_INFO_STREAM("d[" << i << "] : " << d[i]);
      }
    }
    ct = orunav_generic::computeCoordinatedTimesFromDeltaTs(d);
    
    
    if (cts_hole_width_ > 0) {
      ct.createHoles(cts_hole_width_);
    }
    
    ct.addOffset(ros::Time::now().toSec()+start_time_offset_);
    return orunav_conversions::getCoordinatorTimeFromDoubleVec(ct);
  }

  bool validTaskMsg(const orunav_msgs::Task &task) const {
    // Any path points?
    if (task.path.path.size() < 3) {
      ROS_ERROR_STREAM("Not a valid task: path to short, current length : " << task.path.path.size()); 
      return false;
    }
    if (use_ct_) {
      if (task.dts.dts.size() < 2) { // 2- the fastest and slowest
        ROS_ERROR_STREAM("Not a valid task: no dts vectors, current length: " << task.dts.dts.size());
        return false;
      }
      if (task.path.path.size() != task.dts.dts[0].dt.size()) {
        ROS_ERROR_STREAM("Not a valid task: dts[0] length different from path length: " << task.dts.dts[0].dt.size());
        return false;
      }
    }
    if (task.path.path.size() < task.constraints.constraints.size()) {
      ROS_ERROR_STREAM("Not a valid task: amount of constraints larger the path length: " << task.constraints.constraints.size());
      return false;
    }
    return true;
  }
  
  // Service callbacks
  bool setTaskCB(orunav_msgs::SetTask::Request &req,
                 orunav_msgs::SetTask::Response &res)
  {
    ROS_INFO("[CoordinatorFake] RID:%d - received setTask", (int)req.task.target.robot_id);

    // Verify the task
    if (!validTaskMsg(req.task)) {
      ROS_ERROR("[CoordinatorFake] invalid task.");
      return false;
    }

    // Simply just call the execute directly using the same task.
    task_ = req.task;


    // All items below needs to be set by the coordinator.
    task_.update = true;
    task_.target.task_id = task_id_inc_++;
    
    if (use_ct_) {
      task_.cts.ts.push_back(computeCTSDirectlyWithoutDoingAnyCoordinationAtAll(req.task.dts));
    }
  
    // This doesn't work...
#if 0    
    {
      ROS_INFO("[CooridnatorFake]: CALLING");
      ros::ServiceClient client = nh_.serviceClient<orunav_msgs::SetTask>("execute_task");
      ROS_INFO("[CooridnatorFake]: CALLING2");
      orunav_msgs::SetTask srv;
      srv.request.task = req.task;
      
      if (client.call(srv)) {
        ROS_INFO("[CoordinatorFakeNode] - execute_task call successful");
      }
      else
      {
        ROS_ERROR("[CoordinatorFakeNode] - Failed to call service: execute_task");
        res.result = 0;
        return false;
      }
      ROS_INFO_STREAM("[CoordinatorFakeNode] - execute_task return value : " << srv.response.result);
    }
#endif
    res.result = 1;
    cond_.notify_one();
    return true;
  }

  bool updateTaskCB(orunav_msgs::ExecuteTask::Request &req,
                     orunav_msgs::ExecuteTask::Response &res)
  {
    ROS_INFO("[CoordinatorFake] RID:%d - received updateTask", (int)req.task.target.robot_id);
    // Simply just call the execute directly using the same task.
    task_ = req.task;
    task_.update = true;
    res.result = 1;
    cond_.notify_one();
    return true;
  }

  void run() {
      
    while(!b_shutdown_) {
      usleep(100000);
   
      boost::unique_lock<boost::mutex> uniqueLock(run_mutex_);

      cond_.wait(uniqueLock);
      
      ROS_INFO("[CoordinatorFake] waking up");

      {
        ros::ServiceClient client;
        std::string service_name;
        if (multiple_vehicles_) {
          service_name = orunav_generic::getRobotTopicName(task_.target.robot_id, this->execute_task_name_);
        }
        else {
          service_name = this->execute_task_name_;
        }
        client = nh_.serviceClient<orunav_msgs::SetTask>(service_name);
        
        orunav_msgs::SetTask srv;
        srv.request.task = task_;
        
        if (client.call(srv)) {
          ROS_INFO_STREAM("[CoordinatorFakeNode] - " << service_name << " call successful");
      }
        else
        {
          ROS_ERROR_STREAM("[CoordinatorFakeNode] - Failed to call service: " << service_name);
        }
        ROS_INFO_STREAM("[CoordinatorFakeNode] - " << service_name << " return value : " << srv.response.result);
      }	    
      
    } // while
  }




};


int main(int argc, char** argv) {

    ros::init(argc,argv,"coordinator_fake");
    ros::NodeHandle params ("~");

    CoordinatorFakeNode cf(params);

    ros::spin();


}
