#ifndef CHANNEL_MODEL_H
#define CHANNEL_MODEL_H

//ros
#include <ros/node_handle.h>
#include <ros/common.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

//messages
#include <orunav_msgs/RobotReport.h>
#include <orunav_msgs/Task.h>
#include <orunav_msgs/ExecuteTask.h>

//libraries
#include <vector>
#include <string>
#include <boost/thread.hpp>
#include <random>
#include <functional>
#include <memory>

//Get parameters from launch file
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

namespace channel_model {
  class ChannelModel {
    ros::NodeHandle nh_;
    std::vector<int> robotIDs_;
    
    //channel parameters
    double packet_loss_probability_;
    int number_of_replicas_;
    int min_tx_delay_millis_;
    int max_tx_delay_millis_;
    std::vector<int> task_counters_;
    
    //topic naming
    std::string prefix_;
    std::string report_topic_;
    std::string execute_task_topic_;
    std::string execute_task_service_;
    
    //publisher and subscrbers
    std::vector<ros::Subscriber> report_subscribers_;
    std::vector<ros::Publisher> delayed_report_publishers_;
    std::vector<ros::Subscriber> execute_task_subscribers_;
    std::vector<ros::ServiceClient> execute_task_clients_;
    
    //utilities
    float getRand(const float lb, const float ub);
    int getRobotIndex(const int robot_id);
    
    //callback and functions
    void onNewRobotReport(const orunav_msgs::RobotReport::Ptr msg);
    bool delayRobotReport(const orunav_msgs::RobotReport::Ptr msg);
    void onNewTaskMsg(const orunav_msgs::Task::Ptr msg);
    bool delayTaskExecution(const orunav_msgs::Task msg); 
    
  public:
    ChannelModel(ros::NodeHandle &paramHandle);
  };  
};

#endif