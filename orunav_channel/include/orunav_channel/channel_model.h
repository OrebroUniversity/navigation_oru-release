#ifndef CHANNEL_MODEL_H
#define CHANNEL_MODEL_H

//ros
#include <ros/node_handle.h>
#include <ros/common.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

//std library
#include <vector>
#include <string>

//Get parameters from launch file
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

namespace channel_model {
  class ChannelModel {
    ros::NodeHandle nh_, pnh_;
    std::vector<int> robotIDs_;
    std::vector<ros::Subscriber> report_subscribers_;
    std::vector<ros::Publisher> delayed_report_publishers_;
    double packet_loss_probability_;
    int min_tx_delay_millis_;
    int max_tx_delay_millis_;
    
  public:
    ChannelModel();
  };  
};

#endif