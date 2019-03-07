#include <ros/ros.h>
#include <orunav_channel/channel_model.h>

using namespace channel_model;

int main(int argc, char** argv){
  ros::init(argc, argv, "channel_model");

  ros::NodeHandle nh;
  ChannelModel ch;
  
  ros::spin();

  return(0);
}