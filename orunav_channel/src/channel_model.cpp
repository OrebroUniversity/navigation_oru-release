#include <orunav_channel/channel_model.h>

using namespace channel_model;

ChannelModel::ChannelModel() : pnh_("~"), 
			       nh_() {

    //get parameters from the launch file
    pnh_.param("packet_loss_probability", packet_loss_probability_, 0.0);
    ROS_INFO_STREAM("packet_loss_probability: " << packet_loss_probability_);
    pnh_.param("min_tx_delay_millis", min_tx_delay_millis_, 0);
    ROS_INFO_STREAM("min_tx_delay_millis: " << min_tx_delay_millis_);
    pnh_.param("max_tx_delay_millis", max_tx_delay_millis_, 0);
    ROS_INFO_STREAM("max_tx_delay_millis: " << max_tx_delay_millis_);
    XmlRpc::XmlRpcValue robot_ids;
    if (pnh_.getParam("robot_ids", robot_ids)) {
      ROS_ASSERT(robot_ids.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (int i = 0; i < robot_ids.size(); i++) {
	ROS_ASSERT(robot_ids[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
	robotIDs_.push_back(static_cast<int>(robot_ids[i]));
      }
    }
    else 
      ROS_WARN_STREAM("Robot IDs not found! Using default configuration: robotIDs = [1].");
}
