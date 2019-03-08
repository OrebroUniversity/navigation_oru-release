#include <orunav_channel/channel_model.h>

using namespace channel_model;

ChannelModel::ChannelModel(ros::NodeHandle &paramHandle) : nh_() {

    //get parameters from the launch file
    paramHandle.param<double>("packet_loss_probability", packet_loss_probability_, 0.0);
    paramHandle.param<int>("min_tx_delay_millis", min_tx_delay_millis_, 0);
    paramHandle.param<int>("max_tx_delay_millis", max_tx_delay_millis_, 0);
    XmlRpc::XmlRpcValue robot_ids;
    if (paramHandle.getParam("robot_ids", robot_ids)) {
      ROS_ASSERT(robot_ids.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (int i = 0; i < robot_ids.size(); i++) {
	ROS_ASSERT(robot_ids[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
	robotIDs_.push_back(static_cast<int>(robot_ids[i]));
      }
    }
    else 
      ROS_WARN_STREAM("Robot IDs not found! Using default configuration: robotIDs = [1].");
    
    //Define publisher and subscribers
    std::string tf_prefix("/robot");
    report_subscribers_.resize(robotIDs_.size());
    delayed_report_publishers_.resize(robotIDs_.size());
    for(int i = 0; i < robotIDs_.size(); i++) {
      report_subscribers_[i] = nh_.subscribe(tf_prefix+std::to_string(robotIDs_[i])+"/report", 1, &ChannelModel::onNewRobotReport, this);
      delayed_report_publishers_[i] = nh_.advertise<orunav_msgs::RobotReport>(tf_prefix+std::to_string(robotIDs_[i])+"/report_delayed", 10);
    }

}

void ChannelModel::onNewRobotReport(const orunav_msgs::RobotReport::Ptr msg)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);
  if (packet_loss_probability_ > 0 ? dis(gen) > packet_loss_probability_ : true) {
    int delay = min_tx_delay_millis_ + static_cast<int>(dis(gen)*(max_tx_delay_millis_-min_tx_delay_millis_));
    boost::thread* th = new boost::thread(&ChannelModel::delayRobotReport, this, msg, delay);
  }
  else
    ROS_DEBUG_STREAM("Lost RobotReport to Robot" << msg->robot_id);
    
}

bool ChannelModel::delayRobotReport(const orunav_msgs::RobotReport::Ptr msg, int delay)
{
  auto iter = std::find(robotIDs_.begin(), robotIDs_.end(), msg->robot_id);
  if (iter == robotIDs_.end()) {
    ROS_WARN_STREAM("Wrong Robot ID in RobotReport.");
    return false;
  }
  ros::Duration(1e-3*delay).sleep();
  delayed_report_publishers_[std::distance(robotIDs_.begin(),iter)].publish(msg);
  
  return true;
}
