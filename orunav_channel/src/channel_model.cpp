#include <orunav_channel/channel_model.h>

using namespace channel_model;

ChannelModel::ChannelModel(ros::NodeHandle &paramHandle) : nh_() {

    //get parameters from the launch file
    paramHandle.param<double>("packet_loss_probability", packet_loss_probability_, 0.0);
    double max_unsafety_probability;
    paramHandle.param<double>("max_unsafety_probability", max_unsafety_probability, 0.0001);
    paramHandle.param<int>("min_tx_delay_millis", min_tx_delay_millis_, 0);
    paramHandle.param<int>("max_tx_delay_millis", max_tx_delay_millis_, 0);    
    number_of_replicas_ = (packet_loss_probability_ > 0 && max_unsafety_probability > 0) ? std::max(static_cast<int>(log(1.0-sqrt(1.0-max_unsafety_probability))/log(packet_loss_probability_)), 1) : 1;
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
    paramHandle.param<std::string>("tf_prefix", prefix_, "/robot");
    paramHandle.param<std::string>("report_topic", report_topic_, "report");
    paramHandle.param<std::string>("execute_task_topic", execute_task_topic_, "unreliable_execute_task");
    paramHandle.param<std::string>("execute_task_service", execute_task_service_, "execute_task");
    
    //Define publisher and subscribers
    report_subscribers_.resize(robotIDs_.size());
    delayed_report_publishers_.resize(robotIDs_.size());
    execute_task_subscribers_.resize(robotIDs_.size());
    execute_task_clients_.resize(robotIDs_.size());
    task_counters_.resize(robotIDs_.size());
    for(int i = 0; i < robotIDs_.size(); i++) {
      report_subscribers_[i] = nh_.subscribe(prefix_+std::to_string(robotIDs_[i])+"/"+report_topic_, 1, &ChannelModel::onNewRobotReport, this);
      delayed_report_publishers_[i] = nh_.advertise<orunav_msgs::RobotReport>(prefix_+std::to_string(robotIDs_[i])+"/report_delayed", 10);
      execute_task_subscribers_[i] = nh_.subscribe(prefix_+std::to_string(robotIDs_[i])+"/"+execute_task_topic_, 1, &ChannelModel::onNewTaskMsg, this);
      execute_task_clients_[i] = nh_.serviceClient<orunav_msgs::ExecuteTask>(prefix_+std::to_string(robotIDs_[i])+"/"+execute_task_service_);
      task_counters_[i] = -2;
    }
}

float ChannelModel::getRand(const float lb, const float ub) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(std::min(lb,ub), std::max(lb,ub));
  return dis(gen);
}

int ChannelModel::getRobotIndex(const int robot_id) {
  auto iter = std::find(robotIDs_.begin(), robotIDs_.end(), robot_id);
  if (iter == robotIDs_.end())
    return -1;
  return std::distance(robotIDs_.begin(),iter);
}

void ChannelModel::onNewRobotReport(const orunav_msgs::RobotReport::Ptr msg)
{
  if (packet_loss_probability_ > 0 ? getRand(0.0,1.0) > packet_loss_probability_ : true) 
    boost::thread* th = new boost::thread(&ChannelModel::delayRobotReport, this, msg);
  else
    ROS_DEBUG_STREAM("Lost RobotReport to Robot" << msg->robot_id);
}

bool ChannelModel::delayRobotReport(const orunav_msgs::RobotReport::Ptr msg)
{
  int index = getRobotIndex(msg->robot_id);
  if (index == -1) {
    ROS_WARN_STREAM("Wrong Robot ID in RobotReport.");
    return false;
  }
  int delayInMillis = min_tx_delay_millis_ + static_cast<int>(getRand(0.0,1.0)*(max_tx_delay_millis_-min_tx_delay_millis_));
  ros::Duration(1e-3*delayInMillis).sleep();
  delayed_report_publishers_[index].publish(msg);
  return true;
}

void ChannelModel::onNewTaskMsg(const orunav_msgs::Task::Ptr msg)
{
  int index = getRobotIndex(msg->robot_id);
  if (index == -1) {
    ROS_WARN_STREAM("Wrong Robot ID in Task.");
    return;
  }
  if (msg->seq > task_counters_[index]) {
    task_counters_[index] = msg->seq;
    bool send = false;
    for (int i = 0; i < number_of_replicas_; i++) {
      send = getRand(0.0,1.0) >= packet_loss_probability_;
      if (send) break;
    }
    if (send) 
      boost::thread* th = new boost::thread(&ChannelModel::delayTaskExecution, this, *msg);
 }
}

bool ChannelModel::delayTaskExecution(const orunav_msgs::Task msg) {
  int delayInMillis = min_tx_delay_millis_ + static_cast<int>(getRand(0.0,1.0)*(max_tx_delay_millis_-min_tx_delay_millis_));
  ros::Duration(1e-3*delayInMillis).sleep();
  orunav_msgs::ExecuteTaskRequest req;
  req.task = msg;
  orunav_msgs::ExecuteTaskResponse res;
  if (!execute_task_clients_[getRobotIndex(msg.robot_id)].call(req,res))
    ROS_ERROR_STREAM("Failed to start execution of goal " << msg.criticalPoint << " Robot: " << msg.robot_id << ".");
  return true;
}