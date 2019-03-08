#include <orunav_channel/channel_model.h>

using namespace channel_model;

ChannelModel::ChannelModel(ros::NodeHandle &paramHandle) : nh_() {

    //get parameters from the launch file
    paramHandle.param<double>("packet_loss_probability", packet_loss_probability_, 0.0);
    double max_unsafety_probability;
    paramHandle.param<double>("max_unsafety_probability", max_unsafety_probability, 0.0001);
    paramHandle.param<int>("min_tx_delay_millis", min_tx_delay_millis_, 0);
    paramHandle.param<int>("max_tx_delay_millis", max_tx_delay_millis_, 0);    
    number_of_replicas_ = (packet_loss_probability_ > 0 && max_unsafety_probability > 0) ? static_cast<int>(log(max_unsafety_probability)/log(packet_loss_probability_)) : 1;
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
      task_counters_[i] = -1;
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

bool ChannelModel::delayRobotReport(const orunav_msgs::RobotReport::Ptr msg, const int delay)
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

void ChannelModel::onNewTaskMsg(const orunav_msgs::Task::Ptr msg)
{
  auto iter = std::find(robotIDs_.begin(), robotIDs_.end(), msg->constraints.robot_id);
  if (iter == robotIDs_.end()) {
    ROS_WARN_STREAM("Wrong Robot ID in Task.");
    return;
  }
  int index = std::distance(robotIDs_.begin(), iter);
  if (msg->seq > task_counters_[index]) {
    task_counters_[index] = msg->seq;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    bool send = false;
    for (int i = 0; i < number_of_replicas_; i++) {
      send = dis(gen) > packet_loss_probability_;
      if (send) break;
    }
    if (send) {
      int delay = min_tx_delay_millis_ + static_cast<int>(dis(gen)*(max_tx_delay_millis_-min_tx_delay_millis_));
       boost::thread* th = new boost::thread(&ChannelModel::delayTaskExecution, this, msg, delay);
    }
  }
}

bool ChannelModel::delayTaskExecution(const orunav_msgs::Task::Ptr msg, const int delay) {
  ros::Duration(1e-3*delay).sleep();
  orunav_msgs::ExecuteTaskRequest req;
  req.task = *msg;
  orunav_msgs::ExecuteTaskResponse res;
  if (execute_task_clients_[msg->constraints.robot_id].call(req,res)) {
    if (res.result)
      ROS_WARN_STREAM("Failed to start execution of goal " << msg->criticalPoint << " for robot " << msg->constraints.robot_id << ".");
  }
  else
    ROS_ERROR_STREAM("Failed to call task execution service. Goal: " << msg->criticalPoint << " Robot: " << msg->constraints.robot_id << ".");
  
  return true;
}