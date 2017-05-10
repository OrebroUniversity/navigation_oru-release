#include <ros/ros.h>
#include <orunav_generic/utils.h>
#include <orunav_msgs/ComputeTask.h>
#include <orunav_msgs/SetTask.h>
#include <geometry_msgs/PoseStamped.h>
#include <orunav_vehicle_execution/io.h>

#include <boost/program_options.hpp>


namespace po = boost::program_options;

//! Simple client to the vehicle execution service that listen to the /robotX/goal topic, converts it into a RobotTarget message.

class PointNClickTargetClientNode
{
private:
  ros::Subscriber *robot_goal_subs_;
  ros::NodeHandle nh_;
  int task_id_inc_;
  bool load_operation_;
  bool save_targets_;
  std::vector<orunav_msgs::RobotTarget> targets_;
  bool multiple_vehicles_;

public:
  PointNClickTargetClientNode(ros::NodeHandle paramHandle) : task_id_inc_(0)
 {
    XmlRpc::XmlRpcValue my_list;
    paramHandle.getParam("robot_ids", my_list);
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_WARN("creating %d subscribers",my_list.size());
    robot_goal_subs_ = new ros::Subscriber[my_list.size()];
    for (int32_t i = 0; i < my_list.size(); ++i) 
    {
      ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
      int my_id = static_cast<int>(my_list[i]);
      
      // Goal topic
      {
        boost::function<void (PointNClickTargetClientNode*, const geometry_msgs::PoseStampedConstPtr&, int)> f;
        boost::function<void (const geometry_msgs::PoseStampedConstPtr&)> f0;
        f = &PointNClickTargetClientNode::process_goal;
        f0 = boost::bind(f,this,_1,my_id);
        char topic_name[400];
        snprintf(topic_name, 399,"/robot%d/goal",my_id);
        robot_goal_subs_[i] = nh_.subscribe(topic_name,10,f0);
        ROS_INFO("[PointNClickTargetClientNode] - subscribing to : %s", topic_name);
      }
    }
    paramHandle.param<bool>("load_operation",load_operation_,false);
    paramHandle.param<bool>("save_targets", save_targets_, true);
    paramHandle.param<bool>("multiple_vehicles", multiple_vehicles_, false);
 }

  void process_goal(const geometry_msgs::PoseStampedConstPtr &msg, int my_robot_id ) {
    ROS_INFO("[PointNClickTargetClientNode] - process_goal, my id is %d", my_robot_id);  
    
    // Call the server...
    orunav_msgs::RobotTarget target;
    
    target.goal.pose = msg->pose;
    target.goal.steering = 0.;
    target.goal_op.operation = target.goal_op.NO_OPERATION;
    target.start_op.operation = target.start_op.NO_OPERATION;
    if (load_operation_) {
      target.goal_load.status = target.goal_load.EUR_PALLET;
      if ((task_id_inc_ % 2) == 0) {
        target.goal_op.operation = target.goal_op.LOAD;
        if (task_id_inc_ > 0) {
          target.start_op.operation = target.start_op.UNLOAD;
        }
      }
      else {
        target.goal_op.operation = target.goal_op.UNLOAD;
      }
    }
    
    ROS_INFO_STREAM("[PointNClickTargetClientNode] target.goal_op.operation : " << target.goal_op.operation); 
    ROS_INFO_STREAM("[PointNClickTargetClientNode] target.start_op.operation : " << target.start_op.operation); 
    
    
    target.robot_id = my_robot_id;
    target.task_id = task_id_inc_++;  // This will anyway be handled by the coordinator.
    target.goal_id = target.task_id;  // Obsolete?

    target.start_earliest = ros::Time::now();
    target.start_deadline = ros::Time::now() + ros::Duration(60);

    targets_.push_back(target);
    if (save_targets_) {
      std::string file_name("robot_targets.dat");
      saveRobotTargets(targets_, file_name);
    }

    ros::Publisher path_pub = nh_.advertise<orunav_msgs::RobotTarget>("/forIran/path", 1000);
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        path_pub.publish(target);
      	loop_rate.sleep();
      	//path_pub.publish(target);
      	//path_pub.publish(target2);
      	ros::spinOnce();
    }

    // TODO - fix the other start times...
    /*orunav_msgs::Task task;
    {
      ros::ServiceClient client;
      if (multiple_vehicles_) {
        ROS_INFO_STREAM("[PointNClickTargetClientNode] topic : " << orunav_generic::getRobotTopicName(target.robot_id, "/compute_task"));
        client = nh_.serviceClient<orunav_msgs::ComputeTask>(orunav_generic::getRobotTopicName(target.robot_id, "/compute_task"));
      } else {
        client = nh_.serviceClient<orunav_msgs::ComputeTask>("compute_task");
      }
      orunav_msgs::ComputeTask srv;
      
      srv.request.target = target;
      srv.request.start_from_current_state = true;
      
      if (client.call(srv)) {
        ROS_INFO("[PointNClickTargetClientNode] - compute_task sucessfull");
      }
      else
      {
        ROS_ERROR("[PointNClickTargetClientNode] - Failed to call service: compute_task");
        return;
      }
      task = srv.response.task;
      ROS_INFO_STREAM("[PointNClickTargetClientNode] - compute_task return value : " << srv.response.result);
    }

    {
      // Send the task to the coordinator
      ros::ServiceClient client = nh_.serviceClient<orunav_msgs::SetTask>("set_task");
      orunav_msgs::SetTask srv;
      srv.request.task = task;
      
      if (client.call(srv)) {
        ROS_INFO("[PointNClickTargetClientNode] - set_task sucessfull");
      }
      else
      {
        ROS_ERROR("[PointNClickTargetClientNode] - Failed to call service: set_task");
        return;
      }
      ROS_INFO_STREAM("[PointNClickTargetClientNode] - set_task return value : " << srv.response.result);
    }*/	    
  }
    
 };

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_n_click_target_publisher");
  ros::NodeHandle params ("~");

  PointNClickTargetClientNode p(params);
  
  ros::spin();
}


