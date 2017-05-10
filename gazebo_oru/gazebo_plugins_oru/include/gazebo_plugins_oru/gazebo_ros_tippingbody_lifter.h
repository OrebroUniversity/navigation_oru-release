#ifndef TIPPINGBODY_LIFTER_PLUGIN_HH
#define TIPPINGBODY_LIFTER_PLUGIN_HH

#include <map>

// Gazebo
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

class Joint;
class Entity;


class GazeboRosTippingbodyLifter : public ModelPlugin {

public:
    GazeboRosTippingbodyLifter();
    ~GazeboRosTippingbodyLifter();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
    virtual void UpdateChild();
    virtual void FiniChild();

private:
    GazeboRosPtr gazebo_ros_;
    physics::ModelPtr parent;
    //void publishOdometry(double step_time);
    void publishTF(); /// publishes the wheel tf's
    void publishJointState();
    void motorController(double target_tippingbody, double dt);

    event::ConnectionPtr update_connection_;

    physics::JointPtr joint_tippingbody_;

    double tippingbody_torque_;

    std::string robot_namespace_;
    std::string command_topic_;
    std::string robot_base_frame_;

    // ROS STUFF
    ros::Subscriber cmd_tippingbody_subscriber_;
    boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
    sensor_msgs::JointState joint_state_;
    ros::Publisher joint_state_publisher_;

    boost::mutex lock;

    // Custom Callback Queue
    ros::CallbackQueue queue_;
    boost::thread callback_queue_thread_;
    void QueueThread();

    // DiffDrive stuff
    void cmdTippingbodyCallback(const geometry_msgs::Point::ConstPtr& cmd_msg);

    /// updates the relative robot pose based on the wheel encoders
    void UpdateTippingbodyEncoder();

    double tippingbody_cmd_;

    bool alive_;
    double tippingbody_encoder_;
    common::Time last_encoder_update_;
    common::PID  joint_pid_;

    // Update Rate
    double update_rate_;
    double update_period_;
    common::Time last_actuator_update_;

    // Flags
    bool publishTF_;
    bool publishJointState_;

    bool use_velocity_control_;
    double max_velocity_;
};

}

#endif

