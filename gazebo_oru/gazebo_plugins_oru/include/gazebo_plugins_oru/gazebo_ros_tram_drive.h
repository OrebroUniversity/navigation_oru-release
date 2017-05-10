#ifndef TRAM_DRIVE_PLUGIN_HH
#define TRAM_DRIVE_PLUGIN_HH

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


class GazeboRosTramDrive : public ModelPlugin {

    class TramDriveCmd {
    public:
        TramDriveCmd():speed(0), angle(0) {};
        double speed;
        double angle;
    };

    enum OdomSource
    {
        ENCODER = 0,
        WORLD = 1,
    };
public:
    GazeboRosTramDrive();
    ~GazeboRosTramDrive();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
    virtual void UpdateChild();
    virtual void FiniChild();

private:
    GazeboRosPtr gazebo_ros_;
    physics::ModelPtr parent;
    void publishOdometry(double step_time);
    void publishWheelTF(); /// publishes the wheel tf's
    void publishWheelJointState();
    void motorController(double target_speed, double target_steering_speed, double dt);

    event::ConnectionPtr update_connection_;

    physics::JointPtr joint_drive_CSNDE3_;
    physics::JointPtr joint_drive_NCSNDE3_;

    physics::JointPtr joint_steer_wheel_CSDE3_;
    physics::JointPtr joint_steer_wheel_NCSDE3_;
    physics::JointPtr joint_steer_wheel_CSNDE3_;
    physics::JointPtr joint_steer_wheel_NCSNDE3_;

    double wheel_diameter_;
    double wheel_acceleration_;
    double wheel_deceleration_;
    double wheel_speed_tolerance_;
    double steering_angle_tolerance_;
    double steering_speed_;
    double separation_encoder_wheel_;

    OdomSource odom_source_;
    double drive_torque_;

    std::string robot_namespace_;
    std::string command_topic_;
    std::string odometry_topic_;
    std::string odometry_frame_;
    std::string robot_base_frame_;

    // ROS STUFF
    ros::Publisher odometry_publisher_;
    ros::Subscriber cmd_vel_subscriber_;
    boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
    sensor_msgs::JointState joint_state_;
    ros::Publisher joint_state_publisher_;
    nav_msgs::Odometry odom_;

    boost::mutex lock;


    // Custom Callback Queue
    ros::CallbackQueue queue_;
    boost::thread callback_queue_thread_;
    void QueueThread();

    // DiffDrive stuff
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

    /// updates the relative robot pose based on the wheel encoders
    void UpdateOdometryEncoder();

    TramDriveCmd cmd_;
    bool alive_;
    geometry_msgs::Pose2D pose_encoder_;
    common::Time last_odom_update_;

    // Update Rate
    double update_rate_;
    double update_period_;
    common::Time last_actuator_update_;

    // Flags
    bool publishWheelTF_;
    bool publishWheelJointState_;

};

}

#endif

