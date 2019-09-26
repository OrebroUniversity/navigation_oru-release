/**
 * \file  gazebo_ros_steer_drive.cpp
 * \brief A fork lifter plugin for gazebo - taken from the old SAUNA repo.
 * \author  Henrik Andreasson <henrik.andreasson@oru.se>
 */


#include <algorithm>
#include <assert.h>

#include <gazebo_plugins_oru/gazebo_ros_fork_lifter.h>

//#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo
{

GazeboRosForkLifter::GazeboRosForkLifter() {}

// Destructor
GazeboRosForkLifter::~GazeboRosForkLifter() {}

// Load the controller
void GazeboRosForkLifter::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "ForkLifter" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<double> ( fork_torque_, "forkTorque", 1000.0 );
    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_fork" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_link" );

    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );
    // Gazebo specific height to align the forks to the origin.
    gazebo_ros_->getParameter<double> ( fork_height_origin_, "forkHeightOrigin", 0.08 );

    gazebo_ros_->getParameterBoolean ( publishTF_, "publishTF", false );
    gazebo_ros_->getParameterBoolean ( publishJointState_, "publishJointState", false );

    gazebo_ros_->getParameterBoolean ( use_velocity_control_, "useVelocityControl", true );
    gazebo_ros_->getParameter<double> ( max_velocity_, "maxVelocity", 0.08 );

    double pid_p, pid_i, pid_d;
    gazebo_ros_->getParameter<double> ( pid_p, "pidP", 2000 );
    gazebo_ros_->getParameter<double> ( pid_i, "pidI", 0 );
    gazebo_ros_->getParameter<double> ( pid_d, "pidD", 10 );

    joint_fork_ = gazebo_ros_->getJoint ( parent, "forkJoint", "fork_joint" );
#if GAZEBO_MAJOR_VERSION > 2
    joint_fork_->SetParam ( "fmax", 0, fork_torque_);
#else
    joint_fork_->SetMaxForce ( 0, fork_torque_ );
#endif
    
    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;
#if GAZEBO_MAJOR_VERSION >= 8
    last_actuator_update_ = parent->GetWorld()->SimTime();
#else
    last_actuator_update_ = parent->GetWorld()->GetSimTime();
#endif
    
    // Initialize velocity stuff
    alive_ = true;

    //initialize PID
    if (use_velocity_control_)
      this->joint_pid_.Init( pid_p, pid_i, pid_d, 100, -100, max_velocity_, -max_velocity_);
    else
      this->joint_pid_.Init( pid_p, pid_i, pid_d, 100, -100, fork_torque_, -fork_torque_);


    if ( this->publishJointState_ ) {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState> ( "joint_states", 1000 );
        ROS_INFO ( "%s: Advertise joint_states!", gazebo_ros_->info() );
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster> ( new tf::TransformBroadcaster() );

    // ROS: Subscribe to the velocity command topic (cmd_fork)
    ROS_INFO ( "%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str() );

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Point> ( command_topic_, 1,
                boost::bind ( &GazeboRosForkLifter::cmdForkCallback, this, _1 ),
                ros::VoidPtr(), &queue_ );

    cmd_fork_subscriber_ = gazebo_ros_->node()->subscribe ( so );
    ROS_INFO ( "%s: Subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str() );

    // odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry> ( odometry_topic_, 1 );
    // ROS_INFO ( "%s: Advertise odom on %s !", gazebo_ros_->info(), odometry_topic_.c_str() );

    // start custom queue for diff drive
    this->callback_queue_thread_ = boost::thread ( boost::bind ( &GazeboRosForkLifter::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosForkLifter::UpdateChild, this ) );

    fork_height_cmd_ = 0.;
}

void GazeboRosForkLifter::publishJointState()
{
    std::vector<physics::JointPtr> joints;
    joints.push_back ( joint_fork_ );

    ros::Time current_time = ros::Time::now();
    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( joints.size() );
    joint_state_.position.resize ( joints.size() );
    joint_state_.velocity.resize ( joints.size() );
    joint_state_.effort.resize ( joints.size() );
    for ( std::size_t i = 0; i < joints.size(); i++ ) {
        joint_state_.name[i] = joints[i]->GetName();
#if GAZEBO_MAJOR_VERSION >= 8
	joint_state_.position[i] = joints[i]->Position ( 0 );
#else
	joint_state_.position[i] = joints[i]->GetAngle ( 0 ).Radian();
#endif
        joint_state_.velocity[i] = joints[i]->GetVelocity ( 0 );
        joint_state_.effort[i] = joints[i]->GetForce ( 0 );
    }
    joint_state_publisher_.publish ( joint_state_ );
}

void GazeboRosForkLifter::publishTF()
{
    ros::Time current_time = ros::Time::now();
    std::vector<physics::JointPtr> joints;
    joints.push_back ( joint_fork_ );

    for ( std::size_t i = 0; i < joints.size(); i++ ) {
        std::string frame = gazebo_ros_->resolveTF ( joints[i]->GetName() );
        std::string parent_frame = gazebo_ros_->resolveTF ( joints[i]->GetParent()->GetName() );

#if GAZEBO_MAJOR_VERSION >= 8
	ignition::math::Pose3d pose = joints[i]->GetChild()->RelativePose();
#else
        ignition::math::Pose3d pose = joints[i]->GetChild()->GetRelativePose().Ign();
#endif
   
	tf::Quaternion qt ( pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W() );
	tf::Vector3 vt ( pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() );

        tf::Transform transform ( qt, vt );
        transform_broadcaster_->sendTransform ( tf::StampedTransform ( transform, current_time, parent_frame, frame ) );
    }

}
// Update the controller
void GazeboRosForkLifter::UpdateChild()
{
    UpdateForkEncoder();
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    
    double seconds_since_last_update = ( current_time - last_actuator_update_ ).Double();
    if ( seconds_since_last_update > update_period_ ) {

        if ( publishJointState_ ) publishJointState();

        double target_fork_height = fork_height_cmd_ + this->fork_height_origin_;

        motorController ( target_fork_height, seconds_since_last_update );
        last_actuator_update_ += common::Time ( update_period_ );
    }

#if 0
    if ( odom_source_ == ENCODER ) UpdateOdometryEncoder();
    common::Time current_time = parent->GetWorld()->GetSimTime();
    double seconds_since_last_update = ( current_time - last_actuator_update_ ).Double();
    if ( seconds_since_last_update > update_period_ ) {

        publishOdometry ( seconds_since_last_update );
        if ( publishWheelTF_ ) publishWheelTF();
        if ( publishWheelJointState_ ) publishWheelJointState();

        double target_wheel_roation_speed = cmd_.speed / ( wheel_diameter_ / 2.0 );
        double target_steering_angle_speed = cmd_.angle;

        motorController ( target_wheel_roation_speed, target_steering_angle_speed, seconds_since_last_update );

        //ROS_INFO("v = %f, w = %f !", target_wheel_roation_speed, target_steering_angle);

        last_actuator_update_ += common::Time ( update_period_ );
    }
#endif
}


void GazeboRosForkLifter::motorController ( double target_fork_height, double dt )
{
  // Use the PID class...
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Pose3d pose = joint_fork_->GetChild()->RelativePose();
#else
  ignition::math::Pose3d pose = joint_fork_->GetChild()->GetRelativePose().Ign();
#endif
  double current_fork_height = pose.Pos().Z();

  
  double error = current_fork_height - target_fork_height;
  double control_value = this->joint_pid_.Update(error, dt);
#if GAZEBO_MAJOR_VERSION > 2
  if (use_velocity_control_)
    joint_fork_->SetParam ( "vel", 0, control_value );
  else
    joint_fork_->SetForce(0, control_value);

#else
  if (use_velocity_control_)
    joint_fork_->SetVelocity(0, control_value );
  else
    joint_fork_->SetForce(0, control_value);
#endif
}

// Finalize the controller
void GazeboRosForkLifter::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void GazeboRosForkLifter::cmdForkCallback ( const geometry_msgs::Point::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    fork_height_cmd_ = cmd_msg->z;
}

void GazeboRosForkLifter::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void GazeboRosForkLifter::UpdateForkEncoder()
{
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double step_time = ( current_time - last_encoder_update_ ).Double();
    last_encoder_update_ = current_time;
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Pose3d pose = joint_fork_->GetChild()->RelativePose();
#else
  ignition::math::Pose3d pose = joint_fork_->GetChild()->GetRelativePose().Ign();
#endif
  fork_height_encoder_ = pose.Pos().Z();
}


GZ_REGISTER_MODEL_PLUGIN ( GazeboRosForkLifter )
}

