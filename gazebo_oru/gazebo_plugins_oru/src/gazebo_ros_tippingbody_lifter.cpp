/**
 * \file  gazebo_ros_tippingbody_lifter.cpp
 * \brief A tippping body lifter plugin for gazebo (for volvo xa15)
 * \author  Henrik Andreasson <henrik.andreasson@oru.se>
 */


#include <algorithm>
#include <assert.h>

#include <gazebo_plugins_oru/gazebo_ros_tippingbody_lifter.h>

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

GazeboRosTippingbodyLifter::GazeboRosTippingbodyLifter() {}

// Destructor
GazeboRosTippingbodyLifter::~GazeboRosTippingbodyLifter() {}

// Load the controller
void GazeboRosTippingbodyLifter::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "TippingbodyLifter" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<double> ( tippingbody_torque_, "tippingbodyTorque", 1000.0 );
    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_tippingbody" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_link" );

    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );

    gazebo_ros_->getParameterBoolean ( publishTF_, "publishTF", false );
    gazebo_ros_->getParameterBoolean ( publishJointState_, "publishJointState", false );

    gazebo_ros_->getParameterBoolean ( use_velocity_control_, "useVelocityControl", true );
    gazebo_ros_->getParameter<double> ( max_velocity_, "maxVelocity", 0.08 );

    double pid_p, pid_i, pid_d;
    gazebo_ros_->getParameter<double> ( pid_p, "pidP", 2000 );
    gazebo_ros_->getParameter<double> ( pid_i, "pidI", 0 );
    gazebo_ros_->getParameter<double> ( pid_d, "pidD", 10 );
 
    joint_tippingbody_ = gazebo_ros_->getJoint ( parent, "tippingbodyJoint", "base2tippingbody_joint" );
#if GAZEBO_MAJOR_VERSION > 2
    joint_tippingbody_->SetParam ( "fmax", 0, tippingbody_torque_ );
#else
    joint_tippingbody_->SetMaxForce ( 0, tippingbody_torque_ );
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
      this->joint_pid_.Init( pid_p, pid_i, pid_d, 100, -100, tippingbody_torque_, -tippingbody_torque_);


    if ( this->publishJointState_ ) {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState> ( "joint_states", 1000 );
        ROS_INFO ( "%s: Advertise joint_states!", gazebo_ros_->info() );
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster> ( new tf::TransformBroadcaster() );

    // ROS: Subscribe to the velocity command topic (cmd_tippingbody)
    ROS_INFO ( "%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str() );

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Point> ( command_topic_, 1,
                boost::bind ( &GazeboRosTippingbodyLifter::cmdTippingbodyCallback, this, _1 ),
                ros::VoidPtr(), &queue_ );

    cmd_tippingbody_subscriber_ = gazebo_ros_->node()->subscribe ( so );
    ROS_INFO ( "%s: Subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str() );

    // odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry> ( odometry_topic_, 1 );
    // ROS_INFO ( "%s: Advertise odom on %s !", gazebo_ros_->info(), odometry_topic_.c_str() );

    // start custom queue for diff drive
    this->callback_queue_thread_ = boost::thread ( boost::bind ( &GazeboRosTippingbodyLifter::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosTippingbodyLifter::UpdateChild, this ) );

    tippingbody_cmd_ = 0.;
}

void GazeboRosTippingbodyLifter::publishJointState()
{
    std::vector<physics::JointPtr> joints;
    joints.push_back ( joint_tippingbody_ );

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

void GazeboRosTippingbodyLifter::publishTF()
{
    ros::Time current_time = ros::Time::now();
    std::vector<physics::JointPtr> joints;
    joints.push_back ( joint_tippingbody_ );

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
void GazeboRosTippingbodyLifter::UpdateChild()
{
    UpdateTippingbodyEncoder();
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_actuator_update_ ).Double();
    if ( seconds_since_last_update > update_period_ ) {

        if ( publishJointState_ ) publishJointState();

        double target_tippingbody = tippingbody_cmd_;

        motorController ( target_tippingbody, seconds_since_last_update );
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


void GazeboRosTippingbodyLifter::motorController ( double target_tippingbody, double dt )
{
  // Use the PID class...
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Pose3d pose = joint_tippingbody_->GetChild()->RelativePose();
#else
  ignition::math::Pose3d pose = joint_tippingbody_->GetChild()->GetRelativePose().Ign();
#endif
  double current_tippingbody = pose.Pos().Z();
  double error = current_tippingbody - target_tippingbody;
  double control_value = this->joint_pid_.Update(error, dt);

  if (use_velocity_control_)
#if GAZEBO_MAJOR_VERSION > 2
    joint_tippingbody_->SetParam("vel", 0, control_value );
#else
    joint_tippingbody_->SetVelocity(0, control_value );
#endif
  else
    joint_tippingbody_->SetForce(0, control_value);
}

// Finalize the controller
void GazeboRosTippingbodyLifter::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void GazeboRosTippingbodyLifter::cmdTippingbodyCallback ( const geometry_msgs::Point::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    tippingbody_cmd_ = cmd_msg->z;
}

void GazeboRosTippingbodyLifter::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void GazeboRosTippingbodyLifter::UpdateTippingbodyEncoder()
{
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double step_time = ( current_time - last_encoder_update_ ).Double();
    last_encoder_update_ = current_time;

#if GAZEBO_MAJOR_VERSION >= 8
    tippingbody_encoder_ = joint_tippingbody_->Position(0);
#else
    tippingbody_encoder_ = joint_tippingbody_->GetAngle(0).Radian();
#endif
}


GZ_REGISTER_MODEL_PLUGIN ( GazeboRosTippingbodyLifter )
}

