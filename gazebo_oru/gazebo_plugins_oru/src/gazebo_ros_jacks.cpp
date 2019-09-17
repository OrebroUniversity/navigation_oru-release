/**
 * \file  gazebo_ros_steer_drive.cpp
 * \brief A plugin for jacks for gazebo - copyed from the fork lifter plugin.
 * \author  Robert Lundh <robert.lundh@se.atlascopco.com>
 */


#include <algorithm>
#include <assert.h>

#include <gazebo_plugins_oru/gazebo_ros_jacks.h>

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

GazeboRosJacks::GazeboRosJacks() {}

// Destructor
GazeboRosJacks::~GazeboRosJacks() {}

// Load the controller
void GazeboRosJacks::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "Jacks" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<double> ( jack_CSDE_torque_, "jackCSDETorque", 1000.0 );
    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_jack_CSDE" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_link" );

    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );
    // Gazebo specific height to align the jacks to the origin.
    gazebo_ros_->getParameter<double> ( jack_CSDE_height_origin_, "jackCSDEHeightOrigin", 0.08 );

    gazebo_ros_->getParameterBoolean ( publishTF_, "publishTF", false );
    gazebo_ros_->getParameterBoolean ( publishJointState_, "publishJointState", false );

    gazebo_ros_->getParameterBoolean ( use_velocity_control_, "useVelocityControl", true );
    gazebo_ros_->getParameter<double> ( max_velocity_, "maxVelocity", 0.08 );

    double pid_p, pid_i, pid_d;
    gazebo_ros_->getParameter<double> ( pid_p, "pidP", 2000 );
    gazebo_ros_->getParameter<double> ( pid_i, "pidI", 0 );
    gazebo_ros_->getParameter<double> ( pid_d, "pidD", 10 );
    joint_jack_CSDE_ = gazebo_ros_->getJoint ( parent, "jackCSDEJoint", "jack_CSDE_joint" );
    joint_jack_CSNDE_ = gazebo_ros_->getJoint ( parent, "jackCSNDEJoint", "jack_CSNDE_joint" );
    joint_jack_NCSDE_ = gazebo_ros_->getJoint ( parent, "jackNCSDEJoint", "jack_NCSDE_joint" );
    joint_jack_NCSNDE_ = gazebo_ros_->getJoint ( parent, "jackNCSNDEJoint", "jack_NCSNDE_joint" );
#if GAZEBO_MAJOR_VERSION > 2
    joint_jack_CSDE_->SetParam ( "fmax", 0, jack_CSDE_torque_ );
    joint_jack_CSNDE_->SetParam ( "fmax", 0, jack_CSDE_torque_ );
    joint_jack_NCSDE_->SetParam ( "fmax", 0, jack_CSDE_torque_ );
    joint_jack_NCSNDE_->SetParam ( "fmax", 0, jack_CSDE_torque_ );
#else
    joint_jack_CSDE_->SetMaxForce ( 0, jack_CSDE_torque_ );
    joint_jack_CSNDE_->SetMaxForce ( 0, jack_CSDE_torque_ );
    joint_jack_NCSDE_->SetMaxForce ( 0, jack_CSDE_torque_ );
    joint_jack_NCSNDE_->SetMaxForce ( 0, jack_CSDE_torque_ );
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
      this->joint_pid_.Init( pid_p, pid_i, pid_d, 100, -100, jack_CSDE_torque_, -jack_CSDE_torque_);


    if ( this->publishJointState_ ) {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState> ( "joint_states", 1000 );
        ROS_INFO ( "%s: Advertise joint_states!", gazebo_ros_->info() );
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster> ( new tf::TransformBroadcaster() );

    // ROS: Subscribe to the velocity command topic (cmd_jack_CSDE)
    ROS_INFO ( "%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str() );

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Point> ( command_topic_, 1,
                boost::bind ( &GazeboRosJacks::cmdJackCSDECallback, this, _1 ),
                ros::VoidPtr(), &queue_ );

    cmd_jack_CSDE_subscriber_ = gazebo_ros_->node()->subscribe ( so );
    ROS_INFO ( "%s: Subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str() );

    // odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry> ( odometry_topic_, 1 );
    // ROS_INFO ( "%s: Advertise odom on %s !", gazebo_ros_->info(), odometry_topic_.c_str() );

    // start custom queue for diff drive
    this->callback_queue_thread_ = boost::thread ( boost::bind ( &GazeboRosJacks::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosJacks::UpdateChild, this ) );

    jack_CSDE_height_cmd_ = 0.;
}

void GazeboRosJacks::publishJointState()
{
    std::vector<physics::JointPtr> joints;
    joints.push_back ( joint_jack_CSDE_ );
    joints.push_back ( joint_jack_CSNDE_ );
    joints.push_back ( joint_jack_NCSDE_ );
    joints.push_back ( joint_jack_NCSNDE_ );

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

void GazeboRosJacks::publishTF()
{
    ros::Time current_time = ros::Time::now();
    std::vector<physics::JointPtr> joints;
    joints.push_back ( joint_jack_CSDE_ );
    joints.push_back ( joint_jack_CSNDE_ );
    joints.push_back ( joint_jack_NCSDE_ );
    joints.push_back ( joint_jack_NCSNDE_ );

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
void GazeboRosJacks::UpdateChild()
{
    UpdateJackCSDEEncoder();
    #if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_actuator_update_ ).Double();
    if ( seconds_since_last_update > update_period_ ) {

        if ( publishJointState_ ) publishJointState();

        double target_jack_CSDE_height = jack_CSDE_height_cmd_ + this->jack_CSDE_height_origin_;

        motorController ( target_jack_CSDE_height, seconds_since_last_update );
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


void GazeboRosJacks::motorController ( double target_jack_CSDE_height, double dt )
{
  // Use the PID class...
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Pose3d pose = joint_jack_CSDE_->GetChild()->RelativePose();
#else
  ignition::math::Pose3d pose = joint_jack_CSDE_->GetChild()->GetRelativePose().Ign();
#endif
  double current_jack_CSDE_height =  pose.Pos().Z();
  double error = current_jack_CSDE_height - target_jack_CSDE_height;
  double control_value = this->joint_pid_.Update(error, dt);

  if (use_velocity_control_) {
#if GAZEBO_MAJOR_VERSION > 2
    joint_jack_CSDE_->SetParam("vel", 0, control_value );
    joint_jack_CSNDE_->SetParam("vel", 0, control_value );
    joint_jack_NCSDE_->SetParam("vel", 0, control_value );
    joint_jack_NCSNDE_->SetParam("vel", 0, control_value );
#else    
    joint_jack_CSDE_->SetVelocity(0, control_value );
    joint_jack_CSNDE_->SetVelocity(0, control_value );
    joint_jack_NCSDE_->SetVelocity(0, control_value );
    joint_jack_NCSNDE_->SetVelocity(0, control_value );
#endif
  }
  else {
    joint_jack_CSDE_->SetForce(0, control_value);
    joint_jack_CSNDE_->SetForce(0, control_value);
    joint_jack_NCSDE_->SetForce(0, control_value);
    joint_jack_NCSNDE_->SetForce(0, control_value);
  }
}

// Finalize the controller
void GazeboRosJacks::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void GazeboRosJacks::cmdJackCSDECallback ( const geometry_msgs::Point::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    jack_CSDE_height_cmd_ = cmd_msg->z;
}

void GazeboRosJacks::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void GazeboRosJacks::UpdateJackCSDEEncoder()
{
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double step_time = ( current_time - last_encoder_update_ ).Double();
    last_encoder_update_ = current_time;

#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = joint_jack_CSDE_->GetChild()->RelativePose();
#else
    ignition::math::Pose3d pose = joint_jack_CSDE_->GetChild()->GetRelativePose().Ign();
#endif
    jack_CSDE_height_encoder_ = pose.Pos().Z();
}

#if 0
void GazeboRosJacks::publishOdometry ( double step_time )
{

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
    std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );

    tf::Quaternion qt;
    tf::Vector3 vt;

    if ( odom_source_ == ENCODER ) {
        // getting data form encoder integration
        qt = tf::Quaternion ( odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
        vt = tf::Vector3 ( odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );

    }
    if ( odom_source_ == WORLD ) {
        // getting data form gazebo world
        math::Pose pose = parent->GetWorldPose();
        qt = tf::Quaternion ( pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w );
        vt = tf::Vector3 ( pose.pos.x, pose.pos.y, pose.pos.z );

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        // get velocity in /odom frame
        math::Vector3 linear;
        linear = parent->GetWorldLinearVel();
        odom_.twist.twist.angular.z = parent->GetWorldAngularVel().z;

        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.rot.GetYaw();
	odom_.twist.twist.linear.x = cosf ( yaw ) * linear.X() + sinf ( yaw ) * linear.Y();
	odom_.twist.twist.linear.y = cosf ( yaw ) * linear.Y() - sinf ( yaw ) * linear.X();
    }

    tf::Transform base_footprint_to_odom ( qt, vt );
    transform_broadcaster_->sendTransform (
        tf::StampedTransform ( base_footprint_to_odom, current_time,
                               odom_frame, base_footprint_frame ) );


    // set covariance - TODO, fix this(!)
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;


    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish ( odom_ );
}
#endif

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosJacks )
}

