/**
 * \file  gazebo_ros_tram_drive.cpp
 * \brief A steer drive plugin for gazebo - taken from the old SAUNA repo.
 * \author  Henrik Andreasson <henrik.andreasson@oru.se>
 */


#include <algorithm>
#include <assert.h>

#include <gazebo_plugins_oru/gazebo_ros_tram_drive.h>

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

enum {
  DRIVE
};

GazeboRosTramDrive::GazeboRosTramDrive() {}

// Destructor
GazeboRosTramDrive::~GazeboRosTramDrive() {}

// Load the controller
void GazeboRosTramDrive::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "TramDrive" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<double> ( wheel_diameter_, "wheelDiameter", 1.2 );
    gazebo_ros_->getParameter<double> ( drive_torque_, "driveTorque", 50.0 );
    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
    gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_link" );

    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );
    gazebo_ros_->getParameter<double> ( wheel_acceleration_, "wheelAcceleration", 0 );
    gazebo_ros_->getParameter<double> ( wheel_deceleration_, "wheelDeceleration", wheel_acceleration_ );

    gazebo_ros_->getParameterBoolean ( publishWheelTF_, "publishWheelTF", false );
    gazebo_ros_->getParameterBoolean ( publishWheelJointState_, "publishWheelJointState", false );

    joint_drive_CSNDE3_ = gazebo_ros_->getJoint ( parent, "driveJointCS", "drive_wheel_CS_joint" );
    joint_drive_NCSNDE3_ = gazebo_ros_->getJoint ( parent, "driveJointNCS", "drive_wheel_NCS_joint" );
    
    // Note, this joints are not used for anything apart from getting the TF's.
    joint_steer_wheel_CSDE3_ = gazebo_ros_->getJoint ( parent, "steerWheelCSDE3", "steer2sd_wheel_CSDE3_joint" );
    joint_steer_wheel_NCSDE3_ = gazebo_ros_->getJoint ( parent, "steerWheelNCSDE3", "steer2sd_wheel_NCSDE3_joint" );
    joint_steer_wheel_CSNDE3_ = gazebo_ros_->getJoint ( parent, "steerWheelCSNDE3", "steer2sd_wheel_CSNDE3_joint" );
    joint_steer_wheel_NCSNDE3_ = gazebo_ros_->getJoint ( parent, "steerWheelNCSNDE3", "steer2sd_wheel_NCSNDE3_joint" );
    

    // joint_fixed_wheel_CSNDE2_ = gazebo_ros_->getJoint ( parent, "fixedWheelCSNDE2Joint", "fixed_wheel_CSNDE2_joint" );
    // joint_fixed_wheel_CSNDE1_ = gazebo_ros_->getJoint ( parent, "fixedWheelCSNDE1Joint", "fixed_wheel_CSNDE1_joint" );
    // joint_fixed_wheel_CSDE3_ = gazebo_ros_->getJoint ( parent, "fixedWheelCSDE3Joint", "fixed_wheel_CSDE3_joint" );
    // joint_fixed_wheel_CSDE2_ = gazebo_ros_->getJoint ( parent, "fixedWheelCSDE2Joint", "fixed_wheel_CSDE2_joint" );
    // joint_fixed_wheel_CSDE1_ = gazebo_ros_->getJoint ( parent, "fixedWheelCSDE1Joint", "fixed_wheel_CSDE1_joint" );

    // joint_fixed_wheel_NCSNDE2_ = gazebo_ros_->getJoint ( parent, "fixedWheelNCSNDE2Joint", "fixed_wheel_NCSNDE2_joint" );
    // joint_fixed_wheel_NCSNDE1_ = gazebo_ros_->getJoint ( parent, "fixedWheelNCSNDE1Joint", "fixed_wheel_NCSNDE1_joint" );
    // joint_fixed_wheel_NCSDE3_ = gazebo_ros_->getJoint ( parent, "fixedWheelNCSDE3Joint", "fixed_wheel_NCSDE3_joint" );
    // joint_fixed_wheel_NCSDE2_ = gazebo_ros_->getJoint ( parent, "fixedWheelNCSDE2Joint", "fixed_wheel_NCSDE2_joint" );
    // joint_fixed_wheel_NCSDE1_ = gazebo_ros_->getJoint ( parent, "fixedWheelNCSDE1Joint", "fixed_wheel_NCSDE1_joint" );

    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );

#if GAZEBO_MAJOR_VERSION > 2
    joint_drive_CSNDE3_->SetParam ( "fmax", 0, drive_torque_);
    joint_drive_NCSNDE3_->SetParam ( "fmax", 0, drive_torque_);
#else
    joint_drive_CSNDE3_->SetMaxForce ( 0, drive_torque_ );
    joint_drive_NCSNDE3_->SetMaxForce ( 0, drive_torque_ );
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

    if ( this->publishWheelJointState_ ) {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState> ( "joint_states", 1000 );
        ROS_INFO ( "%s: Advertise joint_states!", gazebo_ros_->info() );
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster> ( new tf::TransformBroadcaster() );

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO ( "%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str() );

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist> ( command_topic_, 1,
                boost::bind ( &GazeboRosTramDrive::cmdVelCallback, this, _1 ),
                ros::VoidPtr(), &queue_ );

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe ( so );
    ROS_INFO ( "%s: Subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str() );

    odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry> ( odometry_topic_, 1 );
    ROS_INFO ( "%s: Advertise odom on %s !", gazebo_ros_->info(), odometry_topic_.c_str() );

    // start custom queue for diff drive
    this->callback_queue_thread_ = boost::thread ( boost::bind ( &GazeboRosTramDrive::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosTramDrive::UpdateChild, this ) );

}

void GazeboRosTramDrive::publishWheelJointState()
{
    std::vector<physics::JointPtr> joints;
    joints.push_back ( joint_drive_CSNDE3_ );
    joints.push_back ( joint_drive_NCSNDE3_ );

    joints.push_back( joint_steer_wheel_CSDE3_ );
    joints.push_back( joint_steer_wheel_NCSDE3_ );
    joints.push_back( joint_steer_wheel_CSNDE3_ );
    joints.push_back( joint_steer_wheel_NCSNDE3_ );

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

void GazeboRosTramDrive::publishWheelTF()
{
    ros::Time current_time = ros::Time::now();
    std::vector<physics::JointPtr> joints;

    joints.push_back ( joint_drive_CSNDE3_ );
    joints.push_back ( joint_drive_NCSNDE3_ );

    joints.push_back( joint_steer_wheel_CSDE3_ );
    joints.push_back( joint_steer_wheel_NCSDE3_ );
    joints.push_back( joint_steer_wheel_CSNDE3_ );
    joints.push_back( joint_steer_wheel_NCSNDE3_ );

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
void GazeboRosTramDrive::UpdateChild()
{
    if ( odom_source_ == ENCODER ) UpdateOdometryEncoder();
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
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
}


void GazeboRosTramDrive::motorController ( double target_speed, double target_steering_speed, double dt )
{
  // TODO add the accelerations etc. properly
    double applied_cs_speed = target_speed;
    double applied_ncs_speed = target_speed;

    double cs_speed = joint_drive_CSNDE3_->GetVelocity ( 0 );
    double ncs_speed = joint_drive_NCSNDE3_->GetVelocity ( 0 );
    if ( wheel_acceleration_ > 0 ) {
      // TODO
      // applied_speed = ...;
      // applied_steering_speed = ...;
    }
    if ( target_steering_speed > 0 ) {
       applied_cs_speed = 0.1 * target_speed;
    }
    else if ( target_steering_speed < 0 ) {
       applied_ncs_speed = 0.1 * target_speed;
    }
#if GAZEBO_MAJOR_VERSION > 2
    joint_drive_CSNDE3_->SetParam ( "vel", 0, applied_cs_speed );
    joint_drive_NCSNDE3_->SetParam ( "vel", 0, applied_ncs_speed );
#else
    joint_drive_CSNDE3_->SetVelocity ( 0, applied_cs_speed );
    joint_drive_NCSNDE3_->SetVelocity ( 0, applied_ncs_speed );
#endif    
    ROS_INFO ( "target: [%3.2f, %3.2f], current: [%3.2f, %3.2f]",
               applied_cs_speed, applied_ncs_speed,
               cs_speed, ncs_speed );
}

// Finalize the controller
void GazeboRosTramDrive::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void GazeboRosTramDrive::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    cmd_.speed = cmd_msg->linear.x;
    cmd_.angle = cmd_msg->angular.z;
}

void GazeboRosTramDrive::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void GazeboRosTramDrive::UpdateOdometryEncoder()
{
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double step_time = ( current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;

    double odom_alpha = 0;

    // Distance travelled drive wheel
    double drive_dist = step_time * wheel_diameter_ / 2 * joint_drive_CSNDE3_->GetVelocity(0);

    double dd = 0.;
    double da = 0.;
    
    if (fabs(odom_alpha) < 0.000001) // Avoid dividing with a very small number...
      {
	dd = drive_dist;
	da = 0.;
      }
    else
      {
        // TODO: Fix odometry.
    double r_stear = 1;
    double r_fix =  1;
	
	dd = r_fix / r_stear * drive_dist; // Adjust the actual forward movement (located between the fixed front wheels) based on the radius of the drive wheel).
	da = drive_dist / r_stear;
      }
    
    // Update the current estimate
    double dx = dd * cos( pose_encoder_.theta + da / 2. );
    double dy = dd * sin( pose_encoder_.theta + da / 2. );

    // Compute odometric pose
    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += da;

    double w = da/step_time;
    double v = dd/step_time;

    tf::Quaternion qt;
    tf::Vector3 vt;
    qt.setRPY ( 0,0,pose_encoder_.theta );
    vt = tf::Vector3 ( pose_encoder_.x, pose_encoder_.y, 0 );

    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    odom_.twist.twist.angular.z = w;
    odom_.twist.twist.linear.x = dx/step_time;
    odom_.twist.twist.linear.y = dy/step_time;
}

void GazeboRosTramDrive::publishOdometry ( double step_time )
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
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = parent->WorldPose();
#else
        ignition::math::Pose3d pose = parent->GetWorldPose().Ign();
#endif
	qt = tf::Quaternion ( pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W() );
	vt = tf::Vector3 ( pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() );

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        // get velocity in /odom frame
	ignition::math::Vector3d linear;
#if GAZEBO_MAJOR_VERSION >= 8
	linear = parent->WorldLinearVel();
	odom_.twist.twist.angular.z = parent->WorldAngularVel().Z();
#else
	linear = parent->GetWorldLinearVel().Ign();
        odom_.twist.twist.angular.z = parent->GetWorldAngularVel().Ign().Z();
#endif

        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.Rot().Yaw();
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

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosTramDrive )
}

