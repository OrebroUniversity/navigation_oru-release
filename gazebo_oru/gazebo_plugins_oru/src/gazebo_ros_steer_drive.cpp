/**
 * \file  gazebo_ros_steer_drive.cpp
 * \brief A steer drive plugin for gazebo - taken from the old SAUNA repo.
 * \author  Henrik Andreasson <henrik.andreasson@oru.se>
 */


#include <algorithm>
#include <assert.h>

#include <gazebo_plugins_oru/gazebo_ros_steer_drive.h>

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
  DRIVE,
  STEER,
};

GazeboRosSteerDrive::GazeboRosSteerDrive() {}

// Destructor
GazeboRosSteerDrive::~GazeboRosSteerDrive() {}

// Load the controller
void GazeboRosSteerDrive::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "SteerDrive" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<double> ( wheel_diameter_, "wheelDiameter", 0.16 );
    gazebo_ros_->getParameter<double> ( steer_torque_, "steerTorque", 5.0 );
    gazebo_ros_->getParameter<double> ( drive_torque_, "driveTorque", 5.0 );
    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
    gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom" );
    gazebo_ros_->getParameter<std::string> ( odometry_enc_topic_, "odometryEncTopic", "odom_enc" );
    gazebo_ros_->getParameter<std::string> ( odometry_enc_child_frame_, "odometryEncChildFrame", "odom_enc" );

    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_link" );

    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );
    gazebo_ros_->getParameter<double> ( wheel_acceleration_, "wheelAcceleration", 0 );
    gazebo_ros_->getParameter<double> ( wheel_deceleration_, "wheelDeceleration", wheel_acceleration_ );
    gazebo_ros_->getParameter<double> ( steering_fix_wheel_distance_x_, "steeringFixWheelDistanceX", 0.68 );
    gazebo_ros_->getParameter<double> ( steering_fix_wheel_distance_y_, "steeringFixWheelDistanceY", 0. );
    gazebo_ros_->getParameter<double> ( odom_enc_steering_angle_offset_, "odomEncSteeringAngleOffset", 0. );
    
    gazebo_ros_->getParameterBoolean ( publishWheelTF_, "publishWheelTF", false );
    gazebo_ros_->getParameterBoolean ( publishWheelJointState_, "publishWheelJointState", false );
    bool init_odom_enc;
    gazebo_ros_->getParameterBoolean ( init_odom_enc, "initialize_odom_enc", true );
    odom_enc_initialized_ = !init_odom_enc;

    joint_steer_ = gazebo_ros_->getJoint ( parent, "steerJoint", "steer_joint" );
    joint_drive_ = gazebo_ros_->getJoint ( parent, "driveJoint", "drive_joint" );
    
    // Note, this joints are not used for anything apart from getting the TF's.
    joint_fixed_wheel_left_ = gazebo_ros_->getJoint ( parent, "fixedWheelLeftJoint", "fixed_wheel_left_joint" );
    joint_fixed_wheel_right_ = gazebo_ros_->getJoint ( parent, "fixedWheelRightJoint", "fixed_wheel_right_joint" );

    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );

#if GAZEBO_MAJOR_VERSION > 2
    joint_drive_->SetParam ( "fmax", 0, drive_torque_ );
    joint_steer_->SetParam ( "fmax", 0, steer_torque_ );
#else
    joint_drive_->SetMaxForce ( 0, drive_torque_ );
    joint_steer_->SetMaxForce ( 0, steer_torque_ );
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
                boost::bind ( &GazeboRosSteerDrive::cmdVelCallback, this, _1 ),
                ros::VoidPtr(), &queue_ );

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe ( so );
    ROS_INFO ( "%s: Subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str() );

    odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry> ( odometry_topic_, 1 );
    ROS_INFO ( "%s: Advertise odom on %s !", gazebo_ros_->info(), odometry_topic_.c_str() );

    odometry_enc_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry> ( odometry_enc_topic_, 1 );
    ROS_INFO ( "%s: Advertise odom on %s !", gazebo_ros_->info(), odometry_enc_topic_.c_str() );

    // Todo add state publisher here(!)...

    // start custom queue for diff drive
    this->callback_queue_thread_ = boost::thread ( boost::bind ( &GazeboRosSteerDrive::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosSteerDrive::UpdateChild, this ) );

}

void GazeboRosSteerDrive::publishWheelJointState()
{
    std::vector<physics::JointPtr> joints;
    joints.push_back ( joint_drive_ );
    joints.push_back ( joint_steer_ );
    joints.push_back( joint_fixed_wheel_left_ );
    joints.push_back( joint_fixed_wheel_right_ );

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

void GazeboRosSteerDrive::publishWheelTF()
{
    ros::Time current_time = ros::Time::now();
    std::vector<physics::JointPtr> joints;
    joints.push_back ( joint_steer_ );
    joints.push_back ( joint_drive_ );
    joints.push_back( joint_fixed_wheel_left_ );
    joints.push_back( joint_fixed_wheel_right_ );

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
void GazeboRosSteerDrive::UpdateChild()
{
    UpdateOdometryEncoder();
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

        double target_wheel_rotation_speed = cmd_.speed / ( wheel_diameter_ / 2.0 );
        double target_steering_angle_speed = cmd_.angle;

        motorController ( target_wheel_rotation_speed, target_steering_angle_speed, seconds_since_last_update );

	//        ROS_INFO("v = %f, w = %f !", target_wheel_rotation_speed, target_steering_angle_speed);

        last_actuator_update_ += common::Time ( update_period_ );
    }
}


void GazeboRosSteerDrive::motorController ( double target_speed, double target_steering_speed, double dt )
{
  // TODO add the accelerations etc. properly
    double applied_speed = target_speed;
    double applied_steering_speed = target_steering_speed;

    double current_speed = joint_drive_->GetVelocity ( 0 );
    double current_steering_speed = joint_steer_->GetVelocity ( 0 );
    if ( wheel_acceleration_ > 0 ) {
      // TODO
      // applied_speed = ...;
      // applied_steering_speed = ...;
    }
#if GAZEBO_MAJOR_VERSION > 2
    joint_drive_->SetParam ( "vel", 0, applied_speed );
    joint_steer_->SetParam ( "vel", 0, applied_steering_speed );
#else
    joint_drive_->SetVelocity ( 0, applied_speed );
    joint_steer_->SetVelocity ( 0, applied_steering_speed );
#endif    
    // ROS_INFO ( "target: [%3.2f, %3.2f], current: [%3.2f, %3.2f]", 
    //    	       target_speed, target_steering_speed, 
    //    	       current_speed, current_steering_speed );
}

// Finalize the controller
void GazeboRosSteerDrive::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void GazeboRosSteerDrive::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    cmd_.speed = cmd_msg->linear.x;
    cmd_.angle = cmd_msg->angular.z;
}

void GazeboRosSteerDrive::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void GazeboRosSteerDrive::UpdateOdometryEncoder()
{
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double step_time = ( current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;

#if GAZEBO_MAJOR_VERSION >= 8
    double odom_alpha = joint_steer_->Position(0);
#else
    double odom_alpha = joint_steer_->GetAngle(0).Radian();
#endif
    odom_alpha += odom_enc_steering_angle_offset_;

    // Distance travelled drive wheel
    double drive_dist = step_time * wheel_diameter_ / 2 * joint_drive_->GetVelocity(0);

    double dd = 0.;
    double da = 0.;
    
    if (fabs(odom_alpha) < 0.000001) // Avoid dividing with a very small number...
      {
	dd = drive_dist;
	da = 0.;
      }
    else
      {
	double r_stear = steering_fix_wheel_distance_x_ / sin(odom_alpha);
	double r_fix = r_stear * cos(odom_alpha) - steering_fix_wheel_distance_y_;
	
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

    odom_enc_.pose.pose.position.x = vt.x();
    odom_enc_.pose.pose.position.y = vt.y();
    odom_enc_.pose.pose.position.z = vt.z();

    odom_enc_.pose.pose.orientation.x = qt.x();
    odom_enc_.pose.pose.orientation.y = qt.y();
    odom_enc_.pose.pose.orientation.z = qt.z();
    odom_enc_.pose.pose.orientation.w = qt.w();

    odom_enc_.twist.twist.angular.z = w;
    odom_enc_.twist.twist.linear.x = dx/step_time;
    odom_enc_.twist.twist.linear.y = dy/step_time;
}

void GazeboRosSteerDrive::publishOdometry ( double step_time )
{

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
    std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );

    tf::Quaternion qt;
    tf::Vector3 vt;

    if ( odom_source_ == ENCODER ) {
        // getting data form encoder integration
        odom_ = odom_enc_;
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

    // publish the encoder based odometry
    {    

      if (!odom_enc_initialized_) 
      {
        pose_encoder_.x = odom_.pose.pose.position.x;
        pose_encoder_.y = odom_.pose.pose.position.y;
        pose_encoder_.theta = tf::getYaw(odom_.pose.pose.orientation);
        odom_enc_initialized_ = true;
        
        odom_enc_ = odom_;
      }

      qt = tf::Quaternion ( odom_enc_.pose.pose.orientation.x, odom_enc_.pose.pose.orientation.y, odom_enc_.pose.pose.orientation.z, odom_enc_.pose.pose.orientation.w );
      vt = tf::Vector3 ( odom_enc_.pose.pose.position.x, odom_enc_.pose.pose.position.y, odom_enc_.pose.pose.position.z );


      std::string odom_enc_child_frame = gazebo_ros_->resolveTF ( odometry_enc_child_frame_ );

      tf::Transform odom_enc_child_to_odom ( qt, vt );
      transform_broadcaster_->sendTransform (
                                             tf::StampedTransform ( odom_enc_child_to_odom, current_time,
                                                                    odom_frame, odom_enc_child_frame ) );
      
      
      odom_enc_.pose.covariance = odom_.pose.covariance; // TODO...
      odom_enc_.header.stamp = current_time;
      odom_enc_.header.frame_id = odom_frame; // Note - this is typically /world
      odom_enc_.child_frame_id = odom_enc_child_frame;
      odometry_enc_publisher_.publish ( odom_enc_ );
    }
}




GZ_REGISTER_MODEL_PLUGIN ( GazeboRosSteerDrive )
}

