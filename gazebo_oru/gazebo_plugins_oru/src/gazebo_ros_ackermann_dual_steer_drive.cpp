/**
 * \file  gazebo_ros_ackermann_steer_drive.cpp
 * \brief ackermann plugin for gazebo - useful for th xa15 model, this version will utilize two pairs of steering wheels.
 * \author  Henrik Andreasson <henrik.andreasson@oru.se>
 */


#include <algorithm>
#include <assert.h>

#include <gazebo_plugins_oru/gazebo_ros_ackermann_dual_steer_drive.h>

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

GazeboRosAckermannDualSteerDrive::GazeboRosAckermannDualSteerDrive() {}

// Destructor
GazeboRosAckermannDualSteerDrive::~GazeboRosAckermannDualSteerDrive() {}

// Load the controller
void GazeboRosAckermannDualSteerDrive::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "AckermannDualSteerDrive" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<double> ( wheel_diameter_, "wheelDiameter", 0.59 );
    gazebo_ros_->getParameter<double> ( steer_torque_, "steerTorque", 5.0 );
    gazebo_ros_->getParameter<double> ( drive_torque_, "driveTorque", 5.0 );
    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
    gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_link" );

    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );
    gazebo_ros_->getParameter<double> ( wheel_acceleration_, "wheelAcceleration", 0 );
    gazebo_ros_->getParameter<double> ( wheel_deceleration_, "wheelDeceleration", wheel_acceleration_ );
    { 
      double x, y;
      gazebo_ros_->getParameter<double> ( x, "fl_wheel_pos_x_", 3.2 );
      gazebo_ros_->getParameter<double> ( y, "fl_wheel_pos_y_", 1.11 );
      ackermann_model_.addWheel(orunav_generic::SteerDriveWheel(x,y));
      gazebo_ros_->getParameter<double> ( x, "fr_wheel_pos_x_", 3.2 );
      gazebo_ros_->getParameter<double> ( y, "fr_wheel_pos_y_", -1.11 );
      ackermann_model_.addWheel(orunav_generic::SteerDriveWheel(x,y));
      gazebo_ros_->getParameter<double> ( x, "bl_wheel_pos_x_", 0.);
      gazebo_ros_->getParameter<double> ( y, "bl_wheel_pos_y_", 1.11 );
      ackermann_model_.addWheel(orunav_generic::SteerDriveWheel(x,y));
      gazebo_ros_->getParameter<double> ( x, "bl_wheel_pos_x_", 0.);
      gazebo_ros_->getParameter<double> ( y, "bl_wheel_pos_y_", -1.11 );
      ackermann_model_.addWheel(orunav_generic::SteerDriveWheel(x,y));
      gazebo_ros_->getParameter<double> ( x, "sd_wheel_pos_x_", 3.2);
      gazebo_ros_->getParameter<double> ( y, "sd_wheel_pos_y_", 0. );
      ackermann_model_.setSteerDriveWheelPos(x, y);
    }
      
    gazebo_ros_->getParameterBoolean ( publishWheelTF_, "publishWheelTF", false );
    gazebo_ros_->getParameterBoolean ( publishWheelJointState_, "publishWheelJointState", false );


    // Virual steering wheel
    joint_front_steer_ = gazebo_ros_->getJoint ( parent, "frontSteerJoint", "front_steer_joint" );
    joint_rear_steer_ = gazebo_ros_->getJoint ( parent, "rearSteerJoint", "rear_steer_joint" );
    
    // 4WD steering
    joint_wheel_fl_steer_ = gazebo_ros_->getJoint( parent, "wheelFrontLeftSteerJoint", "base2steering_fl_joint");
    joint_wheel_fr_steer_ = gazebo_ros_->getJoint( parent, "wheelFrontRightSteerJoint", "base2steering_fr_joint");
    joint_wheel_bl_steer_ = gazebo_ros_->getJoint( parent, "wheelBackLeftSteerJoint", "base2steering_bl_joint");
    joint_wheel_br_steer_ = gazebo_ros_->getJoint( parent, "wheelBackRightSteerJoint", "base2steering_br_joint");

    // 4WD
    joint_wheel_fl_drive_ = gazebo_ros_->getJoint( parent, "wheelFrontLeftDriveJoint", "steering2wheel_fl_joint");
    joint_wheel_fr_drive_ = gazebo_ros_->getJoint( parent, "wheelFrontRightDriveJoint", "steering2wheel_fr_joint");
    joint_wheel_bl_drive_ = gazebo_ros_->getJoint( parent, "wheelBackLeftDriveJoint", "steering2wheel_bl_joint");
    joint_wheel_br_drive_ = gazebo_ros_->getJoint( parent, "wheelBackRightDriveJoint", "steering2wheel_br_joint");
    
    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );

#if GAZEBO_MAJOR_VERSION > 2
    joint_front_steer_->SetParam ( "fmax", 0, steer_torque_ );
    joint_rear_steer_->SetParam ( "fmax", 0, steer_torque_ );

    joint_wheel_fl_steer_->SetParam ( "fmax", 0, steer_torque_ );
    joint_wheel_fr_steer_->SetParam ( "fmax", 0, steer_torque_ );
    joint_wheel_bl_steer_->SetParam ( "fmax", 0, steer_torque_ );
    joint_wheel_br_steer_->SetParam ( "fmax", 0, steer_torque_ );

    joint_wheel_fl_drive_->SetParam ( "fmax", 0, drive_torque_ );
    joint_wheel_fr_drive_->SetParam ( "fmax", 0, drive_torque_ );
    joint_wheel_bl_drive_->SetParam ( "fmax", 0, drive_torque_ );
    joint_wheel_br_drive_->SetParam ( "fmax", 0, drive_torque_ );
#else
    joint_front_steer_->SetMaxForce ( 0, steer_torque_ );
    joint_rear_steer_->SetMaxForce ( 0, steer_torque_ );

    joint_wheel_fl_steer_->SetMaxForce ( 0, steer_torque_ );
    joint_wheel_fr_steer_->SetMaxForce ( 0, steer_torque_ );
    joint_wheel_bl_steer_->SetMaxForce ( 0, steer_torque_ );
    joint_wheel_br_steer_->SetMaxForce ( 0, steer_torque_ );

    joint_wheel_fl_drive_->SetMaxForce ( 0, drive_torque_ );
    joint_wheel_fr_drive_->SetMaxForce ( 0, drive_torque_ );
    joint_wheel_bl_drive_->SetMaxForce ( 0, drive_torque_ );
    joint_wheel_br_drive_->SetMaxForce ( 0, drive_torque_ );
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
                boost::bind ( &GazeboRosAckermannDualSteerDrive::cmdVelCallback, this, _1 ),
                ros::VoidPtr(), &queue_ );

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe ( so );
    ROS_INFO ( "%s: Subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str() );

    odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry> ( odometry_topic_, 1 );
    ROS_INFO ( "%s: Advertise odom on %s !", gazebo_ros_->info(), odometry_topic_.c_str() );

    // start custom queue for diff drive
    this->callback_queue_thread_ = boost::thread ( boost::bind ( &GazeboRosAckermannDualSteerDrive::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosAckermannDualSteerDrive::UpdateChild, this ) );

    ROS_ERROR("Load() - completed");
}

void GazeboRosAckermannDualSteerDrive::publishWheelJointState()
{
    std::vector<physics::JointPtr> joints;
    joints.push_back ( joint_front_steer_ );
    joints.push_back ( joint_rear_steer_ );
    joints.push_back ( joint_wheel_fl_steer_ );
    joints.push_back ( joint_wheel_fr_steer_ );
    joints.push_back ( joint_wheel_bl_steer_ );
    joints.push_back ( joint_wheel_br_steer_ );

    joints.push_back ( joint_wheel_fl_drive_ );
    joints.push_back ( joint_wheel_fr_drive_ );
    joints.push_back ( joint_wheel_bl_drive_ );
    joints.push_back ( joint_wheel_br_drive_ );

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

void GazeboRosAckermannDualSteerDrive::publishWheelTF()
{
    ros::Time current_time = ros::Time::now();
    std::vector<physics::JointPtr> joints;
    joints.push_back ( joint_front_steer_ );
    joints.push_back ( joint_rear_steer_ );
    joints.push_back ( joint_wheel_fl_steer_ );
    joints.push_back ( joint_wheel_fr_steer_ );
    joints.push_back ( joint_wheel_bl_steer_ );
    joints.push_back ( joint_wheel_br_steer_ );

    joints.push_back ( joint_wheel_fl_drive_ );
    joints.push_back ( joint_wheel_fr_drive_ );
    joints.push_back ( joint_wheel_bl_drive_ );
    joints.push_back ( joint_wheel_br_drive_ );

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
void GazeboRosAckermannDualSteerDrive::UpdateChild()
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

        double target_wheel_rotation_speed = cmd_.speed / ( wheel_diameter_ / 2.0 );
        double target_front_steering_angle_speed = cmd_.angle1;
        double target_rear_steering_angle_speed = cmd_.angle2;

        motorController ( target_wheel_rotation_speed, target_front_steering_angle_speed, target_rear_steering_angle_speed, seconds_since_last_update );

        //        ROS_INFO_STREAM("wheel_diameter_ : " << wheel_diameter_ << " v = " << target_wheel_rotation_speed << " w1 = " << target_front_steering_angle_speed << " w2 = " << target_rear_steering_angle_speed);

        last_actuator_update_ += common::Time ( update_period_ );
    }
}

inline void jointPositionPDControl(physics::JointPtr joint,
                                   double target, double kp, double kd) {
#if GAZEBO_MAJOR_VERSION >= 8
    double current = joint->Position(0);
#else
    double current = joint->GetAngle(0).Radian();
#endif
  double v = kp * (target - current) - kd * joint->GetVelocity(0);
  // ROS_INFO_STREAM("current : " << current << " target : " << target << " output v : " << v);
#if GAZEBO_MAJOR_VERSION > 2
  joint->SetParam( "vel", 0, v);
#else
  joint->SetVelocity(0, v);
#endif
}

void GazeboRosAckermannDualSteerDrive::motorController ( double target_speed, double target_front_steering_speed, double target_rear_steering_speed, double dt )
{
  // Each joint is controlled independently (instead of having a set of parallel links).
  
  // 1), update the fake steering wheel with the velocity given
  // 2), utilize a combination of the current position of the fake wheel including the control velocity inorder to update the left and right wheel.
#if GAZEBO_MAJOR_VERSION > 2
  joint_front_steer_->SetParam ( "vel", 0, target_front_steering_speed );
  joint_rear_steer_->SetParam ( "vel", 0, target_rear_steering_speed );
#else
  joint_front_steer_->SetVelocity ( 0, target_front_steering_speed );
  joint_rear_steer_->SetVelocity ( 0, target_rear_steering_speed );
#endif

#if GAZEBO_MAJOR_VERSION >= 8
  double steering1_angle = joint_front_steer_->Position(0);
  double steering2_angle = joint_rear_steer_->Position(0);
#else
  double steering1_angle = joint_front_steer_->GetAngle(0).Radian();
  double steering2_angle = joint_rear_steer_->GetAngle(0).Radian();
#endif

  double steering1_angle_velocity = joint_front_steer_->GetVelocity(0);
  double steering2_angle_velocity = joint_rear_steer_->GetVelocity(0);
  
  //  std::cout << " -- s1 : " << steering1_angle << " s2 : " << steering2_angle << std::endl;

  ackermann_model_.updateSteeringAngle(steering1_angle, steering2_angle);
  
  // Run a simple positioning control on the steering angle.
  
  // Update left and right steering. This is the target values...
  double steering_fl = ackermann_model_.wheels[0].steeringAngle;
  double steering_fr = ackermann_model_.wheels[1].steeringAngle;
  double steering_bl = ackermann_model_.wheels[2].steeringAngle;
  double steering_br = ackermann_model_.wheels[3].steeringAngle;

  //  ROS_INFO_STREAM("steering front : " << steering1_angle << " fl : " << steering_fl << " fr : " << steering_fr << " rear : " << steering2_angle << " bl : " << steering_bl << " br : " << steering_br);
  jointPositionPDControl(joint_wheel_fl_steer_, steering_fl, 10, 1);
  jointPositionPDControl(joint_wheel_fr_steer_, steering_fr, 10, 1);
  jointPositionPDControl(joint_wheel_bl_steer_, steering_bl, 10, 1);
  jointPositionPDControl(joint_wheel_br_steer_, steering_br, 10, 1);
#if GAZEBO_MAJOR_VERSION > 2
  joint_front_steer_->SetParam( "vel", 0, target_front_steering_speed );
  joint_rear_steer_->SetParam( "vel", 0, target_rear_steering_speed );

  joint_wheel_fl_drive_->SetParam( "vel", 0, ackermann_model_.wheels[0].velocityFactor*target_speed );
  joint_wheel_fr_drive_->SetParam( "vel", 0, ackermann_model_.wheels[1].velocityFactor*target_speed );
  joint_wheel_bl_drive_->SetParam( "vel", 0, ackermann_model_.wheels[2].velocityFactor*target_speed );
  joint_wheel_br_drive_->SetParam( "vel", 0, ackermann_model_.wheels[3].velocityFactor*target_speed );
#else    
  joint_front_steer_->SetVelocity( 0, target_front_steering_speed );
  joint_rear_steer_->SetVelocity( 0, target_rear_steering_speed );
  
  joint_wheel_fl_drive_->SetVelocity( 0, ackermann_model_.wheels[0].velocityFactor*target_speed );
  joint_wheel_fr_drive_->SetVelocity( 0, ackermann_model_.wheels[1].velocityFactor*target_speed );
  joint_wheel_bl_drive_->SetVelocity( 0, ackermann_model_.wheels[2].velocityFactor*target_speed );
  joint_wheel_br_drive_->SetVelocity( 0, ackermann_model_.wheels[3].velocityFactor*target_speed );
#endif
}

// Finalize the controller
void GazeboRosAckermannDualSteerDrive::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void GazeboRosAckermannDualSteerDrive::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    cmd_.speed = cmd_msg->linear.x;
    cmd_.angle1 = cmd_msg->angular.z;
    cmd_.angle2 = cmd_msg->angular.y;
    //    std::cout << "s : " << cmd_.speed << " a1 : " << cmd_.angle1 << " a2 : " << cmd_.angle2 << std::endl;

}

void GazeboRosAckermannDualSteerDrive::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void GazeboRosAckermannDualSteerDrive::UpdateOdometryEncoder()
{
  // This is currently based on old code with a combined steer and drive wheel (e.g. snowwhite / cititruck)
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double step_time = ( current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;

#if GAZEBO_MAJOR_VERSION >= 8
    double odom_alpha = (joint_wheel_fl_steer_->Position(0) +
                         joint_wheel_fr_steer_->Position(0)) * 0.5;
#else
    double odom_alpha = (joint_wheel_fl_steer_->GetAngle(0).Radian() +
                         joint_wheel_fr_steer_->GetAngle(0).Radian()) * 0.5;
#endif

    // Distance travelled (this is utilizing only the front wheels...)
    double drive_dist = step_time * wheel_diameter_ / 2 * 
      (joint_wheel_fl_steer_->GetVelocity(0) + 
       joint_wheel_fr_steer_->GetVelocity(0));

    double dd = 0.;
    double da = 0.;
    
    if (fabs(odom_alpha) < 0.000001) // Avoid dividing with a very small number...
      {
	dd = drive_dist;
	da = 0.;
      }
    else
      {
	double r_stear = ackermann_model_.getSteerDriveWheelPos()[0] / sin(odom_alpha);
	double r_fix = r_stear * cos(odom_alpha) - ackermann_model_.getSteerDriveWheelPos()[1];
	
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

void GazeboRosAckermannDualSteerDrive::publishOdometry ( double step_time )
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

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosAckermannDualSteerDrive )
}

