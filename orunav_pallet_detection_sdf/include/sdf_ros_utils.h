#ifndef SDF_ROS_UTILS_H
#define SDF_ROS_UTILS_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <orunav_rviz/orunav_rviz.h>

namespace sdf {

  // TODO - find some suitable representations for transformations(!) to avoid parsing them around.
  inline geometry_msgs::Quaternion sdf_quaternionToOrientation(tf::Quaternion quat) {
    geometry_msgs::Quaternion orientation;
    orientation.x = quat.x();
    orientation.y = quat.y();
    orientation.z = quat.z();
    orientation.w = quat.w();
    return orientation;
  }
  
  inline geometry_msgs::Point sdf_vectorToPosition(tf::Vector3 point) {
    geometry_msgs::Point position;
    position.x = point.x();
    position.y = point.y();
    position.z = point.z();
    return position;
  }
  inline geometry_msgs::Pose sdf_transformToPose(tf::Transform transform) {
    geometry_msgs::Pose pose;
    pose.orientation = sdf_quaternionToOrientation( transform.getRotation() );
    pose.position = sdf_vectorToPosition( transform.getOrigin() );
    return pose;
  }
  
  void assignColor(visualization_msgs::Marker &m, int color)
  {
    double r,g,b = 0.;
    int color_mod = color % 4;
    
    switch (color_mod)
      {
      case 0:
	r = 1.; g = 0.; b = 0.;
	break;
      case 1:
	r = 0.; g = 1.; b = 0.;
	break;
      case 2:
	r = 0.; g = 0.; b = 1.;
	break;
	
      default:
	r = 0.5; g = 0.5; b = 0.5;
	break;
      };
    
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 0.3;
    
    if (color < 0) {
      m.color.r = 1.;
      m.color.g = 1.;
      m.color.b = 1.;
      m.color.a = 1.;
    }
  }

  void drawPalletWithTime(const Eigen::Affine3d &T, const std::string &frame, int id, const ros::Time &time, ros::Publisher &pub) {
    // Any better way to do this?!?!
    tf::Transform tf_T;
    tf::poseEigenToTF (T, tf_T);
    geometry_msgs::Pose pose = sdf_transformToPose(tf_T);
    // orunav_generic::Pose2d p = orunav_conversions::createPose2dFromMsg(pose);
    // orunav_rviz::drawPalletWithTime(p, frame, id, time, pub);
    
    
    visualization_msgs::Marker marker;
    assignColor(marker, id);
    marker.header.frame_id = frame;
    marker.header.stamp = time;
    marker.ns = "pallet";
    marker.id = id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 1.2;
    marker.scale.y = 0.8;
    marker.scale.z = 0.1;
    // marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    // marker.scale.x = 0.0254;
    // marker.scale.y = 0.0254;
    // marker.scale.z = 0.0254;
    // marker.mesh_resource = "package://euro_pallet/meshes/EuroPallet.dae";
        
    pub.publish( marker );
  }

  void drawPallet(const Eigen::Affine3d &T, const std::string &frame, int id, ros::Publisher &pub) {
    drawPalletWithTime(T, frame, id, ros::Time(), pub);
  }

  void drawPalletHalfWithTime(const Eigen::Affine3d &T, const std::string &frame, int id, const ros::Time &time, ros::Publisher &pub) {
    // Any better way to do this?!?!
    tf::Transform tf_T;
    tf::poseEigenToTF (T, tf_T);
    geometry_msgs::Pose pose = sdf_transformToPose(tf_T);
    // orunav_generic::Pose2d p = orunav_conversions::createPose2dFromMsg(pose);
    // orunav_rviz::drawPalletWithTime(p, frame, id, time, pub);
    
    
    visualization_msgs::Marker marker;
    assignColor(marker, id);
    marker.header.frame_id = frame;
    marker.header.stamp = time;
    marker.ns = "pallet_half";
    marker.id = id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.6;
    marker.scale.y = 0.8;
    marker.scale.z = 0.18; // This is the KUKA booth pallet (the aass lab one is 14)
    marker.pose.position.z += 0.09; // This is the KUKA booth pallet (the aass lab one is 7)
    // marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    // marker.scale.x = 0.0254;
    // marker.scale.y = 0.0254;
    // marker.scale.z = 0.0254;
    // marker.mesh_resource = "package://euro_pallet/meshes/EuroPallet.dae";
        
    pub.publish( marker );
  }

  void drawPalletHalf(const Eigen::Affine3d &T, const std::string &frame, int id, ros::Publisher &pub) {
    drawPalletHalfWithTime(T, frame, id, ros::Time(), pub);
  }
  
} // namespace

#endif
