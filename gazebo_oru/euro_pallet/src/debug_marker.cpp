#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <resource_retriever/retriever.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "debug_marker");
  ros::NodeHandle params ("~");
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;

  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

  std::string mesh_resource = "package://euro_pallet/meshes/EuroPallet.dae";
  //std::string mesh_resource = "package://cititruck_description/meshes/sick_s300_laser.dae";
  //std::string mesh_resource = "file:///home/han/catkin_ws/src/orunav/gazebo_oru/euro_pallet/meshes/EuroPallet.dae";
  resource_retriever::Retriever ret;
  resource_retriever::MemoryResource resource;
  try
  {
     resource = ret.get(mesh_resource); 
  }
  catch (resource_retriever::Exception& e)
  {
    ROS_ERROR("Failed to retrieve file: %s", e.what());
    return 1;
  }
  std::cout << "mesh found : " << mesh_resource << std::endl;

  ros::Rate r(10);
  while (ros::ok()) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "pallet";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 2;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    //    marker.scale.x = 0.0254;
    //    marker.scale.y = 0.0254;
    //    marker.scale.z = 0.0254;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.mesh_resource = mesh_resource; //"package://euro_pallet/meshes/EuroPallet.dae";
    marker.mesh_use_embedded_materials = false;
// "file://home/han/catkin_ws/src/orunav/gazebo_oru/euro_pallet/meshespackage://euro_pallet/meshes/EuroPallet.dae";


    marker_pub_.publish( marker );
    std::cout << "." << std::flush;

    r.sleep();
  }
}
