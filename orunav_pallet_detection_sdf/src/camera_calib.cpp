#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub_model;
ros::Publisher pub_cloud;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud, cloud_f;
  pcl::fromROSMsg (*input, cloud);

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  
  // Optional
  seg.setOptimizeCoefficients (true);
  
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (inliers, coefficients);

#if 0
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud.makeShared ());
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (cloud_f);

  std::cerr << "Unfiltered : " << cloud << std::endl;
  std::cerr << "Filtered   : " << cloud_f << std::endl;
#endif

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud.makeShared ());
  extract.setIndices (boost::make_shared<std::vector<int> > (inliers.indices));
  extract.setNegative (false);
  extract.filter (cloud_f);

  cloud_f.header.frame_id = cloud.header.frame_id;

  pub_cloud.publish (cloud_f);
  
  // Publish the model coefficients
  pub_model.publish (coefficients);

  std::cerr << coefficients << std::endl;

  std::cerr << "Height of the camera : " << coefficients.values[3] << std::endl;
  // Camera rotations:
  std::cout << "Rotations (in rad), given in the SDF frame :" << std::endl;
  std::cout << "   X : " << atan2(coefficients.values[2], -coefficients.values[1]) << " -> the forward rotated angle (how much the up-side-down camera is looking downwards -> 0 would be straight fwd" << std::endl; 
  std::cout << "   Y : " << atan2(coefficients.values[0], -coefficients.values[1]) << " -> additional rotation around the fwd axis (the axis of the forks) - this should be rather small" << std::endl;
  
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "camera_calib");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_model = nh.advertise<pcl::ModelCoefficients> ("output", 1);
  pub_cloud = nh.advertise<PointCloud> ("segmented_plane_points", 1);

  // Spin
  ros::spin ();
}
