#ifndef CAMERA_POSE_CALIB_NODE_HH
#define CAMERA_POSE_CALIB_NODE_HH

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#define EIGEN_NO_DEBUG
#include <sdf.h>
#include <sdf_tracker_refbased.h> //includes highgui, imgproc, cvcore, eigen
#include <cititruck_forks_sdf.h>
#include <euro_pallet_sdf.h>
#include <euro_pallet_half_sdf.h>
#include <sdf_ros_utils.h>

#include <pcl_ros/point_cloud.h>


#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Plane segmentation...
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <orunav_generic/utils.h>
#include <orunav_generic/io.h>

//! This is used for the forward fork camera calibration (we simply assume we're driving on a plane here (to get the height + roll + pitch. x, y and yaw is currently not computed)
class CameraPoseCalibNode {

    private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Publisher pointcloud_pub_;
    ros::Publisher camera_poses_pub_;

    ros::Subscriber pointcloud_sub_;
  
    Eigen::Vector3d cam_pos;
    Eigen::Vector3d cam_rot;
    Eigen::Vector4f cam_params_;
    bool visualize, visualize_sdf;

    std::vector<sdf::Primitive*> geometry;

    Eigen::Affine3d Tcam_floor, Tcam_sdf, Tcam;

    sdf::SDF_Parameters params_;
    bool use_pallet_to_calib_;

    bool save_background_indices_;
    std::string background_indices_file_name_;

    int counter;

public: 
	CameraPoseCalibNode (ros::NodeHandle &paramHandle) {
            

            
            // Initial estimation parameters
            paramHandle.param<double>("x_offset", cam_pos(0), 0.0);
            paramHandle.param<double>("y_offset", cam_pos(1), 0.0);
            paramHandle.param<double>("z_offset", cam_pos(2), 0.0);
            
            paramHandle.param<double>("X_offset", cam_rot(0), 0.0);
            paramHandle.param<double>("Y_offset", cam_rot(1), 0.0);
            paramHandle.param<double>("Z_offset", cam_rot(2), 0.0);

	    paramHandle.param<bool>("visualize",visualize,false);
            paramHandle.param<bool>("visualize_sdf",visualize_sdf,true);

            paramHandle.param<double>("Dmax", params_.Dmax, 0.1);

            paramHandle.param<bool>("use_pallet_to_calib", use_pallet_to_calib_, false);
            
            paramHandle.param<bool>("save_background_indices", save_background_indices_, false);
            background_indices_file_name_ = std::string("background.dat");
            

  	    if (visualize)
	    {
		std::cout << "The output is visualized using visualization_markers (in rviz)." << std::endl;
		marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
	    }
            
            if (use_pallet_to_calib_) {
              //geometry = sdf::defineEuroPallet();
              geometry = sdf::defineEuroPalletHalf();
            }
            else {
                geometry = sdf::defineCitiTruckForks();
            }

            pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("pointcloud",10,&CameraPoseCalibNode::process_pointcloud,this);

            pointcloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("filtered_points", 1);

            cam_params_ << 570.342f, 570.342f, 314.5f, 235.5f; // From ASUS.
            cam_params_ << 589.3664548852777, 589.3664548852777, 320.5, 240.5; // From RGB camera_info (gazebo).

            camera_poses_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("camera_poses",10);
            counter = 0;
        }

    std::vector<int> negateVector(const std::vector<int> &vec, size_t size) {
        std::vector<int> neg_vec;
        size_t vec_idx = 0;
        for (size_t i = 0; i < size; i++) {
            if (vec[vec_idx] == i) {
                vec_idx++;
            }
            else {
                neg_vec.push_back(i);
            }
        }
        return neg_vec;
    }

    std::vector<int> getOutliers(const pcl::PointCloud<pcl::PointXYZ> cloud, const std::vector<int> &inliers) {
        // Return the inverse of the inliers -> to get the outliers... this is a bit nuts
        return negateVector(inliers, cloud.width*cloud.height);
    }

    void process_pointcloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        
        ROS_INFO("[CameraPoseCalibNode] - processing");


        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud, cloud_f;
        pcl::fromROSMsg (*msg, cloud);

        pcl::ModelCoefficients coefficients;
        pcl::PointIndices inliers;
        
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        
        // Optional
        seg.setOptimizeCoefficients (/*true*/false);
        
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
        { 
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (cloud.makeShared ());
            extract.setIndices (boost::make_shared<std::vector<int> > (inliers.indices));
            extract.setNegative (false);
            extract.filter (cloud_f);
        }
        // For simplicily keep the cloud intact -> fill the values .z = -1.
        for (size_t i = 0; i < inliers.indices.size(); i++) {
            cloud[inliers.indices[i]].z = -1;
        }

        if (save_background_indices_) {
            seg.setDistanceThreshold (0.03);
            seg.segment (inliers, coefficients);

            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (cloud.makeShared ());
            extract.setIndices (boost::make_shared<std::vector<int> > (inliers.indices));
            extract.setNegative (true);           
            extract.filter (cloud_f);
            pointcloud_pub_.publish(cloud_f);
            
            // inliers belong to the floor -> would like to keep all indices that do not contain the floor
            // This should be a very simple operation in PCL - go figure how...
            if (inliers.indices.size() > 0) {
                std::vector<int> outliers = getOutliers(cloud, inliers.indices);
                orunav_generic::saveIntVecTextFile(outliers, background_indices_file_name_);
                ROS_INFO("Saved background indices (%d) file : %s", (int)outliers.size(), background_indices_file_name_.c_str());
            }
            else {
                ROS_WARN("No inliers found(!!!)");
            }
            return;
        }
        else {
            pointcloud_pub_.publish(cloud_f);
        }

        cam_pos(2) = -coefficients.values[3];

        if (cam_pos(2) < 0) {// We're 'upside-down'
            cam_pos(2) *= -1.;
            coefficients.values[0] = -coefficients.values[0];
            coefficients.values[1] = -coefficients.values[1];
            coefficients.values[2] = -coefficients.values[2];
        }

        cam_rot(0) = -atan2(coefficients.values[0], -coefficients.values[1]);
        cam_rot(1) = M_PI - atan2(coefficients.values[2], -coefficients.values[1]);
        
        // Define an inital camera pose based on the plane estimation
        Tcam_floor = Eigen::Affine3d::Identity();
        Tcam_floor.translation() = Eigen::Vector3d(0., 0., 0.); // Tcam_floor handles only the rotation... (to be able to move the camera around on the 'floor coord system'.
        
        Tcam_floor.linear() = (Eigen::AngleAxisd(0., Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(cam_rot(1), Eigen::Vector3d::UnitY()) * 
                               Eigen::AngleAxisd(cam_rot(0), Eigen::Vector3d::UnitX()) ).toRotationMatrix(); //rotation ab


        // Add the rotation step (to align the coord system with the camera one).
        Eigen::Affine3d Tcam_sdf = Eigen::Affine3d::Identity();
        Tcam_sdf.linear() = (Eigen::AngleAxisd(-M_PI/2., Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
                          Eigen::AngleAxisd(-M_PI/2., Eigen::Vector3d::UnitX()) ).toRotationMatrix();
        
        Tcam = Tcam_floor*Tcam_sdf;


        // This doesn't tell us the complete story, we need to determine the offsets in position x, position y, and rotation around Z.
        // Utilize a sdf model of the forks.
        cam_pos(0) = 0.7;
        cam_pos(1) = 0.;
        cam_rot(2) = 0.;

        Eigen::Affine3d Tcam_offset = Eigen::AngleAxisd(cam_rot(2), Eigen::Vector3d::UnitZ())*Eigen::Translation3d(Eigen::Vector3d(cam_pos(0), cam_pos(1), cam_pos(2)));

        Eigen::Affine3d init_Tcam =  Tcam_offset * Tcam;
//        Eigen::Affine3d init_Tcam_non_sdf = Tcam_offset * Tcam_floor;

        Eigen::Affine3d aligned_Tcam = init_Tcam;

        aligned_Tcam = sdf::EstimatePalletPosePC(init_Tcam,        //cam pose
                                                 480,         //height
                                                 640,         //width 
                                                 cam_params_,  //camera parameters
                                                 100,         //max number of steps
                                                 5,           //max ray lentgh (meters)
                                                 params_.Dmax,           //max sdf value
                                                 geometry,     //geometry vector
                                                 cloud);

        if (visualize_sdf) {
            
//            aligned_Tcam = Eigen::Translation3d(Eigen::Vector3d(1.2, 0, 0)) * Tcam;
            
            sdf::RenderPC(aligned_Tcam,        //cam pose
                          480,         //height
                          640,         //width 
                          cam_params_,  //camera parameters
                          100,         //max number of steps
                          5,           //max ray lentgh (meters)
                          params_.Dmax,           //max sdf value
                          geometry,     //geometry vector
                          cloud);
   
            sdf::RenderDiffPC(aligned_Tcam,        //cam pose
                              480,         //height
                              640,         //width 
                              cam_params_,  //camera parameters
                              100,         //max number of steps
                              5,           //max ray lentgh (meters)
                              params_.Dmax,           //max sdf value
                              geometry,     //geometry vector
                              cloud);

            sdf::Render(aligned_Tcam,
                        480,
                        640,
                        cam_params_,
                        100,
                        5,
                        0.0001,
                        geometry);

   }
   
   // 

   {

       // We need to output the /camera_link frame, the ones we are currently using are either /camera_rgb_frame (depth_registred points) or /camera_depth_frame (depth points).
       // As long as we are consistent in the calibration the depth_regsitred vs. depth doesn't matter.
       Eigen::Affine3d Tcam_link_to_rgb = Eigen::Affine3d::Identity();
       Tcam_link_to_rgb.translation() = Eigen::Vector3d(0., -0.045, 0.); // Tcam_floor handles only the rotation... (to be able to move the camera around on the 'floor coord system'.
        

       tf::Transform tf_T;
       Eigen::Affine3d aligned_Tcam_non_sdf = aligned_Tcam*Tcam_sdf.inverse()*Tcam_link_to_rgb.inverse();
       tf::poseEigenToTF (aligned_Tcam_non_sdf, tf_T);
       geometry_msgs::PoseStamped pose_stamped;
       pose_stamped.header.stamp = msg->header.stamp;
       pose_stamped.header.frame_id = "/world";
       pose_stamped.pose = sdf::sdf_transformToPose(tf_T);
       camera_poses_pub_.publish(pose_stamped);

       // Print out the pose data to create a transform...
       std::cout << "---- CUT AND PASTE TO HAVE A TF TRANSFORMATION ----" << std::endl;
       std::cout << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"robotX_kinect_1\" args=\"" << tf_T.getOrigin().x() << " " << tf_T.getOrigin().y() << " " << tf_T.getOrigin().z() << " " << tf_T.getRotation().x() << " " << tf_T.getRotation().y() << " " << tf_T.getRotation().z() << " " << tf_T.getRotation().w() << " /world /camera_link 10\"/>" << std::endl;
       
       Eigen::Vector3d rpy;
       tf::Matrix3x3(tf_T.getRotation()).getRPY(rpy(0), rpy(1), rpy(2));
       
       std::cout << "CALIBRATION PARAMS:" << std::endl;
       std::cout << " - position: " << std::endl;
       std::cout << " -- x " << tf_T.getOrigin().x() << " (forward direction) " << std::endl;
       std::cout << " -- y " << tf_T.getOrigin().y() << std::endl;
       std::cout << " -- z " << tf_T.getOrigin().z() << " (height of the camera abouve the floor)" << std::endl;

       std::cout << " - rotations (in rad): " << std::endl;
       std::cout << " -- X " << rpy(0) << " -> additional rotation around the fwd axis (the axis of the forks) - this should be rather small" << std::endl;
       std::cout << " -- Y " << rpy(1)  << " -> the forward rotated angle (how much the up-side-down camera is looking downwards -> 0 would be straight fwd" << std::endl; 
       std::cout << " -- Z " << rpy(2)  << " -> rotation in the floor plane" << std::endl;
   }    
   // {
   //     Eigen::Affine3d camera_floor_offset = aligned_Tcam*Tcam.inverse();
   //     tf::Transform tf_T;
   //     tf::TransformEigenToTF (camera_floor_offset, tf_T);

   //     std::cout << "CALIBRATION PARAMS:" << std::endl;
   //     std::cout << " - position: " << std::endl;
   //     std::cout << " -- x " << tf_T.getOrigin().x() << " (forward direction) " << std::endl;
   //     std::cout << " -- y " << tf_T.getOrigin().y() << std::endl;
   //     std::cout << " -- z " << tf_T.getOrigin().z() << " (height of the camera abouve the floor)" << std::endl;
       
   //     Eigen::Vector3d rot;
   //     rot(0) = atan2(coefficients.values[0], -coefficients.values[1]);
   //     rot(1) = M_PI - atan2(coefficients.values[2], -coefficients.values[1]);
   //     rot(2) = tf::getYaw(tf_T.getRotation());
       
   //     std::cout << " - rotations (in rad): " << std::endl;
   //     std::cout << " -- X " << rot(0) << " -> additional rotation around the fwd axis (the axis of the forks) - this should be rather small" << std::endl;
   //     std::cout << " -- Y " << rot(1)  << " -> the forward rotated angle (how much the up-side-down camera is looking downwards -> 0 would be straight fwd" << std::endl; 
   //     std::cout << " -- Z " << rot(2)  << " -> rotation in the floor plane" << std::endl;

   //     std::cout << "---- CUT AND PASTE TO HAVE A TF TRANSFORMATION ----" << std::endl;
   //     std::cout << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"robotX_kinect_1\" args=\"" << tf_T.getOrigin().x() << " " << tf_T.getOrigin().y() << " " << tf_T.getOrigin().z() << " " << rot(2) << " " << rot(1) << " " << rot(0) << " /world /camera_link 10\"/>" << std::endl;
   // }
   
   // {
   //     tf::Transform tf_T;
   //     tf::TransformEigenToTF ((aligned_Tcam*Tcam.inverse()).inverse(), tf_T);
   //     geometry_msgs::PoseStamped pose_stamped;
   //     pose_stamped.header.stamp = msg->header.stamp;
   //     pose_stamped.header.frame_id = "/world";
   //     pose_stamped.pose = sdf::sdf_transformToPose(tf_T);
   //     camera_poses_pub_.publish(pose_stamped);
   // }    
}


};

#endif
