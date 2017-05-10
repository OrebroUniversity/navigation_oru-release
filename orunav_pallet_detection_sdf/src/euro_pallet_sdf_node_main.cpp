#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>

#define EIGEN_NO_DEBUG
#include <sdf.h>
#include <sdf_tracker_refbased.h> //includes highgui, imgproc, cvcore, eigen
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

#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>
#include <orunav_conversions/conversions.h>
#include <orunav_msgs/ObjectPoseEstimation.h>
#include <orunav_msgs/ObjectPose.h>


class EuroPalletSDFNode {

private:
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher pallet_poses_pub_;

  ros::Subscriber depth_img_sub_;
  ros::Subscriber pointcloud_sub_;

  ros::ServiceServer service_;

  std::vector<sdf::Primitive*> geometry;

  Eigen::Affine3d Tcam;
  Eigen::Vector4f cam_params_;
  bool visualize, visualize_sdf;
    
  sdf::SDF_Parameters params_;
  bool removeFloor_;
  bool publish_pose_cum;

  Eigen::Vector3d cam_pos;
  Eigen::Vector3d cam_rot;
  Eigen::Vector3d initPalletPose_;
   
  //poses stored as affine matrices for convenience
  Eigen::Affine3d pposeTrack;

  //vector for kalman update
  Eigen::Vector3d vposeTrack;
  //covariance for kalman update
  Eigen::Matrix3d cposeTrack;

  int robot_id_;
    
  tf::TransformListener tf_listener;
    
  int errorcnt;
  double sensor_time_offset_;
    
  bool do_nothing;

  std::string depth_frame_id_;
  std::string camera_frame_id_;
  std::string tf_base_link_;

  double floor_distance_thresh_;
  int min_nb_matched_points_;
  double min_match_nb_points_ratio_;

  std::vector<int> background_indices_;

  //The last good pose we published
  geometry_msgs::PoseStamped pose_stamped;
  double residual_thresh, cov_thresh;

  double max_new_target_dist_diff_;
  double max_new_target_angular_diff_;

  bool active_;
  orunav_generic::Pose2d object_pose_init_;
  double cov_thresh_;

  boost::mutex targets_mutex_;

  bool asus_camera_;
  //  std::string object_type_;
  unsigned int object_type_;


public: 
  EuroPalletSDFNode (ros::NodeHandle &paramHandle) : active_(false) {
    paramHandle.param<int>("robot_id", robot_id_, 1);
        
    pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("pointcloud",1,&EuroPalletSDFNode::process_pointcloud,this);

    service_ = nh_.advertiseService(orunav_generic::getRobotTopicName(robot_id_, "/pallet_estimation_service"), &EuroPalletSDFNode::objectPoseEstimationCB, this);
            
    paramHandle.param<bool>("visualize",visualize,true);
    paramHandle.param<bool>("visualize_sdf",visualize_sdf,true);
        
    //choose which pose estimate to compute
    paramHandle.param<bool>("publish_pose_cum",publish_pose_cum,false);

    // Use the standard coordinate system to specify the coordinates of the camera...
    // It is only a rotation step that needs to be added.
        
    // This should encode the position of the camera relative to the ground floor (note that the optical frame is currently connected to the base_footprint which is located on the floor -> the same number should go here...).
    paramHandle.param<double>("x_offset", cam_pos(0), 0.5);
    paramHandle.param<double>("y_offset", cam_pos(1), 0);
    paramHandle.param<double>("z_offset", cam_pos(2), 0.7);
        
    paramHandle.param<double>("X_offset", cam_rot(0), 0.0);
    paramHandle.param<double>("Y_offset", cam_rot(1), 2.5);
    paramHandle.param<double>("Z_offset", cam_rot(2), 0.0);
        
    paramHandle.param<double>("Dmax", params_.Dmax, 0.1);
    paramHandle.param<bool>("removeFloor", removeFloor_, true);
        
    paramHandle.param<double>("sensor_time_offset", sensor_time_offset_, 0.);
        
    paramHandle.param<bool>("do_nothing", do_nothing, false);

    paramHandle.param<std::string>("depth_frame_id", depth_frame_id_, std::string("/kinect_optical_frame"));
    paramHandle.param<std::string>("camera_frame_id", camera_frame_id_, std::string(""));
    paramHandle.param<std::string>("tf_base_link", tf_base_link_, orunav_generic::getRobotBaseLinkTF(robot_id_));
    paramHandle.param<double>("floor_distance_thresh", floor_distance_thresh_, 0.01);
    paramHandle.param<int>("min_nb_matched_points", min_nb_matched_points_, 10000);
    paramHandle.param<double>("min_match_nb_points_ratio", min_match_nb_points_ratio_, 0.2);
    
    std::string background_indices_file_name;
    paramHandle.param<std::string>("background_indices_file_name", background_indices_file_name, std::string(""));
    paramHandle.param<double>("residual_thresh", residual_thresh, 0.05);
    paramHandle.param<double>("cov_thresh", cov_thresh, 0.05);

    paramHandle.param<double>("max_new_target_dist_diff", max_new_target_dist_diff_, 0.5);
    paramHandle.param<double>("max_new_target_angular_diff", max_new_target_angular_diff_, 0.5);
        
    paramHandle.param<bool>("asus_camera", asus_camera_, true);

    if (removeFloor_) {
      pointcloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("filtered_points", 1);
    }
        
    if (visualize)
    {
      ROS_INFO("The output is visualized using visualization_markers (in rviz).");
      marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
    }
        
    pallet_poses_pub_ = nh_.advertise<orunav_msgs::ObjectPose>(orunav_generic::getRobotTopicName(robot_id_, "/pallet_poses"),10);

    //    pallet_poses_vis_only_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(orunav_generic::getRobotTopicName(robot_id_, "/pallet_poses_vis_only"),10);
        
    Eigen::Affine3d Tcam_floor = Eigen::Affine3d::Identity();
    Tcam_floor.translation() = Eigen::Vector3d(0., 0., 0.); // Tcam_floor handles only the rotation... (to be able to move the camera around on the 'floor coord system'.
        
    Tcam_floor.linear() = (Eigen::AngleAxisd(0., Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(cam_rot(1), Eigen::Vector3d::UnitY()) * 
                           Eigen::AngleAxisd(cam_rot(0), Eigen::Vector3d::UnitX()) ).toRotationMatrix(); //rotation ab
        
       
    // Add the rotation step (to align the coord system with the camera one).
    Eigen::Affine3d Tcam_sdf = Eigen::Affine3d::Identity();
    Tcam_sdf.linear() = (Eigen::AngleAxisd(-M_PI/2., Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
                         Eigen::AngleAxisd(-M_PI/2., Eigen::Vector3d::UnitX()) ).toRotationMatrix();
        
    // Final movements (on the floor coord).
    Eigen::Affine3d Tcam_offset = Eigen::AngleAxisd(cam_rot(2), Eigen::Vector3d::UnitZ())*Eigen::Translation3d(Eigen::Vector3d(cam_pos(0), cam_pos(1), cam_pos(2)));


    Tcam = Tcam_offset*Tcam_floor*Tcam_sdf;

    if (!camera_frame_id_.empty()) {

      // Wait until we have the time...
      while (!ros::Time::isValid()) {
        usleep(1000*1000);
      }
          
      // Find the frame instead... -> this is anyway constant.
      tf::StampedTransform transform;
      tf_listener.waitForTransform(tf_base_link_, camera_frame_id_, ros::Time::now()+ros::Duration(-2.), ros::Duration(10.0));
      try{
        tf_listener.lookupTransform(tf_base_link_, camera_frame_id_, ros::Time::now()+ros::Duration(-2.), transform);
                
        tf::poseTFToEigen(transform, Tcam_offset);

        Tcam = Tcam_offset*Tcam_sdf;
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }             
    }

    cam_params_ << 589.3664548852777, 589.3664548852777, 320.5, 240.5; // From RGB camera_info (gazebo).
    if (asus_camera_) {
      cam_params_ << 570.342f, 570.342f, 314.5f, 235.5f; // From ASUS.
    }

    if (!background_indices_file_name.empty()) {
      // Load the file (a column based text file...) created by the camera calibration node.
      background_indices_ = orunav_generic::loadIntVecTextFile(background_indices_file_name);
    }
    pose_stamped.header.frame_id = "/world";

    object_type_ = 0;

  }
    
  ~EuroPalletSDFNode() {
    // TODO add cleaning of geometry.
  }
    
    

  bool objectPoseEstimationCB(orunav_msgs::ObjectPoseEstimation::Request &req,
                              orunav_msgs::ObjectPoseEstimation::Response &res) 
  {
    targets_mutex_.lock();
    object_type_ = req.object.type;
    if (req.object.type == req.object.EUR_PALLET) {
      ROS_INFO_STREAM("[EuroPalletSDFNode] - EUR pallet");
      geometry = sdf::defineEuroPallet();
    }
    else if (req.object.type == req.object.HALF_PALLET) {
      ROS_INFO_STREAM("[EuroPalletSDFNode] - half pallet");
      geometry = sdf::defineEuroPalletHalf();
    }
    else {
      ROS_WARN_STREAM("[EuroPalletSDFNode] - unknown object_type : " << req.object.type << ". Assume a standard EUR pallet");
      geometry = sdf::defineEuroPallet();
      object_type_ = req.object.EUR_PALLET;
    }
     
    active_ = req.active;
    object_pose_init_ = orunav_conversions::createPose2dFromMsg(req.pose);
    cov_thresh_ = req.cov_thresh;
    
    vposeTrack(0) = object_pose_init_(0);
    vposeTrack(1) = object_pose_init_(1);
    vposeTrack(2) = object_pose_init_(2);
    cposeTrack.setIdentity();
    cposeTrack(0,0) = 10;
    cposeTrack(1,1) = 10;
    cposeTrack(2,2) = 10;

    pposeTrack = Eigen::Translation3d(Eigen::Vector3d(vposeTrack(0), vposeTrack(1), 0))
      *Eigen::AngleAxisd(vposeTrack(2), Eigen::Vector3d::UnitZ());
    
    targets_mutex_.unlock();
    res.result = 1;
    return true;
  }
    
    
  double normalizeM_PI(const double &angle) {
    double a = angle;
    while (a > M_PI/2.)
      a -= M_PI;
    while (a < -M_PI/2.)
      a += M_PI;
    
    return a;
  }

  void kalman_update(const Eigen::Vector3d &measure_X, const Eigen::Matrix3d &measure_cov, Eigen::Vector3d &est_X, Eigen::Matrix3d &est_cov) {

    //kalman update on pose vectors and covariance
    Eigen::Matrix3d Rk, Sk, Kk, Ik, Sk_inv;
    Ik.setIdentity();
    Eigen::Vector3d yk;
    
    bool wtf;
    double det;
    
    orunav_generic::Pose2d measured, estimated, residual, corrected, correction;
    
    measured(0) = measure_X(0);
    measured(1) = measure_X(1);
    measured(2) = measure_X(2);

    estimated(0) = est_X(0);
    estimated(1) = est_X(1);
    estimated(2) = est_X(2);

    residual = orunav_generic::subPose2d(estimated, measured);
    
    // Normalize the residual(2), note since the euro pallet object is symmetric (factor M_PI/2.), we might endup with a "flip".
    {
      double a = normalizeM_PI(residual(2));
      residual(2) = a;
    }
    
    yk(0) = residual(0); yk(1) = residual(1); yk(2) = residual(2);
    
    //    can_update = (yk.norm() > residual_thresh);
    Rk = measure_cov;
    Sk = est_cov + Rk;
    Sk.computeInverseAndDetWithCheck(Sk_inv,det,wtf);
    if (!wtf) {
      ROS_WARN("[EuroPalletSDF]: wtf, matrix not invertible");
      return;
    }
    Kk = est_cov*Sk_inv;
    yk = Kk*yk;
    correction(0) = yk(0); correction(1) = yk(1); correction(2) = yk(2);  
    corrected = orunav_generic::addPose2d(estimated, correction);
    
    est_X(0) = corrected(0); est_X(1) = corrected(1); est_X(2) = corrected(2); 
    est_cov = (Ik - Kk)*est_cov;
  }
  
  void process_pointcloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    
    if (!active_)
      return;
    
    targets_mutex_.lock();
    orunav_generic::Pose2d target_pose = object_pose_init_;
    targets_mutex_.unlock();
    
    // Use the first entry to get an inital estimate where the pallet could be (in global coords)
    // Need to get the vehicle pose in global coords
    orunav_generic::Pose2d vehicle_pose;
    tf::StampedTransform transform;
    Eigen::Affine3d baseToWorld;
    tf_listener.waitForTransform("world", tf_base_link_, msg->header.stamp,ros::Duration(1.0));
    try{
      tf_listener.lookupTransform("world", tf_base_link_,msg->header.stamp/* + ros::Duration(sensor_time_offset_)*/, transform);
      vehicle_pose(0) = transform.getOrigin().x();
      vehicle_pose(1) = transform.getOrigin().y();
      vehicle_pose(2) = tf::getYaw(transform.getRotation());
      errorcnt = 0;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("[EuroPalletSDF]: %s",ex.what());
      errorcnt++;
      return;
    }
    tf::poseTFToEigen(transform, baseToWorld);
    
    //	std::cerr<< "Base to world is "<<baseToWorld.matrix()<<std::endl;
    //	std::cerr<< "err counter is at : "<<errorcnt<<std::endl;
    
    // Ok, we have the vehicle pose and the target pose. The compute the pose between the vehicle's pose and the target.
    orunav_generic::Pose2d offset = orunav_generic::subPose2d(vehicle_pose, target_pose);
    if (publish_pose_cum) { // base the initial guess on the current pose estimate
      offset = orunav_generic::subPose2d(vehicle_pose, vposeTrack);
    }
    
    Eigen::Affine3d init_Tcam =  Eigen::AngleAxisd(-offset(2), Eigen::Vector3d::UnitZ())*Eigen::Translation3d(Eigen::Vector3d(-offset(0), -offset(1), 0)) * Tcam;
    
    
    if (visualize) { // TODO fix this ugly drawing stuff
      // Red - initial pose... -> important to use the correct timestamp when drawing this...
      if (object_type_ == 1) {
        sdf::drawPalletWithTime(init_Tcam.inverse(), depth_frame_id_,0,msg->header.stamp, marker_pub_);
      }
      else if (object_type_ == 2) {
        sdf::drawPalletHalfWithTime(init_Tcam.inverse(), depth_frame_id_,0,msg->header.stamp, marker_pub_);
      }
      else {
        
      }

    }
    
    if (do_nothing)
      return;
    
    pcl::PointCloud<pcl::PointXYZ> cloud;
    
    pcl::fromROSMsg<pcl::PointXYZ>( *msg, cloud);
    
    if (removeFloor_) {
      // Clear the PC from floor points - TODO take the coefficients from the calibration data and not from a plane estimation model.
      pcl::PointCloud<pcl::PointXYZ> cloud_f;
      pcl::ModelCoefficients coefficients;
      pcl::PointIndices inliers;
      
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
            
      // Optional
      seg.setOptimizeCoefficients (true);
            
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (floor_distance_thresh_);
            
      seg.setInputCloud (cloud.makeShared ());
      seg.segment (inliers, coefficients);
            
            
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (cloud.makeShared ());
      extract.setIndices (boost::make_shared<std::vector<int> > (inliers.indices));
      extract.setNegative (true);
      extract.filter (cloud_f);
            
            
      // For simplicily keep the cloud intact -> fill the values .z = -1.
      for (unsigned int i = 0; i < inliers.indices.size(); i++) {
        cloud[inliers.indices[i]].z = -1;
      }
      // If we have any background data -> the forks
      if (!background_indices_.empty()) {
        ROS_INFO("[EuroPalletSDF]: Number of pre-stored background points : %lu", background_indices_.size());
        for (unsigned int i = 0; i < background_indices_.size(); i++) {
          cloud[background_indices_[i]].z = -1;
        }
      }
      pointcloud_pub_.publish(cloud_f);
    }


    double t1 = orunav_generic::getDoubleTime();
    Eigen::Affine3d aligned_Tcam = sdf::EstimatePalletPosePC(init_Tcam,        //cam pose
                                                             480,         //height
                                                             640,         //width 
                                                             cam_params_,  //camera parameters
                                                             100,         //max number of steps
                                                             5,           //max ray lentgh (meters)
                                                             params_.Dmax,           //max sdf value
                                                             geometry,     //geometry vector
                                                             cloud);


    double t2 = orunav_generic::getDoubleTime();
    ROS_INFO("[EuroPalletSDF]: Registration took %lf",t2-t1);
        
    if (visualize_sdf) {
            
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

    std::pair<double, int> res = sdf::AlignmentPC(aligned_Tcam,        //cam pose
                                                  480,         //height
                                                  640,         //width 
                                                  cam_params_,  //camera parameters
                                                  100,         //max number of steps
                                                  5,           //max ray lentgh (meters)
                                                  params_.Dmax,           //max sdf value
                                                  geometry,     //geometry vector
                                                  cloud);
        
    ROS_INFO("[EuroPalletSDF]: Current match : %f, number of points used %d", res.first, res.second);
    
    // Is there a match -> very simplistic heuristics for now.
    if (res.second > min_nb_matched_points_) {
      if (res.first / (double)res.second > min_match_nb_points_ratio_)
      {
        Eigen::Affine3d in_global; 
        in_global = baseToWorld*((aligned_Tcam*Tcam.inverse()).inverse());
        Eigen::Vector3d vmeasure;
        vmeasure(0) = in_global.translation()(0);
        vmeasure(1) = in_global.translation()(1);
        vmeasure(2) = in_global.rotation().matrix().eulerAngles(0,1,2)(2);
        Eigen::Matrix3d cmeasure;
        cmeasure.setIdentity();
        cmeasure = (2*res.first/(double)res.second)*cmeasure; //fixed cov to start with
            
        kalman_update(vmeasure, cmeasure, vposeTrack, cposeTrack);

            
        orunav_rviz::drawPose2d(vposeTrack, 2, marker_pub_);
        orunav_rviz::drawPose2d(vmeasure, 1, marker_pub_);
            
        pposeTrack = Eigen::Translation3d(Eigen::Vector3d(vposeTrack(0), vposeTrack(1), 0))
          *Eigen::AngleAxisd(vposeTrack(2), Eigen::Vector3d::UnitZ());
            
      }
      else {
        ROS_INFO("[EuroPalletSDF]: not enough quality");
      }
    }	
    else {
      ROS_INFO("[EuroPalletSDF]: not enough points");
    }
    //else we simply don't update the estimates!	
    
    if (visualize) {
      
      Eigen::Affine3d Tcam_inv = aligned_Tcam.inverse();
      if (object_type_ == 1) {
        sdf::drawPalletWithTime(Tcam_inv, depth_frame_id_,1,msg->header.stamp,marker_pub_);
        sdf::drawPallet(pposeTrack,"world",2,marker_pub_);
      }
      else if (object_type_ == 2) {
        sdf::drawPalletHalfWithTime(Tcam_inv, depth_frame_id_,1,msg->header.stamp,marker_pub_);
        sdf::drawPalletHalf(pposeTrack,"world",2,marker_pub_);
      }
      else {

      }
    }
    
    //check if all of the diagonal ellements of the covariance are below the threshold
    bool can_pub = (cposeTrack(0,0) < cov_thresh && 
                    cposeTrack(1,1) < cov_thresh && cposeTrack(2,2) < cov_thresh);
    
    if (!active_)
      return;
    
    if(!can_pub) {
      ROS_INFO_STREAM("[EuroPalletSDF]: cannot publish updated pose, it is too uncertain still [" << cov_thresh << "]. Covariance matrix is: "<<cposeTrack);
    }
    else {
      tf::Transform tf_out;
      tf::poseEigenToTF(pposeTrack, tf_out);
      pose_stamped.pose = sdf::sdf_transformToPose(tf_out);
      pose_stamped.header.stamp = msg->header.stamp;
      
      orunav_msgs::ObjectPose object_pose;
      object_pose.pose = pose_stamped;
      object_pose.object.type = object_type_;
      pallet_poses_pub_.publish(object_pose);
    }
  }
  
  };

  using namespace std;

  int main(int argc, char** argv) {

    ros::init(argc,argv,"euro_pallet_sdf_node");
    ros::NodeHandle params ("~");

    EuroPalletSDFNode euro_pallet_sdf(params);

    ros::spin();

  }
