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

//-------------OBBICP------------------
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

// OpenCV specific includes
#include <opencv2/highgui/highgui.hpp>

#include <registration_obbicp.h>

using namespace cv;
using namespace std;

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

  std::string depth_frame_id_{"depth_frame_default"};
  std::string base_link_frame_id_;
  std::string global_frame_id_;
  std::string camera_frame_id_;

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

  //-----------------------------OBBICP----------------------------       
  ros::Publisher pointsRGB_pub, markers_pub;
  ros::Subscriber depth_sub_, semantic_sub_;

  std::string full_pallet_name;
  std::string half_pallet_name;

  std::string pallet_pose_topic;

  bool obbicp_based_;
  bool save_ground_depthmap;
  bool using_bagfile;
  bool visual_model;        
  bool deepLearning_based, ICP_based;
  bool downsample;

  double background_thresh;
  double downsample_ratio;
  double EC_segThresh;
  int maxSegPoints, minSegPoints;
  double tolerance[3];
  double overlap_dst_thresh, overlap_score_thresh;
  std::string ground_depthmap_dir, models_dir;

  registrationOBBICP *myOBBICP;
  std::vector<ObjectModel> models;
  std::vector<clusterOBBICP> myclusters;
  std::vector<std::string> objectNames;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr myCloud;

  cv::Mat depth, semantic_image;
  //----------------------------OBBICP-----------------------------

public: 
  EuroPalletSDFNode (ros::NodeHandle &paramHandle) : active_(false) 
  {
    paramHandle.param<int>("robot_id", robot_id_, 4);

    //---------------------- Para For OBBICP  ----------------------//
    
    myOBBICP = new registrationOBBICP();
    myCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    paramHandle.param<std::string>("full_pallet_name", full_pallet_name, "full_pallet");
    paramHandle.param<std::string>("half_pallet_name", half_pallet_name, "half_pallet");
    paramHandle.param<std::string>("pallet_pose_topic", pallet_pose_topic, "pallet_poses");
    
    paramHandle.param<bool>("using_bagfile", using_bagfile, false);
    paramHandle.param<bool>("visual_model", visual_model, true);             
    paramHandle.param<bool>("obbicp_based", obbicp_based_, false);
    paramHandle.param<bool>("save_ground_depthmap", save_ground_depthmap, true);
    paramHandle.param<bool>("deep_learning_based", deepLearning_based, false);
    paramHandle.param<bool>("icp_based", ICP_based, true);
    paramHandle.param<bool>("downsample", downsample, false);

    paramHandle.param<double>("background_thresh", background_thresh, 0.05);
    paramHandle.param<double>("downsample_ratio", downsample_ratio, 0.05);
    paramHandle.param<double>("EC_segThresh", EC_segThresh, 0.05);
    paramHandle.param<int>("maxSegPoints", maxSegPoints, 9900000);
    paramHandle.param<int>("minSegPoints", minSegPoints, 100);
    paramHandle.param<double>("tolerance_X", tolerance[0], 0.2);
    paramHandle.param<double>("tolerance_Y", tolerance[1], 0.2);
    paramHandle.param<double>("tolerance_Z", tolerance[2], 0.6);
    paramHandle.param<double>("overlap_dst_thresh", overlap_dst_thresh, 0.05);
    paramHandle.param<double>("overlap_score_thresh", overlap_score_thresh, 0.8);
    
    paramHandle.param<std::string>("ground_depthmap_dir", ground_depthmap_dir, "");
    paramHandle.param<std::string>("models_dir", models_dir, "");

    if(obbicp_based_)
    {
      depth_sub_ = nh_.subscribe<sensor_msgs::Image>("depthmap", 2, 
                      &EuroPalletSDFNode::process_depthmap, this);
      semantic_sub_ = nh_.subscribe<sensor_msgs::Image>("semanticmap", 1, 
                      &EuroPalletSDFNode::process_semantic, this);             
      pointsRGB_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("filtered_points", 1);
      markers_pub = nh_.advertise<visualization_msgs::MarkerArray>( "OBBs", 1);
      loadPointCloudModel();

      cv::Mat semanticImg(cv::Size(640, 480), CV_8UC1, Scalar(0));
      semantic_image = semanticImg;
    }

    //---------------------- Para For OBBICP Finish  ----------------------//
    pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("pointcloud",1,
                          &EuroPalletSDFNode::process_pointcloud,this);
    service_ = nh_.advertiseService("pallet_estimation_service", &EuroPalletSDFNode::objectPoseEstimationCB, this);
            
    paramHandle.param<bool>("visualize",visualize,false);
    paramHandle.param<bool>("visualize_sdf",visualize_sdf,false);
        
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

    paramHandle.param<std::string>("base_link_frame_id", base_link_frame_id_, std::string("base_link"));
    paramHandle.param<std::string>("camera_frame_id", camera_frame_id_, std::string(""));
    paramHandle.param<std::string>("global_frame_id", global_frame_id_, std::string("/world"));

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
        
    pallet_poses_pub_ = nh_.advertise<orunav_msgs::ObjectPose>(pallet_pose_topic,10);

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
      tf_listener.waitForTransform(base_link_frame_id_, camera_frame_id_, ros::Time::now()+ros::Duration(-2.), ros::Duration(10.0));
      try{
        tf_listener.lookupTransform(base_link_frame_id_, camera_frame_id_, ros::Time::now()+ros::Duration(-2.), transform);
                
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
    pose_stamped.header.frame_id = global_frame_id_;

    object_type_ = 0;

    std::cerr << "Waiting for sensor data ...";

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
  
  void process_pointcloud(const sensor_msgs::PointCloud2::ConstPtr &msg) 
  {
    pose_stamped.header.stamp = msg->header.stamp;
    if(obbicp_based_) return;

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
    tf_listener.waitForTransform(global_frame_id_, base_link_frame_id_, msg->header.stamp,ros::Duration(1.0));
    try{
      tf_listener.lookupTransform(global_frame_id_, base_link_frame_id_,msg->header.stamp/* + ros::Duration(sensor_time_offset_)*/, transform);
      vehicle_pose(0) = transform.getOrigin().x();
      vehicle_pose(1) = transform.getOrigin().y();
      vehicle_pose(2) = tf::getYaw(transform.getRotation());
      errorcnt = 0;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("[EuroPalletSDF]: %s",ex.what());
      errorcnt++;
      //std::cerr << "cuong 0" << "\n";
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
      /* for (unsigned int i = 0; i < inliers.indices.size(); i++) {
        cloud[inliers.indices[i]].z = -1;
      } */
      // If we have any background data -> the forks
      if (!background_indices_.empty()) {
        ROS_INFO("[EuroPalletSDF]: Number of pre-stored background points : %lu", background_indices_.size());
        for (unsigned int i = 0; i < background_indices_.size(); i++) {
          cloud[background_indices_[i]].z = -1;
        }
      }
      pointcloud_pub_.publish(cloud);
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
    //visualize = false;
    if (visualize) {
      
      Eigen::Affine3d Tcam_inv = aligned_Tcam.inverse();
      if (object_type_ == 1) {
        sdf::drawPalletWithTime(Tcam_inv, depth_frame_id_,1,msg->header.stamp,marker_pub_);
        sdf::drawPallet(pposeTrack,global_frame_id_,2,marker_pub_);
      }
      else if (object_type_ == 2) {
        sdf::drawPalletHalfWithTime(Tcam_inv, depth_frame_id_,1,msg->header.stamp,marker_pub_);
        sdf::drawPalletHalf(pposeTrack,global_frame_id_,2,marker_pub_);
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

  //-----------------------------------OBBICP------------------------------------

  void publish_pallet_pose(const Eigen::Matrix4f& pose_matrix)
  {
    pposeTrack.matrix() = pose_matrix.cast<double>();
    tf::Transform tf_out;
    tf::poseEigenToTF(pposeTrack, tf_out);

    tf::StampedTransform worldToBaseLink;
    tf_listener.waitForTransform(global_frame_id_, base_link_frame_id_, ros::Time(0), ros::Duration(1.0));
    try{
        tf_listener.lookupTransform(global_frame_id_, base_link_frame_id_, ros::Time(0), worldToBaseLink);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("[EuroPalletSDF]: %s",ex.what());
        return;
    }

    pose_stamped.header.frame_id = global_frame_id_;
    pose_stamped.pose = sdf::sdf_transformToPose(worldToBaseLink * tf_out);
    
    orunav_msgs::ObjectPose object_pose;
    object_pose.pose = pose_stamped;
    object_pose.object.type = object_type_;
    pallet_poses_pub_.publish(object_pose);

    ROS_INFO_STREAM_THROTTLE(1, "Homogeneous matrix of closest pallet: " << pose_matrix);

  }

  void loadPointCloudModel()
  {
    std::vector<std::string> dirs;

    objectNames.push_back(full_pallet_name);
    std::string dir_full_pallet_model = models_dir + full_pallet_name + ".ply";
    dirs.push_back(dir_full_pallet_model);

    objectNames.push_back(half_pallet_name);
    std::string dir_half_pallet_model = models_dir + half_pallet_name + ".ply";
    dirs.push_back(dir_half_pallet_model);

    myOBBICP->loadModels(dirs, objectNames, models);
  }
 
  void depthToCloud(const cv::Mat& depthImg, pcl::PointCloud<pcl::PointXYZ> &cloud)
  {
      float cx = 319.5; float cy = 239.5; float fx = 580.0; float fy = 580.0;
      cv::Mat ground_depthImg;
      ground_depthImg = cv::imread(ground_depthmap_dir, -1);
      ground_depthImg.convertTo(ground_depthImg, CV_32FC1, 0.001);
      for(int row = 0; row < depthImg.rows; row++)
      {
          for(int col = 0; col < depthImg.cols; col++)       
          {
              if(isnan(depthImg.at<float>(row, col))) continue;
              if(isnan(ground_depthImg.at<float>(row, col))) continue;

              double depth = depthImg.at<float>(row, col);
              unsigned char semantic = semantic_image.at<unsigned char>(row, col);
              double groundDepth = ground_depthImg.at<float>(row, col);
              
              pcl::PointXYZRGB point;
              point.x = (col-cx) * depth / fx;
              point.y = (row-cy) * depth / fy;
              point.z = depth;
              
              if(abs(depth-groundDepth) < background_thresh & semantic >= 0)
              {
                  point.r = 0; point.g = 0; point.b = 255;
                  myCloud->push_back(point);
              } 
              else
              {
                  pcl::PointXYZ objPoint;
                  objPoint.x = point.x;
                  objPoint.y = point.y;
                  objPoint.z = point.z;
                  cloud.push_back(objPoint);
              }
          }
      }
  }
 
  void process_semantic (const sensor_msgs::Image::ConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr bridge;
    try
    {
      bridge = cv_bridge::toCvCopy(msg, "8UC1");
      semantic_image = bridge->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Failed to transform rgb image.");
      return;
    }
  }

  void process_depthmap (const sensor_msgs::Image::ConstPtr& msg)
  {
      if(!active_) {
          ROS_INFO_STREAM_THROTTLE(5, "Inactive...");
          return;
      }
      ROS_INFO("New depth image received. ACTIVE!");

      if(myCloud->size()) myCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>); 

      cv_bridge::CvImageConstPtr bridge;

      try
      {
          bridge = cv_bridge::toCvCopy(msg, "32FC1");
          if(save_ground_depthmap)
          {
              cv::Mat depth = bridge->image.clone();      
              depth.convertTo(depth, CV_16UC1, 1000.0);
              cv::imwrite(ground_depthmap_dir, depth);
              ROS_DEBUG_STREAM_THROTTLE(1, ("Depth saved to file: " + ground_depthmap_dir).c_str());
              return;
          }
      }
      catch (cv_bridge::Exception& e)
      {
          ROS_ERROR("Failed to transform depth image.");
          return;
      }

      depth = bridge->image.clone();
      depth_frame_id_ = msg->header.frame_id;
      pcl::PointCloud<pcl::PointXYZ> cloud;
      depthToCloud(depth, cloud);
      if(ICP_based) obbicp_pipeline(cloud);
      else nonICP_pipeline(cloud);
  }

  void nonICP_pipeline(pcl::PointCloud<pcl::PointXYZ> &cloud)
  {
      if(!using_bagfile)
      {
          tf::StampedTransform transform;
          Eigen::Affine3d Tcam_offset;
          try
          {
              tf_listener.lookupTransform(base_link_frame_id_, depth_frame_id_, ros::Time(0), transform);
              tf::poseTFToEigen(transform, Tcam_offset);
              pcl::transformPointCloud (cloud, cloud, Tcam_offset);
              pcl::transformPointCloud (*myCloud, *myCloud, Tcam_offset);
          }
          catch (tf::TransformException ex)
          {
              ROS_ERROR("%s",ex.what());
          }
      }

      std::vector<pcl::PointIndices> cluster_indices;
      if(myclusters.size()) myclusters.clear();
      
      if(downsample) myOBBICP->downSample(cloud, cloud, downsample_ratio);
      ROS_DEBUG_STREAM_THROTTLE(5, "Scene Points after removing ground and downsample: " << cloud.size());
      
      myOBBICP->pclEuclideanDistanceSegmentation(cloud, cluster_indices, EC_segThresh, maxSegPoints, minSegPoints);
      if(cluster_indices.size() == 0)
      {
          std::cerr << "No cluster!" << "\n";
          return;
      }
      
      myOBBICP->colorSegments(cloud, cluster_indices, *myCloud);
      myOBBICP->getClusters(cloud, cluster_indices, myclusters);
      myOBBICP->obbDimensionalCheck(models, myclusters, tolerance);

      Eigen::Matrix4f pallet_pose_matrix;
      double dstToOrigin = 99999999;
      bool pallet_ready = false;

      for(int k = 0; k < myclusters.size(); k++)
      {
        if(!myclusters[k].passOBBcandidates.size()) continue;

        std::cerr << "\n" << "Pallet pose homogeneous matrix:" << "\n";
        std::cerr << myclusters[k].OBB.toOrigin.inverse() << "\n";
        std::cerr << "\n" << "OBB center:" << "\n";
        std::cerr << myclusters[k].OBB.center.x << " ";
        std::cerr << myclusters[k].OBB.center.y << " ";
        std::cerr << myclusters[k].OBB.center.z << "\n";
        
        double dst = myclusters[k].OBB.center.x*myclusters[k].OBB.center.x +
                    myclusters[k].OBB.center.y*myclusters[k].OBB.center.y +
                    myclusters[k].OBB.center.z*myclusters[k].OBB.center.z;
        if(dstToOrigin > dst)
        {
          pallet_pose_matrix = myclusters[k].OBB.toOrigin.inverse();
          dstToOrigin = dst;
          pallet_ready = true;
        }
      }
      if(pallet_ready) publish_pallet_pose(pallet_pose_matrix);
      pallet_ready = false;

      if(!using_bagfile) myCloud->header.frame_id = base_link_frame_id_; 
      else myCloud->header.frame_id = depth_frame_id_; 
      
      pointsRGB_pub.publish(*myCloud);
      publishMarkers();
  }

  void obbicp_pipeline(pcl::PointCloud<pcl::PointXYZ> &cloud)
  {
      if(!using_bagfile)
      {
          tf::StampedTransform transform;
          Eigen::Affine3d Tcam_offset;
          try
          {
              tf_listener.lookupTransform(base_link_frame_id_, depth_frame_id_, ros::Time(0), transform);
              tf::poseTFToEigen(transform, Tcam_offset);
              pcl::transformPointCloud (cloud, cloud, Tcam_offset);
              pcl::transformPointCloud (*myCloud, *myCloud, Tcam_offset);
          }
          catch (tf::TransformException ex)
          {
              ROS_ERROR("%s",ex.what());
          }
      }

      std::vector<pcl::PointIndices> cluster_indices;
      if(myclusters.size()) myclusters.clear();
      
      if(downsample) myOBBICP->downSample(cloud, cloud, downsample_ratio);
      ROS_DEBUG_STREAM_THROTTLE(5, "Scene after removing ground and downsample: " << cloud.size());
      
      myOBBICP->pclEuclideanDistanceSegmentation(cloud, cluster_indices, EC_segThresh, maxSegPoints, minSegPoints);
      if(cluster_indices.size() == 0)
      {
          std::cerr << "No cluster!" << "\n";
          return;
      }
      
      myOBBICP->colorSegments(cloud, cluster_indices, *myCloud);
      myOBBICP->getClusters(cloud, cluster_indices, myclusters);
      myOBBICP->obbDimensionalCheck(models, myclusters, tolerance);
      myOBBICP->coarseToFineRegistration(myclusters, models, 
      overlap_dst_thresh, overlap_score_thresh);

      Eigen::Matrix4f pallet_pose_matrix;
      double dstToOrigin = 99999999;
      bool pallet_ready = false;

      // Visualize model matching
      if(visual_model)
      {
          for(int i=0; i < myclusters.size(); i++)
          {
              if(myclusters[i].recognizedObj != "")
              {
                  pcl::PointCloud<pcl::PointXYZRGB> clusterRGB;
                  pcl::copyPointCloud(myclusters[i].colorModelPoints, clusterRGB);
                  *myCloud += clusterRGB;

                  ROS_INFO_STREAM_THROTTLE(2, "Pallet pose homogeneous matrix:" << myclusters[i].OBB.toOrigin.inverse());

                  ROS_INFO_STREAM_THROTTLE(2, "OBB center:" << "(" << myclusters[i].OBB.center.x << ","
                  << myclusters[i].OBB.center.y << " " << myclusters[i].OBB.center.z << ")");

                  double dst = myclusters[i].OBB.center.x*myclusters[i].OBB.center.x +
                              myclusters[i].OBB.center.y*myclusters[i].OBB.center.y +
                              myclusters[i].OBB.center.z*myclusters[i].OBB.center.z;
                  if(dstToOrigin > dst)
                  {
                    pallet_pose_matrix = myclusters[i].OBB.toOrigin.inverse();
                    dstToOrigin = dst;
                    pallet_ready = true;
                  }           
              }
          }
      }

      if(pallet_ready) publish_pallet_pose(pallet_pose_matrix);
      pallet_ready = false;

      if(!using_bagfile) myCloud->header.frame_id = base_link_frame_id_; 
      else myCloud->header.frame_id = depth_frame_id_; 

      pointsRGB_pub.publish(*myCloud);
      publishMarkers();
  }

  void publishMarkers()
  {
      visualization_msgs::MarkerArray multiMarker;
      visualization_msgs::Marker OBB;
      geometry_msgs::Point p;
      
      if(!using_bagfile) OBB.header.frame_id = base_link_frame_id_; 
      else OBB.header.frame_id = depth_frame_id_;
      OBB.header.stamp = ros::Time::now();
      OBB.ns = "OBBs";
      OBB.id = 0;
      OBB.type = visualization_msgs::Marker::LINE_LIST;
      OBB.action = visualization_msgs::Marker::ADD;
      OBB.pose.position.x = 0;
      OBB.pose.position.y = 0;
      OBB.pose.position.z = 0;
      OBB.pose.orientation.x = 0.0;
      OBB.pose.orientation.y = 0.0;
      OBB.pose.orientation.z = 0.0;
      OBB.pose.orientation.w = 1.0;
      OBB.scale.x = 0.02; OBB.scale.y = 0.02; OBB.scale.z = 0.02;
      OBB.color.r = 0.0f; OBB.color.g = 1.0f; OBB.color.b = 0.0f; OBB.color.a = 8.0;

      if(myclusters.size() == 0) 
      { 
          OBB.lifetime = ros::Duration();
          multiMarker.markers.push_back(OBB);             
          markers_pub.publish(multiMarker);
          return;
      };

      for(int k = 0; k < myclusters.size(); k++)
      {
          if(ICP_based & myclusters[k].recognizedObj == "") continue;
          if(!ICP_based & !myclusters[k].passOBBcandidates.size()) continue;
          
          for(int i = 0; i < 8; i++)
          {
              if(i == 3 || i == 7)
              {
                  p.x = myclusters[k].OBB.cornerPoints.points[i].x;
                  p.y = myclusters[k].OBB.cornerPoints.points[i].y; 
                  p.z = myclusters[k].OBB.cornerPoints.points[i].z;
                  OBB.points.push_back(p);
                  p.x = myclusters[k].OBB.cornerPoints.points[i-3].x;
                  p.y = myclusters[k].OBB.cornerPoints.points[i-3].y; 
                  p.z = myclusters[k].OBB.cornerPoints.points[i-3].z;
                  OBB.points.push_back(p);
                  if(i == 3)
                  {
                      p.x = myclusters[k].OBB.cornerPoints.points[i].x;
                      p.y = myclusters[k].OBB.cornerPoints.points[i].y; 
                      p.z = myclusters[k].OBB.cornerPoints.points[i].z;
                      OBB.points.push_back(p);
                      p.x = myclusters[k].OBB.cornerPoints.points[5].x;
                      p.y = myclusters[k].OBB.cornerPoints.points[5].y; 
                      p.z = myclusters[k].OBB.cornerPoints.points[5].z;
                      OBB.points.push_back(p);
                  }
                  if(i == 7)
                  {
                      p.x = myclusters[k].OBB.cornerPoints.points[i].x;
                      p.y = myclusters[k].OBB.cornerPoints.points[i].y; 
                      p.z = myclusters[k].OBB.cornerPoints.points[i].z;
                      OBB.points.push_back(p);
                      p.x = myclusters[k].OBB.cornerPoints.points[1].x;
                      p.y = myclusters[k].OBB.cornerPoints.points[1].y; 
                      p.z = myclusters[k].OBB.cornerPoints.points[1].z;
                      OBB.points.push_back(p);
                  }
              }
              else
              {
                  p.x = myclusters[k].OBB.cornerPoints.points[i].x;
                  p.y = myclusters[k].OBB.cornerPoints.points[i].y; 
                  p.z = myclusters[k].OBB.cornerPoints.points[i].z;
                  OBB.points.push_back(p);
                  p.x = myclusters[k].OBB.cornerPoints.points[i+1].x;
                  p.y = myclusters[k].OBB.cornerPoints.points[i+1].y; 
                  p.z = myclusters[k].OBB.cornerPoints.points[i+1].z;
                  OBB.points.push_back(p);
                  if(i == 0)
                  {
                      p.x = myclusters[k].OBB.cornerPoints.points[i].x;
                      p.y = myclusters[k].OBB.cornerPoints.points[i].y; 
                      p.z = myclusters[k].OBB.cornerPoints.points[i].z;
                      OBB.points.push_back(p);
                      p.x = myclusters[k].OBB.cornerPoints.points[6].x;
                      p.y = myclusters[k].OBB.cornerPoints.points[6].y; 
                      p.z = myclusters[k].OBB.cornerPoints.points[6].z;
                      OBB.points.push_back(p);
                  }
                  if(i == 2)
                  {
                      p.x = myclusters[k].OBB.cornerPoints.points[i].x;
                      p.y = myclusters[k].OBB.cornerPoints.points[i].y; 
                      p.z = myclusters[k].OBB.cornerPoints.points[i].z;
                      OBB.points.push_back(p);
                      p.x = myclusters[k].OBB.cornerPoints.points[4].x;
                      p.y = myclusters[k].OBB.cornerPoints.points[4].y; 
                      p.z = myclusters[k].OBB.cornerPoints.points[4].z;
                      OBB.points.push_back(p);
                  }
              }
          }
      }

      OBB.lifetime = ros::Duration();
      multiMarker.markers.push_back(OBB);
      markers_pub.publish(multiMarker);

      // publish palet pose:

  }
  
  //------------------------------------OBBICP--------------------------------------
  
  };

  using namespace std;

  int main(int argc, char** argv) 
  {

    ros::init(argc,argv,"euro_pallet_sdf_node");
    ros::NodeHandle params ("~");

    EuroPalletSDFNode euro_pallet_sdf(params);

    ros::spin();

  }
