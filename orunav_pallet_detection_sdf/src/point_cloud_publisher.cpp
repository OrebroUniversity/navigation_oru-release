
#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#define SYNC_FRAMES 20


using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image> SyncPolicyRGB;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicyDepth;

class pointCloudPublisher
{

  public:
  pointCloudPublisher()
  { 
    n_ = ros::NodeHandle();
    nh_=ros::NodeHandle("~");
    nh_.param<std::string>("c_name",camera_name_,"camera");
    nh_.param<std::string>("im_name",image_name_,"image");
    nh_.param<std::string>("point_name",points_name_,"points");
    nh_.param("depth_registered",depth_registered_,false);
    nh_.param("EnableColor", enableColor_, false);
    nh_.param("stampNow", stampNow_, false);

    std::string topic_points;
    if(enableColor_)
    {
      if(depth_registered_)
      {
        
        topic_points = "/"+camera_name_+"/depth_registered/"+points_name_;
        sub_depth_ = new message_filters::Subscriber<sensor_msgs::Image>(n_, camera_name_+"/depth_registered/"+image_name_, 5);
        sub_depth_info_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(n_, camera_name_+"/depth_registered/camera_info", 1);
      }
      else
      {
        topic_points = "/"+camera_name_+"/depth/"+points_name_;
        sub_depth_ = new message_filters::Subscriber<sensor_msgs::Image>(n_, camera_name_+"/depth/"+image_name_, 5);
        sub_depth_info_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(n_, camera_name_+"/depth/camera_info", 1);
      }  
      
      sub_color_ = new message_filters::Subscriber<sensor_msgs::Image>(n_, camera_name_+"/rgb/image_color", 5);
      cloud_pub_ = n_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >(topic_points, 5, true);
      sync_RGB_ = new message_filters::Synchronizer< SyncPolicyRGB >(SyncPolicyRGB(SYNC_FRAMES), *sub_depth_, *sub_depth_info_, *sub_color_);
      sync_RGB_->registerCallback(boost::bind(&pointCloudPublisher::imageCb_with_RGB, this, _1, _2, _3));
      cerr<<" all setup color true"<<endl;
      cerr<<" input topics "<<sub_depth_->getTopic ()<<"  "<<sub_depth_info_->getTopic ()<<"  "<<sub_color_->getTopic ()<<endl;
      cerr<<" output topci "<<cloud_pub_.getTopic()<<endl;
    }
    else
    {
      if(depth_registered_)
      {
        topic_points = "/"+camera_name_+"/depth_registered/"+points_name_;
        sub_depth_ = new message_filters::Subscriber<sensor_msgs::Image>(n_, camera_name_+"/depth_registered/"+image_name_, 5);
        sub_depth_info_= new message_filters::Subscriber<sensor_msgs::CameraInfo>(n_, camera_name_+"/depth_registered/camera_info", 1);
      }
      else
      {
        topic_points = "/"+camera_name_+"/depth/"+points_name_;
        sub_depth_ = new message_filters::Subscriber<sensor_msgs::Image>(n_, camera_name_+"/depth/"+image_name_, 5);
        sub_depth_info_= new message_filters::Subscriber<sensor_msgs::CameraInfo>(n_, camera_name_+"/depth/camera_info", 1); 
      }
      
      cloud_pub_ = n_.advertise< pcl::PointCloud<pcl::PointXYZ> >(topic_points, 5, true
);
      sync_depth_ = new message_filters::Synchronizer< SyncPolicyDepth >(SyncPolicyDepth(SYNC_FRAMES), *sub_depth_, *sub_depth_info_);
      sync_depth_->registerCallback(boost::bind(&pointCloudPublisher::imageCb, this, _1, _2));
      cerr<<" all setup color false"<<endl;
    }  
  };

  ~pointCloudPublisher()
  {
    delete sub_depth_;
    delete sub_color_;
    delete sub_depth_info_;
    delete sync_RGB_; 
    delete sync_depth_;
  };

  void imageCb(const sensor_msgs::ImageConstPtr& depth_image,
             const sensor_msgs::CameraInfoConstPtr& depth_cam_info)
  {
//       cerr<<" got synchronized data..."<<endl;
    // Update the camera model (usually a no-op)
    model_.fromCameraInfo(depth_cam_info);
    
    cv_bridge::CvImageConstPtr bridge;
    try
    {
      bridge = cv_bridge::toCvShare(depth_image, "32FC1");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Failed to transform depth image.");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
    if(!depth_registered_)
    msg->header.frame_id = "/"+camera_name_+"_depth_optical_frame";
    else
    msg->header.frame_id = "/"+camera_name_+"_rgb_optical_frame";
   
    msg->height = bridge->image.rows;
    msg->width = bridge->image.cols;
    msg->is_dense = 1;
    msg->header.stamp = ros::Time::now();//raw_image->header.stamp();
    //msg->header.stamp = depth_image->header.stamp;


    for(int row=0; row<bridge->image.rows; ++row)
    { 
      const float* Drow = bridge->image.ptr<float>(row);
      for(int col=0; col<bridge->image.cols-0; ++col)
      {
        cv::Point2d uv = cv::Point2d(col,row);
        cv::Point3d xyz = model_.projectPixelTo3dRay(uv)*Drow[col];    
        msg->points.push_back (pcl::PointXYZ(xyz.x, xyz.y, xyz.z));
      }
    }
//     cerr<<" publishing "<<endl;
    cloud_pub_.publish(msg);
    return;
  };

  void imageCb_with_RGB(const sensor_msgs::ImageConstPtr& depth_image,
                        const sensor_msgs::CameraInfoConstPtr& depth_cam_info,
                        const sensor_msgs::ImageConstPtr& color_image)
  {
            cerr<<" got synchronized data rgb..."<<endl;

    // Update the camera model (usually a no-op)
    model_.fromCameraInfo(depth_cam_info);
    
    cv_bridge::CvImageConstPtr bridge;
    try
    {
      bridge = cv_bridge::toCvShare(depth_image, "32FC1");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Failed to transform depth image.");
      return;
    }

    cv_bridge::CvImageConstPtr bridge_RGB;
    try
    {
      bridge_RGB = cv_bridge::toCvShare(color_image, "8UC3");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Failed to transform color image.");
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg (new pcl::PointCloud<pcl::PointXYZRGB>);
    if(!depth_registered_)
    msg->header.frame_id = "/"+camera_name_+"_depth_optical_frame";
    else
    msg->header.frame_id = "/"+camera_name_+"_rgb_optical_frame";

    msg->height = bridge->image.rows;
    msg->width = bridge->image.cols;
    msg->is_dense = 1;
    if(stampNow_) {
	msg->header.stamp = ros::Time::now();//raw_image->header.stamp();
    } else {
	msg->header.stamp = color_image->header.stamp;
    }
    //msg->header.stamp = depth_image->header.stamp;
    //msg->header.stamp = color_image->header.stamp;

    for(int row=0; row<bridge->image.rows; ++row)
    { 
      const float* Drow = bridge->image.ptr<float>(row);
      for(int col=0; col<bridge->image.cols-0; ++col)
      {
       
        cv::Point2d uv = cv::Point2d(col,row);
        cv::Point3d xyz = model_.projectPixelTo3dRay(uv)*Drow[col];    
    
        pcl::PointXYZRGB p;
        uint8_t R = bridge_RGB->image.at<cv::Vec3b>(row,col)[0];
        uint8_t G = bridge_RGB->image.at<cv::Vec3b>(row,col)[1];
        uint8_t B = bridge_RGB->image.at<cv::Vec3b>(row,col)[2];
        uint32_t rgb = ((uint32_t)R << 16 | (uint32_t)G << 8 | (uint32_t)B);
        p.rgb = *reinterpret_cast<float*>(&rgb);
        p.x = xyz.x;
        p.y = xyz.y;
        p.z = xyz.z;

        msg->points.push_back (p);
      }
    }
    cloud_pub_.publish(msg);
    cerr<<" publishing in rgb"<<endl;
    return;
  };

  protected:

  bool depth_registered_;
  bool enableColor_;
  bool stampNow_;
  ros::NodeHandle n_;
  ros::NodeHandle nh_;
  ros::Publisher cloud_pub_;
  std::string camera_name_;
  std::string image_name_;
  std::string points_name_;
  
  message_filters::Synchronizer< SyncPolicyRGB > *sync_RGB_;
  message_filters::Synchronizer< SyncPolicyDepth > *sync_depth_;

  message_filters::Subscriber<sensor_msgs::Image> *sub_depth_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *sub_depth_info_;  
  message_filters::Subscriber<sensor_msgs::Image> *sub_color_;

  image_geometry::PinholeCameraModel model_;
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "point_cloud_publisher"); 
  pointCloudPublisher pcp;
  ros::spin();

  return 0;
}
