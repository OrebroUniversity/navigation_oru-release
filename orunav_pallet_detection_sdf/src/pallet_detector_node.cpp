// Author: Dinh-Cuong Hoang, cuong.hoang@oru.se
// Author: Dinh-Cuong Hoang, cuong.hoang@oru.se

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

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

class PalletDetectorNode 
{
    private:
        ros::NodeHandle nh_;        
        ros::Publisher pointsRGB_pub, markers_pub;
        ros::Subscriber pointcloud_sub_;
        ros::Subscriber depth_sub_;

        std::string pallet_name;
        bool using_bagfile;
        bool visual_model;
        bool save_ground_depthmap;
        bool deepLearning_based, ICP_based;
        bool downsample;

        double background_thresh;
        double downsample_ratio;
        double EC_segThresh;
        int maxSegPoints, minSegPoints;
        double tolerance[3];
        double overlap_dst_thresh, overlap_score_thresh;
        std::string ground_depthmap_dir, models_dir;

        std::string pallet_sensor_frame_id_prefix_;
        std::string OBB_frame_id;

        registrationOBBICP *myOBBICP;
        std::vector<ObjectModel> models;
        std::vector<clusterOBBICP> myclusters;
        std::vector<std::string> objectNames;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr myCloud;

        cv::Mat depth;
        tf::TransformListener tf_listener;


    public: 
        PalletDetectorNode (ros::NodeHandle &paramHandle)
        {
            myOBBICP = new registrationOBBICP();
            myCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

            paramHandle.param<std::string>("pallet_name", pallet_name, "full_pallet");
            paramHandle.param<bool>("using_bagfile", using_bagfile, true);
            paramHandle.param<bool>("visual_model", visual_model, true);             
            paramHandle.param<bool>("save_ground_depthmap", save_ground_depthmap, true);
            paramHandle.param<bool>("deepLearning_based", deepLearning_based, false);
            paramHandle.param<bool>("ICP_based", ICP_based, true);
            paramHandle.param<bool>("downsample", downsample, false);

            paramHandle.param<double>("background_thresh", background_thresh, 0.05);
            paramHandle.param<double>("downsample_ratio", downsample_ratio, 0.05);
            paramHandle.param<double>("EC_segThresh", EC_segThresh, 0.05);
            paramHandle.param<int>("maxSegPoints", maxSegPoints, 9900000);
            paramHandle.param<int>("minSegPoints", minSegPoints, 100);
            paramHandle.param<double>("tolerance_X", tolerance[0], 0.2);
            paramHandle.param<double>("tolerance_Y", tolerance[1], 0.2);
            paramHandle.param<double>("tolerance_Z", tolerance[2], 0.2);
            paramHandle.param<double>("overlap_dst_thresh", overlap_dst_thresh, 0.05);
            paramHandle.param<double>("overlap_score_thresh", overlap_score_thresh, 0.8);
            
            paramHandle.param<std::string>("ground_depthmap_dir", ground_depthmap_dir, "");
            paramHandle.param<std::string>("models_dir", models_dir, "");
            paramHandle.param<std::string>("pallet_sensor_frame_id_prefix", pallet_sensor_frame_id_prefix_, "camera_");
            paramHandle.param<std::string>("OBB_frame_id", OBB_frame_id, "camera_");
    
            //pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("pointcloud", 1, &PalletDetectorNode::process_pointcloud, this);     
            depth_sub_ = nh_.subscribe<sensor_msgs::Image>("depthmap", 1, &PalletDetectorNode::process_depthmap, this);             
            pointsRGB_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("filtered_points", 10);
            markers_pub = nh_.advertise<visualization_msgs::MarkerArray>( "OBBs", 1);   
            
            loadPointCloudModel();
        }
    
        ~PalletDetectorNode() 
        {
        }

        void loadPointCloudModel()
        {
            std::vector<std::string> dirs;
            objectNames.push_back(pallet_name);
            std::string dir_full_pallet_model = models_dir + pallet_name + ".ply";
            dirs.push_back(dir_full_pallet_model);
            myOBBICP->loadModels(dirs, objectNames, models);
        }

        void process_pointcloud(const sensor_msgs::PointCloud2::ConstPtr &msg) 
        {    
            pcl::PointCloud<pcl::PointXYZ> cloud;           
            pcl::fromROSMsg<pcl::PointXYZ>(*msg, cloud);
            //myOBBICP->pipeline(cloud);
            pointsRGB_pub.publish(cloud);
        }

        void depthToCloud(const cv::Mat& depthImg, pcl::PointCloud<pcl::PointXYZ> &cloud)
        {
            float cx = 319.5; float cy = 239.5; float fx = 580.0; float fy = 580.0;
            cv::Mat ground_depthImg;
            ground_depthImg = cv::imread(ground_depthmap_dir, -1);
            ground_depthImg.convertTo(ground_depthImg, CV_32FC1, 1.0); // should be 0.001 for mm scale, 1.0 for metre
            for(int row = 0; row < depthImg.rows; row++)
            {
                for(int col = 0; col < depthImg.cols; col++)       
                {
                    if(isnan(depthImg.at<float>(row, col))) continue;
                    if(isnan(ground_depthImg.at<float>(row, col))) continue;

                    double depth = depthImg.at<float>(row, col)/1000.0; // should be 1000 for metre scale, 1 for mm scale
                    double groundDepth = ground_depthImg.at<float>(row, col)/1000.0;  // should be 1000 for metre scale, 1 for mm scale
                    
                    pcl::PointXYZRGB point;
                    point.x = (col-cx) * depth / fx;
                    point.y = (row-cy) * depth / fy;
                    point.z = depth;
                    
                    if(abs(depth-groundDepth) < background_thresh)
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
        
        void semanticImprove(const cv::Mat& depthImg, const std::vector<clusterOBBICP> &myclusters, cv::Mat& semanticMap)
        {
            float cx = 319.5; float cy = 239.5; float fx = 550.0; float fy = 550.0;
            cv::Mat visual(cv::Size(depthImg.cols, depthImg.rows), CV_8UC3, Scalar(0, 0, 0)); ;

            for(int i=0; i < myclusters.size(); i++)
            {
                if(myclusters[i].recognizedObj != "")
                {
                    for(int j = 0; j < myclusters[i].colorModelPoints.size(); j++)
                    {
                        double x = myclusters[i].colorModelPoints.points[j].x;
                        double y = myclusters[i].colorModelPoints.points[j].y;
                        double z = myclusters[i].colorModelPoints.points[j].z;                        
                        int col =  double(x*fx / (z+0.0000001)) + cx;
                        int row =  double(y*fy / (z+0.0000001)) + cy;
                        if(col > 0 & col < 640 & row < 480 & row > 0)
                        {
                            visual.at<cv::Vec3b>(row, col)[0] = 0;
                            visual.at<cv::Vec3b>(row, col)[1] = 255;
                            visual.at<cv::Vec3b>(row, col)[2] = 0;
                            semanticMap.at<unsigned char>(row, col) = 7;
                        } 
                    }

                    for(int row = 3; row < semanticMap.rows-3; row++)
                    {
                        for(int col = 3; col < semanticMap.cols-3; col++)       
                        {
                            if(semanticMap.at<unsigned char>(row, col) == 0)
                            if
                            (
                                ( semanticMap.at<unsigned char>(row-1, col) != 0 || 
                                semanticMap.at<unsigned char>(row-2, col) != 0 ||
                                semanticMap.at<unsigned char>(row-3, col) !=0 ) &&

                                ( semanticMap.at<unsigned char>(row+1, col) != 0 || 
                                semanticMap.at<unsigned char>(row+2, col) != 0 ||
                                semanticMap.at<unsigned char>(row+3, col) !=0 ) &&
                                
                                ( semanticMap.at<unsigned char>(row, col-1) != 0 || 
                                semanticMap.at<unsigned char>(row, col-2) != 0 ||
                                semanticMap.at<unsigned char>(row, col-3) !=0 ) &&
                                
                                ( semanticMap.at<unsigned char>(row, col+1) != 0 || 
                                semanticMap.at<unsigned char>(row, col+2) != 0 ||
                                semanticMap.at<unsigned char>(row, col+3) !=0 )
                            )
                            {
                                visual.at<cv::Vec3b>(row, col)[0] = 0;
                                visual.at<cv::Vec3b>(row, col)[1] = 255;
                                visual.at<cv::Vec3b>(row, col)[2] = 0;
                                semanticMap.at<unsigned char>(row, col) = 7;
                            }
                        }
                    }

                }
            }
             
            cv::imshow("Improved Segmentation", visual);
            cv::waitKey(3);
        }

        void process_depthmap (const sensor_msgs::Image::ConstPtr& msg)
        {
            std::cerr << "\n" << "//----------- New Depth Frame Recieved -----------//" << "\n";
            if(myCloud->size()) myCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>); 

            cv_bridge::CvImageConstPtr bridge;

            try
            {
                bridge = cv_bridge::toCvCopy(msg, "32FC1");
                if(save_ground_depthmap)
                {
                    cv::Mat depth = bridge->image.clone();      
                    depth.convertTo(depth, CV_16UC1, 1.0); // Should be 1.0 for metre scale, 1000 for mm
                    cv::imwrite(ground_depthmap_dir, depth);
                    std::cerr << "Depth saved to: " << "\n" << ground_depthmap_dir << "\n";
                    return;
                }
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("Failed to transform depth image.");
                return;
            }

            depth = bridge->image.clone();
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
                    tf_listener.lookupTransform(pallet_sensor_frame_id_prefix_ + "link", pallet_sensor_frame_id_prefix_ + "depth_optical_frame", ros::Time(0), transform);
                    tf::poseTFToEigen(transform, Tcam_offset);
                    pcl::transformPointCloud (cloud, cloud, Tcam_offset);
                    
                    tf_listener.lookupTransform("world", pallet_sensor_frame_id_prefix_ + "link", ros::Time(0), transform);
                    tf::poseTFToEigen(transform, Tcam_offset);
                    pcl::transformPointCloud (cloud, cloud, Tcam_offset);
                }
                catch (tf::TransformException ex)
                {
                    ROS_ERROR("%s",ex.what());
                }
            }

            std::vector<pcl::PointIndices> cluster_indices;
            if(myclusters.size()) myclusters.clear();
            
            if(downsample) myOBBICP->downSample(cloud, cloud, downsample_ratio);
            std::cerr << "Scene Points after removing ground and downsample: " << cloud.size() << "\n";
            
            myOBBICP->pclEuclideanDistanceSegmentation(cloud, cluster_indices, EC_segThresh, maxSegPoints, minSegPoints);
            if(cluster_indices.size() == 0)
            {
                std::cerr << "No cluster!" << "\n";
                return;
            }
            
            myOBBICP->colorSegments(cloud, cluster_indices, *myCloud);
            myOBBICP->getClusters(cloud, cluster_indices, myclusters);
            myOBBICP->obbDimensionalCheck(models, myclusters, tolerance);
            
            if(!using_bagfile) myCloud->header.frame_id = "world"; 
            else myCloud->header.frame_id = pallet_sensor_frame_id_prefix_ + "depth_optical_frame";

            pointsRGB_pub.publish(*myCloud);
            markersPublish(); 
        }

        void obbicp_pipeline(pcl::PointCloud<pcl::PointXYZ> &cloud)
        {
            if(!using_bagfile)
            {
                tf::StampedTransform transform;
                Eigen::Affine3d Tcam_offset;
                try
                {
                    tf_listener.lookupTransform(pallet_sensor_frame_id_prefix_ + "link", pallet_sensor_frame_id_prefix_ + "depth_optical_frame", ros::Time(0), transform);
                    tf::poseTFToEigen(transform, Tcam_offset);
                    pcl::transformPointCloud (cloud, cloud, Tcam_offset);
		    pcl::transformPointCloud (*myCloud, *myCloud, Tcam_offset);
                    
                    tf_listener.lookupTransform("world", pallet_sensor_frame_id_prefix_ + "link", ros::Time(0), transform);
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
            std::cerr << "\n" << "Scene after removing ground and downsample: " << cloud.size() << "\n";
            
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
                    }
                }
            }

            if(!using_bagfile) myCloud->header.frame_id = "world"; 
            else
	      myCloud->header.frame_id = pallet_sensor_frame_id_prefix_ + "depth_optical_frame";

	    std::cerr << "Scene Points after removing ground and downsample #2: " << myCloud->size() << "\n";
       	    

            pointsRGB_pub.publish(*myCloud);
            markersPublish();

            //cv::Mat label(cv::Size(640, 480), CV_8UC1, Scalar(0)); 
            //semanticImprove(depth, myclusters, label);
        }

        void markersPublish()
        {
            visualization_msgs::MarkerArray multiMarker;
            visualization_msgs::Marker OBB;
            geometry_msgs::Point p;

            if(!using_bagfile) OBB.header.frame_id = "world"; 
            else OBB.header.frame_id = OBB_frame_id;
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
            OBB.scale.x = 0.01; OBB.scale.y = 0.01; OBB.scale.z = 0.01;
            OBB.color.r = 1.0f; OBB.color.g = 1.0f; OBB.color.b = 1.0f; OBB.color.a = 8.0;

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
        }
};

int main(int argc, char** argv) {

    ros::init(argc,argv,"pallet_detector_node");
    ros::NodeHandle params ("~");

    PalletDetectorNode pallet_detector_sdf(params);

    ros::spin();

}
