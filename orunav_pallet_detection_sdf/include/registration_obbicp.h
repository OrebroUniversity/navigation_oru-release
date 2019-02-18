// Author: Dinh-Cuong Hoang, cuong.hoang@oru.se

#ifndef REGISTRATION_OBBICP
#define REGISTRATION_OBBICP

#include <cmath>
#include <iostream>
#include <time.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>

struct boundingBBox
{
    double length[3];
    pcl::PointXYZ minPoint, maxPoint; //min max Point after trasferring to origin
    pcl::PointXYZ center;
    Eigen::Matrix4f toOrigin; //pcl::transformPointCloud(input, output, toOrigin);
    pcl::PointCloud<pcl::PointXYZ> cornerPoints;

};

struct ObjectModel
{
    std::string name;
	pcl::PointCloud<pcl::PointXYZ> points;
	pcl::PointCloud<pcl::PointXYZRGB> colorPoints;
    boundingBBox OBB;   
};

struct clusterOBBICP
{
    std::string recognizedObj;
    std::vector<std::string> passOBBcandidates;
	pcl::PointCloud<pcl::PointXYZ> points;
	pcl::PointCloud<pcl::PointXYZ> modelPoints;
	pcl::PointCloud<pcl::PointXYZRGB> colorModelPoints;
    boundingBBox OBB;
};

class registrationOBBICP
{
    public:
	    int image_height;
	    registrationOBBICP();
        virtual ~registrationOBBICP();
        void pipeline(const pcl::PointCloud<pcl::PointXYZ> &cloud);
        void pclEuclideanDistanceSegmentation(const pcl::PointCloud<pcl::PointXYZ> &cloud, 
                                            std::vector<pcl::PointIndices> &cluster_indices, 
                                            const double distThresh, const int maxNumPoints, const int minNumPoints);
        void downSample(const pcl::PointCloud<pcl::PointXYZ> &input, 
                        pcl::PointCloud<pcl::PointXYZ> &output, 
                        const double leafSize);
        void colorSegments(const pcl::PointCloud<pcl::PointXYZ> &input, 
                            const std::vector<pcl::PointIndices> &cluster_indices, 
                            pcl::PointCloud<pcl::PointXYZRGB> &output);
        void computeOBB(const pcl::PointCloud<pcl::PointXYZ> &input, boundingBBox &OBB);
        bool loadModel(const std::string dir, const std::string name, ObjectModel& objModel);
        bool loadModels(const std::vector<std::string> dirs, const std::vector<std::string> Names,
                        std::vector<ObjectModel> &models);
        bool getClusters(const pcl::PointCloud<pcl::PointXYZ> &input, 
                        const std::vector<pcl::PointIndices> &cluster_indices,
                        std::vector<clusterOBBICP> &myclusters);
        void obbDimensionalCheck(const std::vector<ObjectModel> &models, 
                                std::vector<clusterOBBICP> &myclusters, const double *toler);
        void coarseToFineRegistration(std::vector<clusterOBBICP> &myclusters, 
                                    std::vector<ObjectModel> &models, 
                                    const double &overlap_dst, const double &overlap_score);
        double overlapPortion(const pcl::PointCloud<pcl::PointXYZ> &source, 
                                const pcl::PointCloud<pcl::PointXYZ> &target,
                                const double &max_dist);
	    
        std::vector<ObjectModel> models;
};

#endif