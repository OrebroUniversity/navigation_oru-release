// Author: Dinh-Cuong Hoang, cuong.hoang@oru.se

#include "registration_obbicp.h"

registrationOBBICP::registrationOBBICP()
{
}

registrationOBBICP::~registrationOBBICP()
{  
}

void registrationOBBICP::pipeline(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZ> cloud_noNAN;
    pcl::removeNaNFromPointCloud(cloud, cloud_noNAN, indices);

    downSample(cloud_noNAN, cloud_noNAN, 0.03);

    std::vector<pcl::PointIndices> cluster_indices;
    pclEuclideanDistanceSegmentation(cloud_noNAN, cluster_indices, 0.03, 100, 9900000);
}

void registrationOBBICP::pclEuclideanDistanceSegmentation(const pcl::PointCloud<pcl::PointXYZ> &input_cloud, 
                                                        std::vector<pcl::PointIndices> &cluster_indices, 
                                                        const double dstThresh, 
                                                        const int maxNumPoints, 
                                                        const int minNumPoints)
{
    // PCL segmentation based Euclidean distance

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(input_cloud, *cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(dstThresh);
	ec.setMinClusterSize(minNumPoints);
	ec.setMaxClusterSize(maxNumPoints);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);
}

void registrationOBBICP::downSample(const pcl::PointCloud<pcl::PointXYZ> &input, 
                                    pcl::PointCloud<pcl::PointXYZ> &output, 
                                    const double leafSize)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr DownsampleCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(input, *DownsampleCloud);

	// Downsample
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(DownsampleCloud);
	sor.setLeafSize(leafSize, leafSize, leafSize);
	DownsampleCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	sor.filter(*DownsampleCloud);

	if ((int)output.size() != 0) output.clear();
	pcl::copyPointCloud(*DownsampleCloud, output);
}

void registrationOBBICP::colorSegments(const pcl::PointCloud<pcl::PointXYZ> &input, 
                            const std::vector<pcl::PointIndices> &cluster_indices, 
                            pcl::PointCloud<pcl::PointXYZRGB> &output)
{
    int j= 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
            pcl::PointXYZRGB point;
            point.x = input.points[*pit].x;
            point.y = input.points[*pit].y;
            point.z = input.points[*pit].z;

            if (j == 0) // Red (255,0,0)
                {
                    point.r = 255;
                    point.g = 0;
                    point.b = 0;
                }
            else if (j == 1) // Green (0,255,0)
                {
                    point.r = 0;
                    point.g = 255;
                    point.b = 0;
                }
            else if (j == 2) // coral (255,127,80)
                {
                    point.r = 255;
                    point.g = 127;
                    point.b = 80;
                }
            else if (j == 3) // Yellow	(255,255,0)
                {
                    point.r = 255;
                    point.g = 255;
                    point.b = 0;
                }
            else if (j == 4) //Cyan	(0,255,255)
                {
                    point.r = 0;
                    point.g = 255;
                    point.b = 255;
                }
            else if (j == 5) // Magenta	(255,0,255)
                {
                    point.r = 255;
                    point.g = 0;
                    point.b = 255;
                }
            else if (j == 6) // Olive (128,128,0)
                {
                    point.r = 128;
                    point.g = 128;
                    point.b = 0;
                }
            else if (j == 7) // Teal (0,128,128)
                {
                    point.r = 0;
                    point.g = 128;
                    point.b = 128;
                }
            else if (j == 8) // Purple (128,0,128)
                {
                    point.r = 128;
                    point.g = 0;
                    point.b = 128;
                }
            else
                {
                    if (j % 2 == 0)
                    {
                        point.r = 255 * j / (cluster_indices.size());
                        point.g = 128;
                        point.b = 50;
                    }
                    else
                    {
                        point.r = 0;
                        point.g = 255 * j / (cluster_indices.size());
                        point.b = 128;
                    }
                }
            output.push_back(point);
        }
        j++;
    }
}

void registrationOBBICP::computeOBB(const pcl::PointCloud<pcl::PointXYZ> &input, boundingBBox &OBB)
{
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(input, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(input, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(input, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    
    OBB.length[0] = maxPoint.x - minPoint.x; //MAX length OBB
    OBB.length[1] = maxPoint.y - minPoint.y; //MID length OBB
    OBB.length[2] = maxPoint.z - minPoint.z; //MIN length OBB

    if(OBB.length[0] < OBB.length[1])
    {
        float buf = OBB.length[0]; OBB.length[0] = OBB.length[1]; 
        OBB.length[1] = buf;

        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        transform_2.rotate (Eigen::AngleAxisf (M_PI/2.0, Eigen::Vector3f::UnitZ()));
        pcl::transformPointCloud (*cloudPointsProjected, *cloudPointsProjected, transform_2);
        projectionTransform = transform_2.matrix()*projectionTransform;
    }
    if(OBB.length[0] < OBB.length[2])
    {
        float buf = OBB.length[0]; OBB.length[0] = OBB.length[2]; 
        OBB.length[2] = buf;

        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        transform_2.rotate (Eigen::AngleAxisf (M_PI/2.0, Eigen::Vector3f::UnitY()));
        pcl::transformPointCloud (*cloudPointsProjected, *cloudPointsProjected, transform_2);
        projectionTransform = transform_2.matrix()*projectionTransform;
    }
    if(OBB.length[1] < OBB.length[2])
    {
        float buf = OBB.length[1]; OBB.length[1] = OBB.length[2]; 
        OBB.length[2] = buf;

        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        transform_2.rotate (Eigen::AngleAxisf (M_PI/2.0, Eigen::Vector3f::UnitX()));
        pcl::transformPointCloud (*cloudPointsProjected, *cloudPointsProjected, transform_2);
        projectionTransform = transform_2.matrix()*projectionTransform;
    }

    pcl::getMinMax3D(*cloudPointsProjected, OBB.minPoint, OBB.maxPoint);
    OBB.toOrigin = projectionTransform;

    pcl::PointXYZ OBB_points;

    OBB.cornerPoints.push_back(OBB.minPoint); // Min Point
    OBB_points.x = OBB.minPoint.x; OBB_points.y = OBB.maxPoint.y; OBB_points.z = OBB.minPoint.z;
    OBB.cornerPoints.push_back(OBB_points);
    OBB_points.x = OBB.minPoint.x; OBB_points.y = OBB.maxPoint.y; OBB_points.z = OBB.maxPoint.z;
    OBB.cornerPoints.push_back(OBB_points);
    OBB_points.x = OBB.minPoint.x; OBB_points.y = OBB.minPoint.y; OBB_points.z = OBB.maxPoint.z;
    OBB.cornerPoints.push_back(OBB_points);

    OBB.cornerPoints.push_back(OBB.maxPoint); //Max point
    OBB_points.x = OBB.maxPoint.x; OBB_points.y = OBB.minPoint.y; OBB_points.z = OBB.maxPoint.z;
    OBB.cornerPoints.push_back(OBB_points);
    OBB_points.x = OBB.maxPoint.x; OBB_points.y = OBB.minPoint.y; OBB_points.z = OBB.minPoint.z;
    OBB.cornerPoints.push_back(OBB_points);
    OBB_points.x = OBB.maxPoint.x; OBB_points.y = OBB.maxPoint.y; OBB_points.z = OBB.minPoint.z;
    OBB.cornerPoints.push_back(OBB_points);

    pcl::transformPointCloud(OBB.cornerPoints, OBB.cornerPoints, projectionTransform.inverse());

    OBB.center.x = 0; OBB.center.y = 0; OBB.center.z = 0;
    for (int i = 0; i < OBB.cornerPoints.size(); i++)
    {
        OBB.center.x += OBB.cornerPoints[i].x;
        OBB.center.y += OBB.cornerPoints[i].y;
        OBB.center.z += OBB.cornerPoints[i].z;
    }
    OBB.center.x = OBB.center.x / OBB.cornerPoints.size();
    OBB.center.y = OBB.center.y / OBB.cornerPoints.size();
    OBB.center.z = OBB.center.z / OBB.cornerPoints.size();
}

bool registrationOBBICP::loadModel(const std::string dir, const std::string name, ObjectModel& objModel)
{
    // Load point cloud model from database and tranfering the model to origin by OBB unit vectors

    pcl::PLYReader Reader;
    if(Reader.read(dir, objModel.colorPoints) == -1) return false;
    int i = 1;
    while(objModel.points.size()==0 || objModel.points.size() > 5000)
    {
        pcl::copyPointCloud(objModel.colorPoints, objModel.points);
        downSample(objModel.points, objModel.points, i*0.01);
        i++;
    }
    std::cerr << "Number of model points after downsampling: " << objModel.points.size() << "\n";   
    objModel.name = name;

    computeOBB(objModel.points, objModel.OBB);
    pcl::transformPointCloud(objModel.points, objModel.points, objModel.OBB.toOrigin);
    pcl::transformPointCloud(objModel.colorPoints, objModel.colorPoints, objModel.OBB.toOrigin);

    std::cerr << dir << "\n" << "OBB Length: "; 
    std::cerr << objModel.OBB.length[0] << " " << objModel.OBB.length[1] << " " << objModel.OBB.length[2] << "\n";

    return 1;
}

bool registrationOBBICP::loadModels(const std::vector<std::string> dirs, 
    const std::vector<std::string> Names, std::vector<ObjectModel> &models)
{
    for(int i=0; i < dirs.size(); i++)
    {
        ObjectModel model;
        if(!loadModel(dirs[i], Names[i], model)) 
        {
            std::cerr << "Fail to load models" << "\n";
            return false;
        }
        models.push_back(model);
    }
    return true;
}

bool registrationOBBICP::getClusters(const pcl::PointCloud<pcl::PointXYZ> &input, 
                                    const std::vector<pcl::PointIndices> &cluster_indices,
                                    std::vector<clusterOBBICP> &myclusters)
{
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        clusterOBBICP cls;
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            pcl::PointXYZ point;
            point.x = input.points[*pit].x;
            point.y = input.points[*pit].y;
            point.z = input.points[*pit].z;
            cls.points.push_back(point);
        }
        computeOBB(cls.points, cls.OBB);
        cls.recognizedObj = "";
        myclusters.push_back(cls);
    }
}


void registrationOBBICP::obbDimensionalCheck(const std::vector<ObjectModel> &models, 
                                            std::vector<clusterOBBICP> &myclusters, 
                                            const double *toler)
{
    for(int i=0; i < myclusters.size(); i++)
    {
        for(int j=0; j < models.size(); j++)
        {
            double d_X = fabs(myclusters[i].OBB.length[0] - models[j].OBB.length[0]);
            double d_Y = fabs(myclusters[i].OBB.length[1] - models[j].OBB.length[1]);
            double d_Z = fabs(myclusters[i].OBB.length[2] - models[j].OBB.length[2]);
            if(d_X < models[j].OBB.length[0] * toler[0])
            if(d_Y < models[j].OBB.length[1] * toler[1])
            if(d_Z < models[j].OBB.length[2] * toler[2])
            {
                myclusters[i].passOBBcandidates.push_back(models[j].name);
                std::cerr << "OBB passed: " << myclusters[i].OBB.length[0]
                 << " " << myclusters[i].OBB.length[1] << " " << myclusters[i].OBB.length[2] << "\n";
            }
       }
    }
}

double registrationOBBICP::overlapPortion(const pcl::PointCloud<pcl::PointXYZ> &source, 
                                        const pcl::PointCloud<pcl::PointXYZ> &target, const double &max_dist)
{
	if (source.size() == 0 || target.size() == 0) return -1;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(target, *target_cloud);
	pcl::copyPointCloud(source, *source_cloud);
	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(target_cloud);
	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);

	int overlap_Points = 0;
	for (int i = 0; i < source.size(); ++i)
	{
		if (kdtree.nearestKSearch(source_cloud->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
            if(sqrt(pointNKNSquaredDistance[0]) < max_dist)
			    overlap_Points++;
		}
	}

	//calculating the mean distance
	double portion = (double) overlap_Points / source.size();
	return portion;
}

void registrationOBBICP::coarseToFineRegistration(std::vector<clusterOBBICP> &myclusters, 
                                    std::vector<ObjectModel> &models,
                                    const double &overlap_dst_thresh, 
                                    const double &overlap_score_thresh)
{
    std::cerr << "Number of clusters: " << myclusters.size() << "\n";
    for(int i=0; i < myclusters.size(); i++)
    {
        if(!myclusters[i].passOBBcandidates.size()) continue;
        std::cerr << "\n" << "Registration phase: " << "\n";
        std::cerr << "NumberofPoints of clusters: " << myclusters[i].points.size() << "\n";

        pcl::PointCloud<pcl::PointXYZ>::Ptr origCluster (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(myclusters[i].points, *origCluster, myclusters[i].OBB.toOrigin);
        
        Eigen::Matrix4f bestMat(Eigen::Matrix4f::Identity());
        double bestScore = -9999999;
        Eigen::Matrix4f bestCoarseMat;
        Eigen::Matrix4f bestFineMat;
        int bestModel = -1;

        for(int j=0; j < myclusters[i].passOBBcandidates.size(); j++)
        {            
            for(int k=0; k < models.size(); k++)
            {
                if(models[k].name == myclusters[i].passOBBcandidates[j])
                {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud (new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::copyPointCloud(models[k].points, *targetCloud);

                    for(double RX = 0; RX <= M_PI; RX+=M_PI)
                    for(double RY = 0; RY <= M_PI; RY+=M_PI)
                    for(double RZ = 0; RZ < M_PI; RZ+=M_PI)
                    {
                        pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud (new pcl::PointCloud<pcl::PointXYZ>);;
                        
                        Eigen::Affine3f ROT = Eigen::Affine3f::Identity();
                        ROT.rotate (Eigen::AngleAxisf (RX, Eigen::Vector3f::UnitX()));
                        ROT.rotate (Eigen::AngleAxisf (RY, Eigen::Vector3f::UnitY()));
                        ROT.rotate (Eigen::AngleAxisf (RZ, Eigen::Vector3f::UnitZ()));                        
                        pcl::transformPointCloud (*origCluster, *sourceCloud, ROT);
                    
                        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                        icp.setInputSource(sourceCloud);
                        icp.setInputTarget(targetCloud);
                        icp.setMaximumIterations (100);
                        icp.setMaxCorrespondenceDistance(0.2);
                        icp.setRANSACOutlierRejectionThreshold(1);
                        pcl::PointCloud<pcl::PointXYZ> Final;
                        icp.align(Final);

                        double overlapScore = overlapPortion(*targetCloud, Final, overlap_dst_thresh);
                        std::cerr << "overlap score: " << overlapScore << "\n";
                        if(overlapScore < overlap_score_thresh) continue; 

                        if(icp.hasConverged() & bestScore < overlapScore)                                             
                        {
                            bestMat = icp.getFinalTransformation();
                            myclusters[i].recognizedObj = models[k].name;
                            bestCoarseMat = ROT.matrix();
                            bestFineMat = icp.getFinalTransformation();
                            bestModel = k;
                            bestScore = overlapScore;
                        }

                        /* if(icp.hasConverged() || bestScore > icp.getFitnessScore())                                             
                        {
                            bestMat = icp.getFinalTransformation();
                            myclusters[i].recognizedObj = models[k].name;
                            bestCoarseMat = ROT.matrix();
                            bestFineMat = icp.getFinalTransformation();
                            bestModel = k;
                            bestScore = icp.getFitnessScore();
                        } */
                    }
                }
            }
        }

        Eigen::Matrix4f finalMat = bestFineMat*bestCoarseMat*myclusters[i].OBB.toOrigin;
        if (bestModel > -1)
        {
            pcl::transformPointCloud(models[bestModel].points, 
                                    myclusters[i].modelPoints, finalMat.inverse());
            pcl::transformPointCloud(models[bestModel].colorPoints, 
                                    myclusters[i].colorModelPoints,  finalMat.inverse()); 
            myclusters[i].OBB.cornerPoints.clear();
            computeOBB(myclusters[i].modelPoints, myclusters[i].OBB);            
            std::cerr << "bestScore: " << bestScore << "\n";
            std::cerr << "Recognised object: " << models[bestModel].name << "\n";
            std::cerr << "Finish registration: " << "Successful!" << "\n";
        }
        else std::cerr << "Fail to registering to model" << "\n";
    }
}