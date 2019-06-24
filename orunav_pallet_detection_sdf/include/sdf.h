#ifndef PALLET_DETECTION_SDF_H
#define PALLET_DETECTION_SDF_H


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace Eigen{ typedef Eigen::Matrix<double,6,1> Vector6d; }

namespace sdf
{
  class SDF_Parameters
  {
  public:
    SDF_Parameters()
      {  
	image_width = 640;
	image_height = 480;
	
	makeTris = false;
	makeIP = true;
	makeFV = true;
	interactive_mode = true;
	use_texture = true;
	Wmax = 64.0;
	resolution = 0.01;
	XSize = 256;
	YSize = 256;
	ZSize = 256;
	Dmax = 0.1;
	Dmin = -0.04;
	pose_offset = Eigen::MatrixXd::Identity(4,4);
	robust_statistic_coefficient = 0.02;
	regularization = 0.01;
	min_pose_change = 0.01;
	min_parameter_update = 0.0001;
	raycast_steps = 12;
	fx = 520.0;
	fy = 520.0;
	cx = 319.5;
	cy = 239.5;
      }
    
    ~SDF_Parameters() {}
    
    bool use_texture;
    bool makeTris;
    bool interactive_mode;
    bool makeIP;
    bool makeFV;
    int XSize;
    int YSize;
    int ZSize;
    int raycast_steps;
    int image_height;
    int image_width;
    double fx;
    double fy;
    double cx;
    double cy;
    double Wmax;
    double resolution;
    double Dmax; 
    double Dmin;
    Eigen::Matrix4d pose_offset;
    double robust_statistic_coefficient;
    double regularization;
    double min_parameter_update;
    double min_pose_change;
  };
  
  enum  operation_t { bool_union, bool_subtraction, bool_intersection};
  
  /// Virtual Base class, from which other primitives are derived. 
  class Primitive
  {
  public:    
    virtual double signedDistance(Eigen::Vector3d &location) = 0;

    /// flips the sign of the distance field so that positive/negative = outside/inside are reversed
    bool invert;
    /// defines which set operation should be applied with this primitive relative to the others
    operation_t operation;
    /// local transformation of the primitive relative to an axis aligned, zero-centered pose
    Eigen::Affine3d transformation;
    /// parameters for determining the geometry of the primitive
    Eigen::Vector4d parameters;
  };

  /// SDF of a box. Parameters to the constructor should be dimensions in x y and z (the fourth element of the parameter vector is not used)
  class Box : public Primitive
  {
  public:
    Box( operation_t op, Eigen::Affine3d &T, Eigen::Vector4d &params, bool inv=false)
    {
      invert = inv;
      operation = op;
      transformation = T;
      parameters = params;  
    }
    ~Box(){}
  
    double signedDistance(Eigen::Vector3d &location)
    { 
   
      Eigen::Vector3d dist;
      dist(0) = fabs(location(0)) - (parameters(0)/2);
      dist(1) = fabs(location(1)) - (parameters(1)/2);
      dist(2) = fabs(location(2)) - (parameters(2)/2);
      Eigen::Vector3d mdz( std::max(dist(0), 0.0),std::max(dist(1), 0.0),std::max(dist(2), 0.0));

      double value = std::min(  std::max(  dist(0), std::max(  dist(1), dist(2)  ) ), 0.0  )  + sqrt(mdz(0)*mdz(0) + mdz(1)*mdz(1) + mdz(2)*mdz(2)); 
      return (invert) ? -value : value;
    }
  };

  /// This is not a truly Euclidean SDF of a box, as the distance field is always parallel to one of the faces. Parameters to the constructor should be dimensions in x y and z (the fourth element of the parameter vector is not used)
  class badBox : public Primitive
  {
  public:
    badBox( operation_t op, Eigen::Affine3d &T, Eigen::Vector4d &params, bool inv=false)
    {
      invert = inv;
      operation = op;
      transformation = T;
      parameters = params;
    }
    double signedDistance(Eigen::Vector3d &location)
    {
      Eigen::Vector3d dist;
      dist(0) = fabs(location(0)) - (parameters(0)/2);
      dist(1) = fabs(location(1)) - (parameters(1)/2);
      dist(2) = fabs(location(2)) - (parameters(2)/2);

      double value = std::max(    std::min(  dist(0),std::min( dist(1),dist(2) )  ) , std::max( dist(0), std::max(dist(1),dist(2) )  )    ); 
      return (invert) ? -value : value;

    }
  };

  /// SDF of a sphere. Parameters to the constructor should contain the radius as the first element, others are disregarded.
  class Sphere : public Primitive
  {
  public:
    Sphere( operation_t op, Eigen::Affine3d &T, Eigen::Vector4d &params, bool inv=false)
    {
      invert = inv;
      operation = op;
      transformation = T;
      parameters = params;  
    }
    ~Sphere(){}

    double signedDistance(Eigen::Vector3d &location)
    { 
      double value = location.norm() - parameters(0);
      return (invert) ? -value : value;
    }  
  };
  
  /// SDF of an infinite cylinder. Parameters to the constructor are the offsets in x,y from the origin, and the radius (the last element is disregarded). The cylinder is extruded along the z direction.
  class Cylinder : public Primitive
  {
  public:
    Cylinder( operation_t op, Eigen::Affine3d &T, Eigen::Vector4d &params, bool inv=false)
    {
      invert = inv;
      operation = op;
      transformation = T;
      parameters = params;  
    }
    ~Cylinder(){}

    double signedDistance(Eigen::Vector3d &location)
    {
      Eigen::Vector2d loc_xz(location(0), location(2));
      Eigen::Vector2d cyl_xy(parameters(0), parameters(1));
      double value = (loc_xz-cyl_xy).norm() - parameters(2);
      return (invert) ? -value : value;
    }
  };

  /// SDF of a torus. Parameters to the constructor are the radius of the sweeping disc and the radius of the circle aound which the sweep is performed (the remaining 2 elements are disregarded). 
  class Torus : public Primitive
  {
  public:
    Torus( operation_t op, Eigen::Affine3d &T, Eigen::Vector4d &params, bool inv=false)
    {
      invert = inv;
      operation = op;
      transformation = T;
      parameters = params;  
    }
    ~Torus(){}

    double signedDistance(Eigen::Vector3d &location)
    {  
      Eigen::Vector2d loc_xz(location(0), location(2));
      Eigen::Vector2d q(loc_xz.norm() - parameters(0) , location(1));
      double value = q.norm() - parameters(1);
      return (invert) ? -value : value;
    }
  };

  /// SDF of a plane. Parameters to the constructor are the three elements of the surface normal direction plus an offset. The field on one side of the plane is positive, the other negative.
  class Plane : public Primitive
  {
  public:
    Plane( operation_t op, Eigen::Affine3d &T, Eigen::Vector4d &params, bool inv=false)
    {
      invert = inv;
      operation = op;
      transformation = T;   
      parameters = params;

    }
    ~Plane(){}

    double signedDistance(Eigen::Vector3d &location)
    {  
      
      Eigen::Vector3d normal(parameters(0),parameters(1),parameters(2));
      double value = normal.normalized().dot(location) + parameters(3);
      return (invert) ? -value : value;
    }   
  };

  /// helper function for raycasting. makes a unit ray given a pixel row, column and depth value, along with camera parameters
  Eigen::Vector3d 
  makeRay(int row, int column, float depth, float fx, float fy, float cx, float cy);

  /// Evaluates the SDF at a given location for a vector containing primitives (set operations are applied in order of placement in the vector)
  double 
  SDF(const Eigen::Vector3d &location, std::vector<Primitive*> &primitives);
  
  /// Evalueate the numeric gradient of the SDF at the given location for a vector containing primitives. 
  Eigen::Vector3d 
  SDFGradient(const Eigen::Vector3d &location, std::vector<Primitive*> &primitives);
  
  /// Evaluate the numeric gradient of the SDF at the given location for a vector containing primitives. The result is normalized and always in the positive octant.
  Eigen::Vector3d 
  SDFGradientPosNorm(const Eigen::Vector3d &location, std::vector<Primitive*> &primitives);

  /// Ray-casts an image of size rows x cols of the SDF at a camera whose pose is given by the supplied transformation. 
  void Render(const Eigen::Affine3d &transformation, 
              const int rows, 
              const int cols, 
              Eigen::Vector4f &cam_params, 
              const int max_steps, 
              const float max_ray_length, 
              const float precision, 
              std::vector<Primitive*> &primitives);

 void RenderPC(
    const Eigen::Affine3d &transformation, 
    const int rows, 
    const int cols, 
    Eigen::Vector4f &cam_params, 
    const int max_steps, 
    const float max_ray_length, 
    const float precision, 
    std::vector<Primitive*> &primitives,
const pcl::PointCloud<pcl::PointXYZ> &depth);
  
    std::pair<double, int> AlignmentPC(const Eigen::Affine3d &transformation, 
              const int rows, 
              const int cols, 
              Eigen::Vector4f &cam_params, 
              const int max_steps, 
              const float max_ray_length, 
              const float precision, 
              std::vector<Primitive*> &primitives,
	      const pcl::PointCloud<pcl::PointXYZ> &depth);

    Eigen::Affine3d AlignmentSearchPC(const Eigen::Affine3d &transformation, 
				      const int rows, 
				      const int cols, 
				      Eigen::Vector4f &cam_params, 
				      const int max_steps, 
				      const float max_ray_length, 
				      const float precision, 
				      std::vector<Primitive*> &primitives,
				      const pcl::PointCloud<pcl::PointXYZ> &depth);

  void RenderDiffPC(const Eigen::Affine3d &transformation, 
		    const int rows, 
		    const int cols, 
		    Eigen::Vector4f &cam_params, 
		    const int max_steps, 
		    const float max_ray_length, 
		    const float precision, 
		    std::vector<Primitive*> &primitives,
		    const pcl::PointCloud<pcl::PointXYZ> &depth);

 Eigen::Affine3d
  EstimatePalletPosePC(const Eigen::Affine3d &transformation,
		       const int rows, 
		       const int cols, 
		       Eigen::Vector4f &cam_params, 
		       const int max_steps, 
		       const float max_ray_length, 
		       const float precision, 
		       std::vector<Primitive*> &primitives,
		       const pcl::PointCloud<pcl::PointXYZ> &depth);
  

}//sdf

#endif
