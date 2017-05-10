//#include <image_transport/image_transport.h>
#include <boost/thread/mutex.hpp>

#include <fstream>
#include <iostream>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/StdVector>
#include <time.h>
#include "sdf_descriptor.h"

#ifndef SDF_TEXTURE
#define SDF_TEXTURE
typedef struct
{
  Eigen::Matrix4d transformation;
  cv::Mat image;
} Texture;
#endif


#define EIGEN_USE_NEW_STDVECTOR

#ifndef SDF_TRACKER_REF
#define SDF_TRACKER_REF

class SDF_Parameters
{
public:
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

  SDF_Parameters();
  virtual ~SDF_Parameters();
};

typedef Eigen::Matrix<double,6,1> Vector6d; 


#define TEXTURE_LIMIT 250

class SDFTrackerRef
{
  protected:
  // variables
  Texture* textures_[TEXTURE_LIMIT];
  std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d> > transformations_;
  std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d> > interest_points_;
  Eigen::Matrix4d Transformation_;
  Vector6d Pose_;
  Vector6d cumulative_pose_;
  cv::Mat *depthImage_;
  cv::Mat *depthImage_denoised_;

  boost::mutex transformation_mutex_;
  boost::mutex texture_mutex_;
  boost::mutex depth_mutex_;
  boost::mutex depthDenoised_mutex_;
  std::string camera_name_;
  
  bool** validityMask_;
  float*** myGrid_; 
  float*** weightArray_;    
  Texture*** *textureReference_;
  int current_texture_;
  bool quit_;
  bool first_frame_;
  bool saved_texture_;
  bool refresh_textures_; 

  SDF_Parameters parameters_;

  std::ofstream triangle_stream_;
  std::ofstream IP_stream_;
  std::ofstream FV_stream_;
  // functions 
  Eigen::Vector3d VertexInterp(double iso, Eigen::Vector4d &p1d, Eigen::Vector4d &p2d,double valp1, double valp2);
  void marchingTetrahedrons(Eigen::Vector4d &Origin, int tetrahedron);
  void init(SDF_Parameters &parameters);

  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //virtual void subscribeTopic(const std::string topic = std::string("default"));    
  //virtual void advertiseTopic(const std::string topic = std::string("default"));    
  virtual double SDF(const Eigen::Vector4d &location);
  virtual double SDFGradient(const Eigen::Vector4d &location, int stepSize, int dim);
  bool validGradient(const Eigen::Vector4d &location);
  bool validGradientStrict(const Eigen::Vector4d &location);
  virtual Vector6d EstimatePose(void); 
  cv::Point2d To2D(const Eigen::Vector4d &location, double fx, double fy, double cx, double cy);
  Eigen::Matrix4d Twist(const Vector6d &xi);
  Eigen::Vector4d To3D(int row, int column, double depth, double fx, double fy, double cx, double cy);
  virtual void FuseDepth(const cv::Mat &depth);
  virtual void AddViewpoint(const cv::Mat &rgb);   
  virtual void Render(void);
  virtual void Render(const Eigen::Matrix4d &customTransformation);
  void saveTriangles(const std::string filename = std::string("triangles.obj"));
  void saveInterestPoints(const std::string filename = std::string("interest_points.m"));
  void saveDescriptors(const std::string filename = std::string("feature_vectors.m"));
  void getInterestPoints(std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d> > &IP_vec);
  bool quit(void);
  Eigen::Matrix4d getCurrentTransformation(void);

  void realOnMouse(int event, int x, int y, int flags);
  //void publishDepthDenoisedImage(const ros::TimerEvent& event);
  void getDenoisedImage(cv::Mat &img); 
  
  SDFTrackerRef();
  SDFTrackerRef(SDF_Parameters &parameters);
  virtual ~SDFTrackerRef();    
};

#endif


/*SDF_REGISTRATION*/
