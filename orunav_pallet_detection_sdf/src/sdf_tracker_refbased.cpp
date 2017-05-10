//#include <image_transport/image_transport.h>
#include <cmath>
#include <iostream>
#include <limits>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
// #include <boost/foreach.hpp>
#include <opencv2/core/core.hpp>

#include <time.h>
#include "sdf_tracker_refbased.h"


static void wrappedOnMouseRef(int event, int x, int y, int flags, void* ptr)
{
    SDFTrackerRef* mcPtr = (SDFTrackerRef*)ptr;
    if(mcPtr != NULL) mcPtr->realOnMouse(event, x, y, flags);
}


SDF_Parameters::SDF_Parameters()
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

SDF_Parameters::~SDF_Parameters()
{}

SDFTrackerRef::SDFTrackerRef() {
    SDF_Parameters myparams = SDF_Parameters();
    this->init(myparams);
}

SDFTrackerRef::SDFTrackerRef(SDF_Parameters &parameters)
{
  this->init(parameters);
}

SDFTrackerRef::~SDFTrackerRef()
{
  for (int i = 0; i < parameters_.XSize; ++i)
  {
    for (int j = 0; j < parameters_.YSize; ++j)
    {
      if (myGrid_[i][j]!=NULL)
      delete[] myGrid_[i][j];

      if (weightArray_[i][j]!=NULL)
      delete[] weightArray_[i][j];

      if (textureReference_[i][j]!=NULL)
      delete[] textureReference_[i][j];
    }
     
    if (myGrid_[i]!=NULL)
    delete[] myGrid_[i];

    if (weightArray_[i]!=NULL)
    delete[] weightArray_[i];   

    if (textureReference_[i]!=NULL)
    delete[] textureReference_[i];   
  }

  delete[] myGrid_;
  delete[] weightArray_;  
  delete[] textureReference_;  
  
  for (int i = 0; i < parameters_.image_height; ++i)
  {
    if ( validityMask_[i]!=NULL)
    delete[] validityMask_[i];
  }
  delete[] validityMask_;
  
  if(depthImage_!=NULL)
  delete depthImage_;

  if(depthImage_denoised_!=NULL)
  delete depthImage_denoised_;
  
};

void SDFTrackerRef::init(SDF_Parameters &parameters)
{
  parameters_ = parameters;

  int downsample=1;
  switch(parameters_.image_height)
  {
  case 480: downsample = 1; break; //VGA
  case 240: downsample = 2; break; //QVGA
  case 120: downsample = 4; break; //QQVGA
  }
  parameters_.fx /= downsample;
  parameters_.fy /= downsample;
  parameters_.cx /= downsample;
  parameters_.cy /= downsample;

  depthImage_ = new cv::Mat(parameters_.image_height,parameters_.image_width,CV_32FC1); 
  depthImage_denoised_ = new cv::Mat( parameters_.image_height,parameters_.image_width,CV_32FC1);

  validityMask_ = new bool*[parameters_.image_height];
  for (int i = 0; i < parameters_.image_height; ++i)
  {
    validityMask_[i] = new bool[parameters_.image_width];
  }   

  myGrid_ = new float**[parameters_.XSize];
  weightArray_ = new float**[parameters_.XSize];
  textureReference_ = new Texture***[parameters_.XSize];

  for (int i = 0; i < parameters_.XSize; ++i)
  {
    myGrid_[i] = new float*[parameters_.YSize];
    weightArray_[i] = new float*[parameters_.YSize];
    textureReference_[i] = new Texture**[parameters_.YSize];

    for (int j = 0; j < parameters_.YSize; ++j)
    {
      myGrid_[i][j] = new float[parameters_.ZSize];
      weightArray_[i][j] = new float[parameters_.ZSize];
      textureReference_[i][j] = new Texture*[parameters_.ZSize];
    }
  }
    
  for (int x = 0; x < parameters_.XSize; ++x)
  {
    for (int y = 0; y < parameters_.YSize; ++y)
    {
      for (int z = 0; z < parameters_.ZSize; ++z)
      {
        myGrid_[x][y][z]=parameters_.Dmax;
        weightArray_[x][y][z]=0.0f;
      }
    }
  }
  current_texture_ = 0;
  saved_texture_ = false;
  refresh_textures_ = true;
  quit_ = false;
  first_frame_ = true;
  Pose_ << 0.0,0.0,0.0,0.0,0.0,0.0;
  cumulative_pose_ << 0.0,0.0,0.0,0.0,0.0,0.0;
  Transformation_=parameters_.pose_offset*Eigen::MatrixXd::Identity(4,4);

  if(parameters_.interactive_mode)
  {
    cv::namedWindow( "Render", 0 );
    cv::setMouseCallback( "Render", wrappedOnMouseRef, (void*)this );
  }
};

void SDFTrackerRef::saveInterestPoints(const std::string filename)
{
  IP_stream_.open(filename);
  IP_stream_ << "IP = [..." << std::endl;
  for (uint i = 0; i < interest_points_.size(); ++i)
  {
    IP_stream_ << interest_points_[i].transpose() << ";..." << std::endl;    
  }
  IP_stream_ << "];" << std::endl;
  IP_stream_.close();
}

void SDFTrackerRef::saveDescriptors(const std::string filename)
{
    FV_stream_.open(filename);
    FV_stream_ << "fv = [..." << std::endl;          
      for (uint i = 0; i < interest_points_.size(); ++i)
    {
      SDFDescriptor desc = SDFDescriptor(&interest_points_[i],this);
      FV_stream_ << desc.feature << ";..." << std::endl;    
    }
    FV_stream_ << "];";
    FV_stream_.close();
}


void SDFTrackerRef::saveTriangles(const std::string filename)
{
  triangle_stream_.open(filename);
  for (int i = 1; i < parameters_.XSize-2; ++i)
  {
    for (int j = 1; j < parameters_.YSize-2; ++j)
    {
      for (int k = 1; k < parameters_.ZSize-2; ++k)
      {
        Eigen::Vector4d CellOrigin = Eigen::Vector4d(double(i),double(j),double(k),1.0);
        //if(!validGradient(CellOrigin*parameters_.resolution)) continue;
        /*1*/marchingTetrahedrons(CellOrigin,1);
        /*2*/marchingTetrahedrons(CellOrigin,2);
        /*3*/marchingTetrahedrons(CellOrigin,3);
        /*4*/marchingTetrahedrons(CellOrigin,4);
        /*5*/marchingTetrahedrons(CellOrigin,5);
        /*6*/marchingTetrahedrons(CellOrigin,6);
      }
    }
  }
  triangle_stream_.close();
}

Eigen::Vector3d 
SDFTrackerRef::VertexInterp(double iso, Eigen::Vector4d &p1d, Eigen::Vector4d &p2d,double valp1, double valp2)
{
  double mu;
  Eigen::Vector3d p;
  Eigen::Vector3d p1 = Eigen::Vector3d(p1d(0) , p1d(1), p1d(2) );
  Eigen::Vector3d p2 = Eigen::Vector3d(p2d(0) , p2d(1), p2d(2) );

  if (fabs(iso-valp1) < 0.000001)
    return(p1);
  if (fabs(iso-valp2) < 0.000001)
    return(p2);
  if (fabs(valp1-valp2) < 0.000001)
    return(p1);
  mu = (iso - valp1) / (valp2 - valp1);
  p(0) = p1d(0) + mu * (p2d(0) - p1d(0));
  p(1) = p1d(1) + mu * (p2d(1) - p1d(1));
  p(2) = p1d(2) + mu * (p2d(2) - p1d(2));

  return(p);  
};

Eigen::Matrix4d 
SDFTrackerRef::Twist(const Vector6d &xi)
{
  Eigen::Matrix4d M;
  
  M << 0.0  , -xi(2),  xi(1), xi(3),
       xi(2), 0.0   , -xi(0), xi(4),
      -xi(1), xi(0) , 0.0   , xi(5),
       0.0,   0.0   , 0.0   , 0.0  ;
  
  return M;
};

Eigen::Vector4d 
SDFTrackerRef::To3D(int row, int column, double depth, double fx, double fy, double cx, double cy)
{

  Eigen::Vector4d ret(double(column-cx)*depth/(fx),
                      double(row-cy)*depth/(fy),
                      double(depth),
                      1.0f);
  return ret;
};

cv::Point2d 
SDFTrackerRef::To2D(const Eigen::Vector4d &location, double fx, double fy, double cx, double cy)
{
  cv::Point2d pixel(0,0);  
  if(location(2) != 0)
  {
     pixel.x = (cx) + location(0)/location(2)*(fx);
     pixel.y = (cy) + location(1)/location(2)*(fy);
  }
  
  return pixel;  
};

bool 
SDFTrackerRef::validGradientStrict(const Eigen::Vector4d &location)
{
 /* 
 The function tests the current location and its adjacent
 voxels for valid values (NOT TRUNCATED) to 
 determine if derivatives at this location are 
 computable in all three directions.

 Since the function SDF(Eigen::Vector4d &location) is a 
 trilinear interpolation between neighbours, testing the
 validity of the gradient involves looking at all the 
 values that would contribute to the final  gradient. 
 If any of these have a weight equal to zero, the result
 is false.
                      X--------X
                    /        / |
                  X--------X   ----X
                  |        |   | / |
              X----        |   X-------X
            /     |        | /       / |
          X-------X--------X-------X   |
          |     /        / |       |   |
          |   X--------X   |       |   |
     J    |   |        |   |       | /
     ^    X----        |   X-------X
     |        |        | / |  |
      --->I   X--------X   |  X
    /             |        | /
   v              X--------X
  K                                                */

  float eps = 1e-6; 
  double i,j,k;
  modf(location(0)/parameters_.resolution + parameters_.XSize/2, &i);
  modf(location(1)/parameters_.resolution + parameters_.YSize/2, &j);  
  modf(location(2)/parameters_.resolution + parameters_.ZSize/2, &k);
  
  if(std::isnan(i) || std::isnan(j) || std::isnan(k)) return false;

  int I = int(i)-1; int J = int(j)-1;   int K = int(k)-1;  
  
  if(I>=parameters_.XSize-4 || J>=parameters_.YSize-3 || K>=parameters_.ZSize-3 || I<=1 || J<=1 || K<=1)return false;

  float* D10 = &myGrid_[I+1][J+0][K];
  float* D20 = &myGrid_[I+2][J+0][K];
 
  float* D01 = &myGrid_[I+0][J+1][K];
  float* D11 = &myGrid_[I+1][J+1][K];
  float* D21 = &myGrid_[I+2][J+1][K];
  float* D31 = &myGrid_[I+3][J+1][K];
  
  float* D02 = &myGrid_[I+0][J+2][K];
  float* D12 = &myGrid_[I+1][J+2][K];
  float* D22 = &myGrid_[I+2][J+2][K];
  float* D32 = &myGrid_[I+3][J+2][K];

  float* D13 = &myGrid_[I+1][J+3][K];
  float* D23 = &myGrid_[I+2][J+3][K];


  if( fabsf(D10[1]-parameters_.Dmax) < eps || fabsf(D10[2]-parameters_.Dmax) < eps || 
      fabsf(D20[1]-parameters_.Dmax) < eps || fabsf(D20[2]-parameters_.Dmax) < eps || 
      
      fabsf(D01[1]-parameters_.Dmax) < eps || fabsf(D01[2]-parameters_.Dmax) < eps ||
      fabsf(D11[0]-parameters_.Dmax) < eps || fabsf(D11[1]-parameters_.Dmax) < eps || fabsf(D11[2]-parameters_.Dmax) < eps || fabsf(D11[3]-parameters_.Dmax) < eps ||
      fabsf(D21[0]-parameters_.Dmax) < eps || fabsf(D21[1]-parameters_.Dmax) < eps || fabsf(D21[2]-parameters_.Dmax) < eps || fabsf(D21[3]-parameters_.Dmax) < eps ||
      fabsf(D31[1]-parameters_.Dmax) < eps || fabsf(D31[2]-parameters_.Dmax) < eps ||
      
      fabsf(D02[1]-parameters_.Dmax) < eps || fabsf(D02[2]-parameters_.Dmax) < eps ||
      fabsf(D12[0]-parameters_.Dmax) < eps || fabsf(D12[1]-parameters_.Dmax) < eps || fabsf(D12[2]-parameters_.Dmax) < eps || fabsf(D12[3]-parameters_.Dmax) < eps ||
      fabsf(D22[0]-parameters_.Dmax) < eps || fabsf(D22[1]-parameters_.Dmax) < eps || fabsf(D22[2]-parameters_.Dmax) < eps || fabsf(D22[3]-parameters_.Dmax) < eps ||
      fabsf(D32[1]-parameters_.Dmax) < eps || fabsf(D32[2]-parameters_.Dmax) < eps ||
      
      fabsf(D13[1]-parameters_.Dmax) < eps || fabsf(D13[2]-parameters_.Dmax) < eps ||
      fabsf(D23[1]-parameters_.Dmax) < eps || fabsf(D23[2]-parameters_.Dmax) < eps 
      ) return false;
  else return true;
};


bool 
SDFTrackerRef::validGradient(const Eigen::Vector4d &location)
{
 /* 
 The function tests the current location and its adjacent
 voxels for valid values (written at least once) to 
 determine if derivatives at this location are 
 computable in all three directions.

 Since the function SDF(Eigen::Vector4d &location) is a 
 trilinear interpolation between neighbours, testing the
 validity of the gradient involves looking at all the 
 values that would contribute to the final  gradient. 
 If any of these have a weight equal to zero, the result
 is false.
                      X--------X
                    /        / |
                  X--------X   ----X
                  |        |   | / |
              X----        |   X-------X
            /     |        | /       / |
          X-------X--------X-------X   |
          |     /        / |       |   |
          |   X--------X   |       |   |
     J    |   |        |   |       | /
     ^    X----        |   X-------X
     |        |        | / |  |
      --->I   X--------X   |  X
    /             |        | /
   v              X--------X
  K                                                */

  float eps = 1e-6; 
  double i,j,k;
  modf(location(0)/parameters_.resolution + parameters_.XSize/2, &i);
  modf(location(1)/parameters_.resolution + parameters_.YSize/2, &j);  
  modf(location(2)/parameters_.resolution + parameters_.ZSize/2, &k);
  
  if(std::isnan(i) || std::isnan(j) || std::isnan(k)) return false;

  int I = int(i)-1; int J = int(j)-1;   int K = int(k)-1;  
  
  if(I>=parameters_.XSize-4 || J>=parameters_.YSize-3 || K>=parameters_.ZSize-3 || I<=1 || J<=1 || K<=1)return false;

  float* W10 = &weightArray_[I+1][J+0][K];
  float* W20 = &weightArray_[I+2][J+0][K];
 
  float* W01 = &weightArray_[I+0][J+1][K];
  float* W11 = &weightArray_[I+1][J+1][K];
  float* W21 = &weightArray_[I+2][J+1][K];
  float* W31 = &weightArray_[I+3][J+1][K];
  
  float* W02 = &weightArray_[I+0][J+2][K];
  float* W12 = &weightArray_[I+1][J+2][K];
  float* W22 = &weightArray_[I+2][J+2][K];
  float* W32 = &weightArray_[I+3][J+2][K];

  float* W13 = &weightArray_[I+1][J+3][K];
  float* W23 = &weightArray_[I+2][J+3][K];

  if( W10[1] < eps || W10[2] < eps || 
      W20[1] < eps || W20[2] < eps || 
      
      W01[1] < eps || W01[2] < eps ||
      W11[0] < eps || W11[1] < eps || W11[2] < eps || W11[3] < eps ||
      W21[0] < eps || W21[1] < eps || W21[2] < eps || W21[3] < eps ||
      W31[1] < eps || W31[2] < eps ||
      
      W02[1] < eps || W02[2] < eps ||
      W12[0] < eps || W12[1] < eps || W12[2] < eps || W12[3] < eps ||
      W22[0] < eps || W22[1] < eps || W22[2] < eps || W22[3] < eps ||
      W32[1] < eps || W32[2] < eps ||
      
      W13[1] < eps || W13[2] < eps ||
      W23[1] < eps || W23[2] < eps 
      ) return false;
  else return true;
}


double 
SDFTrackerRef::SDFGradient(const Eigen::Vector4d &location, int stepSize, int dim )
{
  double delta=parameters_.resolution*stepSize;
  Eigen::Vector4d location_offset = Eigen::Vector4d(0,0,0,1);
  location_offset(dim) = delta;

  return ((SDF(location+location_offset)) - (SDF(location-location_offset)))/(2.0*delta);
};

void 
SDFTrackerRef::marchingTetrahedrons(Eigen::Vector4d &Origin, int tetrahedron)
{
  /*
  The following part is adapted from code found at:
  http://paulbourke.net/geometry/polygonise/

  (Paul Bourke / David Thoth)


  Function that outputs polygons from the SDF. The function is called
  giving a 3D location  of the (zero, zero) vertex and an index. 
  The index indicates which of the six possible tetrahedrons that can 
  be formed within a cube should be checked. 

      04===============05
      |\\              |\\
      ||\\             | \\
      || \\            |  \\
      ||  07===============06
      ||  ||           |   ||
      ||  ||           |   ||
      00--||-----------01  ||
       \\ ||            \  ||
        \\||             \ ||
         \||              \||
          03===============02

  Polygonise a tetrahedron given its vertices within a cube
  This is an alternative algorithm to Marching Cubes.
  It results in a smoother surface but more triangular facets.

              + 0
             /|\
            / | \
           /  |  \
          /   |   \
         /    |    \
        /     |     \
       +-------------+ 1
      3 \     |     /
         \    |    /
          \   |   /
           \  |  /
            \ | /
             \|/
              + 2

  Typically, for each location in space one would call:

  marchingTetrahedrons(CellOrigin,1);
  marchingTetrahedrons(CellOrigin,2);
  marchingTetrahedrons(CellOrigin,3);
  marchingTetrahedrons(CellOrigin,4);
  marchingTetrahedrons(CellOrigin,5);
  marchingTetrahedrons(CellOrigin,6);              
  */


  float val0, val1, val2, val3;
  val0 = val1 = val2 = val3 = parameters_.Dmax;

  Eigen::Vector4d V0, V1, V2, V3;
    
  int i = int(Origin(0));
  int j = int(Origin(1));
  int k = int(Origin(2));

  switch(tetrahedron)
  {
    case 1:
    val0 = myGrid_[i][j][k+1];
    V0 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2)+1,1.0)*parameters_.resolution;
    val1 = myGrid_[i+1][j][k];
    V1 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2),1.0)*parameters_.resolution;
    val2 = myGrid_[i][j][k];
    V2 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2),1.0)*parameters_.resolution;
    val3 = myGrid_[i][j+1][k];
    V3 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2),1.0)*parameters_.resolution;      
    break;
    
    case 2:  
    val0 = myGrid_[i][j][k+1];
    V0 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2)+1,1.0)*parameters_.resolution;
    val1 = myGrid_[i+1][j][k];
    V1 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2),1.0)*parameters_.resolution;
    val2 = myGrid_[i+1][j+1][k];
    V2 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2),1.0)*parameters_.resolution;
    val3 = myGrid_[i][j+1][k];
    V3 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2),1.0)*parameters_.resolution;      
    break;
    
    case 3:
    val0 = myGrid_[i][j][k+1];
    V0 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2)+1,1.0)*parameters_.resolution;
    val1 = myGrid_[i][j+1][k+1];
    V1 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2)+1,1.0)*parameters_.resolution;
    val2 = myGrid_[i+1][j+1][k];
    V2 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2),1.0)*parameters_.resolution;
    val3 = myGrid_[i][j+1][k];
    V3 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2),1.0)*parameters_.resolution;      
    break;
    
    case 4:      
    val0 = myGrid_[i][j][k+1];
    V0 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2)+1,1.0)*parameters_.resolution;
    val1 = myGrid_[i+1][j+1][k];
    V1 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2),1.0)*parameters_.resolution;
    val2 = myGrid_[i+1][j][k+1];
    V2 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2)+1,1.0)*parameters_.resolution;
    val3 = myGrid_[i+1][j][k];
    V3 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2),1.0)*parameters_.resolution;      
    break;
      
    case 5:
    val0 = myGrid_[i][j][k+1];
    V0 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2)+1,1.0)*parameters_.resolution;
    val1 = myGrid_[i+1][j+1][k];
    V1 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2),1.0)*parameters_.resolution;
    val2 = myGrid_[i+1][j][k+1];
    V2 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2)+1,1.0)*parameters_.resolution;
    val3 = myGrid_[i][j+1][k+1];
    V3 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2)+1,1.0)*parameters_.resolution;      
    break;
    
    case 6:
    val0 = myGrid_[i+1][j+1][k+1];
    V0 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2)+1,1.0)*parameters_.resolution;
    val1 = myGrid_[i+1][j+1][k];
    V1 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2),1.0)*parameters_.resolution;
    val2 = myGrid_[i+1][j][k+1];
    V2 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2)+1,1.0)*parameters_.resolution;
    val3 = myGrid_[i][j+1][k+1];
    V3 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2)+1,1.0)*parameters_.resolution;      
    break;
  }  
    
  if(val0>parameters_.Dmax-parameters_.resolution || val1>parameters_.Dmax-parameters_.resolution || val2>parameters_.Dmax-parameters_.resolution || val3>parameters_.Dmax-parameters_.resolution )
  
    return;

  int count = 0;
  if(val0 < 0)count++;
  if(val1 < 0)count++;
  if(val2 < 0)count++;
  if(val3 < 0)count++;

  {
    switch(count)
    {
      case 0:
      case 4:
      break;

      case 1:
      /*One voxel has material*/
      if(val0 < 0) /*03,02,01*/
      {       
        if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 || tetrahedron == 6)
        {
          /*correct tetrahedrons for this winding: 1,3,4,6*/
          /*wrong tetrahedrons for this winding: 2,5*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        if(tetrahedron == 2 || tetrahedron == 5)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val1 < 0) /*01,13,12*/
      {     
        if(tetrahedron == 2 ||tetrahedron == 5)
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 1 ||tetrahedron == 3||tetrahedron == 4|| tetrahedron == 6)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val2 < 0) /*02,12,23*/
      {     
        if(tetrahedron == 2 || tetrahedron == 5)
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 3||tetrahedron == 1 ||tetrahedron == 4 ||tetrahedron == 6)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val3 < 0) /*03,32,31*/
      {     
        if(tetrahedron == 2 ||tetrahedron == 5)
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V2,val3,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V1,val3,val1).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 ||tetrahedron == 6)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V3,V1,val3,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V2,val3,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      break;

      case 2:
      /*two voxels have material*/
      if(val0 < 0 && val3 < 0)   /*01,02,31;31,02,32*/
      { 
        if(tetrahedron == 2 ||tetrahedron == 5 )
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V1,val3,val1).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V0,val2,val0).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V2,val3,val2).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 4||tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 6)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V3,V1,val3,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V2,val3,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V0,val2,val0).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val1 < 0 && val2 < 0) /*13,32,02;02,01,13*/
      {     
        if(tetrahedron == 1 ||tetrahedron == 4 || tetrahedron == 6 || tetrahedron == 3)
        {
          /*correct tetrahedrons for this winding: 1,3,4,6*/
          /*wrong tetrahedrons for this winding: 2,5*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V2,val3,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 2||tetrahedron == 5)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V2,val3,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val2 < 0 && val3 < 0)/*03,02,13;13,02,12*/
      {     
        if(tetrahedron == 2 || tetrahedron == 5)
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 ||tetrahedron == 6)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val0 < 0 && val1 < 0)/*03,02,13;13,02,12*/
      {     
        if(tetrahedron == 3 ||tetrahedron == 6 || tetrahedron == 1 || tetrahedron == 4)
        {
          /*correct tetrahedrons for this winding: 1,3,4,6*/
          /*wrong tetrahedrons for this winding: 2,5*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 2 || tetrahedron == 5 )
        {
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val1 < 0 && val3 < 0)/*01,12,32;32,30,01*/
      {     
        if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 || tetrahedron == 6)
        {
          /*correct tetrahedrons for this winding: 1,3,4,6*/
          /*wrong tetrahedrons for this winding: 2,5*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V0,val3,val0).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 2 ||tetrahedron == 5)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V0,val3,val0).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val0 < 0 && val2 < 0)/*01,03,32;32,12,01*/
      {
        if(tetrahedron == 1  ||tetrahedron == 3 ||tetrahedron == 4||tetrahedron == 6)
        {
          /*correct tetrahedrons for this winding: 1,3,4,6*/
          /*wrong tetrahedrons for this winding: 2,5*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 5||tetrahedron == 2)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      break;
      
      case 3:
      /*three voxels have material*/
      if(val0 > 0)/*03,01,02*/
      {       
        if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 ||tetrahedron == 6 )
        {
          /*correct tetrahedrons for this winding: 1,3,4,6*/
          /*wrong tetrahedrons for this winding: 2,5*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 2 || tetrahedron == 5)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val1 > 0)/*10,12,13*/
      {
        if(tetrahedron == 2 ||tetrahedron == 5 )
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 ||tetrahedron == 6 )
        {
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val2 > 0) /*20,23,21*/
      {
        if(tetrahedron == 2 || tetrahedron == 5 )
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V1,val2,val1).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        } 
        else if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 ||tetrahedron == 6)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V2,V1,val2,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val3 > 0)/*30,31,32*/
      {
        if(tetrahedron == 2 ||tetrahedron == 5  )
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 3 ||tetrahedron == 1 ||tetrahedron == 4 ||tetrahedron == 6)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      break;
    }
  }
};

void 
SDFTrackerRef::FuseDepth(const cv::Mat& depth)
{
 
  depth_mutex_.lock();
  depth.copyTo(*depthImage_);
  depth_mutex_.unlock();

  for(int row=0; row<depthImage_->rows-0; ++row)
  { 
    const float* Drow = depthImage_->ptr<float>(row);
    #pragma omp parallel for 
    for(int col=0; col<depthImage_->cols-0; ++col)
    { 
      if(!std::isnan(Drow[col]) && Drow[col]>0.4)
      {
      validityMask_[row][col]=true;
      }else
      {
        validityMask_[row][col]=false;
      }
    }
  }
  
  bool hasfused;
  if(!first_frame_)
  {
    hasfused = true;
    Pose_ = EstimatePose();
  } 
  else
  {
    hasfused = false;
    first_frame_ = false;
    if(depthImage_->rows!=parameters_.image_height) std::cout << "depth image rows do not match given image height parameter"<<std::endl;

  } 
  
//--  Transformation_ = Twist(Pose_).exp()*Transformation_;
  
  transformations_.push_back(Transformation_);
  cumulative_pose_ += Pose_;
  Pose_ = Pose_ * 0.0;

  if(cumulative_pose_.norm() < parameters_.min_pose_change && hasfused ){/*Render();*/ return;}
  cumulative_pose_ *= 0.0;
  
  Eigen::Matrix4d camToWorld = Transformation_.inverse();
  Eigen::Vector4d camera = camToWorld * Eigen::Vector4d(0.0,0.0,0.0,1.0);
  //Main 3D reconstruction loop
  
  for(int x = 0; x<parameters_.XSize; ++x)
  { 
  #pragma omp parallel for \
  shared(x)
    for(int y = 0; y<parameters_.YSize;++y)
    { 
      float* previousD = &myGrid_[x][y][0];
      float* previousW = &weightArray_[x][y][0];      
      for(int z = 0; z<parameters_.ZSize; ++z)
      {           
        //define a ray and point it into the center of a node
        Eigen::Vector4d ray((x-parameters_.XSize/2)*parameters_.resolution, (y- parameters_.YSize/2)*parameters_.resolution , (z- parameters_.ZSize/2)*parameters_.resolution, 1);        
        ray = camToWorld*ray;
        if(ray(2)-camera(2) < 0) continue;
        
        cv::Point2d uv;
        uv=To2D(ray,parameters_.fx,parameters_.fy,parameters_.cx,parameters_.cy );
        
        int j=floor(uv.x);
        int i=floor(uv.y);      
        
        //if the projected coordinate is within image bounds
        if(i>0 && i<depthImage_->rows-1 && j>0 && j <depthImage_->cols-1 && validityMask_[i][j] &&    
            validityMask_[i-1][j] && validityMask_[i][j-1])
        {
          const float* Di = depthImage_->ptr<float>(i);
          double Eta; 
          const float W=1/((1+Di[j])*(1+Di[j]));
            
          Eta=(double(Di[j])-ray(2));       
            
          if(Eta >= parameters_.Dmin)
          {
            if(refresh_textures_)//Eta < parameters_.Dmax && refresh_textures_)
            {
              texture_mutex_.lock();
              Texture* tx = textures_[current_texture_];
              texture_mutex_.unlock();
              if(tx != NULL)
              {
                //Eigen::Vector4d texture_position = tx->transformation*Eigen::Vector4d(0.0,0.0,0.0,1.0);
                if(previousW[z] > 1.0f)//textureReference_[x][y][z] == NULL)
                {
                  textureReference_[x][y][z] = textures_[current_texture_];//&textures_.back();
                  saved_texture_=true;
                }
              } 
              // condition to replace existing texture (TODO)
              // else if(used_texture_)
              // {
              //   Eigen::Vector4d old_texture_position = textureReference_[x][y][z]->transformation*Eigen::Vector4d(0.0,0.0,0.0,1.0);
              //   if( (ray - texture_position).normalized().dot((ray - camera).normalized()) > (ray - old_texture_position).normalized().dot((ray - camera).normalized())) 
              //     textureReference_[x][y][z] = &textures_[current_texture_];
              // }
            }
              
            double D = std::min(Eta,parameters_.Dmax);//*copysign(1.0,Eta);*perpendicular
                
            previousD[z] = (previousD[z] * previousW[z] + float(D) * W) /
                      (previousW[z] + W);

            previousW[z] = std::min(previousW[z] + W , float(parameters_.Wmax));

          }//within visible region 
        }//within bounds      
      }//z   
    }//y
  }//x
 // Render();
  return;
};

void
SDFTrackerRef::AddViewpoint(const cv::Mat &rgb)
{
  //If render hasn't said we need new textures, dont bother
  if(!refresh_textures_) return;

  //If fuse ha saved the current texture, we can stop refreshing
  if(saved_texture_)
   {
    //but the next texture needs a different reference
      texture_mutex_.lock();
      ++current_texture_;
      //we got the message from fuse, so we can set this back
      saved_texture_ = false;
      refresh_textures_ = false;
      texture_mutex_.unlock();
   }

  // printf("added a texture (first one!)\n");
  Texture* tex = new Texture;
  tex->transformation = Transformation_;
  rgb.copyTo(tex->image);

  texture_mutex_.lock();
  textures_[current_texture_] = tex;
  texture_mutex_.unlock();
  
  if (current_texture_ >= TEXTURE_LIMIT)
  {
    printf("texture limit reached!\n");
    --current_texture_;
  }
  
  return;
};

double 
SDFTrackerRef::SDF(const Eigen::Vector4d &location)
{
  double i,j,k;
  double x,y,z;
  
  if(std::isnan(location(0)+location(1)+location(2))) return parameters_.Dmax;
  
  x = modf(location(0)/parameters_.resolution + parameters_.XSize/2, &i);
  y = modf(location(1)/parameters_.resolution + parameters_.YSize/2, &j);  
  z = modf(location(2)/parameters_.resolution + parameters_.ZSize/2, &k);
    
  if(i>=parameters_.XSize-1 || j>=parameters_.YSize-1 || k>=parameters_.ZSize-1 || i<0 || j<0 || k<0)return parameters_.Dmax;

  int I = int(i); int J = int(j);   int K = int(k);
  
  float* N1 = &myGrid_[I][J][K];
  float* N2 = &myGrid_[I][J+1][K];
  float* N3 = &myGrid_[I+1][J][K];
  float* N4 = &myGrid_[I+1][J+1][K];

  double a1,a2,b1,b2;
  a1 = double(N1[0]*(1-z)+N1[1]*z);
  a2 = double(N2[0]*(1-z)+N2[1]*z);
  b1 = double(N3[0]*(1-z)+N3[1]*z);
  b2 = double(N4[0]*(1-z)+N4[1]*z);
    
  return double((a1*(1-y)+a2*y)*(1-x) + (b1*(1-y)+b2*y)*x);
};

Vector6d 
SDFTrackerRef::EstimatePose(void) 
{
  Vector6d xi;
  xi<<0.0,0.0,0.0,0.0,0.0,0.0; // + (Pose-previousPose)*0.1;
  Vector6d xi_prev = xi;
  const double eps = 1.0e-9;
  const double c = parameters_.robust_statistic_coefficient*parameters_.Dmax;
  
  const int iterations[3]={12, 8, 2};
  const int stepSize[3] = {4, 2, 1};

  for(int lvl=0; lvl < 3; ++lvl)
  {
    for(int k=0; k<iterations[lvl]; ++k)
    {

      const Eigen::Matrix4d camToWorld = Twist(xi).exp()*Transformation_;
      
      double A00=0.0,A01=0.0,A02=0.0,A03=0.0,A04=0.0,A05=0.0;
      double A10=0.0,A11=0.0,A12=0.0,A13=0.0,A14=0.0,A15=0.0;
      double A20=0.0,A21=0.0,A22=0.0,A23=0.0,A24=0.0,A25=0.0;
      double A30=0.0,A31=0.0,A32=0.0,A33=0.0,A34=0.0,A35=0.0;
      double A40=0.0,A41=0.0,A42=0.0,A43=0.0,A44=0.0,A45=0.0;
      double A50=0.0,A51=0.0,A52=0.0,A53=0.0,A54=0.0,A55=0.0;
      
      double g0=0.0, g1=0.0, g2=0.0, g3=0.0, g4=0.0, g5=0.0;
      
      for(int row=0; row<depthImage_->rows-0; row+=stepSize[lvl])
      {          
        #pragma omp parallel for \
        default(shared) \
        reduction(+:g0,g1,g2,g3,g4,g5,A00,A01,A02,A03,A04,A05,A10,A11,A12,A13,A14,A15,A20,A21,A22,A23,A24,A25,A30,A31,A32,A33,A34,A35,A40,A41,A42,A43,A44,A45,A50,A51,A52,A53,A54,A55)
        for(int col=0; col<depthImage_->cols-0; col+=stepSize[lvl])
        {
          if(!validityMask_[row][col]) continue;
          double depth = double(depthImage_->ptr<float>(row)[col]); 
          Eigen::Vector4d currentPoint = camToWorld*To3D(row,col,depth,parameters_.fx,parameters_.fy,parameters_.cx,parameters_.cy);
          
          if(!validGradient(currentPoint)) continue;
          double D = (SDF(currentPoint));
          double Dabs = fabs(D);
          if(D > parameters_.Dmax - eps || D < parameters_.Dmin + eps) continue;
          
          //partial derivative of SDF wrt position  
          Eigen::Matrix<double,1,3> dSDF_dx(SDFGradient(currentPoint,1,0),
                                            SDFGradient(currentPoint,1,1),
                                            SDFGradient(currentPoint,1,2) 
                                            );
          //partial derivative of position wrt optimizaiton parameters
          Eigen::Matrix<double,3,6> dx_dxi; 
          dx_dxi << 0, currentPoint(2), -currentPoint(1), 1, 0, 0,
                    -currentPoint(2), 0, currentPoint(0), 0, 1, 0,
                    currentPoint(1), -currentPoint(0), 0, 0, 0, 1;

          //jacobian = derivative of SDF wrt xi (chain rule)
          Eigen::Matrix<double,1,6> J = dSDF_dx*dx_dxi;
          
          //double tukey = (1-(Dabs/c)*(Dabs/c))*(1-(Dabs/c)*(Dabs/c));
          double huber = Dabs < c ? 1.0 : c/Dabs;
          
          //Gauss - Newton approximation to hessian
          Eigen::Matrix<double,6,6> T1 = huber * J.transpose() * J;
          Eigen::Matrix<double,1,6> T2 = huber * J.transpose() * D;
          
          g0 = g0 + T2(0); g1 = g1 + T2(1); g2 = g2 + T2(2);
          g3 = g3 + T2(3); g4 = g4 + T2(4); g5 = g5 + T2(5);
          
          A00+=T1(0,0);A01+=T1(0,1);A02+=T1(0,2);A03+=T1(0,3);A04+=T1(0,4);A05+=T1(0,5);
	        A10+=T1(1,0);A11+=T1(1,1);A12+=T1(1,2);A13+=T1(1,3);A14+=T1(1,4);A15+=T1(1,5);
          A20+=T1(2,0);A21+=T1(2,1);A22+=T1(2,2);A23+=T1(2,3);A24+=T1(2,4);A25+=T1(2,5);
          A30+=T1(3,0);A31+=T1(3,1);A32+=T1(3,2);A33+=T1(3,3);A34+=T1(3,4);A35+=T1(3,5);
          A40+=T1(4,0);A41+=T1(4,1);A42+=T1(4,2);A43+=T1(4,3);A44+=T1(4,4);A45+=T1(4,5);
          A50+=T1(5,0);A51+=T1(5,1);A52+=T1(5,2);A53+=T1(5,3);A54+=T1(5,4);A55+=T1(5,5);

        }//col
      }//row
  
      Eigen::Matrix<double,6,6> A;

      A<< A00,A01,A02,A03,A04,A05,
          A10,A11,A12,A13,A14,A15,
          A20,A21,A22,A23,A24,A25,
          A30,A31,A32,A33,A34,A35,
          A40,A41,A42,A43,A44,A45,
          A50,A51,A52,A53,A54,A55;
     double scaling = 1/A.maxCoeff();
      
      Vector6d g;
      g<< g0, g1, g2, g3, g4, g5;
      
      g *= scaling;
      A *= scaling;
      
      A = A + (parameters_.regularization)*Eigen::MatrixXd::Identity(6,6);
      xi = xi - A.ldlt().solve(g);
      Vector6d Change = xi-xi_prev;  
      double Cnorm = Change.norm();
      xi_prev = xi;
      if(Cnorm < parameters_.min_parameter_update) break;
    }//k
  }//level
  if(std::isnan(xi.sum())) xi << 0.0,0.0,0.0,0.0,0.0,0.0;
  return xi;
};//function

void 
SDFTrackerRef::Render(void)
{
  double minStep = parameters_.resolution/4;
  cv::Mat depthImage_out(parameters_.image_height,parameters_.image_width,CV_32FC1);
  cv::Mat preview(parameters_.image_height,parameters_.image_width,CV_8UC3);
  
  uint textured_pixels=0;
  uint total_pixels=0;

  const Eigen::Matrix4d expmap = Transformation_;
  const Eigen::Vector4d camera = expmap * Eigen::Vector4d(0.0,0.0,0.0,1.0);
  const Eigen::Vector4d viewAxis = (expmap * Eigen::Vector4d(0.0,0.0,1.0,0.0)).normalized();
  const double max_ray_length = 5.0;
  
  //Rendering loop
 #pragma omp parallel for 
  for(int u = 0; u < parameters_.image_height; ++u)
  {
    for(int v = 0; v < parameters_.image_width; ++v)
    {
      bool hit = false;

      Eigen::Vector4d p = expmap*To3D(u,v,1.0,parameters_.fx,parameters_.fy,parameters_.cx,parameters_.cy) - camera;
      p.normalize();
            
      double scaling = validityMask_[u][v] ? double(depthImage_->ptr<float>(u)[v])*0.80 : parameters_.Dmax;
      
      double scaling_prev=0;
      int steps=0;
      double D = parameters_.resolution;
      while(steps<parameters_.raycast_steps && scaling < max_ray_length && !hit)
      { 

        double D_prev = D;
        D = SDF(camera + p*scaling);
        
        if(D < 0.0)
        {
          scaling = scaling_prev + (scaling-scaling_prev)*D_prev/(D_prev - D);
          hit = true;
          ++total_pixels;
          Eigen::Vector4d currentPoint = camera + p*scaling;

          Eigen::Vector4d normal_vector = Eigen::Vector4d::Zero();
          if(parameters_.interactive_mode && parameters_.use_texture)
          {
            
            for(int ii=0; ii<3; ++ii)
            {
              normal_vector(ii) = fabs(SDFGradient(currentPoint,1,ii));            
            }
            normal_vector.normalize();

            preview.at<cv::Vec3b>(u,v)[0]=normal_vector(0)*255;
            preview.at<cv::Vec3b>(u,v)[1]=normal_vector(1)*255;
            preview.at<cv::Vec3b>(u,v)[2]=normal_vector(2)*255;

            double i,j,k;
            //double x,y,z;
            /*x = */modf(currentPoint(0)/parameters_.resolution + parameters_.XSize/2, &i);
            /*y = */modf(currentPoint(1)/parameters_.resolution + parameters_.YSize/2, &j);  
            /*z = */modf(currentPoint(2)/parameters_.resolution + parameters_.ZSize/2, &k);
            
            int I = int(i);
            int J = int(j);
            int K = int(k);

            if(I>=0 && I<parameters_.XSize && J>=0 && J<parameters_.YSize && K>=0 && K<parameters_.ZSize)
            {   
              Texture* tx = textureReference_[I][J][K];
              if(tx!=NULL)
              {
                //Position of the current texture in world-frame
                //Eigen::Vector4d texture_position = tx->transformation*Eigen::Vector4d(0.0,0.0,0.0,1.0);         
                //View direction from which the texture was captured 
                Eigen::Vector4d color_ray = (tx->transformation.inverse())*(currentPoint);

                cv::Point2d uv = To2D(color_ray,parameters_.fx,parameters_.fy,parameters_.cx,parameters_.cy);
                int pxx = floor(uv.x);
                int pxy = floor(uv.y);

                if(pxx>=0 && pxx<parameters_.image_width-0 && pxy>=0 && pxy<parameters_.image_height-0)
                {
                  if(true)//(currentPoint - texture_position).normalized().dot(normal_vector) > 0.0 )
                  {                      
                    preview.at<cv::Vec3b>(u,v)[0] = tx->image.at<cv::Vec3b>(pxy,pxx)[2];//uchar(floorf(B));
                    preview.at<cv::Vec3b>(u,v)[1] = tx->image.at<cv::Vec3b>(pxy,pxx)[1];//uchar(floorf(G));
                    preview.at<cv::Vec3b>(u,v)[2] = tx->image.at<cv::Vec3b>(pxy,pxx)[0];//uchar(floorf(R));
                    ++textured_pixels;  
                  }
                }
              }
            }
          }
          else if(parameters_.interactive_mode)
          {  
            for(int ii=0; ii<3; ++ii)
            {
              normal_vector(ii) = fabs(SDFGradient(camera + p*scaling,1,ii));            
            }   
            normal_vector.normalize();

            preview.at<cv::Vec3b>(u,v)[0]=normal_vector(0)*255;
            preview.at<cv::Vec3b>(u,v)[1]=normal_vector(1)*255;
            preview.at<cv::Vec3b>(u,v)[2]=normal_vector(2)*255;
          }
          
          depthImage_out.at<float>(u,v)=scaling*(viewAxis.dot(p));
          break;
        }

        scaling_prev = scaling;
        scaling += std::max(parameters_.resolution, D);//abs(D-parameters_.Dmax) < 1e-12 ? D : D*D;  
        ++steps;        
       }//ray
      if(!hit)     
      {
        //Input values are better than nothing.
        depthImage_out.at<float>(u,v)=depthImage_->ptr<float>(u)[v];  
  
        if(parameters_.interactive_mode)
        {
          preview.at<cv::Vec3b>(u,v)[0]=uchar(30);
          preview.at<cv::Vec3b>(u,v)[1]=uchar(30);
          preview.at<cv::Vec3b>(u,v)[2]=uchar(30);
        }
      }//no hit
    }//col
  }//row
  float ratio = float(textured_pixels)/float(total_pixels);
  if(ratio < 0.70f) refresh_textures_ = true;

  if(parameters_.makeIP)
  {
    for (uint i = 0; i < interest_points_.size(); ++i)
    {
      cv::Point2d uv = To2D(expmap.inverse()*interest_points_[i],parameters_.fx,parameters_.fy,parameters_.cx,parameters_.cy);
      
      int pxx = floor(uv.x);
      int pxy = floor(uv.y);
      if(pxx<0+1 || pxx>=parameters_.image_width-1 || pxy<0+1 || pxy>parameters_.image_height-1) continue;

      preview.at<cv::Vec3b>(pxy,pxx)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy,pxx)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy,pxx)[2]=uchar(0);

      preview.at<cv::Vec3b>(pxy,pxx+1)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy,pxx+1)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy,pxx+1)[2]=uchar(0);

      preview.at<cv::Vec3b>(pxy,pxx-1)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy,pxx-1)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy,pxx-1)[2]=uchar(0);

      preview.at<cv::Vec3b>(pxy+1,pxx+1)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy+1,pxx+1)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy+1,pxx+1)[2]=uchar(0);

      preview.at<cv::Vec3b>(pxy-1,pxx+1)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy-1,pxx+1)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy-1,pxx+1)[2]=uchar(0);

      preview.at<cv::Vec3b>(pxy+1,pxx-1)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy+1,pxx-1)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy+1,pxx-1)[2]=uchar(0);

      preview.at<cv::Vec3b>(pxy-1,pxx-1)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy-1,pxx-1)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy-1,pxx-1)[2]=uchar(0);

      preview.at<cv::Vec3b>(pxy+1,pxx)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy+1,pxx)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy+1,pxx)[2]=uchar(0);

      preview.at<cv::Vec3b>(pxy-1,pxx)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy-1,pxx)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy-1,pxx)[2]=uchar(0);


    }
  }

  depthDenoised_mutex_.lock();
  depthImage_out.copyTo(*depthImage_denoised_);
  depthDenoised_mutex_.unlock();    
  if(parameters_.interactive_mode)
  {
  
    // struct timespec stamp;
    // clock_gettime(CLOCK_MONOTONIC, &stamp);
    
    
    // std::stringstream filenameImage;//, filenameRenderCloud, filenameDepthMapCloud;
    // filenameImage << "/home/danielcanelhas/Desktop/frame" << stamp.tv_sec << ".png";
    // cv::imwrite(filenameImage.str(),preview);

    cv::imshow("Render", preview);//depthImage_denoised);
    char q = cv::waitKey(3);
    if(q == 'q' || q  == 27 || q  == 71 ) { quit_ = true; }//int(key)
  }
  return;
};

void 
SDFTrackerRef::Render(const Eigen::Matrix4d &custom_transformation)
{
  double minStep = parameters_.resolution/16;
  cv::Mat depthImage_out(parameters_.image_height,parameters_.image_width,CV_32FC1);
  cv::Mat preview(parameters_.image_height,parameters_.image_width,CV_8UC3);
  
  uint textured_pixels=0;
  uint total_pixels=0;

  const Eigen::Matrix4d expmap = custom_transformation;
  const Eigen::Vector4d camera = expmap * Eigen::Vector4d(0.0,0.0,0.0,1.0);
  const Eigen::Vector4d viewAxis = (expmap * Eigen::Vector4d(0.0,0.0,1.0,0.0)).normalized();
  const double max_ray_length = 5.0;
  
  //Rendering loop
 #pragma omp parallel for 
  for(int u = 0; u < parameters_.image_height; ++u)
  {
    for(int v = 0; v < parameters_.image_width; ++v)
    {
      bool hit = false;

      Eigen::Vector4d p = expmap*To3D(u,v,1.0,parameters_.fx,parameters_.fy,parameters_.cx,parameters_.cy) - camera;
      p.normalize();
            
      double scaling = validityMask_[u][v] ? double(depthImage_->ptr<float>(u)[v])*0.0 : parameters_.Dmax;
      
      double scaling_prev=0;
      int steps=0;
      double D = parameters_.resolution;
      while(steps<parameters_.raycast_steps*2 && scaling < max_ray_length && !hit)
      { 

        double D_prev = D;
        D = SDF(camera + p*scaling);
        
        if(D < 0.0)
        {
          scaling = scaling_prev + (scaling-scaling_prev)*D_prev/(D_prev - D);
          hit = true;
          ++total_pixels;
          Eigen::Vector4d currentPoint = camera + p*scaling;

          Eigen::Vector4d normal_vector = Eigen::Vector4d::Zero();
          if(parameters_.interactive_mode && parameters_.use_texture)
          {
            
            for(int ii=0; ii<3; ++ii)
            {
              normal_vector(ii) = fabs(SDFGradient(currentPoint,1,ii));            
            }
            normal_vector.normalize();

            preview.at<cv::Vec3b>(u,v)[0]=normal_vector(0)*255;
            preview.at<cv::Vec3b>(u,v)[1]=normal_vector(1)*255;
            preview.at<cv::Vec3b>(u,v)[2]=normal_vector(2)*255;

            double i,j,k;
            //double x,y,z;
            /*x = */modf(currentPoint(0)/parameters_.resolution + parameters_.XSize/2, &i);
            /*y = */modf(currentPoint(1)/parameters_.resolution + parameters_.YSize/2, &j);  
            /*z = */modf(currentPoint(2)/parameters_.resolution + parameters_.ZSize/2, &k);
            
            int I = int(i);
            int J = int(j);
            int K = int(k);

            if(I>=0 && I<parameters_.XSize && J>=0 && J<parameters_.YSize && K>=0 && K<parameters_.ZSize)
            {   
              Texture* tx = textureReference_[I][J][K];
              if(tx!=NULL)
              {
                //Position of the current texture in world-frame
                //Eigen::Vector4d texture_position = tx->transformation*Eigen::Vector4d(0.0,0.0,0.0,1.0);         
                //View direction from which the texture was captured 
                Eigen::Vector4d color_ray = (tx->transformation.inverse())*(currentPoint);

                cv::Point2d uv = To2D(color_ray,parameters_.fx,parameters_.fy,parameters_.cx,parameters_.cy);
                int pxx = floor(uv.x);
                int pxy = floor(uv.y);

                if(pxx>=0 && pxx<parameters_.image_width-0 && pxy>=0 && pxy<parameters_.image_height-0)
                {
                  if(true)//(currentPoint - texture_position).normalized().dot(normal_vector) > 0.0 )
                  {                      
                    preview.at<cv::Vec3b>(u,v)[0] = tx->image.at<cv::Vec3b>(pxy,pxx)[2];//uchar(floorf(B));
                    preview.at<cv::Vec3b>(u,v)[1] = tx->image.at<cv::Vec3b>(pxy,pxx)[1];//uchar(floorf(G));
                    preview.at<cv::Vec3b>(u,v)[2] = tx->image.at<cv::Vec3b>(pxy,pxx)[0];//uchar(floorf(R));
                    ++textured_pixels;  
                  }
                }
              }
            }
          }
          else if(parameters_.interactive_mode)
          {  
            for(int ii=0; ii<3; ++ii)
            {
              normal_vector(ii) = fabs(SDFGradient(camera + p*scaling,1,ii));            
            }   
            normal_vector.normalize();

            preview.at<cv::Vec3b>(u,v)[0]=normal_vector(0)*255;
            preview.at<cv::Vec3b>(u,v)[1]=normal_vector(1)*255;
            preview.at<cv::Vec3b>(u,v)[2]=normal_vector(2)*255;
          }
          
          depthImage_out.at<float>(u,v)=scaling*(viewAxis.dot(p));
          break;
        }
        scaling_prev = scaling;
        scaling += std::max(parameters_.resolution, D);//abs(D-parameters_.Dmax) < 1e-12 ? D : D*D;  
        ++steps;        
      }//ray
      if(!hit)     
      {

        depthImage_out.at<float>(u,v)=std::numeric_limits<float>::quiet_NaN( );//depthImage_->ptr<float>(u)[v];  
  
        if(parameters_.interactive_mode)
        {
          preview.at<cv::Vec3b>(u,v)[0]=uchar(30);
          preview.at<cv::Vec3b>(u,v)[1]=uchar(30);
          preview.at<cv::Vec3b>(u,v)[2]=uchar(30);
        }
      }//no hit
    }//col
  }//row
  float ratio = float(textured_pixels)/float(total_pixels);
  if(ratio < 0.70f) refresh_textures_ = true;

  if(parameters_.makeIP)
  {
    for (uint i = 0; i < interest_points_.size(); ++i)
    {
      cv::Point2d uv = To2D(expmap.inverse()*interest_points_[i],parameters_.fx,parameters_.fy,parameters_.cx,parameters_.cy);
      
      int pxx = floor(uv.x);
      int pxy = floor(uv.y);
      if(pxx<0+1 || pxx>=parameters_.image_width-1 || pxy<0+1 || pxy>parameters_.image_height-1) continue;

      preview.at<cv::Vec3b>(pxy,pxx)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy,pxx)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy,pxx)[2]=uchar(0);

      preview.at<cv::Vec3b>(pxy,pxx+1)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy,pxx+1)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy,pxx+1)[2]=uchar(0);

      preview.at<cv::Vec3b>(pxy,pxx-1)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy,pxx-1)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy,pxx-1)[2]=uchar(0);

      preview.at<cv::Vec3b>(pxy+1,pxx+1)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy+1,pxx+1)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy+1,pxx+1)[2]=uchar(0);

      preview.at<cv::Vec3b>(pxy-1,pxx+1)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy-1,pxx+1)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy-1,pxx+1)[2]=uchar(0);

      preview.at<cv::Vec3b>(pxy+1,pxx-1)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy+1,pxx-1)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy+1,pxx-1)[2]=uchar(0);

      preview.at<cv::Vec3b>(pxy-1,pxx-1)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy-1,pxx-1)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy-1,pxx-1)[2]=uchar(0);

      preview.at<cv::Vec3b>(pxy+1,pxx)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy+1,pxx)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy+1,pxx)[2]=uchar(0);

      preview.at<cv::Vec3b>(pxy-1,pxx)[0]=uchar(0);
      preview.at<cv::Vec3b>(pxy-1,pxx)[1]=uchar(255);
      preview.at<cv::Vec3b>(pxy-1,pxx)[2]=uchar(0);


    }
  }

  depthDenoised_mutex_.lock();
  depthImage_out.copyTo(*depthImage_denoised_);
  depthDenoised_mutex_.unlock();    
  if(parameters_.interactive_mode)
  {
  
    // struct timespec stamp;
    // clock_gettime(CLOCK_MONOTONIC, &stamp);
    
    
    // std::stringstream filenameImage;//, filenameRenderCloud, filenameDepthMapCloud;
    // filenameImage << "/home/danielcanelhas/Desktop/frame" << stamp.tv_sec << ".png";
    // cv::imwrite(filenameImage.str(),preview);

    cv::imshow("Render", preview);//depthImage_denoised);
    char q = cv::waitKey(3);
    if(q == 'q' || q  == 27 || q  == 71 ) { quit_ = true; }//int(key)
  }
  return;
};




void SDFTrackerRef::realOnMouse( int event, int x, int y, int )
{
  if (!parameters_.makeIP) return;
  //if the event that called this function was anything but a
  //left button click, return.
  if(event!=CV_EVENT_LBUTTONDOWN) return;
  //pixel range check
  if(x<0 || x>=parameters_.image_width || y<0 || y>parameters_.image_height) return;

  int u = y;
  int v = x;
  double minStep = parameters_.resolution/4;
  
  //some transformations to get to the camera coordinate system
  const Eigen::Matrix4d expmap = Transformation_;
  const Eigen::Vector4d camera = expmap * Eigen::Vector4d(0.0,0.0,0.0,1.0);

  Eigen::Vector4d p = expmap*To3D(u,v,1.0,parameters_.fx,parameters_.fy,parameters_.cx,parameters_.cy) - camera;
  p.normalize();
        
  double scaling = parameters_.Dmax;
  double scaling_prev=0;
  int steps=0;
  double D = parameters_.resolution;

  //cast a single ray through the clicked-on pixel
  while(steps<parameters_.raycast_steps*2)
  { 
    double D_prev = D;
    D = SDF(camera + p*scaling);
 
    if(D < 0.0)
    {
      scaling = scaling_prev + (scaling-scaling_prev)*D_prev/(D_prev - D);
      
      Eigen::Vector4d currentPoint = camera + p*scaling;

      double i,j,k;  
      modf(currentPoint(0)/parameters_.resolution + parameters_.XSize/2, &i);
      modf(currentPoint(1)/parameters_.resolution + parameters_.YSize/2, &j);  
      modf(currentPoint(2)/parameters_.resolution + parameters_.ZSize/2, &k);
      int I = int(i);
      int J = int(j);
      int K = int(k);

      //If raycast terminates within the reconstructed volume, keep the surface point.
      if(I>=0 && I<parameters_.XSize && J>=0 && J<parameters_.YSize && K>=0 && K<parameters_.ZSize)
      {   
        interest_points_.push_back(currentPoint);   
        return;
      }
    }
    scaling_prev = scaling;
    scaling += std::max(parameters_.resolution, D);
    ++steps;        
  }//ray
  
  return;
};

void SDFTrackerRef::getDenoisedImage(cv::Mat &img) 
{
    depthDenoised_mutex_.lock();
    depthImage_denoised_->copyTo(img);
    depthDenoised_mutex_.unlock();          
}


bool SDFTrackerRef::quit(void)
{
  return quit_;
}

Eigen::Matrix4d SDFTrackerRef::getCurrentTransformation(void)
{
  Eigen::Matrix4d T;
  transformation_mutex_.lock();
  T = Transformation_;
  transformation_mutex_.unlock();
  return T;
}

void SDFTrackerRef::getInterestPoints(std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d> > &IP_vec)
{
  for (uint i = 0; i < interest_points_.size(); ++i)
  {
    IP_vec.push_back(interest_points_[i]);  
  }
}
