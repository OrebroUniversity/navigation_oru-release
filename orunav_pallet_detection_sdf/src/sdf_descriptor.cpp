#include <sdf_descriptor.h>
#include <random>
#include <limits>
#include <vector>
#include <algorithm>
#include <Eigen/Core>

SDFDescriptor::SDFDescriptor(Eigen::Vector4d* interest_point, SDFTrackerRef* tracker) 
{ 
  if(tracker == NULL) return;  

  //SDF returns the upper truncation limits for nan input. (the upper limit is private in the tracker)
  double SDF_MaxVal = tracker->SDF(Eigen::Vector4d(DBL_MAX, DBL_MAX, DBL_MAX, 1)); 

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0,SDF_FEATURE_RADIUS);
  
  std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d> > samples;
  for (int sample = 0; sample < SDF_FEATURE_SAMPLES; ++sample)
  {
    double offset_x = distribution(generator);
    double offset_y = distribution(generator);
    double offset_z = distribution(generator);
    
    samples.push_back(*interest_point+Eigen::Vector4d(offset_x, offset_y, offset_z, 0.0));
  }

  double feat[6]={0.0,0.0,0.0,0.0,0.0,0.0};
  std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > gradients;

  //for every sample, compute point-based features and gradients
  for (std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d> >::iterator it = samples.begin(); it != samples.end(); ++it)
  {

    //compute the point-based features
    double dist = tracker->SDF(*it); 
    feat[0] += (dist > 0) ? dist : 0;   
    feat[1] += (dist < 0) ? dist : 0;
    feat[2] += fabs(dist - SDF_MaxVal) < 10e-5 ? 1 : 0;
    feat[3] += (dist <0 ) ? 1: 0;

    if(tracker->validGradientStrict(*it))
    {
      //and gradients
      Eigen::Vector3d gradient = Eigen::Vector3d(tracker->SDFGradient(*it, 1, 0),
                                                 tracker->SDFGradient(*it, 1, 1),
                                                 tracker->SDFGradient(*it, 1, 2));
      gradient.normalize();
      if(!std::isnan(gradient(0)+gradient(1)+gradient(2)))
      gradients.push_back(gradient);
    }
  }
  
  //compute the gradient-based features
  for (uint i = 0;  i<gradients.size(); ++i)
  {
    for (uint j = i+1; j<gradients.size(); ++j)
    {
      feat[4] += gradients[i].dot(gradients[j]);
      feat[5] +=  std::max(fabs(gradients[i](0)-gradients[j](0)), 
                  std::max(fabs(gradients[i](1)-gradients[j](1)),
                           fabs(gradients[i](2)-gradients[j](2))) );
    }
  }

  //assemble features into a vector
  feature = Eigen::MatrixXd(1,6);
  feature << feat[0],feat[1],feat[2],feat[3],feat[4],feat[5];
}

SDFDescriptor::~SDFDescriptor()
{ }