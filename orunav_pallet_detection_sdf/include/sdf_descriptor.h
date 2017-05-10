#ifndef SDF_DESCRIPTOR
#define SDF_DESCRIPTOR

#include <Eigen/StdVector>
#include <Eigen/Core>
#include <sdf_tracker_refbased.h>

#define SDF_FEATURE_RADIUS 0.03
#define SDF_FEATURE_SAMPLES 32

class SDFDescriptor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SDFDescriptor(Eigen::Vector4d*, SDFTrackerRef*);
  ~SDFDescriptor();
  
  Eigen::MatrixXd feature;
};
#endif