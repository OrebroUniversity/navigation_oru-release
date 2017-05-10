#ifndef TVL1DENOISE
#define TVL1DENOISE
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <Eigen/Sparse>
#include <Eigen/Core>
#include <Eigen/Eigen>

typedef Eigen::Triplet<double> trip;
namespace TVL1
{
  void make_nabla(Eigen::SparseMatrix<double> &nabla, uint M, uint N);
  void denoise(cv::Mat &img_src, double lambda, unsigned int max_iter);
}

#endif