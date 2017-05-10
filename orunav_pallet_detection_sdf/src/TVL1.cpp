
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/Core>
#include <stdio.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "TVL1.h"

namespace TVL1
{

typedef Eigen::Triplet<double> trip;

void make_nabla(Eigen::SparseMatrix<double> &nabla, uint M, uint N)
{
  //  o----->x(M) 
  //  |
  //  |
  //  ¡ýy(N)      
  // 
  // Kxf{matrix for gradient_x}
  // 
  std::vector<trip> coefs;  
  
  for(uint y = 1; y<=M-1; ++y) 
  {
    for(uint x = 1; x<=N; ++x)
    {
      coefs.push_back(trip( y+(x-1)*M-1, y+(x-1)*M-1,-1));
      coefs.push_back(trip( y+(x-1)*M-1, y+(x-1)*M  , 1));
    }
  }
  uint offset = M*N;
  for(uint y = 1; y<=M; ++y) 
  {
    for(uint x = 1; x<=N-1; ++x)
    {   
      coefs.push_back(trip(y+(x-1)*M -1 + offset , y+(x-1)*M -1, -1));
      coefs.push_back(trip(y+(x-1)*M -1 + offset , y+(x)*M -1  , 1));          
    }
  }
  nabla.setFromTriplets(coefs.begin(), coefs.end());
}

//    image denoising based TV-L1 model via Algorithm 1 for optimization
//    [1] Antonin Chambolle and Thomas Pock, A first-order primal-dual
//    algorithm with applications to imaging, Technical Report, 2010
// 
//    License: BSD

void denoise(cv::Mat &img_src, double lambda, unsigned int max_iter)
{
  
  // cv::imshow("debug",img_src);
  // cv::waitKey();
  uint n_row = img_src.rows;
  uint n_col = img_src.cols;

  const long long unsigned int N  = n_row*n_col;
  
  Eigen::MatrixXd img_temp;
  cv::cv2eigen(img_src,img_temp);

  Eigen::MatrixXd img(N,1);

  for(unsigned int i = 0; i<n_row ; i++)
  {
    for(unsigned int j = 0; j<n_col ; j++)
    {
      img(i*n_col+j,0) = img_temp(i,j);
    }
  }
  
  Eigen::SparseMatrix<double> nabla(N*2,N); 
  make_nabla(nabla,img_src.cols,img_src.rows);
  
  Eigen::MatrixXd noise_image = img;
  Eigen::MatrixXd u = noise_image;
  Eigen::MatrixXd head_u = u;
  Eigen::MatrixXd temp_u = Eigen::MatrixXd::Zero(N,1);
  Eigen::MatrixXd old_u = Eigen::MatrixXd::Zero(N,1);
  Eigen::MatrixXd p = Eigen::MatrixXd::Zero(2*N,1); 
  Eigen::MatrixXd temp_p = Eigen::MatrixXd(2*N,1);
  Eigen::MatrixXd sqrt_p = Eigen::MatrixXd(N,1);
  Eigen::MatrixXd sqrt_p_ = Eigen::MatrixXd(2*N,1);

  double L = sqrt(8);
  double tau = 0.02;
  double sigma = (1/(L*L))/tau;
  double theta = 0.5;
  std::vector<double> energy[max_iter];
  Eigen::SparseMatrix<double> nabla_t = nabla.transpose();

  for(unsigned int n_processing = 0; n_processing < max_iter; ++n_processing)
  {   
    old_u   = u;
    //update dual
    temp_p  = nabla * head_u * sigma + p;

    for(long long unsigned int i = 0; i<N; ++i)
    {
        sqrt_p(i,0)  = sqrt(temp_p(i,0)*temp_p(i,0) + temp_p(N+i)*temp_p(N+i));
        sqrt_p_(i,0) = sqrt_p(i,0);
        sqrt_p_(i+N) = sqrt_p(i,0);
        p(i,  0) = temp_p(i,  0)/std::max(1.0,sqrt_p_(i,  0));
        p(i+N,0) = temp_p(i+N,0)/std::max(1.0,sqrt_p_(i+N,0));
    }
    // update primal
    temp_u = old_u - tau * nabla_t * p;
    for(long long unsigned int i = 0; i<N;++i)
    {
        double a = temp_u(i,0) - noise_image(i,0);
        double b = tau*lambda;
        
        if(a > b) u(i,0) = temp_u(i,0) - b; 
        else if(a < -b) u(i,0) = temp_u(i,0) + b;
        else u(i,0) = noise_image(i,0);

    }
    // calculate head_u
    head_u  = u + theta * (u - old_u);
    temp_p  = nabla * u;

    // compute energy
    double e1 = 0.0;
    double e2 = 0.0;
    for (long long unsigned int i = 0; i < N; ++i)
    {
      e1 += sqrt(temp_p(i,0)*temp_p(i,0) + temp_p(N+i,0)*temp_p(N+i,0));
      e2 += fabs(u(i,0) - noise_image(i,0));
    }
    e1 = e1 + lambda*e2;

  //printf(">>>>>: it = %4d , energy = %.5f\n",n_processing, e1); 
  }

  for(unsigned int i = 0; i<n_row ; i++)
  {
    for(unsigned int j = 0; j<n_col ; j++)
    {
      img_temp(i,j) = u(i*n_col+j,0);
    }
  }

  // cv::eigen2cv(img_temp,img_src);
  // cv::imshow("debug",img_src);
  // cv::waitKey();  
  return;
}
}//namespace

// int main(int argc, char **argv)
// {
//   cv::Mat img_c;
//   img_c = cv::imread(argv[1], CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR ); // Read the file
  
//   cv::Mat img_f;
//   img_c.convertTo(img_f, CV_64F);
//   img_f/=255;
//   denoiseTVL1(img_f, 0.60, 30);
//   return 0;
// }

