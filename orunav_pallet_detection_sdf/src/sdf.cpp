#include "sdf.h"
#include <iostream>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//#define SDF_DRAW_GRAD_DEBUG

namespace sdf
{



  Eigen::Vector3d 
  makeRay(int row, int column, float depth, float fx, float fy, float cx, float cy)
  {

    Eigen::Vector3d ret(double(column-cx)*depth/(fx),
                        double(row-cy)*depth/(fy),
                        double(depth));
    return ret;
  };

  double 
  SDF(const Eigen::Vector3d &location, std::vector<Primitive*> &primitives)
  {
    double d_value = 0.0;

    for(std::vector<Primitive*>::iterator it = primitives.begin(); it != primitives.end(); ++it)
    {
      Eigen::Vector3d location_warped = it[0]->transformation.inverse()*location;
      if( it == primitives.begin() ) d_value = it[0]->signedDistance(location_warped);
      else
      {
        switch(it[0]->operation)
        {
          case bool_union :
            d_value = std::min( it[0]->signedDistance(location_warped), d_value);
          
            break;
          case bool_intersection :
            d_value = std::max( it[0]->signedDistance(location_warped), d_value);
            break;
          
          case bool_subtraction :
            d_value = std::max( -it[0]->signedDistance(location_warped), d_value);
            break;
        } 
      }
    }
    return d_value;
  }
 
  Eigen::Vector3d 
  SDFGradientPosNorm(const Eigen::Vector3d &location,  std::vector<Primitive*> &primitives)
  {
    double delta=1e-05;
    Eigen::Vector3d v = Eigen::Vector3d(
           fabs(SDF(location + Eigen::Vector3d(delta,0,0), primitives) - SDF(location - Eigen::Vector3d(delta,0,0), primitives)),
           fabs(SDF(location + Eigen::Vector3d(0,delta,0), primitives) - SDF(location - Eigen::Vector3d(0,delta,0), primitives)),
           fabs(SDF(location + Eigen::Vector3d(0,0,delta), primitives) - SDF(location - Eigen::Vector3d(0,0,delta), primitives))/(2.0*delta));
    return v.normalized();
  } 
 
  Eigen::Vector3d 
  SDFGradient(const Eigen::Vector3d &location,  std::vector<Primitive*> &primitives)
  {
    double delta=1e-05;
    return Eigen::Vector3d(
           SDF(location + Eigen::Vector3d(delta,0,0), primitives) - SDF(location - Eigen::Vector3d(delta,0,0), primitives),
           SDF(location + Eigen::Vector3d(0,delta,0), primitives) - SDF(location - Eigen::Vector3d(0,delta,0), primitives),
           SDF(location + Eigen::Vector3d(0,0,delta), primitives) - SDF(location - Eigen::Vector3d(0,0,delta), primitives))/(2.0*delta);
  }

Eigen::Matrix4d 
Twist(const Eigen::Vector6d &xi)
{
  Eigen::Matrix4d M;
  
  M << 0.0  , -xi(2),  xi(1), xi(3),
       xi(2), 0.0   , -xi(0), xi(4),
      -xi(1), xi(0) , 0.0   , xi(5),
       0.0,   0.0   , 0.0   , 0.0  ;
  
  return M;
};

Eigen::Affine3d
ComputeAffine3d(const Eigen::Vector6d &xi) {
    Eigen::Affine3d t = Eigen::Affine3d::Identity();
    Eigen::Vector3d pos_(xi(3), xi(4), xi(5));
    Eigen::Vector3d rot_(xi(0), xi(1), xi(2));
    t.translation() = pos_;
    t.linear() = (Eigen::AngleAxisd(rot_(2), Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(rot_(1), Eigen::Vector3d::UnitY()) * 
                  Eigen::AngleAxisd(rot_(0), Eigen::Vector3d::UnitX()) ).toRotationMatrix();
    return t;
}

std::pair<double,int> AlignmentPC(const Eigen::Affine3d &transformation, 
                                  const int rows, 
                                  const int cols, 
                                  Eigen::Vector4f &cam_params, 
                                  const int max_steps, 
                                  const float max_ray_length, 
                                  const float precision, 
                                  std::vector<Primitive*> &primitives,
                                  const pcl::PointCloud<pcl::PointXYZ> &depth)
{
    double S = 0.;
    int N = 0;

    //Rendering loop
#pragma omp parallel for                      \
    default(shared)                           \
    reduction(+:S,N)
    for(int u = 0; u < rows; ++u)
    {
        for(int v = 0; v < cols; ++v)
        {
            const pcl::PointXYZ &pt = depth[u*cols+v];
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
                continue;

            Eigen::Vector3d q = transformation * Eigen::Vector3d(pt.x, pt.y, pt.z);
            
            double D = SDF(q, primitives);
            
            if(D < precision && D > /*0*/ -precision) // !!!!!!!!!!!!!
            {
                N++;
                S+=D;
            }
            if (D < precision)
            {
                // Indicates negative values we're seeing through the model -> very bad bad bad...
                S+=(fabs(D)*10);
            }

        }//col
    }//row 
    return std::pair<double, int>(S,N);
}

Eigen::Affine3d AlignmentSearchPC(const Eigen::Affine3d &transformation, 
                                  const int rows, 
                                  const int cols, 
                                  Eigen::Vector4f &cam_params, 
                                  const int max_steps, 
                                  const float max_ray_length, 
                                  const float precision, 
                                  std::vector<Primitive*> &primitives,
                                  const pcl::PointCloud<pcl::PointXYZ> &depth)

{
    // Try some locations...
    Eigen::Affine3d best_Tcam;
    double best_val = std::numeric_limits<int>::max();
    best_Tcam = transformation;

    for (int i = 1; i < 10; i++) {
        Eigen::Affine3d Tcam = Eigen::Translation3d(Eigen::Vector3d(0.2*i, 0, 0)) * transformation;

        std::pair<double,int> res = AlignmentPC(Tcam,
                                                rows,
                                                cols, 
                                                cam_params,
                                                max_steps,
                                                max_ray_length,
                                                precision,
                                                primitives,
                                                depth);

        double val = res.first / (res.second*1.);
        val = val/(res.second*1.);
        //        std::cout << "res.first : " << res.first << " res.second : " << res.second << " val[" << 0.2*i << "] : " << val << std::endl;
        if (val < best_val) {
            best_val = val;
            best_Tcam = Tcam;
            //            std::cout << "best_val :" << val << std::endl;
        }
        
    }
    return best_Tcam;
   
}

void RenderDiffPC(const Eigen::Affine3d &transformation, 
                  const int rows, 
                  const int cols, 
                  Eigen::Vector4f &cam_params, 
                  const int max_steps, 
                  const float max_ray_length, 
                  const float precision, 
                  std::vector<Primitive*> &primitives,
                  const pcl::PointCloud<pcl::PointXYZ> &depth)
{
    
    cv::Mat preview( rows, cols, CV_8UC3);
    
//Rendering loop
   #pragma omp parallel for 
    for(int u = 0; u < rows; ++u)
    {
      for(int v = 0; v < cols; ++v)
      {
        bool hit = false;
                
         double depth_at_uv = depth[u*cols+v].z;
        if (std::isnan(depth_at_uv))
            depth_at_uv = -1.;
        
        if (depth_at_uv < 0) {
            preview.at<cv::Vec3b>(u,v)[0]=uchar(200);
            preview.at<cv::Vec3b>(u,v)[1]=uchar(30);
            preview.at<cv::Vec3b>(u,v)[2]=uchar(30);
            continue;
        }
        const pcl::PointXYZ &pt = depth[u*cols+v];
        Eigen::Vector3d q = transformation * Eigen::Vector3d(pt.x, pt.y, pt.z);
      
        double D = SDF(q, primitives);
        D = fabs(D); // !!!!!!!!!
        if(D < precision)
        {
            hit = true;
            if (D < 0) {
                preview.at<cv::Vec3b>(u,v)[0]=uchar(30);
                preview.at<cv::Vec3b>(u,v)[1]=uchar(200);
                preview.at<cv::Vec3b>(u,v)[2]=uchar(200);
            } else {
                preview.at<cv::Vec3b>(u,v)[0]= uchar(D*256/precision);
                preview.at<cv::Vec3b>(u,v)[1]= uchar(D*256/precision);
                preview.at<cv::Vec3b>(u,v)[2]= uchar(D*256/precision);
            }

        }
        if(!hit)     
        {
            preview.at<cv::Vec3b>(u,v)[0]=uchar(30);
            preview.at<cv::Vec3b>(u,v)[1]=uchar(200);
            preview.at<cv::Vec3b>(u,v)[2]=uchar(30);
        }//no hit
      }//col
    }//row 

    cv::imshow("RenderDiffPC", preview);
    char q = cv::waitKey(3);
    if(q == 'q' || q  == 27 || q  == 71 ) { exit(0); }
    
    return;
}
 
 void RenderPC(
    const Eigen::Affine3d &transformation, 
    const int rows, 
    const int cols, 
    Eigen::Vector4f &cam_params, 
    const int max_steps, 
    const float max_ray_length, 
    const float precision, 
    std::vector<Primitive*> &primitives,
    const pcl::PointCloud<pcl::PointXYZ> &depth)
 {
    
    cv::Mat preview( rows, cols, CV_8UC3);
            
    //Rendering loop
   #pragma omp parallel for 
    for(int u = 0; u < rows; ++u)
    {
      for(int v = 0; v < cols; ++v)
      {
          double depth_at_uv = depth[u*cols+v].z;
        if (std::isnan(depth_at_uv))
            depth_at_uv = -1.;
        
        if (depth_at_uv < 0) {
            preview.at<cv::Vec3b>(u,v)[0]=uchar(200);
            preview.at<cv::Vec3b>(u,v)[1]=uchar(30);
            preview.at<cv::Vec3b>(u,v)[2]=uchar(30);
            continue;
        }
        

        preview.at<cv::Vec3b>(u,v)[0] = 50*depth_at_uv;
        preview.at<cv::Vec3b>(u,v)[1] = 50*depth_at_uv;
        preview.at<cv::Vec3b>(u,v)[2] = 50*depth_at_uv;
      }//col
    }//row 
   
    cv::imshow("RenderPC", preview);
    char q = cv::waitKey(3);
    if(q == 'q' || q  == 27 || q  == 71 ) { exit(0); }

    return;
  }

 void Render(
    const Eigen::Affine3d &transformation, 
    const int rows, 
    const int cols, 
    Eigen::Vector4f &cam_params, 
    const int max_steps, 
    const float max_ray_length, 
    const float precision, 
    std::vector<Primitive*> &primitives)
  {
    
    cv::Mat preview( rows, cols, CV_8UC3);
    const Eigen::Vector3d camera = transformation * Eigen::Vector3d(0.0,0.0,0.0);
   
    //Rendering loop
   #pragma omp parallel for 
    for(int u = 0; u < rows; ++u)
    {
      for(int v = 0; v < cols; ++v)
      {
        bool hit = false;
                
        // double depth_at_uv = std::isnan(depth.at<float>(u,v)) ? 1.0 : depth.at<float>(u,v);
        // Eigen::Vector3d q = transformation.inverse()*makeRay(u, v, depth_at_uv, cam_params(0), cam_params(1), cam_params(2), cam_params(3));
        // double misalign = SDF(q, primitives);

        Eigen::Vector3d p = transformation*makeRay(u, v, 1.0, cam_params(0), cam_params(1), cam_params(2), cam_params(3)) - camera;

        p.normalize();
              
        double scaling = 0.4;
        
        double scaling_prev=0;
        int steps=0;
        double D = 0.5;
        while(steps<max_steps && scaling < max_ray_length && !hit)
        { 

          double D_prev = D;
          D = SDF(camera + p*scaling, primitives);
          if (u == 240 && v == 320) {
              // std::cout << "=========================================================" << std::endl;
              // std::cout << "SDF value (should be very low) : " << D << std::endl;
              // std::cout << "SDF model, distance : " << scaling << std::endl;
              // std::cout << "Evaluation point (q) : " << camera + p*scaling << std::endl;
              // std::cout << "=========================================================" << std::endl;
          }
          if(D < precision)
          {
                  
              
            scaling = scaling_prev + (scaling-scaling_prev)*D_prev/(D_prev - (D - precision));

            hit = true;
            Eigen::Vector3d normal_vector = SDFGradient(camera + p*scaling,primitives);            
            normal_vector.normalize();

            preview.at<cv::Vec3b>(u,v)[1]=128-rint(normal_vector(0)*127);
            preview.at<cv::Vec3b>(u,v)[2]=128-rint(normal_vector(1)*127);
            preview.at<cv::Vec3b>(u,v)[0]=128-rint(normal_vector(2)*127);

            break;
          }
          scaling_prev = scaling;
          scaling += D;  
          ++steps;        
        }//ray
        if(!hit)     
        {
          preview.at<cv::Vec3b>(u,v)[0]=uchar(30);
          preview.at<cv::Vec3b>(u,v)[1]=uchar(30);
          preview.at<cv::Vec3b>(u,v)[2]=uchar(30);
        
        }//no hit
      }//col
    }//row 
   
    cv::imshow("Render", preview);
    char q = cv::waitKey(3);
    if(q == 'q' || q  == 27 || q  == 71 ) { exit(0); }

    return;
  }


Eigen::Affine3d
EstimatePalletPosePC(
    const Eigen::Affine3d &transformation,
    const int rows, 
    const int cols, 
    Eigen::Vector4f &cam_params, 
    const int max_steps, 
    const float max_ray_length, 
    const float precision, 
    std::vector<Primitive*> &primitives,
    const pcl::PointCloud<pcl::PointXYZ> &depth)
{
   // Incremental offset relative to the provided transformation.
    Eigen::Vector6d xi;
    xi<<0.0,0.0,0.0,0.0,0.0,0.0;
    Eigen::Vector6d xi_prev = xi;
    //const double eps = 1.0e-9;
    double robust_statistic_coefficient = 0.02;
    double   Dmax = 0.1;
    const double c = robust_statistic_coefficient*Dmax; // TODO.
    
//    const int iterations[3]={12, 8, 2};
//    const int stepSize[3] = {4, 2, 1};
    const int iterations[3]={50, 10, 2};
    const int stepSize[3] = {8, 4, 2};

    
    int iter = 0;
    for(int lvl=0; lvl < 3; ++lvl)
    {
        for(int k=0; k<iterations[lvl]; ++k)
        {
#ifdef SDF_DRAW_GRAD_DEBUG
            cv::Mat preview( rows, cols, CV_8UC3);
            preview.setTo(cv::Scalar(0., 0., 0.));
#endif
            double A00=0.0,A01=0.0,A02=0.0,A03=0.0,A04=0.0,A05=0.0;
            double A10=0.0,A11=0.0,A12=0.0,A13=0.0,A14=0.0,A15=0.0;
            double A20=0.0,A21=0.0,A22=0.0,A23=0.0,A24=0.0,A25=0.0;
            double A30=0.0,A31=0.0,A32=0.0,A33=0.0,A34=0.0,A35=0.0;
            double A40=0.0,A41=0.0,A42=0.0,A43=0.0,A44=0.0,A45=0.0;
            double A50=0.0,A51=0.0,A52=0.0,A53=0.0,A54=0.0,A55=0.0;
            
            double g0=0.0, g1=0.0, g2=0.0, g3=0.0, g4=0.0, g5=0.0;
    
//            double Sabs=0.0;
//            double S=0.0;
            for(int u=0; u<rows; u+=stepSize[lvl])
            {          
//#pragma omp parallel for                                            \
//    default(shared)                                                   \
//    reduction(+:g0,g1,g2,g3,g4,g5,A00,A01,A02,A03,A04,A05,A10,A11,A12,A13,A14,A15,A20,A21,A22,A23,A24,A25,A30,A31,A32,A33,A34,A35,A40,A41,A42,A43,A44,A45,A50,A51,A52,A53,A54,A55)
                for(int v=0; v<cols-0; v+=stepSize[lvl])
                {
//--          if(!validityMask_[row][col]) continue;  --> skip this for now.
// Code from rendering....
                    

                    const Eigen::Affine3d transformation_ = ComputeAffine3d(xi)*transformation;
                    
                    double depth_at_uv = depth[u*cols+v].z;

                    if (std::isnan(depth_at_uv))
                        depth_at_uv = -1.;
                    
                    if (depth_at_uv < 0) {
                        continue;
                    }
                    const pcl::PointXYZ &pt = depth[u*cols+v];
                    Eigen::Vector3d q = transformation_ * Eigen::Vector3d(pt.x, pt.y, pt.z);
      
                    double D = SDF(q, primitives);
                    double Dabs = fabs(D);
                    
                    if(Dabs > precision)
                        continue;
                    
                    //partial derivative of SDF wrt position  
                    Eigen::Vector3d tmp = SDFGradient(q, primitives);
                    if (std::isnan(tmp(0)) || std::isnan(tmp(1)) || std::isnan(tmp(2)))
                        continue;

#ifdef SDF_DRAW_GRAD_DEBUG
                    // Debug drawing...
                    Eigen::Vector3d normal_vector = tmp;
                    normal_vector.normalize();
                    preview.at<cv::Vec3b>(u,v)[1]=128-rint(normal_vector(0)*127);
                    preview.at<cv::Vec3b>(u,v)[2]=128-rint(normal_vector(1)*127);
                    preview.at<cv::Vec3b>(u,v)[0]=128-rint(normal_vector(2)*127);
#endif
                    
//                    Sabs += Dabs;
//                    S += D;
                    Eigen::Matrix<double,1,3> dSDF_dx(tmp(0), tmp(1), tmp(2));

                    //partial derivative of position wrt optimizaiton parameters
                    Eigen::Matrix<double,3,6> dx_dxi; 
                    dx_dxi << 0, q(2),-q(1), 1, 0, 0,
                          -q(2),    0, q(0), 0, 1, 0,
                           q(1),-q(0),    0, 0, 0, 1;
                    
                    //jacobian = derivative of SDF wrt xi (chain rule)
                    Eigen::Matrix<double,1,6> J = dSDF_dx*dx_dxi;

                    J(0,0) = 0.;
                    J(0,1) = 0.;
// Optimize in x,y,th (z-axis)
//                    J(0,2) = 0.;
//                    J(0,3) = 0.;
//                    J(0,4) = 0.;
                    J(0,5) = 0.;
                    
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
            
            Eigen::Vector6d g;
            g<< g0, g1, g2, g3, g4, g5;
            
            g *= scaling;
            A *= scaling;
            
            double regularization = 0.01; // TODO
            double   min_parameter_update = 0.0001; // TODO

            
            A = A + regularization*Eigen::MatrixXd::Identity(6,6);
            xi = xi - A.ldlt().solve(g);

            Eigen::Vector6d Change = xi-xi_prev;  
            double Cnorm = Change.norm();
#ifdef SDF_DRAW_GRAD_DEBUG
            cv::imshow("grad", preview);
            cv::waitKey(3);
#endif
            xi_prev = xi;
            if(Cnorm < min_parameter_update) break;
        }//k
    }//level
    if(std::isnan(xi.sum())) {
      //        std::cout << "xi : isnan..." << std::endl;
        xi << 0.0,0.0,0.0,0.0,0.0,0.0;
    }

    return ComputeAffine3d(xi)*transformation;
}//function


}//sdf
