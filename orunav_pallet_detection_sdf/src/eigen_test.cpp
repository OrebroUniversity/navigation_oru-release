#include <Eigen/Core>
#include <Eigen/Geometry>

int main( int argc, char* argv[] )
{
    Eigen::Affine3d cam_to_local = Eigen::Affine3d::Identity();
    cam_to_local.linear() = (Eigen::AngleAxisd(M_PI/2., Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI/2., Eigen::Vector3d::UnitX())).toRotationMatrix();

    

}
