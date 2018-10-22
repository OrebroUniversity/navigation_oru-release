#include <orunav_projector/projector_viz.h>
//#include <ndt_visualisation/ndt_viz.h>
#include <boost/program_options.hpp>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

namespace po = boost::program_options;

int main(int ac, char** av) {
  po::options_description desc("Allowed options");
  Eigen::Vector3f pos, fp;
  float aspect_ratio;
  desc.add_options()
    ("help", "produce help message")
    ("pos_x", po::value<float>(&pos(0))->default_value(0.), "x camera pos (meters)")
    ("pos_y", po::value<float>(&pos(1))->default_value(0.), "y camera pos (meters)")
    ("pos_z", po::value<float>(&pos(2))->default_value(2.), "z camera pos (meters)")
    ("fp_x", po::value<float>(&fp(0))->default_value(1.), "x focal point (meters)")
    ("fp_y", po::value<float>(&fp(1))->default_value(0.), "y focal point (meters)")
    ("fp_z", po::value<float>(&fp(2))->default_value(0.), "z focal point (meters)")
    ("aspect_ratio", po::value<float>(&aspect_ratio)->default_value(1.), "aspect ratio")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);    
  
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }
  

  NDTViz ndt_viz;

  ndt_viz.win3D->setFullScreen(true);
  ndt_viz.win3D->setAspectRatioFactor(aspect_ratio);
  ndt_viz.win3D->switchCamera(std::string("orbit"));
  //ndt_viz.win3D->getCameraPtr()->setFocalPoint(fp);
  //ndt_viz.win3D->getCameraPtr()->setPosition(pos);
  ndt_viz.win3D->setCameraPointingToPoint(fp(0), fp(1), fp(2));
  ndt_viz.win3D->setCameraPosition(pos(0), pos(1), pos(2));
  ndt_viz.addGrid(100, 0.15, 5., 1., 1., 1., 1.);
  ndt_viz.win3D->start_main_loop_own_thread();

  
  while (1) {
    usleep(100000);
    pos = ndt_viz.win3D->getCameraConstPtr()->getPosition();
    fp = ndt_viz.win3D->getCameraConstPtr()->getFocalPoint();
  
    std::cout << "rosrun orunav_projector manual_calibration --pos_x " << pos(0) << " --pos_y " << pos(1) << " --pos_z " << pos(2) << " --fp_x " << fp(0) << " --fp_y " << fp(1) << " --fp_z " << fp(2) << " --aspect_ratio " << aspect_ratio << std::endl;
    
    // Also print out the corresponding tf transform.
    Eigen::Affine3d Tcam = Eigen::Affine3d::Identity();
    Tcam.translation() = Eigen::Vector3d(pos(0), pos(1), pos(2));
    double roll = 0.;
    Eigen::Vector3f diff = fp-pos;
    double yaw = atan2(diff(1),diff(0));
    double pitch = acos(diff(2) / diff.norm());
    // Tcam.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    //                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * 
    //                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) ).toRotationMatrix();

    // tf::Transform tf_T;
    // tf::poseEigenToTF (Tcam, tf_T);
    
    // std::cout << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"robotX_projector\" args=\"" << pos(0) << " " << pos(1) << " " << pos(2) << " " << yaw << " " << pitch << " " << roll << " /robot1/base_footprint /robot1/projector_link 10\"/>" << std::endl;
  }

}
