#include <orunav_projector/projector_viz.h>
#include <boost/program_options.hpp>


namespace po = boost::program_options;

int main(int ac, char** av) {
  po::options_description desc("Allowed options");
  Eigen::Vector3f pos, fp;
  float aspect_ratio;
  std::string slowdown_texture;
  desc.add_options()
    ("help", "produce help message")
    ("slowdown_texture", po::value<std::string>(&slowdown_texture)->default_value(std::string("zebra.png")), "texture to be used withing the slowdown region - hardcoded in this test file")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);    
  
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }
  

  ProjectorViz viz;

  viz.addGrid(100, 0.15, 5., 1., 1., 1., 1.);
  viz.win3D->start_main_loop_own_thread();

  viz.setSlowdownRegionTexture(slowdown_texture);
  
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > pts;
  pts.push_back(Eigen::Vector2d(-1., -1.));
  pts.push_back(Eigen::Vector2d(1., -1.));
  pts.push_back(Eigen::Vector2d(1., 1.));
  pts.push_back(Eigen::Vector2d(-1., 1.));

  viz.addSlowdownRegion(pts);

  while (1) {
    

    usleep(100000);
  }

}
