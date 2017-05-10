#include <orunav_motion_planner/PathFinder.h>
#include <orunav_motion_planner/ModelUtilities.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(int argc, char** argv) {

  std::string prim_dir, tab_dir, model;
  double max_rel_dist_inc;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("prim_dir", po::value<std::string>(&prim_dir)->default_value(std::string("/home/han/catkin_ws/src/orunav/orunav_motion_planner/Primitives/")), "primitives directory (.mprim file)")
    ("tab_dir", po::value<std::string>(&tab_dir)->default_value(std::string("/home/han/catkin_ws/src/orunav/orunav_motion_planner/LookupTabeles/")), "lookup tables directory (.hst file)")
    ("model", po::value<std::string>(&model)->default_value(std::string("test")), "model name -> <model name>.mprim will be loaded")
    ("max_rel_dist_inc", po::value<double>(&max_rel_dist_inc)->default_value(1.2), "max relative distance increase when reducing primitives")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }
  
  po::notify(vm);    


  WP::setPrimitivesDir(prim_dir);
  WP::setTablesDir(tab_dir);
  
  LHDModel* m = new LHDModel(model);
  ModelUtilities::reduceModelMotionPrimitiveSet(m, max_rel_dist_inc); // 1.2 is the max relative increase in distance.


  std::cout << "Done." << std::endl;
  delete m;
  return 0;
}


