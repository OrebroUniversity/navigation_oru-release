#include <orunav_motion_planner/PathFinder.h>
#include <orunav_motion_planner/ModelUtilities.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(int argc, char** argv) {

  std::string prim_dir, tab_dir, model;
  double cost_cutoff, fill_distance;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("prim_dir", po::value<std::string>(&prim_dir)->default_value(std::string("/home/han/catkin_ws/src/orunav/orunav_motion_planner/Primitives/")), "primitives directory (.mprim file)")
    ("tab_dir", po::value<std::string>(&tab_dir)->default_value(std::string("/home/han/catkin_ws/src/orunav/orunav_motion_planner/LookupTables/")), "lookup tables directory (.hst file)")
    ("model", po::value<std::string>(&model)->default_value(std::string("test")), "model name -> <model name>.mprim will be loaded")
    ("cost_cutoff", po::value<double>(&cost_cutoff)->default_value(30.), "cost at when to stop")
    ("fill_distance", po::value<double>(&fill_distance)->default_value(15.), "required distance to fill up the table")
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
  
  CarModel* m = new CarModel(model);
  
  ModelUtilities::generateModelHeuristicTable(m, cost_cutoff, fill_distance);

  std::cout << "Done." << std::endl;

  delete m;
  return 0;
}


