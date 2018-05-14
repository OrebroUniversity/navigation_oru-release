#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>

#include <orunav_conversions/conversions.h>

#include <orunav_constraint_extract/polygon_constraint.h>
#include <orunav_constraint_extract/conversions.h>
#include <orunav_constraint_extract/grid_map.h>
#include <orunav_constraint_extract/utils.h>

#include <orunav_path_smoother/path_smoother_dynamic.h>

#include <orunav_rviz/orunav_rviz.h>

#include <boost/program_options.hpp>


namespace po = boost::program_options;
using namespace std;

int main(int argc, char **argv)
{
  po::options_description desc("Allowed options");
  
  int nb_points;
  int max_nb_points;
  int nb_points_discard;

  desc.add_options()
    ("help", "produce help message")
    ("nb_points", po::value<int>(&nb_points)->default_value(73), "number of points in the trajectory")
    ("max_nb_points", po::value<int>(&max_nb_points)->default_value(10), "number of points in the split trajectories")
    ("nb_points_discard", po::value<int>(&nb_points_discard)->default_value(5), "number of points in discarded in each split");
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  
  if (vm.count("help")) {
      cout << desc << "\n";
      
      return 1;
  }
  
  po::notify(vm);    


  orunav_generic::Trajectory traj;

  for (int i = 0; i < nb_points; i++) {
    traj.addTrajectoryPoint(orunav_generic::Pose2d(i, i, i), i, i, i);
  }

  SplitIndex::Params params;
  params.max_nb_points = max_nb_points;
  params.nb_points_discard = nb_points_discard;

  std::cout << "params : " << params;

  SplitIndex si(params, traj);
  si.printDebug();

  for (int i = 0; i < si.size(); i++) {

    orunav_generic::Trajectory t = si.getTrajectory(i);

    // This test is simply to check that the first and last entry in the trajectory are keept when the are set / updated.
    if (i == 3) {
      t.setSteeringAngle(i*0.1, 0);
      t.setSteeringAngle(i*0.1, t.sizeTrajectory()-1);
    }
    si.setTrajectory(i, t);
  }
  orunav_generic::Trajectory traj2 = si.getTrajectory();

  orunav_generic::saveTrajectoryTextFile(traj2, "split.traj");
  return 0;
}


