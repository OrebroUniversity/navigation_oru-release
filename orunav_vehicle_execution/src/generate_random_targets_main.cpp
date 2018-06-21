#include <ros/ros.h>
#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>
#include <ctime>

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/generator_iterator.hpp>

#include <orunav_generic/interfaces.h>
#include <orunav_generic/io.h>
#include <orunav_generic/random.h>
#include <orunav_conversions/conversions.h>
#include <orunav_msgs/RobotTarget.h>
#include <orunav_vehicle_execution/io.h>

namespace po = boost::program_options;
using namespace std;
typedef boost::minstd_rand base_generator_type;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_random_goal_states"); // Only to use for visualization...

  po::options_description desc("Allowed options");
  std::string output_filename;
  double x_min, y_min, th_min, phi_min, x_max, y_max, th_max, phi_max;
  int nb_goal_poses, seed;
  desc.add_options()
    ("help", "produce help message")
    ("nb_goal_poses", po::value<int>(&nb_goal_poses)->default_value(10), "default number of goal poses")
    ("x_min", po::value<double>(&x_min)->default_value(1.), "min x value")
    ("y_min", po::value<double>(&y_min)->default_value(1.), "min y value")
    ("th_min", po::value<double>(&th_min)->default_value(-M_PI), "min th value")
    ("x_max", po::value<double>(&x_max)->default_value(2.), "max x value")
    ("y_max", po::value<double>(&y_max)->default_value(2.), "max y value")
    ("th_max", po::value<double>(&th_max)->default_value(M_PI), "max th value")
    ("seed", po::value<int>(&seed)->default_value(-1), "seed value, a negative value gives a random one")
    ("output_filename", po::value<std::string>(&output_filename)->default_value(std::string("targets.dat")), "output filename")
    ("append", "use cat file1 file2 file3 > appended_file instead")
    ;
  
     po::variables_map vm;
     po::store(po::parse_command_line(argc, argv, desc), vm);
     
     if (vm.count("help")) {
	  cout << desc << "\n";
	  
	  return 1;
     }

     po::notify(vm);    

     assert(x_min < x_max);
     assert(y_min < y_max);
     assert(th_min < th_max);
     //     assert(phi_min < phi_max);

     // Random number gen.
     base_generator_type generator(static_cast<unsigned int>(seed));
     if (seed < 0) {
       generator.seed(static_cast<unsigned int>(orunav_generic::getSeed()));
     }
     boost::uniform_real<> uni_dist(0,1);
     boost::variate_generator<base_generator_type&, boost::uniform_real<> > uni(generator, uni_dist);

     std::vector<orunav_msgs::RobotTarget> targets;
     orunav_msgs::RobotTarget target;
     // Fill the default values
     target.goal_load.status = target.goal_load.EMPTY;
     target.current_load.status = target.current_load.EMPTY;
     target.start_op.operation = target.start_op.NO_OPERATION;
     target.goal_op.operation = target.goal_op.NO_OPERATION;
     target.robot_id = 1;
     target.task_id = 1;

     orunav_generic::State2d start_state(0., 0., 0., 0.);
     target.start = orunav_conversions::createPoseSteeringMsgFromState2d(start_state);

     orunav_generic::Path goals;
     for (int i = 0; i < nb_goal_poses; i++) {
       double x = uni()*(x_max - x_min) + x_min;
       double y = uni()*(y_max - y_min) + y_min;
       double th = uni()*(th_max - th_min) + th_min;
       double phi = 0.;
       orunav_generic::State2d goal_state(x,y,th,phi);
       target.goal = orunav_conversions::createPoseSteeringMsgFromState2d(goal_state);
       targets.push_back(target);
       target.task_id++;
     }
     
     saveRobotTargets(targets, output_filename);
     
     cout << "Done." << endl;
}
