// Test the conversion
#include <ros/ros.h>
#include <boost/program_options.hpp>
#include <orunav_trajectory_processor/trajectory_processor_naive.h>
//#include <trajectory_processor/trajectory_mpc_smoother.h>
#include <orunav_trajectory_processor/trajectory_processor_naive_ct.h>
#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>
#include <orunav_rviz/orunav_rviz.h>

// For saving the trajectory_processor_ct
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp> // for std::pair

namespace po = boost::program_options;

using namespace std;

int main(int argc, char** argv) {

  ros::init(argc, argv, "path_to_trajectory_test");


  TrajectoryProcessor::Params params;
  string path_file_name, ct_file_name;
  double goal_time, middle_time, control_constraint_speed, min_incr_dist;
  int step_idx_control_constraint;
  int hole_width;  
  int ct_step_idx;
  double ct_step_time;
     po::options_description desc("Allowed options");
     desc.add_options()
       ("help", "produce help message")
       ("debug", "print debug output")
       ("initial_state_zero", "initial state is set to zero")
       ("fileName", po::value<string>(&path_file_name)->required(), "path file to be used")
       ("fileNameCt", po::value<string>(&ct_file_name)->default_value(std::string("")), "ct file to be used")
       ("maxVel", po::value<double>(&params.maxVel)->default_value(1.), "max allowed velocity in the trajectory")
       ("maxAcc", po::value<double>(&params.maxAcc)->default_value(1.), "max allowed acceleration")
       ("maxRotationalVel", po::value<double>(&params.maxRotationalVel)->default_value(1.), "max allowed rotational velocity in the trajectory")
       ("maxSteeringAngleVel", po::value<double>(&params.maxSteeringAngleVel)->default_value(1.), "max allowed steering angle velocity in the trajectory")
       ("goal_time", po::value<double>(&goal_time)->default_value(-1.), "coordination time when the goal should be reached")
       ("middle_time", po::value<double>(&middle_time)->default_value(-1.), "coordination time when the middle point of the path should be reached")
       ("time_step", po::value<double>(&params.timeStep)->default_value(0.06), "fixed time step (to be used when sending the trajectory to the controller)")
       ("step_idx_control_constraint", po::value<int>(&step_idx_control_constraint)->default_value(-1), "idx where a control constraint with (default v = 0.7 and w = 0) will be imposed")
       ("control_constraint_speed", po::value<double>(&control_constraint_speed)->default_value(0.7), "speed at the step_idx_control_constraint step (if provided)")
       ("min_incr_dist", po::value<double>(&min_incr_dist)->default_value(0.0001), "minimum incremental distance in the path")
       ("control_constraint_as_start", "if the control_constraint step should be set to be the start of the generated trajectory")
       ("deltaT_check", "evaluates the deltaT coord time output/input to the coordinator")
       ("skip_acc_ct", "skip the acceleration based even out coordination times changes")
       ("skip_ct_cc", "skip the constraint points used from coordination time")
       ("traj_smooth", "apply a trajectory smoothing step (based on the model predictive control (MPC) scheme)")
       ("creep_speed", po::value<double>(&params.creepSpeed)->default_value(0.), "if a minimum speed should be assigned at the start and end of the trajectory")
       ("creep_distance", po::value<double>(&params.creepDistance)->default_value(0.), "the distance the creeps speed should be applied (acumulated from the end point)")
       ("set_creep_speed_as_end_constraint", "set creep speed as end constraint")
       ("reverse_path", "if the paths goal / start should be changed")
       ("hole_width", po::value<int>(&hole_width)->default_value(-1), "if holes in the coordination times should be given (otherwise the ct will be dense)")
       ("ct_step_idx", po::value<int>(&ct_step_idx)->default_value(-1), "if an increase in ct should be added (ct_step_time)")
       ("ct_step_time", po::value<double>(&ct_step_time)->default_value(10.), "time used in ct_step_idx")
       ("use_rviz", "visualize using rviz")
       ("use_ct_processor", "if the TrajectoryProcessorNaiveCT should be used")
         ;
     
     po::variables_map vm;
     po::store(po::parse_command_line(argc, argv, desc), vm);
     
     if (vm.count("help")) {
	  cout << desc << "\n";
	  return 1;
     }

     po::notify(vm);    
     
     bool debug = vm.count("debug");
     bool deltaT_check =  vm.count("deltaT_check");
     bool control_constraint_as_start = vm.count("control_constraint_as_start");
     bool traj_smooth = vm.count("traj_smooth");
     bool reverse_path = vm.count("reverse_path");
     bool use_rviz = vm.count("use_rviz");
     bool use_ct_processor = vm.count("use_ct_processor");
     params.useCoordTimeAccConstraints = !vm.count("skip_acc_ct");
     params.useCoordTimeContraintPoints = !vm.count("skip_ct_cc");
     params.useInitialState = !vm.count("initial_state_zero");
     params.setCreepSpeedAsEndConstraint = !vm.count("set_creep_speed_as_end_constraint");
     assert(params.useInitialState == true); // Not used.
     params.debug = true;

     // // --------------- DEBUG -> Params from VehicleExecutionNode -----------------

     // params.wheelBaseX = 1.190;
     // params.maxVel = 0.8;
     // params.maxRotationalVel = 0.8;
     // params.maxAcc = 0.2;

     // params.useCoordTimeAccConstraints = true;
     // params.useCoordTimeContraintPoints = true;
     // // ---------------------------------------------------------------------------

     cout << "Used params : " << params << endl;

     TrajectoryProcessorNaive gen;

     cout << "loading : " << path_file_name << endl;
     orunav_generic::Path loaded_path = orunav_generic::loadPathTextFile(path_file_name);
     std::cout << "loaded path sizes : " << loaded_path.sizePath() << std::endl;
     orunav_generic::Path min_dist = orunav_generic::minIncrementalDistancePath(loaded_path, min_incr_dist);
     std::cout << "size before min intermediate dir : " << min_dist.sizePath() << std::endl;
     orunav_generic::Path path = orunav_generic::minIntermediateDirPathPoints(min_dist);
     std::cout << "size after min intermediate dir : " << path.sizePath() << std::endl;
     if (reverse_path)
         path = orunav_generic::getReversePathWithoutChangingDirection(path);

     gen.addPathInterface(path);
     int nb_steps = path.sizePath();
     cout << "number of path steps : " << nb_steps << endl;

     gen.setParams(params);
#if 0
     if (goal_time > 0) {
       // Coordination test.
       std::cout << "adding coordination test, goal_time is set to : " << goal_time << std::endl;
       CoordinatedTimes ct(nb_steps);
       ct.front() = 0.;
       ct.back() = goal_time;
       if (middle_time > 0.) {
	 ct[nb_steps/2] = middle_time;
       }
       gen.addCoordinatedTimes(ct);
     }
#endif

     // Use the delta T's directly as coordination time. - gen2 is only to compute the delta t's.
     orunav_generic::CoordinatedTimes ct;
     if (!ct_file_name.empty()) {
         ct = orunav_generic::loadCoordinatedTimesTextFile(ct_file_name);         
         std::cout << "loading ct : " << ct_file_name << std::endl;
         std::cout << "loaded ct size : " << ct.size() << std::endl;
         if (ct.size() != (unsigned int)nb_steps) {
             std::cerr << "wrong coordination times vector - size don't match - quitting." << std::endl;
             exit(-1);
         }
         gen.addCoordinatedTimes(ct);
     }
     else {
         ct = orunav_generic::CoordinatedTimes(nb_steps);
     }

     double step_idx_control_constraint_time = 0.;
     if (deltaT_check) {
       TrajectoryProcessorNaive gen2;
       gen2.setParams(params);
       gen2.addPathInterface(path);
       //gen2.computeDts();
       orunav_generic::Trajectory traj = gen2.getTrajectory();
       ct.front() = 0.;
       for (unsigned int i = 0; i < gen2.sizeDeltaTVec()-1; i++) {
	 ct[i+1] = ct[i] + gen2.getDeltaT(i);
       }
       
       if (step_idx_control_constraint > 0)
           step_idx_control_constraint_time = ct[step_idx_control_constraint];

       // Make some "holes" in the ct.
       if (hole_width > 0) {
           ct.createHoles(hole_width);
       }
       if (ct_step_idx > 0) {
           ct.createStep(ct_step_idx, ct_step_time);
       }
       if (control_constraint_as_start) {
           ct[step_idx_control_constraint] = step_idx_control_constraint_time;
       }

       ct.removeTimesBefore(step_idx_control_constraint_time);
       gen.addCoordinatedTimes(ct);
     }
     
     orunav_generic::Control c;
     if (step_idx_control_constraint > 0) {
       c.v = control_constraint_speed;
       c.w = 0.;
       if (control_constraint_as_start) {
	 gen.addControlConstraintPointAsStart(step_idx_control_constraint, c);
       }
       else {
	 gen.addControlConstraintPoint(step_idx_control_constraint, c);
       }
     }
     orunav_generic::Trajectory traj;
     if (!use_ct_processor)
         traj = gen.getTrajectory();
     else {
         TrajectoryProcessorNaiveCT gen_ct;
         gen_ct.addPathInterface(path);
         gen_ct.setParams(params);
         gen_ct.setStartIdxTime(step_idx_control_constraint_time);
         gen_ct.addCoordinatedTimes(ct);
         if (step_idx_control_constraint >= 0) {
             if (control_constraint_as_start) {
                 gen_ct.addControlConstraintPointAsStart(step_idx_control_constraint, c);
             }
             else {
                 gen_ct.addControlConstraintPoint(step_idx_control_constraint, c);
             }
         }
         {
             std::cout << "saving ct_processor" << std::endl;
             std::ofstream ofs("tpnct.dat");
             boost::archive::text_oarchive ar(ofs);
             ar & gen_ct;
             std::cout << "saving ct_processor - done" << std::endl;
             gen_ct.printDebug();
         }
         traj = gen_ct.getTrajectory();
         ct = gen_ct.getCoordinatedTimes();
         std::vector<double> global_times = gen_ct.getGlobalPathTimes(step_idx_control_constraint_time);
         orunav_generic::saveDoubleVecTextFile(ct, "output.global_times");
     }
     
     // if (traj_smooth) {
     //     smoothTrajectoryMPCparams mpc_params;
     //   std::cout << "performing MPC based smoothing" << std::endl;
     //   traj = smoothTrajectoryMPC(params, mpc_params, traj);
     //   std::cout << "size of smoothed trajectory : " << traj.sizeTrajectory() << std::endl;
     // }

     std::cout << "size of generated trajectory : " << traj.sizeTrajectory() << std::endl;

     orunav_generic::saveTrajectoryTextFile(traj, "output.traj");
     orunav_generic::saveDoubleVecTextFile(ct, "output.ct");
     
     cout << "Avg sqr. distance trajectory error : " << orunav_generic::calcTrajectoryErrorFixedDt(traj, params.wheelBaseX, params.timeStep);

     cout << "done." << endl;

     // if (use_rviz) {
     //       ros::NodeHandle nh;
     //       ros::Publisher marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
           
     //       orunav_generic::TrajectoryChunks traj_chunks_;
     //       traj_chunks_.push_back(traj);
     //       for (int i = 0; i < 10; i++) {
               
     //           std::cout << ">>publishing marker iter : " << i << std::endl;
     //           ros::Rate r(2);
     //           r.sleep();

     //           // Add trajectory - chunk + coordination times.
     //           orunav_rviz::drawTrajectoryChunksWithControl(traj_chunks_, 1, "chunks", marker_pub_); 

     //           double time_offset = step_idx_control_constraint_time;
     //           orunav_rviz::drawTrajectoryChunksUsingTime(traj_chunks_, 1, "chunks_time", time_offset, 0.1/*time_to_meter_factor_*/, params.timeStep/*dt*/, marker_pub_);
           
     //           // CT
     //           orunav_rviz::drawCoordinatedTimesAsText(path, 
     //                                                  ct,
     //                                                  "ct", 0, 0.2, marker_pub_);
     //           orunav_rviz::drawCoordinatedTimes(path, ct, 0., "ct", 1, 0.1/*time_to_meter_factor_*/, marker_pub_);

     //       }
     // }
     return 1;
}
