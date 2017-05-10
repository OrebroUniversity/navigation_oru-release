// Parse the controller.log file and visualize it.
#include <ros/ros.h>
#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>
#include <ctime>

#include <orunav_generic/interfaces.h>
#include <orunav_generic/io.h>
#include <orunav_generic/path_utils.h>
#include <orunav_rviz/orunav_rviz.h>

#include <orunav_debug/control_log_parser.h>
#include <orunav_debug/sensor_log_parser.h>

namespace po = boost::program_options;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_log_parser");

  po::options_description desc("Allowed options");

  std::string log_filename, sensor_log_filename;
  double wheel_offset;
  desc.add_options()
      ("help", "produce help message")
      ("log_filename", po::value<std::string>(&log_filename)->default_value(std::string("controller.log")), "input log filename")
      ("sensor_log_filename", po::value<std::string>(&sensor_log_filename)->default_value(std::string("")), "input sensor log filename")
      ("wheel_offset", po::value<double>(&wheel_offset)->default_value(1.19), "wheel offset in local x-coords")
      ("draw_actual_velocities", "if the velocities drawn should be derrived from the states and not the control signal sent")
      ("draw_ref_path", "if the reference path should be drawn")
      ("draw_ref_steering_wheel_vel", "if the reference steering velocity should be draw")
      ;
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  
  if (vm.count("help")) {
      cout << desc << "\n";
      
      return 1;
  }
  bool draw_actual_velocites = vm.count("draw_actual_velocities");
  bool draw_ref_path = vm.count("draw_ref_path");
  bool draw_ref_steering_wheel_vel = vm.count("draw_ref_steering_wheel_vel");
  po::notify(vm);    

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  
  ros::Rate r(2);


  std::cout << "Loading the sensor log file : " << log_filename << std::endl;
  ControlLogParser parser(log_filename);
  std::cout << "Number of states loaded : " << parser.getSensorStates().sizePath() << std::endl;

  orunav_generic::Trajectory sensor_enc;
  
  if (!sensor_log_filename.empty()) {
      std::cout << "Loading the sensor log file : " << sensor_log_filename << std::endl;
      SensorLogParser sensor_parser(sensor_log_filename);
      
      std::cout << "Number of sensor states loaded : " << sensor_parser.getSensorStatesAndVelocities().sizeTrajectory() << std::endl;
      sensor_enc = sensor_parser.getSensorStatesAndVelocities();
  }

  orunav_generic::Control c = orunav_generic::computeAvgSqrControlDifference(parser.getReferenceStatesAndControls(),
                                                                           parser.getEstimatedStatesAndUsedControls());

  std::cout << "average squared control diff fwd vel      : " << c.v << std::endl;
  std::cout << "average squared control diff steering vel : " << c.w << std::endl;

  orunav_generic::Trajectory trajectory_driven = orunav_generic::convertPathToTrajectoryWithoutModel(parser.getEstimatedStatesAndUsedControls(), 0.06);
  
  while (ros::ok()) {
      // Visualize in RViz.
      // orunav_rviz::drawPathInterface(parser.getSensorStates(), "sensor", 0, wheel_offset, pub);
      // orunav_rviz::drawPathInterface(parser.getExpectedStates(), "expected", 1, wheel_offset, pub);
      // orunav_rviz::drawPathInterface(parser.getReferenceStatesAndControls(), "reference_path", 1, wheel_offset, pub);
      // orunav_rviz::drawTrajectoryWithControl(parser.getReferenceStatesAndControls(), 1, 0, "reference_t_vel", pub);
      // orunav_rviz::drawTrajectoryWithControl(parser.getEstimatedStatesAndUsedControls(), 0, 0, "computed_t_vel", pub);

      if (draw_ref_path) {
          orunav_rviz::drawPathInterface(parser.getReferenceStatesAndControls(), "ref_path", 1, wheel_offset, pub);
      }
      if (draw_ref_steering_wheel_vel) {
          orunav_rviz::drawSteeringAngleVel(parser.getReferenceStatesAndControls(),
                                           1, "ref_steering_vel", pub);
      }
      
      if (draw_actual_velocites) {
          orunav_rviz::drawTrajectoryRefAndExecPair(parser.getReferenceStatesAndControls(),
                                                   trajectory_driven,
                                                   "ref_exec_pairs", pub);
      }
      else { // Draw computed velocities
          orunav_rviz::drawTrajectoryRefAndExecPair(parser.getReferenceStatesAndControls(),
                                                   parser.getEstimatedStatesAndUsedControls(),
                                                   "ref_exec_pairs", pub);
      }

      if (!sensor_log_filename.empty()) {
          orunav_rviz::drawTrajectoryWithControlType(sensor_enc, 2, 0, 8, 0.010, "sensor_enc", pub);
      }
      r.sleep();
  }
}
