#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>
#include <orunav_rviz/orunav_rviz.h>

#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>

#include <orunav_geometry/geometry.h>
#include <orunav_geometry/pallet_model_2d.h>

#include <orunav_msgs/ComputeTask.h>
#include <orunav_msgs/UpdateTask.h>
#include <orunav_msgs/ExecuteTask.h>

#include <orunav_msgs/GetPath.h>
#include <orunav_msgs/GetGeoFence.h>
#include <orunav_msgs/GetVectorMap.h>
#include <orunav_msgs/GetPolygonConstraints.h>
#include <orunav_msgs/GetSmoothedPath.h>
#include <orunav_msgs/GetSmoothedStraightPath.h>
#include <orunav_msgs/GetDeltaTVec.h>
#include <orunav_msgs/ObjectPoseEstimation.h>
#include <orunav_msgs/ComputeTaskStatus.h>
#include <orunav_msgs/SetTask.h>

#include <orunav_msgs/ControllerTrajectoryChunkVec.h>
#include <orunav_msgs/ControllerTrajectoryChunk.h>
#include <orunav_msgs/ControllerCommand.h>
#include <orunav_msgs/ControllerReport.h>
#include <orunav_msgs/RobotTarget.h>
#include <orunav_msgs/ForkReport.h>
#include <orunav_msgs/ForkCommand.h>
#include <orunav_msgs/RobotReport.h>
#include <orunav_msgs/ObjectPose.h>
#include <orunav_msgs/VectorMap.h>
#include <orunav_msgs/GeoFence.h>
#include <orunav_msgs/EBrake.h>
#include <orunav_msgs/BrakeTask.h>

#include <orunav_constraint_extract/polygon_constraint.h> // Only used for visualization.
#include <orunav_constraint_extract/conversions.h>
#include <orunav_constraint_extract/grid_map.h>
#include <orunav_constraint_extract/utils.h>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp> // for std::pair
#include <iostream>
#include <fstream>

#include <orunav_vehicle_execution/pallet_handling_utils.h>
#include <orunav_vehicle_execution/vehicle_state.h>
#include <orunav_vehicle_execution/trajectory_generation.h>
#include <orunav_node_utils/robot_target_handler.h>
#include <orunav_trajectory_processor/trajectory_processor_naive_ct.h>
#include <orunav_trajectory_processor/trajectory_processor_naive.h>

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <std_msgs/Float64MultiArray.h>


void updateTrajParamsWithVelocityConstraints(TrajectoryProcessor::Params &traj_params, const VehicleState &vehicle_state)  {
  traj_params.maxVel = std::min(traj_params.maxVel, vehicle_state.getMaxLinearVelocityConstraint());
  traj_params.maxVelRev = std::min(traj_params.maxVelRev, vehicle_state.getMaxLinearVelocityConstraintRev());
  traj_params.maxRotationalVel = std::min(traj_params.maxRotationalVel, vehicle_state.getMaxRotationalVelocityConstraint());
  traj_params.maxRotationalVelRev = std::min(traj_params.maxRotationalVelRev, vehicle_state.getMaxRotationalVelocityConstraintRev());
  
  traj_params.maxVel = std::max(traj_params.maxVel, 0.01); // Always allow to drive faster than 1 cm /s.
  traj_params.maxVelRev = std::max(traj_params.maxVelRev, 0.01);
  traj_params.maxRotationalVel = std::max(traj_params.maxRotationalVel, 0.01); // Alway allow to rotate more than 0.01 rad / s.
  traj_params.maxRotationalVelRev = std::max(traj_params.maxRotationalVelRev, 0.01);
}
  


void drawPointCloud(const sensor_msgs::PointCloud &points, const std::string &name, int id, int color, double scale, ros::Publisher &pub)
{

  if (points.points.size() == 0)
    return;
  visualization_msgs::Marker m;
  orunav_rviz::assignDefault(m);
  orunav_rviz::assignColor(m, color);
  m.scale.x = scale;
  m.scale.y = scale;
  m.type = visualization_msgs::Marker::POINTS;
  m.action = visualization_msgs::Marker::ADD;
  m.ns = name + orunav_generic::toString(id);
  m.id = id;

  geometry_msgs::Point p;
  p.z = 0.;

  for (unsigned int i = 0; i < points.points.size(); i++)
  {
    p.x = points.points[i].x;
    p.y = points.points[i].y;
    m.points.push_back(p);
  }
  pub.publish(m);
}

char blue[] = {0x1b, '[', '1', ';', '3', '4', 'm', 0};

class KMOVehicleExecutionNode
{

private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_compute_;
  ros::Publisher compute_status_pub_;
  ros::ServiceServer service_execute_;
  ros::ServiceServer service_brake_;

  ros::Publisher trajectorychunk_pub_;
  ros::Publisher command_pub_;
  ros::Publisher forkcommand_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher report_pub_;
  ros::Publisher planningmap_pub_;

  ros::Subscriber laserscan_sub_;
  ros::Subscriber laserscan2_sub_;

  ros::Subscriber control_report_sub_;
  ros::Subscriber fork_report_sub_;
  ros::Subscriber enc_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber pallet_poses_sub_;

  ros::Subscriber velocity_constraints_sub_;
  ros::Subscriber ebrake_sub_;

  boost::mutex map_mutex_, inputs_mutex_, current_mutex_, run_mutex_;
  boost::thread client_thread_;
  boost::condition_variable cond_;

  VehicleState vehicle_state_;
  orunav_node_utils::RobotTargetHandler target_handler_;

  long current_id_;

  nav_msgs::OccupancyGrid current_map_;
  bool valid_map_;

  orunav_msgs::VectorMap vector_map_;
  orunav_msgs::GeoFence geofence_;

  bool got_first_state_;
  bool valid_control_;

  orunav_generic::Path current_path_;             // This is used to communicate the actual path to be driven...
  orunav_generic::Path current_constraints_path_; // Only used for visualization...
  constraint_extract::PolygonConstraintsVec current_constraints_;
  std::vector<orunav_geometry::Polygon> current_constraints_outer_;

  orunav_msgs::RobotTarget current_target_;
  std::string current_state_str_;
  orunav_generic::CoordinatedTimes current_cts_;

  // Path generation params
  double min_incr_path_dist_;
  int min_nb_path_points_;
  TrajectoryProcessor::Params traj_params_;
  TrajectoryProcessor::Params traj_params_original_;
  TrajectoryProcessor::Params traj_slowdown_params_;
  bool overwrite_traj_params_with_velocity_constraints_;

  // Forklift settings
  int robot_id_;
  bool use_forks_;
  std::string model_name_;

  // Internal flags
  bool b_shutdown_;

  // Driving params
  double max_tracking_error_;

  ros::Timer heartbeat_report_pub_;

  // Visualization params
  bool visualize_;
  ros::Timer heartbeat_slow_visualization_;
  ros::Timer heartbeat_fast_visualization_;
  ros::Timer heartbeat_perception_;
  double current_start_time_;
  double time_to_meter_factor_;

  bool draw_sweep_area_;

  orunav_geometry::RobotModel2dCiTiTruck model1;
  orunav_geometry::RobotModel2dCiTiTruckWithArm model2;
  orunav_geometry::RobotModel2dPitViper model_pitviper; // TODO - this should be removed and placed in ac.
  orunav_geometry::RobotModel2dBtTruck model_bt;
  orunav_geometry::RobotModel2dInterface *model_;

  // Docking params
  double min_docking_distance_;
  double max_docking_distance_;
  double min_docking_driven_distance_;
  double max_target_angular_diff_;
  double max_target_distance_diff_;
  double max_target_distance_diff_fwd_;
  double max_target_distance_diff_side_;
  double max_steering_range_smoothed_path_;
  double overshoot_distance_;
  int docking_max_nb_opt_points_;

  // Coordination parameters
  bool use_ct_; // This is probably obsolete.
  bool provide_dts_; // If the use_ct is not set, compute the dts anyway to provide this information to the coordinator.

  // Motion planner parameters
  bool use_vector_map_and_geofence_;

  // Used for the laser safety functionallities - important, the boost polygon performs really bad when used in debug mode (factor of 100s).
  tf::TransformListener tf_listener_;
  laser_geometry::LaserProjection laser_projection_;
  double ebrake_lookahead_time_;
  double slowdown_drivingslow_lookahead_time_;
  double slowdown_lookahead_time_;
  bool use_safetyregions_;
  orunav_geometry::Polygon current_global_ebrake_area_;
  orunav_geometry::Polygon current_global_slowdown_area_;

  int chunk_idx_connect_offset_;

  ros::Time last_process_fork_report_time_;
  bool cts_clear_first_entry_in_pairs_;
  bool use_ahead_brake_;
  bool visualize_sweep_and_constraints_;

  bool use_update_task_service_;
  bool start_driving_after_recover_;

  bool real_cititruck_;
  bool no_smoothing_;
  bool resolve_motion_planning_error_;

  std::set<int> ebrake_id_set_;

  double max_linear_vel_pallet_picking_;
  double max_rotational_vel_pallet_picking_;
  double max_linear_vel_rev_pallet_picking_;
  double max_rotational_vel_rev_pallet_picking_;
  
public:
  KMOVehicleExecutionNode(ros::NodeHandle &paramHandle)
  {
    bool use_arm;
    // Parameters
    paramHandle.param<int>("robot_id", robot_id_, 1);
    {
      std::vector<int> robot_ids;
      robot_ids.push_back(robot_id_);
      target_handler_.setRobotIDsToCompute(robot_ids);
    }
    paramHandle.param<std::string>("model", model_name_, std::string("cititruck"));
    paramHandle.param<bool>("use_forks", use_forks_, false);
    paramHandle.param<bool>("use_arm", use_arm, false);
    paramHandle.param<double>("time_to_meter_factor", time_to_meter_factor_, 0.02);
    paramHandle.param<double>("max_tracking_error", max_tracking_error_, 100.);

    paramHandle.param<bool>("use_vector_map_and_geofence", use_vector_map_and_geofence_, false);

    paramHandle.param<bool>("traj_debug", traj_params_.debug, true);
    paramHandle.param<double>("wheel_base_x", traj_params_.wheelBaseX, 1.190);
    paramHandle.param<double>("time_step", traj_params_.timeStep, 0.06);
    vehicle_state_.setTimeStep(traj_params_.timeStep);
    paramHandle.param<double>("max_vel", traj_params_.maxVel, 0.1);
    paramHandle.param<double>("max_vel_rev", traj_params_.maxVelRev, traj_params_.maxVel);
    paramHandle.param<double>("max_rotational_vel", traj_params_.maxRotationalVel, 0.1);
    paramHandle.param<double>("max_rotational_vel_rev", traj_params_.maxRotationalVelRev, traj_params_.maxRotationalVel);
    paramHandle.param<double>("max_acc", traj_params_.maxAcc, 0.1);
    paramHandle.param<double>("max_steering_angle_vel", traj_params_.maxSteeringAngleVel, 0.8); // Make sure this is high enough - if the trajectory generation are limited to much by this the velocity profile will vary a lot.
    paramHandle.param<double>("min_incr_path_dist", min_incr_path_dist_, 0.1);
    paramHandle.param<int>("min_nb_path_points_", min_nb_path_points_, 20);
    paramHandle.param<bool>("visualize", visualize_, false);
    traj_params_.useCoordTimeAccConstraints = true;
    traj_params_.useCoordTimeContraintPoints = true;
    traj_params_.debug = true;
    traj_params_.debugPrefix = std::string("ct_traj_gen/");

    paramHandle.param<bool>("overwrite_traj_params_with_velocity_constraints", overwrite_traj_params_with_velocity_constraints_, true);
    traj_params_original_ = traj_params_;
    traj_slowdown_params_ = traj_params_;

    paramHandle.param<double>("max_slowdown_vel", traj_slowdown_params_.maxVel, 0.1);

    paramHandle.param<double>("min_docking_distance", min_docking_distance_, 1.0);
    paramHandle.param<double>("max_docking_distance", max_docking_distance_, 1.3);
    paramHandle.param<double>("min_docking_driven_distance", min_docking_driven_distance_, 0.6);
    paramHandle.param<double>("max_target_angular_diff", max_target_angular_diff_, 0.2);
    paramHandle.param<double>("max_target_distance_diff", max_target_distance_diff_, 0.2);
    paramHandle.param<double>("max_target_distance_diff_fwd", max_target_distance_diff_fwd_, 0.2);
    paramHandle.param<double>("max_target_distance_diff_side", max_target_distance_diff_side_, 0.2);
    paramHandle.param<double>("max_steering_range_smoothed_path", max_steering_range_smoothed_path_, 0.5);
    paramHandle.param<double>("overshoot_distance", overshoot_distance_, 0.);
    paramHandle.param<int>("docking_max_nb_opt_points", docking_max_nb_opt_points_, 40);
    paramHandle.param<bool>("use_ct", use_ct_, false);
    paramHandle.param<bool>("provide_dts", provide_dts_, false);

    paramHandle.param<double>("ebrake_lookahead_time", ebrake_lookahead_time_, 2.);
    paramHandle.param<double>("slowdown_drivingslow_lookahead_time", slowdown_drivingslow_lookahead_time_, 20.);
    paramHandle.param<double>("slowdown_lookahead_time", slowdown_lookahead_time_, 5.);
    paramHandle.param<bool>("use_safetyregions", use_safetyregions_, false);
    std::string safety_laser_topic, safety_laser_topic2;
    paramHandle.param<std::string>("safety_laser_topic", safety_laser_topic, std::string("/laser_scan"));
    paramHandle.param<std::string>("safety_laser_topic2", safety_laser_topic2, std::string("/laser_forkdir_scan"));
    paramHandle.param<int>("chunk_idx_connect_offset", chunk_idx_connect_offset_, 3);
    paramHandle.param<bool>("draw_sweep_area", draw_sweep_area_, false);
    paramHandle.param<bool>("cts_clear_first_entry_in_pairs", cts_clear_first_entry_in_pairs_, true);
    paramHandle.param<bool>("use_ahead_brake", use_ahead_brake_, false);
    paramHandle.param<bool>("visualize_sweep_and_constraints", visualize_sweep_and_constraints_, false);
    paramHandle.param<bool>("use_update_task_service", use_update_task_service_, false);
    paramHandle.param<bool>("start_driving_after_recover", start_driving_after_recover_, true);
    paramHandle.param<bool>("real_cititruck", real_cititruck_, false);
    paramHandle.param<bool>("no_smoothing", no_smoothing_, false);
    paramHandle.param<bool>("resolve_motion_planning_error", resolve_motion_planning_error_, true);

    paramHandle.param<double>("max_linear_vel_pallet_picking", max_linear_vel_pallet_picking_, 0.1);
    paramHandle.param<double>("max_rotational_vel_pallet_picking", max_rotational_vel_pallet_picking_, 0.1);
    paramHandle.param<double>("max_linear_vel_rev_pallet_picking", max_linear_vel_rev_pallet_picking_, max_linear_vel_pallet_picking_);
    paramHandle.param<double>("max_rotational_vel_rev_pallet_picking", max_rotational_vel_rev_pallet_picking_, max_rotational_vel_pallet_picking_);
    
    // Services
    service_compute_ = nh_.advertiseService("compute_task", &KMOVehicleExecutionNode::computeTaskCB, this);
    compute_status_pub_ =  nh_.advertise<orunav_msgs::ComputeTaskStatus>("compute_task/status", 1, true);
    service_execute_ = nh_.advertiseService("execute_task", &KMOVehicleExecutionNode::executeTaskCB, this);
    service_brake_ = nh_.advertiseService("brake_task", &KMOVehicleExecutionNode::brakeTaskCB, this);

    // Publishers
    trajectorychunk_pub_ = nh_.advertise<orunav_msgs::ControllerTrajectoryChunkVec>("control/controller/trajectories", 1000);
    command_pub_ = nh_.advertise<orunav_msgs::ControllerCommand>("control/controller/commands", 1000);
    forkcommand_pub_ = nh_.advertise<orunav_msgs::ForkCommand>("control/fork/command", 1);
    report_pub_ = nh_.advertise<orunav_msgs::RobotReport>("control/report", 1);
    planningmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("debug/planner_map", 1, true);
    
    // Subscribers
    map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &KMOVehicleExecutionNode::process_map, this);
    control_report_sub_ = nh_.subscribe<orunav_msgs::ControllerReport>("control/controller/reports", 10, &KMOVehicleExecutionNode::process_report, this);
    if (use_forks_)
    {
      fork_report_sub_ = nh_.subscribe<orunav_msgs::ForkReport>("control/fork/report", 10, &KMOVehicleExecutionNode::process_fork_report, this);
    }
    //    pallet_poses_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(orunav_generic::getRobotTopicName(robot_id_, "/pallet_poses"), 10, &KMOVehicleExecutionNode::process_pallet_poses,this);
    pallet_poses_sub_ = nh_.subscribe<orunav_msgs::ObjectPose>("pallet_poses", 10, &KMOVehicleExecutionNode::process_pallet_poses, this);

    laserscan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(std::string("sensors/") + safety_laser_topic, 10, &KMOVehicleExecutionNode::process_laserscan, this);
    laserscan2_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(std::string("sensors/") + safety_laser_topic2, 10, &KMOVehicleExecutionNode::process_laserscan, this);

    velocity_constraints_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>(orunav_generic::getRobotTopicName(robot_id_, "/velocity_constraints"), 1, &KMOVehicleExecutionNode::process_velocity_constraints, this);
    ebrake_sub_ = nh_.subscribe<orunav_msgs::EBrake>(orunav_generic::getRobotTopicName(robot_id_, "/ebrake"), 10, &KMOVehicleExecutionNode::process_ebrake, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    heartbeat_slow_visualization_ = nh_.createTimer(ros::Duration(1.0), &KMOVehicleExecutionNode::publish_visualization_slow, this);
    heartbeat_fast_visualization_ = nh_.createTimer(ros::Duration(0.1), &KMOVehicleExecutionNode::publish_visualization_fast, this);
    heartbeat_report_pub_ = nh_.createTimer(ros::Duration(1.0), &KMOVehicleExecutionNode::publish_report, this);
    heartbeat_perception_ = nh_.createTimer(ros::Duration(0.2), &KMOVehicleExecutionNode::handle_perception, this);

    if (use_arm)
    {
      model_ = &model2;
    }
    else
    {
      model_ = &model1;
    }
    if (model_name_ != std::string("cititruck"))
    {
      if (model_name_ == std::string("pitviper"))
      {
        model_ = &model_pitviper;
      }
      else if (model_name_ == std::string("bt"))
      {
        model_ = &model_bt;
      }
    }

    valid_map_ = false;
    b_shutdown_ = false;

    //call worker thread
    client_thread_ = boost::thread(boost::bind(&KMOVehicleExecutionNode::run, this));
  }

  KMOVehicleExecutionNode()
  {
    b_shutdown_ = true;
    cond_.notify_one();
    client_thread_.join();
  }

  // Visualization code
  void visualizeCurrentMission()
  {

    boost::mutex::scoped_lock lock(current_mutex_);
    if (current_path_.sizePath() > 0)
    {
      orunav_generic::Path path = orunav_generic::minIncrementalDistancePath(current_path_, 0.1);
      orunav_rviz::drawPathInterface(path, "current_path_subsampled", 0, traj_params_.wheelBaseX, marker_pub_);
      orunav_rviz::drawPoint2dContainerAsConnectedLineIncZ_(orunav_generic::createPoint2dVecFromPose2dContainerInterface(path), "path_points", 1, 1, 0., 0., false, marker_pub_);

      if (current_cts_.size() == current_path_.sizePath() && vehicle_state_.isDriving())
      {
        orunav_rviz::drawCoordinatedTimes(current_path_, current_cts_, current_start_time_, "cts", 2, time_to_meter_factor_, 0.12, marker_pub_);
        orunav_rviz::drawPose2dTimesAsLines(current_path_, current_cts_, current_start_time_, "cts_lines", robot_id_, time_to_meter_factor_, 0.02, marker_pub_);
      }
    }

    // The trajectory
    orunav_generic::TrajectoryChunks chunks = vehicle_state_.getTrajectoryChunks();
    orunav_rviz::drawTrajectoryChunksWithControl(chunks, 2, "chunks", marker_pub_);

    // Draw the trajetory as a path / cts, each pose2d is separated with 60 ms.
    orunav_rviz::drawTrajectoryChunksUsingTime(chunks, 2, "chunks_dt", current_start_time_, time_to_meter_factor_, traj_params_.timeStep, marker_pub_);

    if (!visualize_sweep_and_constraints_)
      return;

    // The sweep area
    int current_path_idx = vehicle_state_.getCurrentPathIdx();

    if (draw_sweep_area_ && current_path_idx > 0 && current_path_idx < current_path_.sizePath() - 1)
    {
      orunav_geometry::Polygon current_sweep_area = constraint_extract::computeSweepArea(orunav_generic::selectPathIntervall(current_path_, current_path_idx, current_path_.sizePath()), *model_, vehicle_state_.getInternalState2d());
      orunav_rviz::drawPoint2dContainerAsConnectedLine(current_sweep_area, "sweep", 2, 2, marker_pub_);
    }

    // Constrains related
    if (current_constraints_path_.sizePath() != current_constraints_.size())
      return;

    // Draw all active constraints
    for (size_t i = 0; i < current_constraints_.size(); i++)
    {
      const constraint_extract::PolygonConstraint constraint = current_constraints_[i];
      orunav_generic::Pose2d pose = current_constraints_path_.getPose2d(i);

      if (constraint.isActive(pose))
      {
        orunav_rviz::drawPoint2dContainerAsConnectedLine(constraint.getInnerConstraint(), "constraints_xy", i, 0, marker_pub_);
        orunav_rviz::drawPose2d(orunav_generic::Pose2d(pose(0), pose(1), constraint.getThBounds()(0)), i, 3, 0.3, "constraints_th_left", marker_pub_);
        orunav_rviz::drawPose2d(orunav_generic::Pose2d(pose(0), pose(1), constraint.getThBounds()(1)), i, 3, 0.3, "constraints_th_right", marker_pub_);

        // Draw the outer polygon as well if available.
        if (i < current_constraints_outer_.size())
        {
          orunav_rviz::drawPoint2dContainerAsConnectedLine(current_constraints_outer_[i], "constraints_outer_xy", i, 2, marker_pub_);
          orunav_rviz::drawPoint2dContainerAsConnectedLine(constraint.getOuterConstraint(), "constraints_outer2_xy", i, 2, marker_pub_);
        }
      }
    }
  }

  void handle_perception(const ros::TimerEvent &event)
  {
    if (vehicle_state_.activatePerception())
    {
      turnOnPalletEstimation(vehicle_state_.getTask().target);
    }
    if (vehicle_state_.inactivatePerception())
    {
      turnOffPalletEstimation();
    }
  }

  void publish_report(const ros::TimerEvent &event)
  {
    orunav_msgs::RobotReport msg = vehicle_state_.getReport();
    msg.robot_id = robot_id_;
    report_pub_.publish(msg);
  }

  void publish_visualization_slow(const ros::TimerEvent &event)
  {
    if (!visualize_)
      return;

    visualizeCurrentMission();

    // Draw driven trajectory.
    orunav_generic::Trajectory traj = vehicle_state_.getDrivenTrajectory();
    orunav_generic::CoordinatedTimes ct = vehicle_state_.getDrivenTrajectoryTimes();

    assert(traj.sizePath() == ct.size());

    if (!vehicle_state_.isWaiting())
    {
      orunav_rviz::drawCoordinatedTimes(traj, ct, current_start_time_, "driven_ct", robot_id_, time_to_meter_factor_, marker_pub_);
      orunav_rviz::drawTrajectoryWithControl(traj, 0, 0, "driven_traj", marker_pub_);
    }
  }

  void updateSafetyZones()
  {
    if (!use_safetyregions_)
      return;

    if (!vehicle_state_.isDriving())
    {
      current_global_ebrake_area_.clear();
      current_global_slowdown_area_.clear();
      return;
    }

    inputs_mutex_.lock();
    unsigned int current_chunk_idx = vehicle_state_.getCurrentTrajectoryChunkIdx();
    unsigned int current_step_idx = vehicle_state_.getCurrentTrajectoryStepIdx();
    orunav_generic::TrajectoryChunks chunks = vehicle_state_.getTrajectoryChunks();
    orunav_generic::RobotInternalState2d robot_state = vehicle_state_.getInternalState2d();
    inputs_mutex_.unlock();

    unsigned int ebrake_chunk_idx = current_chunk_idx;
    unsigned int ebrake_step_idx = current_step_idx;
    unsigned int slowdown_chunk_idx = current_chunk_idx;
    unsigned int slowdown_step_idx = current_step_idx;

    // Safety ebrake zone
    orunav_generic::updateChunkIdxStepIdxGivenFutureTime(ebrake_chunk_idx, ebrake_step_idx, 10, traj_params_.timeStep, ebrake_lookahead_time_);
    // Slowdown zone
    if (vehicle_state_.isDrivingSlowdown())
    {
      orunav_generic::updateChunkIdxStepIdxGivenFutureTime(slowdown_chunk_idx, slowdown_step_idx, 10, traj_params_.timeStep, slowdown_drivingslow_lookahead_time_);
    }
    else
    {
      orunav_generic::updateChunkIdxStepIdxGivenFutureTime(slowdown_chunk_idx, slowdown_step_idx, 10, traj_params_.timeStep, slowdown_lookahead_time_);
    }
    orunav_generic::Trajectory ebrake = orunav_generic::trajectoryChunksInterfaceToTrajectory(chunks,
                                                                                              current_chunk_idx, current_step_idx, ebrake_chunk_idx, ebrake_step_idx);

    orunav_generic::Trajectory slowdown = orunav_generic::trajectoryChunksInterfaceToTrajectory(chunks, current_chunk_idx, current_step_idx, slowdown_chunk_idx, slowdown_step_idx);
    current_global_ebrake_area_ = constraint_extract::computeSweepArea(orunav_generic::minIncrementalDistancePath(ebrake, 0.5), *model_, robot_state);
    current_global_slowdown_area_ = constraint_extract::computeSweepArea(orunav_generic::minIncrementalDistancePath(slowdown, 0.5), *model_, robot_state);

    //    ROS_INFO_STREAM("current_chunk_idx : " << current_chunk_idx << " current_step_idx : " << current_step_idx << " ebrake_chunk_idx : " << ebrake_chunk_idx << " ebrake_step_idx : " << ebrake_step_idx << " slowdown_chunk_idx : " << slowdown_chunk_idx << " slowdown_step_idx : " << slowdown_step_idx << " ebrake.sizeTrajectory() : " << ebrake.sizeTrajectory());
  }

  void publish_visualization_fast(const ros::TimerEvent &event)
  {

    orunav_rviz::drawText(vehicle_state_.getCurrentState2d().getPose2d(),
                          vehicle_state_.getDebugStringExtended(),
                          "state_str", 1, 1, 0.8, 0.3, marker_pub_);

    orunav_rviz::drawText(vehicle_state_.getCurrentState2d().getPose2d(),
                          orunav_generic::toString(robot_id_),
                          "robot_id_str", 1, 3, 1.0, 2.0, marker_pub_);

    orunav_rviz::drawPosition(vehicle_state_.getCurrentState2d().getPose2d()[0],
                              vehicle_state_.getCurrentState2d().getPose2d()[1],
                              (ros::Time::now().toSec() - current_start_time_) * time_to_meter_factor_,
                              1,
                              marker_pub_);

    // Draw the safety zones.
    if (use_safetyregions_)
    {
      orunav_rviz::drawPoint2dContainerAsConnectedLine(current_global_ebrake_area_, "ebrake", 0, 0, marker_pub_);
      orunav_rviz::drawPoint2dContainerAsConnectedLine(current_global_slowdown_area_, "slowdown", 0, 1, marker_pub_);
    }
    orunav_generic::TrajectoryChunks chunks = vehicle_state_.getTrajectoryChunks();
    orunav_generic::Path path = vehicle_state_.getPath();
    if (!chunks.empty())
    {
#if 0
      {
        unsigned idx = vehicle_state_.getCurrentTrajectoryChunkIdx();
        ROS_INFO_STREAM("current trajectory chunk idx : " << idx);
        if (idx < chunks.size()) {
          orunav_generic::Pose2d p = chunks.getChunk(idx).getPose2d(0);
          orunav_rviz::drawPose2d(p, 1, 1, 2., std::string("current_chunk_idx"), marker_pub_);
        }
      }
      {
        unsigned idx = vehicle_state_.getEarliestConnectChunkIdx();
        ROS_INFO_STREAM("earliest trajectory chunk idx : " << idx);
        if (idx < chunks.size()) {
          orunav_generic::Pose2d p = chunks.getChunk(idx).getPose2d(0);
          orunav_rviz::drawPose2d(p, 1, 2, 2., std::string("earliest_connect_chunk_idx"), marker_pub_);
        }
      }
#endif
      {
        int current_path_idx = vehicle_state_.getCurrentPathIdx();
        //        ROS_INFO_STREAM("current_path_idx : " << current_path_idx);
        if (current_path_idx >= 0 && current_path_idx < path.sizePath())
        {
          orunav_generic::Pose2d p = path.getPose2d(current_path_idx);
          orunav_rviz::drawPose2d(p, 1, 1, 2., std::string("current_path_idx"), marker_pub_);
        }
      }
      {
        int earliest_path_idx = vehicle_state_.getEarliestPathIdxToConnect();
        //        ROS_INFO_STREAM("earliest_path_idx : " << earliest_path_idx);
        if (earliest_path_idx >= 0 && earliest_path_idx < path.sizePath())
        {
          orunav_generic::Pose2d p = path.getPose2d(earliest_path_idx);
          orunav_rviz::drawPose2d(p, 1, 0, 2., std::string("earliest_connect_path_idx"), marker_pub_);
        }
        int docking_path_idx = getDockingPathIdx(path, earliest_path_idx, min_docking_distance_, max_docking_distance_);
        //        ROS_INFO_STREAM("docking_path_idx : " << docking_path_idx);
        if (docking_path_idx >= 0 && docking_path_idx < path.sizePath())
        {
          orunav_generic::Pose2d p = path.getPose2d(docking_path_idx);
          orunav_rviz::drawPose2d(p, 1, 0, 2., std::string("docking_path_idx"), marker_pub_);
        }
      }
    }
    // Critical point
    {
      int critical_point_idx = vehicle_state_.getCriticalPointIdx();
      if (path.sizePath() > 0)
      {
        if (critical_point_idx < 0)
          critical_point_idx = path.sizePath() - 1;
        if (critical_point_idx < path.sizePath())
        {
          orunav_rviz::drawSphere(path.getPose2d(critical_point_idx)[0],
                                  path.getPose2d(critical_point_idx)[1],
                                  0.2,
                                  1,
                                  std::string("critical_point"),
                                  marker_pub_);
        }
      }
      else
      { // If the crit point is zero, no path is loaded to vehicle_state, draw the current state just for illustration.
        if (critical_point_idx == 0)
        {
          orunav_rviz::drawSphere(vehicle_state_.getCurrentState2d().getPose2d()[0],
                                  vehicle_state_.getCurrentState2d().getPose2d()[1],
                                  0.2,
                                  1,
                                  std::string("critical_point"),
                                  marker_pub_);
        }
      }
    }
  }

  bool validTask(const orunav_msgs::Task &task)
  {
    // Check the critical point idx. Must be within the path lenght (or == -1).
    if (task.criticalPoint == -1)
      return true;

    int path_size = task.path.path.size();
    if (task.criticalPoint >= path_size)
    {
      ROS_WARN("[KMOVehicleExecutionNode] RID:%d - invalid critical point idx:%d (size of path:%d)", robot_id_, task.criticalPoint, path_size);
      return false;
    }
    return true;
  }

  bool validTarget(const orunav_msgs::RobotTarget &target)
  {
    // ACTIVATE_SUPPORT_LEGS not valid as a start operation
    if (target.start_op.operation == target.start_op.ACTIVATE_SUPPORT_LEGS)
    {
      ROS_WARN("start_op.operation == start_op.ACTIVATE_SUPPORT_LEGS");
      return false;
    }
    return true;
  }

  // Service callbacks
  bool computeTaskCB(orunav_msgs::ComputeTask::Request &req,
                     orunav_msgs::ComputeTask::Response &res)
  {
    orunav_msgs::ComputeTaskStatus msg;
    msg.task_id = req.target.task_id;
    msg.status = orunav_msgs::ComputeTaskStatus::COMPUTE_TASK_START;
    compute_status_pub_.publish(msg);
    
    ROS_INFO_STREAM("computeTaskCB taskRID: " << req.target.robot_id << " node RID:" << robot_id_);
    if (!validTarget(req.target))
    {
      ROS_WARN("[KMOVehicleExecutionNode] RID:%d - invalid target(!)", robot_id_);
      res.result = 0;
      msg.status = orunav_msgs::ComputeTaskStatus::INVALID_TARGET;
      compute_status_pub_.publish(msg);
      return false;
    }

    ROS_INFO("[KMOVehicleExecutionNode] RID:%d - received computeTask", robot_id_);
    // Do we have a map available? TODO - this should be possible to be sent from the coordination as well(!).
    if (!valid_map_)
    {
      ROS_WARN("[KMOVehicleExecutionNode] RID:%d - empty map(!), cannot computeTask", robot_id_);
      res.result = 0;
      msg.status = orunav_msgs::ComputeTaskStatus::INVALID_MAP;
      compute_status_pub_.publish(msg);
      return false;
    }
    

    if (req.start_from_current_state)
    {
      if (!vehicle_state_.validCurrentState2d())
      {
        ROS_WARN("[KMOVehicleExecutionNode] RID:%d - start from current state2d, current state2d unknown, cannot computeTask", robot_id_);
        res.result = 0;
	msg.status = orunav_msgs::ComputeTaskStatus::INVALID_START;
	compute_status_pub_.publish(msg);
        return false;
      }
      current_mutex_.lock();
      req.target.start = orunav_conversions::createPoseSteeringMsgFromState2d(vehicle_state_.getCurrentState2d());
      current_mutex_.unlock();
    }

    orunav_msgs::RobotTarget target = req.target;
    map_mutex_.lock();

    //relevant if NOT use_vector_map_and_geofence_
    nav_msgs::OccupancyGrid map = current_map_;

    if (use_vector_map_and_geofence_)
    {
      //Get map from ac_mission_server here
      orunav_msgs::GetVectorMap vm_srv;
      ros::ServiceClient vm_client = nh_.serviceClient<orunav_msgs::GetVectorMap>("get_vector_map");
      if (vm_client.call(vm_srv))
      {
        ROS_INFO("[KMOVehicleExecutionNode] - get_vector_map successful");
	msg.status = orunav_msgs::ComputeTaskStatus::VECTOR_MAP_SERVICE_SUCCESS;
	compute_status_pub_.publish(msg);
      }
      else
      {
        ROS_ERROR("[KMOVehicleExecutionNode] - Failed to call service: get_vector_map");
	res.result = 0;
	msg.status = orunav_msgs::ComputeTaskStatus::VECTOR_MAP_SERVICE_FAILURE;
	compute_status_pub_.publish(msg);
        return false;
      }
      vector_map_ = vm_srv.response.vector_map;

      //Get geofence from ac_mission_server here
      orunav_msgs::GetGeoFence gf_srv;
      ros::ServiceClient gf_client = nh_.serviceClient<orunav_msgs::GetGeoFence>("get_geo_fence");
      if (gf_client.call(gf_srv))
      {
        ROS_INFO("[KMOVehicleExecutionNode] - get_geo_fence successful");
	msg.status = orunav_msgs::ComputeTaskStatus::GEOFENCE_CALL_SUCCESS;
	compute_status_pub_.publish(msg);
      }
      else
      {
        ROS_ERROR("[KMOVehicleExecutionNode] - Failed to call service: get_geo_fence");
	res.result = 0;
	msg.status = orunav_msgs::ComputeTaskStatus::GEOFENCE_CALL_FAILURE;
	compute_status_pub_.publish(msg);
        return false;
      }
      geofence_ = gf_srv.response.geo_fence;
    }

    map_mutex_.unlock();

    // Based on the loading operations update the map and goal / start poses.
    if (req.target.goal_op.operation == req.target.goal_op.LOAD_DETECT ||
        req.target.goal_op.operation == req.target.goal_op.LOAD)
    {
      // Update the map with the pallet footprint
      orunav_geometry::PalletModel2dWithState pm = getPalletModelFromRobotTarget(req.target, false);
      constraint_extract::addPolygonToOccupancyMap(pm.getPosePolygon(), map, 100);
      // Update the tgt pose, this will currently use the first suggested pose.
      orunav_generic::Pose2d pickup_pose;
      if (req.target.goal_op.operation == req.target.goal_op.LOAD_DETECT)
	pickup_pose = pm.getPickupPoses().getPose2d(0); // For a pallet - this will contain two poses...
      else // i.e. LOAD
	pickup_pose = pm.getPickupPoses().getPose2d(2); 
      ROS_INFO("Will pick up some goods. Changing the goal pose - ");
      ROS_INFO("(from) Goal :  [%f,%f,%f](%f)", req.target.goal.pose.position.x, req.target.goal.pose.position.y, tf::getYaw(req.target.goal.pose.orientation), req.target.goal.steering);
      target.goal.pose = orunav_conversions::createMsgFromPose2d(pickup_pose);
      ROS_INFO("(to) Goal :  [%f,%f,%f](%f)", target.goal.pose.position.x, target.goal.pose.position.y, tf::getYaw(target.goal.pose.orientation), target.goal.steering);
    }

    if (req.target.start_op.operation == req.target.start_op.UNLOAD)
    {
      // If we're going to unload at the start state -> then we need to force a backward movement and glue these together...
      // Unloading at the start state -> we need to move backwards
      orunav_geometry::PalletModel2dWithState pm = getPalletModelFromRobotTarget(req.target, true);
      constraint_extract::addPolygonToOccupancyMap(pm.getPosePolygon(), map, 100);
      // Update the tgt pose, this will currently use the first suggested pose.
      orunav_generic::Pose2d back_off_pose = pm.getPickupPoses().getPose2d(0); // For a pallet - this will contain two poses...

      ROS_INFO("Will back off from some goods. Changing the start pose - ");
      ROS_INFO("(from) Start :  [%f,%f,%f](%f)", req.target.start.pose.position.x, req.target.start.pose.position.y, tf::getYaw(req.target.start.pose.orientation), req.target.start.steering);
      target.start.pose = orunav_conversions::createMsgFromPose2d(back_off_pose);
      ROS_INFO("(to) Start :  [%f,%f,%f](%f)", target.start.pose.position.x, target.start.pose.position.y, tf::getYaw(target.start.pose.orientation), target.start.steering);
    }

    if (req.extra_obstacles.size() > 0)
    {
      ROS_WARN_STREAM( "extra obstacles: " << req.extra_obstacles.size() );
      // If we have extra obstacles, let's add them to the map
      //crate polygon poly via test/polygon2_test.cpp
      for (int i = 0; i < req.extra_obstacles.size(); i++)
      {
        orunav_generic::Point2dVec pts;
        for (int j = 0; j < req.extra_obstacles[i].points.size(); j++)
        {
          pts.push_back(Eigen::Vector2d(req.extra_obstacles[i].points[j].x, req.extra_obstacles[i].points[j].y));
        }
        orunav_geometry::Polygon poly(pts);
        constraint_extract::addPolygonToOccupancyMap(poly, map, 100);
        //for (unsigned int k = 0; k < poly.sizePoint2d(); k++) std::cout << "pts : " << poly.getPoint2d(k) << std::endl;
      }
    }

    orunav_generic::Path path;
    { // Get path service call related stuff goes here...
      orunav_msgs::GetPath srv;

      if (!use_vector_map_and_geofence_)
      {
        srv.request.map = map;
	planningmap_pub_.publish(map);
      }

      srv.request.target = target;

      if (use_vector_map_and_geofence_)
      {
        srv.request.vector_map = vector_map_;
        srv.request.geofence = geofence_;
	planningmap_pub_.publish(vector_map_);
	ROS_WARN_STREAM("Using geofence map.");
      }

      // Update the target goal pose and map based on the load operations

      srv.request.max_planning_time = 20.0; // TODO param
      // Need to package the target + the map and ask the motion planner.
      ros::ServiceClient client = nh_.serviceClient<orunav_msgs::GetPath>("get_path");

      if (client.call(srv))
      {
        ROS_INFO("[KMOVehicleExecutionNode] - get_path successful");
	msg.status = orunav_msgs::ComputeTaskStatus::PATH_PLANNER_SERVICE_SUCCESS;
	compute_status_pub_.publish(msg);
      }
      else
      {
        ROS_ERROR("[KMOVehicleExecutionNode] - Call to service get_path returns ERROR");
        if (!resolve_motion_planning_error_)
        {
	  res.result = 0;
	  msg.status = orunav_msgs::ComputeTaskStatus::PATH_PLANNER_SERVICE_FAILED;
	  compute_status_pub_.publish(msg);
          return false;
        }
      }

      if (!srv.response.valid)
      {
        ROS_WARN("[KMOVehicleExecutionNode] RID:%d - no path found(!), will attempt to generate another path", robot_id_);
        // Are the start / goal very close?
        if (getTargetStartGoalDistance(target) > sqrt(2) * map.info.resolution)
        { // if the targets are reasonable apart
          // This indicates that there is some real problems finding the path...
          ROS_ERROR("[KMOVehicleExecutionNode] RID:%d - target and goal is to far appart, the motion planner should have found a path", robot_id_);
          res.result = 0;
          msg.status = orunav_msgs::ComputeTaskStatus::PATH_PLANNER_FAILED;
	  compute_status_pub_.publish(msg);
          return false;
        }
        // If they are, try to use the driven path (if any) to generate a repositioning path...
        if (!getRepositioningPathMsgUsingDrivenPath(srv.response.path, target, vehicle_state_.getPathFromLastAssignedTask() /*getPath()*/,
                                                    1.0))
        {
          ROS_WARN("[KMOVehicleExecutionNode] RID:%d - failed to compute repositioning path", robot_id_);
          res.result = 0;
	  msg.status = orunav_msgs::ComputeTaskStatus::PATH_PLANNER_REPOSITIONING_FAILED;
	  compute_status_pub_.publish(msg);
          return false;
        }
        ROS_INFO("[KMOVehicleExecutionNode] - computed repositioning path based on previous path");
      }
      path = orunav_conversions::createPathFromPathMsgUsingTargetsAsFirstLast(srv.response.path);
    }
    // Remove duplicate points in the path.
    orunav_generic::makeValidPathForTrajectoryProcessing(path);
    // Make it less dense... important for the smoothing steps.
    //    path = orunav_generic::minIncrementalDistancePath(path, min_incr_path_dist_);
    path = orunav_generic::minIncrementalDistanceMinNbPointsPath(path, min_incr_path_dist_, min_nb_path_points_);
    ROS_INFO_STREAM("[KMOVehicleExecutionNode] - size of path : " << path.sizePath());

    // Perform smoothing
    orunav_msgs::GetPolygonConstraints srv_constraints;
    orunav_msgs::GetSmoothedPath srv_smoothedpath;

    // init resulting path to the unsmoothed one
    srv_smoothedpath.response.path = orunav_conversions::createPathMsgFromPathAndState2dInterface(path,
                                                                                                  orunav_conversions::createState2dFromPoseSteeringMsg(target.start),
                                                                                                  orunav_conversions::createState2dFromPoseSteeringMsg(target.goal));
    if (!no_smoothing_)
    {

      { // Compute the constraints goes here
        orunav_msgs::GetPolygonConstraints srv;
        srv.request.map = map;
        srv.request.path = orunav_conversions::createPathMsgFromPathAndState2dInterface(path,
                                                                                        orunav_conversions::createState2dFromPoseSteeringMsg(target.start),
                                                                                        orunav_conversions::createState2dFromPoseSteeringMsg(target.goal));

        ros::ServiceClient client = nh_.serviceClient<orunav_msgs::GetPolygonConstraints>("polygonconstraint_service");
        if (client.call(srv))
        {
          ROS_INFO("[KMOVehicleExecutionNode] - polygonconstraint_service - successfull");
	  msg.status = orunav_msgs::ComputeTaskStatus::POLYGONCONSTRAINT_SERVICE_SUCCESS;
	  compute_status_pub_.publish(msg);
        }
        else
        {
          ROS_ERROR("[KMOVehicleExecutionNode] - Failed to call service: PolygonConstraint");
	  res.result = 0;
	  msg.status = orunav_msgs::ComputeTaskStatus::POLYGONCONSTRAINT_SERVICE_FAILED;
	  compute_status_pub_.publish(msg);
          return false;
        }

        // Check that the constraints are valid / add valid flag in the msg.
        srv_constraints = srv;
      }

      {
        // Perform optimization
        orunav_msgs::GetSmoothedPath srv;
        srv.request.path = srv_constraints.request.path;
        srv.request.map = srv_constraints.request.map;
        srv.request.constraints = srv_constraints.response.constraints;

        ros::ServiceClient client = nh_.serviceClient<orunav_msgs::GetSmoothedPath>("get_smoothed_path");
        if (client.call(srv))
        {
          ROS_INFO("[KMOVehicleExecutionNode] - get_smoothed_path - successfull");
	  msg.status = orunav_msgs::ComputeTaskStatus::SMOOTHING_SERVICE_SUCCESS;
	  compute_status_pub_.publish(msg);
	}
        else
        {
          ROS_ERROR("[KMOVehicleExecutionNode] - Failed to call service: GetSmoothedPath");
	  res.result = 0;
	  msg.status = orunav_msgs::ComputeTaskStatus::SMOOTHING_SERVICE_FAILED;
	  compute_status_pub_.publish(msg);
	  return false;
        }
        srv_smoothedpath = srv;
        path = orunav_conversions::createPathFromPathMsg(srv.response.path);

        double max_steering_angle = M_PI / 2.;
        double max_dist_offset = 0.1;
        double max_heading_offset = 0.1;
        if (!orunav_generic::validSmoothedPath(path,
                                               orunav_conversions::createState2dFromPoseSteeringMsg(target.start),
                                               orunav_conversions::createState2dFromPoseSteeringMsg(target.goal),
                                               max_steering_angle,
                                               max_dist_offset,
                                               max_heading_offset))
        {
          ROS_ERROR("Invalid smoothed path(!)");

          std::cout << " path.getPose2d(0) : " << path.getPose2d(0) << std::endl;
          std::cout << " path.getPose2d(path.sizePath()-1) : " << path.getPose2d(path.sizePath() - 1) << std::endl;
          std::cout << " orunav_conversions::createState2dFromPoseSteeringMsg(target.start).getPose2d() : " << orunav_conversions::createState2dFromPoseSteeringMsg(target.start).getPose2d() << std::endl;
          std::cout << " orunav_conversions::createState2dFromPoseSteeringMsg(target.goal).getPose2d() : " << orunav_conversions::createState2dFromPoseSteeringMsg(target.goal).getPose2d() << std::endl;
          res.result = 0;
	  msg.status = orunav_msgs::ComputeTaskStatus::SMOOTHING_FAILED;
	  compute_status_pub_.publish(msg);
	  return false;
        }
        // The path smoother if enabled select different constraints.
        srv_constraints.response.constraints = srv_smoothedpath.response.constraints;
      }
    }

    if (req.target.start_op.operation == req.target.start_op.UNLOAD)
    {
      target.start;     // the docking pose
      req.target.start; // the actual drop pose

      orunav_generic::Path straight_path = orunav_generic::createStraightPathFromStartPose(orunav_conversions::createPose2dFromMsg(req.target.start.pose),
                                                                                           orunav_conversions::createPose2dFromMsg(target.start.pose),
                                                                                           0.1);
      addPathToPath(straight_path, path);
      path = straight_path;
      srv_smoothedpath.response.path = orunav_conversions::createPathMsgFromPathAndState2dInterface(path,
                                                                                                    orunav_conversions::createState2dFromPoseSteeringMsg(req.target.start),
                                                                                                    orunav_conversions::createState2dFromPoseSteeringMsg(req.target.goal));
    }

    if (req.target.goal_op.operation == req.target.goal_op.LOAD)
    {

      target.goal;     // the docking pose
      req.target.goal; // the forklift pose where the lift should be made

      orunav_generic::Path straight_path = orunav_generic::createStraightPathFromStartPose(orunav_conversions::createPose2dFromMsg(target.goal.pose),
                                                                                           orunav_conversions::createPose2dFromMsg(req.target.goal.pose),
                                                                                           0.1);
      addPathToPath(path, straight_path);
      srv_smoothedpath.response.path = orunav_conversions::createPathMsgFromPathAndState2dInterface(path,
                                                                                                    orunav_conversions::createState2dFromPoseSteeringMsg(req.target.start),
                                                                                                    orunav_conversions::createState2dFromPoseSteeringMsg(req.target.goal));
    }

    orunav_msgs::DeltaTVec dts;
    if (use_ct_ || provide_dts_)
    { // Get deltaT's service call related stuff goes here...

      // To perform the trajectory processing there is some requirements that the path has to fullfill.
      // Since there is 1:1 connection between the path indexes and the constraints this needs to be maintaned.
      constraint_extract::PolygonConstraintsVec constraints = orunav_conversions::createPolygonConstraintsVecFromRobotConstraintsMsg(srv_constraints.response.constraints);
      constraint_extract::makeValidPathConstraintsForTrajectoryProcessing(path, constraints);
      srv_constraints.response.constraints = orunav_conversions::createRobotConstraintsFromPolygonConstraintsVec(constraints);
      srv_smoothedpath.response.path = orunav_conversions::createPathMsgFromPathAndState2dInterface(path,
                                                                                                    orunav_conversions::createState2dFromPoseSteeringMsg(req.target.start),
                                                                                                    orunav_conversions::createState2dFromPoseSteeringMsg(req.target.goal));
      orunav_msgs::GetDeltaTVec srv;
      srv.request.path = srv_smoothedpath.response.path;
      srv.request.target = target;
      // Need to package the target + the map and ask the motion planner.
      ros::ServiceClient client = nh_.serviceClient<orunav_msgs::GetDeltaTVec>("deltatvec_service");

      if (client.call(srv))
      {
        ROS_INFO("[KMOVehicleExecutionNode] - deltatvec_service successful");
	msg.status = orunav_msgs::ComputeTaskStatus::DELTATVEC_SERVICE_SUCCESS;
	compute_status_pub_.publish(msg);
      }
      else
      {
        ROS_ERROR("[KMOVehicleExecutionNode] - Failed to call service: deltatvec_service");
        res.result = 0;
	msg.status = orunav_msgs::ComputeTaskStatus::DELTATVEC_SERVICE_FAILURE;
	compute_status_pub_.publish(msg);
	return false;
      }

      if (!srv.response.valid)
      {
        ROS_WARN("[KMOVehicleExecutionNode] RID:%d - couldn't find deltatvecs, cannot computeTask", robot_id_);
        res.result = 0;
	msg.status = orunav_msgs::ComputeTaskStatus::DELTATVEC_CONSTRAINT_FAILURE;
	compute_status_pub_.publish(msg);
        return false;
      }
      dts = srv.response.dts;
    }

#if 0 // Use the input directly
    inputs_mutex_.lock();
    current_constraints_path_ = path;
    path = orunav_generic::minIntermediateDirPathPoints(path);
    current_path_ = path;
    current_constraints_ = orunav_conversions::createPolygonConstraintsVecFromRobotConstraintsMsg(srv_constraints.response.constraints);
    inputs_mutex_.unlock();
    cond_.notify_one();
#endif

    // Packet the message and return it.
    res.task.target = req.target; // Simply return the original target (not the one with modified goal / start poses).
    res.task.path = srv_smoothedpath.response.path;
    res.task.constraints = srv_constraints.response.constraints;
    ROS_INFO_STREAM("KMO res.constraints.constraints_outer.size() : " << res.task.constraints.constraints_outer.size());
    ROS_INFO_STREAM("KMO res.constraints.constraints.size() : " << res.task.constraints.constraints.size());

    res.task.dts = dts;
    res.task.criticalPoint = -1;
    res.task.criticalRobotID = -1;
    res.result = 1;
    msg.status = orunav_msgs::ComputeTaskStatus::COMPUTE_TASK_SUCCESS;
    compute_status_pub_.publish(msg);
    return true;
  }

  bool executeTaskCB(orunav_msgs::ExecuteTask::Request &req,
                     orunav_msgs::ExecuteTask::Response &res)
  {
    // Check the current state of the vehicle
    ROS_INFO("[KMOVehicleExecutionNode] RID:%d [%d] - received executeTask (update:%d)", robot_id_, req.task.target.robot_id, (int)req.task.update);
    ROS_INFO("[KMOVehicleExecutionNode] RID:%d - next critical point (-1 == there is none) (:%d)", robot_id_, (int)req.task.criticalPoint);
    //    ROS_ERROR_STREAM("task.cts : " << req.task.cts);

    if (!vehicle_state_.isWaiting() && !req.task.update)
    {
      ROS_WARN("[KMOVechileExecutionNode] : not in WAITING state(!) this TASK will be IGNORED");
      return false;
    }
    
    if (req.task.update && vehicle_state_.brakeSentUsingServiceCall()) {
        ROS_INFO("[KMOVehicleExecutionNode] - Update and execute task. Calling RECOVER.");
	sendRecoverCommand(VehicleState::BrakeReason::SERVICE_CALL);
	vehicle_state_.setResendTrajectory(true);
    }

    // Any start operation?
    if (req.task.target.start_op.operation != req.task.target.start_op.NO_OPERATION)
    {
      ROS_WARN("start operation requested(!)");
    }
    ROS_INFO_STREAM( "executeTaskCB goal operation: " << req.task.target.goal_op.operation );

    // // Detect the load while driving - turn on the detection?
    // if (req.task.target.goal_op.operation == req.task.target.goal_op.LOAD_DETECT_DRIVING) {
    //   ROS_WARN("goal operation requested");
    //   if (req.task.target.goal_load.status != req.task.target.goal_load.EMPTY) {
    //     // Turn on the load detection

    //     orunav_msgs::ObjectPoseEstimation srv;
    //     srv.request.active = true;
    //     srv.request.pose = req.task.target.goal.pose;
    //     srv.request.object.type = req.task.target.goal_load.status;
    //     ros::ServiceClient client = nh_.serviceClient<orunav_msgs::ObjectPoseEstimation>(orunav_generic::getRobotTopicName(robot_id_, "/pallet_estimation_service"));
    //     if (client.call(srv)) {
    //       ROS_INFO("[KMOVehicleExecutionNode] - pallet_estimation_service - successfull");
    //     }
    //     else
    //     {
    //       ROS_ERROR("[KMOVehicleExecutionNode] - Failed to call service: pallet_estimation_service");
    //       return false;
    //     }
    //   }
    // }

    // Perform some additional tests?
    // The trajectory processor has some requirment on the path. Note, by altering the path the constraints also needs to change accordingly.
    //makeTaskValidForTrajectoryProcessing(req.task); // TODO - preferably make an own Task class when all this is handled...

    if (!validTask(req.task))
    {
      ROS_WARN_STREAM("[KMOVehicleExecutionNode] not a valid task(!) - will be IGNORED");
      return false;
    }

    inputs_mutex_.lock();
    vehicle_state_.update(req.task);
    inputs_mutex_.unlock();

    cond_.notify_one();
    return true;
  }

  bool brakeTaskCB(orunav_msgs::BrakeTask::Request &req,
		   orunav_msgs::BrakeTask::Response &res)
  {
    res.current_path_idx = vehicle_state_.getCurrentPathIdx();
    sendBrakeCommand(VehicleState::BrakeReason::SERVICE_CALL);
    return true;
  }
  
  // Processing callbacks from subscriptions
  void process_map(const nav_msgs::OccupancyGrid::ConstPtr &msg)
  {
    map_mutex_.lock();
    current_map_ = *msg;
    valid_map_ = true;
    map_mutex_.unlock();
  }

  void process_report(const orunav_msgs::ControllerReportConstPtr &msg)
  {

    if (use_forks_)
    {
      if (ros::Time::now().toSec() - last_process_fork_report_time_.toSec() > 5)
      {
        ROS_WARN_STREAM_THROTTLE(5, "[KMOVehicleExecution] - fork reports are not available (!), check use_fork flag?!? using topic : " << fork_report_sub_.getTopic());
      }
    }

    orunav_generic::State2d state = orunav_conversions::createState2dFromControllerStateMsg(msg->state);

    inputs_mutex_.lock();

    //    controller_status_ = msg->status;
    bool completed_target;
    vehicle_state_.update(msg, completed_target, use_forks_);
    inputs_mutex_.unlock();

    updateSafetyZones();

    // Separate thread / timer.
    if (vehicle_state_.getResendTrajectory())
    {
      ROS_ERROR("Resending the trajectory(!)");
      vehicle_state_.setResendTrajectory(false);
      cond_.notify_one();
    }

    if (completed_target)
    {
      // TODO: the vehicle coordinator is simplified and cannot queue up a list with tasks anymore
      // vehicle_state_.setDocking(false);
      ROS_INFO("[KMOVehicleExecutionNode] %d - target completed, time duration: %f", (int)robot_id_, ros::Time::now().toSec() - current_start_time_);
    }

    // Tracking performance check
    double tracking_error = 0.;
    if (msg->traj_values.size() > 0)
    {
      if (msg->traj_values[0].active && msg->traj_values[0].status == 1)
      {
        tracking_error = msg->traj_values[0].value;
      }
    }
    trackingErrorEBrake(tracking_error);
  }

  bool compute_active_docking_path(orunav_generic::Path &new_path, const orunav_generic::Pose2d &pallet_pose)
  {

    // This function modifies the existing path based on the current pallet pose estimate.
    // The goal is that this modifications of the path should be very limited and even possible to
    // run without the coordinator.

    // Some sanity checks
    if (!vehicle_state_.isCurrentChunkIdxValid())
    {
      ROS_ERROR("process_pallet_poses : chunk_idx is not valid");
      return false;
    }

    // For this to work we need the current chunk idx + offset to be valid. Where the offset if based on 1), when the chunk idx can be connected and a number of how many should be left before the end.
    {
      unsigned int chunk_idx = vehicle_state_.getCurrentTrajectoryChunkIdx() + 3; // This is the earliest we can connect to.
      unsigned int chunk_idx_to_end_margin = 5;                                   // After we connect we need some chunks to drive before the end.
      if (!vehicle_state_.isChunkIdxValid(chunk_idx + chunk_idx_to_end_margin + 1))
      {
        return false;
      }
    }
    if (!vehicle_state_.isCurrentPathIdxValid())
    {
      ROS_ERROR("process_pallet_poses : path_idx is not valid");
      return false;
    }

    // The path is only allowed to be altered when:
    // 1) the vehicle is driving in the correct distance
    // 2) the distance to the pickup point is > min_docking_distance_
    // 3) the distance to the pickup point is < max_docking_distance_
    // Find the corresponding path idx (if possible) when all the above statements holds.
    int earliest_path_idx = vehicle_state_.getEarliestPathIdxToConnect();
    ROS_INFO_STREAM("process_pallet_poses : earliest_path_idx : " << earliest_path_idx);
    if (earliest_path_idx < 0)
    {
      return false;
    }

    ROS_INFO_STREAM("process_pallet_poses : getCurrentPathIdx : " << vehicle_state_.getCurrentPathIdx());

    int path_idx = getDockingPathIdx(vehicle_state_.getPath(), earliest_path_idx, min_docking_distance_, max_docking_distance_);
    ROS_INFO_STREAM("process_pallet_poses : path_idx : " << path_idx);

    if (path_idx < 0)
      return false;

    // Additional checks goes here.
    ROS_INFO("[VehicleExecutionNode] RID:%d - check the target pose relative to the pallet estimation", robot_id_);

    orunav_generic::Path orig_path = vehicle_state_.getPath();
    orunav_geometry::PalletModel2dWithState pallet = vehicle_state_.getPalletModelLoadDetect();

    orunav_generic::Pose2dVec pick_poses_close = pallet.getPickupPosesClose();
    // Here only use the first one.
    orunav_generic::Pose2d pick_pose_close = pick_poses_close.getPose2d(0);
    // For the cititruck we can drive +5cm over the object centre (there will be 5cm spare space on the forks...however due to the overshooting that is a bit random this should probably increased even further...
    orunav_generic::Pose2d offset(overshoot_distance_, 0., 0.);
    orunav_generic::Pose2d end_path_pose = orunav_generic::addPose2d(pallet_pose, offset);
    orunav_generic::Path straight_path = orunav_generic::createStraightPathFromStartPose(pick_pose_close, end_path_pose, 0.1);

    bool send_target_failed = false;

    // Check that the distance is not to far off compared to the given target - in that case, send a failure target.
    {
      orunav_generic::Pose2dVec pick_poses = pallet.getPickupPoses();
      orunav_generic::Pose2d pick_pose = pick_poses.getPose2d(0);

      // Compare this with the last path point...
      orunav_generic::Pose2d path_pose = orig_path.getPose2d(orig_path.sizePath() - 1);

      double angular_diff = orunav_generic::getAngularNormDist(pick_pose, path_pose);
      double distance_diff = orunav_generic::getDistBetween(pick_pose, path_pose);
      orunav_generic::Pose2d pose_offset = orunav_generic::subPose2d(path_pose, pick_pose);
      double distance_diff_fwd = fabs(pose_offset[0]);  // Less important...
      double distance_diff_side = fabs(pose_offset[1]); // More important...

      if (angular_diff > max_target_angular_diff_)
      {
        send_target_failed = true;
        ROS_INFO("process_pallet_poses : angular_diff %f > (%f)", angular_diff, max_target_angular_diff_);
      }
      if (distance_diff > max_target_distance_diff_)
      {
        send_target_failed = true;
        ROS_INFO("process_pallet_poses : distance_diff %f > (%f)", distance_diff, max_target_distance_diff_);
      }
      if (distance_diff_fwd > max_target_distance_diff_fwd_)
      {
        send_target_failed = true;
        ROS_INFO("process_pallet_poses : distance_diff_fwd %f > (%f)", distance_diff_fwd, max_target_distance_diff_fwd_);
      }
      if (distance_diff_side > max_target_distance_diff_side_)
      {
        send_target_failed = true;
        ROS_INFO("process_pallet_poses : distance_diff_side %f > (%f)", distance_diff_side, max_target_distance_diff_side_);
      }
      if (send_target_failed)
      {
        return false;
      }
    }

    ROS_INFO("[VehicleExecutionNode] RID:%d - computing smoothed straight path", robot_id_);

    // Compute the path... - use the straight_path_service
    ros::ServiceClient client = nh_.serviceClient<orunav_msgs::GetSmoothedStraightPath>(orunav_generic::getRobotTopicName(robot_id_, "/get_smoothed_straight_path"));
    orunav_msgs::GetSmoothedStraightPath srv;

    orunav_generic::State2d start(orig_path.getPose2d(path_idx), orig_path.getSteeringAngle(path_idx));
    srv.request.start = orunav_conversions::createPoseSteeringMsgFromState2d(start);

    orunav_generic::State2d goal(pick_pose_close, 0.);
    srv.request.goal = orunav_conversions::createPoseSteeringMsgFromState2d(goal);

    srv.request.wheel_base = 1.19;
    srv.request.max_nb_opt_points = docking_max_nb_opt_points_;
    srv.request.resolution = 1. / (srv.request.max_nb_opt_points * 1.);

    if (client.call(srv))
    {
      orunav_generic::Path path_ext = orunav_conversions::createPathFromPathMsg(srv.response.path);
      // Check that the smoothed path is reasonable to drive... for example do we end up at the right pose?
      {
        // Ideally we should get an almost straight path ... a straight path is easy to drive while a lot of turning is not.
        double min_sa, max_sa;
        orunav_generic::getMinMaxSteeringAngle(path_ext, min_sa, max_sa);
        double range_sa = max_sa - min_sa;
        ROS_INFO("process_pallet_poses : range_steering_angle %f, min : %f, max %f", range_sa, min_sa, max_sa);
        if (range_sa > max_steering_range_smoothed_path_)
        {
          ROS_INFO("process_pallet_poses : range_steering_angle %f > (%f)", range_sa, max_steering_range_smoothed_path_);
          send_target_failed = true;
          return false;
        }
      }

      orunav_generic::Path path_final = orunav_generic::createStraightPathFromStartPose(goal.getPose2d(), pallet_pose, 0.05); // Go straight the very last bit...
      ROS_INFO("Got new path(!) -> based on smooth straight...");
      // Fuse the paths together...
      new_path = orunav_generic::selectPathIntervall(orig_path, 0, path_idx - 1);
      addPathToPath(new_path, path_ext);
      addPathToPath(new_path, path_final);
      new_path = orunav_generic::minIncrementalDistancePath(new_path, 0.0001);
      orunav_generic::savePathTextFile(new_path, "online_smoothed.path");

      orunav_rviz::drawPathInterface(path_ext, "path_smoother_straight_path", 0, srv.request.wheel_base, marker_pub_);
      orunav_rviz::drawPose2d(start.getPose2d(), 0, 1, 1., "path_smoother_straight_start_pose2d", marker_pub_);
      orunav_rviz::drawPose2d(goal.getPose2d(), 0, 1, 1., "path_smoother_straigh_goal_pose2d", marker_pub_);
      orunav_rviz::drawPathInterface(new_path, "modified_path", 0, srv.request.wheel_base, marker_pub_);

      return true;
    }
    else
    {
      ROS_ERROR("[VehicleExecutionNode] RID:%d - failed in computing a smoothed straigth path using the service...", robot_id_);
    }

    return false;
  }

  void process_pallet_poses(const orunav_msgs::ObjectPoseConstPtr &msg)
  {
    ROS_INFO("[KMOVehicleExecution] got pallet poses");
    // If we're not about to pick a pallet up that we need to detect - no need process this further.
    if (vehicle_state_.goalOperationLoadDetect())
    {
      // Add some checks on the pose.
      orunav_generic::Pose2d pallet_pose = orunav_conversions::createPose2dFromMsg(msg->pose.pose);
      ROS_INFO("[KMOVehicleExecution] : in LOAD_DETECT, got a pallet estimate.");

      // Need to move the vehicle state to waiting.
      if (!vehicle_state_.setPerceptionReceived())
      {
        ROS_ERROR("failed in setting perception received");
      }
      else
      {
        ROS_INFO("Vehicle state was set to PerceptionReceived");
      }

      // The pose is ok. Send a LOAD request with the pallet pose.
      orunav_msgs::Task task;
      {
        orunav_msgs::ComputeTask srv;

        srv.request.target = vehicle_state_.getTask().target;
        orunav_generic::State2d start = vehicle_state_.getCurrentGoalState();
        srv.request.target.start = orunav_conversions::createPoseSteeringMsgFromState2d(start);
        //srv.request.target.goal.pose = msg->pose.pose;
        // Compute the pose where we need to drive to (given the pallet pose).
        orunav_geometry::PalletModel2dWithState pm = getPalletModelFromRobotTarget(srv.request.target, false);
        pm.update(pallet_pose);
        orunav_generic::Pose2d end_path_pose = pm.getPickupPoseOffset();
        srv.request.target.goal.pose = orunav_conversions::createMsgFromPose2d(end_path_pose);
        srv.request.start_from_current_state = true; // in this case we're fine since we're standing still.
        srv.request.target.goal_op.operation = srv.request.target.goal_op.LOAD;

	ROS_INFO_STREAM( "New velocity constraints for slow picking: (" << max_linear_vel_pallet_picking_ << "," << max_rotational_vel_pallet_picking_ << "," << max_linear_vel_rev_pallet_picking_ << "," << max_rotational_vel_rev_pallet_picking_ << ")");
	vehicle_state_.setNewVelocityConstraints(max_linear_vel_pallet_picking_, max_rotational_vel_pallet_picking_, max_linear_vel_rev_pallet_picking_, max_rotational_vel_rev_pallet_picking_);
	
        if (computeTaskCB(srv.request,
                          srv.response))
        {
          ROS_INFO("[KMOVehicleExecution] - compute_task successful");
        }
        else
        {
          ROS_ERROR("[KMOVehicleExecution] - Failed to compute_task");
          return;
        }
        task = srv.response.task;
        ROS_INFO_STREAM("[KMOVehicleExecution] - compute_task return value : " << srv.response.result);
      }
      if (use_update_task_service_)
      {
        ROS_WARN("[KMOVehicleExecution] Pallet picking via coordinator (make sure it is running)!!");
        // Send the task to the coordinator
        ros::ServiceClient client = nh_.serviceClient<orunav_msgs::UpdateTask>("/coordinator/update_task");
        orunav_msgs::UpdateTask srv;
        srv.request.task = task;
	srv.request.task.update = false;

        if (client.call(srv))
        {
          ROS_INFO("[KMOVehicleExecution] - set(update)_task successful");
        }
        else
        {
          ROS_ERROR("[KMOVehicleExecution] - Failed to call service: set(update)_task");
          return;
        }
        ROS_INFO_STREAM("[KMOVehicleExecution] - set(update)_task return value : " << srv.response.result);
      }
      else
      {
        ROS_WARN("[KMOVehicleExecution] Bypassing coordinator!!");
        orunav_msgs::ExecuteTask srv;
        srv.request.task = task;
        executeTaskCB(srv.request,
                      srv.response);
      }
    }

    return;

//Code bellow is the old code from several year ago. please keep in case useful
#if 0
    if (!vehicle_state_.goalOperationLoadDetect()) {
      ROS_WARN("Receiving pallet pose estimates don't have a load/detect operation to perform");
      return;
    }

    ROS_INFO("process_pallet_poses : have goal operation load");
    if (vehicle_state_.isDocking()) // Perform a check that the poses are approx similar(!)
      return;
    ROS_INFO("process_pallet_poses : not in docking mode - trying to get there");
    if (vehicle_state_.getDockingFailed()) {
      ROS_INFO("process_pallet_poses : docking failed, wait until new task is set");
      return;
    }

    inputs_mutex_.lock();
    orunav_generic::Pose2d pallet_pose = orunav_conversions::createPose2dFromMsg(msg->pose.pose);
    vehicle_state_.setDockingPose(pallet_pose);
    inputs_mutex_.unlock();
    
    // Two options.
    // 1) we can modify the trajectory on the fly while driving.
    // 2) we need to perform a bigger adjustment and try again
    // - for each pickup task we need to have received at least one pallet pose estimate.

    orunav_generic::Path path;
    if (compute_active_docking_path(path, pallet_pose)) {

      vehicle_state_.setDocking(true);

      //      bool use_update_task_service_ = true;
      orunav_msgs::Task task = vehicle_state_.getTask();
      task.path = orunav_conversions::createPathMsgFromPathInterface(path);
      task.update = true;
      //      task.target.goal_op.operation = task.target.goal_op.LOAD;
      
      if (use_update_task_service_) {
        // Send the task to the coordinator
        orunav_msgs::SetTask srv;
        srv.request.task = task;
        ros::ServiceClient client = nh_.serviceClient<orunav_msgs::SetTask>("coodinator/update_task");
        if (client.call(srv)) {
          ROS_INFO("[KMOVehicleExecution] - update_task successful");
        }
        else
        {
          ROS_ERROR("[KMOVehicleExecution] - Failed to call service: update_task");
          return;
        }
        ROS_INFO_STREAM("[KMOVehicleExecution] - update_task return value : " << srv.response.result);
      }
      else {
        orunav_msgs::ExecuteTask srv;
        srv.request.task = task;
        executeTaskCB(srv.request,
                      srv.response);
      }

    }
    else {
      // Need to recompute the path using full steps... need to compute the paths first and then set it. Set the update flag to allow it to continue even if it is active.
      ROS_INFO("[KMOVehicleExecution] - need to recompute a new repositioning path");

      vehicle_state_.setDockingFailed(true);
      vehicle_state_.clearGoalOperation();

      orunav_msgs::Task task;
      {
        orunav_msgs::ComputeTask srv;

        srv.request.target = vehicle_state_.getTask().target;
        //        srv.request.target.start.pose = vehicle_state_.getTask().target.goal.pose; // This goal pose is the pallet pickup pose... should have the actual planned path pose.
        orunav_generic::State2d start = vehicle_state_.getCurrentGoalState();
        srv.request.target.start = orunav_conversions::createPoseSteeringMsgFromState2d(start);
        srv.request.target.goal.pose = msg->pose.pose; // the pallet pose
        srv.request.start_from_current_state = false; // since we're anyway cannot alter the path, drive until the previous goal.

        if (computeTaskCB(srv.request,
                          srv.response)) {
          ROS_INFO("[KMOVehicleExecution] - compute_task successful");
        }
        else
        {
          ROS_ERROR("[KMOVehicleExecution] - Failed to compute_task");
          return;
        }
        task = srv.response.task;
        ROS_INFO_STREAM("[KMOVehicleExecution] - compute_task return value : " << srv.response.result);
      }
      {
        // Send the task to the coordinator
        ros::ServiceClient client = nh_.serviceClient<orunav_msgs::SetTask>("/set_task");
        orunav_msgs::SetTask srv;
        srv.request.task = task;

        if (client.call(srv)) {
          ROS_INFO("[KMOVehicleExecution] - set_task successful");
        }
        else
        {
          ROS_ERROR("[KMOVehicleExecution] - Failed to call service: set_task");
          return;
        }
        ROS_INFO_STREAM("[KMOVehicleExecution] - set_task return value : " << srv.response.result);
      }
    }
#endif
  }

  void process_fork_report(const orunav_msgs::ForkReportConstPtr &msg)
  {

    last_process_fork_report_time_ = ros::Time::now();
    bool completed_target, move_forks, load;
    VehicleState::OperationState operation;
    inputs_mutex_.lock();
    vehicle_state_.update(msg, completed_target, move_forks, load, operation);
    if (completed_target)
    {
      if (vehicle_state_.getDockingFailed())
      {
        // TODO
        ROS_INFO("DOCKING FAILED!");
      }
      vehicle_state_.setDocking(false);
      vehicle_state_.clearTrajectoryChunks();
      vehicle_state_.clearCurrentPath();
    }
    inputs_mutex_.unlock();
    if (move_forks)
    {
      ROS_INFO("[KMOVehicleExecutionNode] %s - moving forks", orunav_node_utils::getIDsString(target_handler_.getLastProcessedID()).c_str());
      orunav_msgs::ForkCommand cmd;
      cmd.robot_id = robot_id_;

      if (operation == VehicleState::LOAD)
      {
        cmd.state.position_z = 0.1;
      }
      //else if (operation == VehicleState::UNLOAD)
      //{
      //  cmd.state.position_z = -0.1;
      //}
      else if (operation == VehicleState::ACTIVATE_SUPPORT_LEGS)
      {
        cmd.state.position_z = -0.1;
      }
      else
      {
        cmd.state.position_z = 0.0;
      }
      forkcommand_pub_.publish(cmd);
      ROS_INFO("[KMOVehicleExecutionNode] - set height %f", cmd.state.position_z ) ;
    }
  }

  // Only valid if only one laser scaner is used.
  void process_laserscan(const sensor_msgs::LaserScanConstPtr &msg)
  {

    if (!use_safetyregions_)
      return;

    // Only run this when driving - the vehicle is moving or if it is in brake state.
    // Verify the readings towards the ebrake and slowdown regions.
    // Would make sense to have a fix static ebrake region as well (since this is directly related to the trajectory sent to the controller), or to assume that this is properly handled by the tracking performance check.

    // 1) Compute the laser scan in global coords.
    // 2) Check the 2d points if they are inside the convexHull of the regions.
    if (!tf_listener_.waitForTransform(
            msg->header.frame_id,
            "/world",
            msg->header.stamp + ros::Duration().fromSec(msg->ranges.size() * msg->time_increment),
            ros::Duration(1.0)))
    {
      return;
    }

    sensor_msgs::PointCloud cloud, cloud_ebrake, cloud_slowdown, cloud_ignore;
    laser_projection_.transformLaserScanToPointCloud("/world", *msg,
                                                     cloud, tf_listener_);

    cloud_ebrake.header = cloud.header;
    cloud_slowdown.header = cloud.header;
    cloud_ignore.header = cloud.header;
    drawPointCloud(cloud, "laserscan_cloud", 0, 1, 0.05, marker_pub_);

    if (!vehicle_state_.isDriving())
    {
      return;
    }

    bool slowdown = false;
    bool sendbrake = false;

    int start_idx = 0;
    int stop_idx = 0;
    if (real_cititruck_)
    {
      start_idx = 70;
      stop_idx = 70;
    }

    for (size_t i = start_idx; i < cloud.points.size() - stop_idx; i++)
    {
      // Self occlusion, for simulation using the nav laser, don't fully get why they have to be this wide the ignore area but otherwise it will detect the frame!
      if (!real_cititruck_)
      {
        if (i >= 205 && i <= 240)
        {
          cloud_ignore.points.push_back(cloud.points[i]);
          continue;
        }
        if (i >= 390 && i <= 425)
        {
          cloud_ignore.points.push_back(cloud.points[i]);
          continue;
        }
      }
      if (current_global_ebrake_area_.collisionPoint2d(Eigen::Vector2d(cloud.points[i].x,
                                                                       cloud.points[i].y)))
      {
        ROS_INFO_STREAM("e-brake area collision point " << i);
        cloud_ebrake.points.push_back(cloud.points[i]);
        sendbrake = true;
        continue;
      }
      if (current_global_slowdown_area_.collisionPoint2d(Eigen::Vector2d(cloud.points[i].x,
                                                                         cloud.points[i].y)))
      {
        //ROS_INFO_STREAM("slowdown collision point " << i);
        cloud_slowdown.points.push_back(cloud.points[i]);
        slowdown = true;
      }
    }

    drawPointCloud(cloud_slowdown, "laserscan_cloud_slowdown", 0, 2, 0.1, marker_pub_);
    drawPointCloud(cloud_ebrake, "laserscan_cloud_ebrake", 0, 0, 0.1, marker_pub_);
    drawPointCloud(cloud_ignore, "laserscan_cloud_ignore", 0, 0, 0.15, marker_pub_);

    if (sendbrake)
    {
      ROS_INFO_STREAM("Sending BRAKE - laser reading in e-brake zone detected");
      if (!vehicle_state_.isBraking())
      {
        sendBrakeCommand(VehicleState::BrakeReason::SENSOR);
      }
      return;
    }
    if (!slowdown)
    {
      if (vehicle_state_.isBraking())
      {

        // Recover.
        sendRecoverCommand(VehicleState::BrakeReason::SENSOR);
        ROS_INFO_STREAM("braking - slowdown area is cleared - recovering");
        // From recovering the state is back to DRIVING.
      }
      if (vehicle_state_.isDrivingSlowdown())
      {
        // Recompute the speed... no point in driving slow.
        vehicle_state_.setDrivingSlowdown(false);
        cond_.notify_one();
        ROS_INFO_STREAM("slowdown mode - slowdown area is cleared - speed up");
        return;
      }
    }
    else
    {
      if (!vehicle_state_.isDrivingSlowdown())
      {
        // Recompute the speed.
        vehicle_state_.setDrivingSlowdown(true);
        cond_.notify_one();
        ROS_INFO_STREAM("normal mode - slowdown area is occupied - entering slowdown mode");
        return;
      }
    }
  }

  void process_velocity_constraints(const std_msgs::Float64MultiArrayConstPtr &msg)
  {
    if (msg->data.size() != 4)
    {
      ROS_ERROR_STREAM("Wrong format on /velocity_constraints topic");
      return;
    }
    double max_linear_velocity_constraint = msg->data[0];
    double max_rotational_velocity_constraint = msg->data[1];
    double max_linear_velocity_constraint_rev = msg->data[2];
    double max_rotational_velocity_constraint_rev = msg->data[3];

    vehicle_state_.setNewVelocityConstraints(max_linear_velocity_constraint, max_rotational_velocity_constraint, max_linear_velocity_constraint_rev, max_rotational_velocity_constraint_rev);
    ROS_INFO_STREAM("New velocity constraint (fwd) [linear: " << msg->data[0] << " | rot: " << msg->data[1] << "] (rev) [linear: " << msg->data[2] << " | rot: " << msg->data[3] << "]");
    if (vehicle_state_.newVelocityConstraints())
    {
      cond_.notify_one();
    }
  }

  void process_ebrake(const orunav_msgs::EBrakeConstPtr &msg)
  {

    if (msg->robot_id != robot_id_)
    {
      ROS_ERROR_STREAM("wrong robot_id");
    }

    // Request to brake or recover?
    if (msg->recover)
    {
      ebrake_id_set_.erase(msg->sender_id);
      if (ebrake_id_set_.empty())
      {
        sendRecoverCommand(VehicleState::BrakeReason::TOPIC_CALL);
      }
      return;
    }

    // Are we already in brake state? That is don't send yet another ebrake command but add to the ebrake sender id set.
    ebrake_id_set_.insert(msg->sender_id);
    if (vehicle_state_.isBraking())
    {
      return;
    }
    sendBrakeCommand(VehicleState::BrakeReason::TOPIC_CALL);
  }

  bool turnOnPalletEstimation(const orunav_msgs::RobotTarget &target)
  {
    // Turn on the load detection
    orunav_msgs::ObjectPoseEstimation srv;
    srv.request.active = true;
    srv.request.pose = target.goal.pose;
    srv.request.object.type = target.goal_load.status;
    ros::ServiceClient client = nh_.serviceClient<orunav_msgs::ObjectPoseEstimation>(orunav_generic::getRobotTopicName(robot_id_, "/pallet_estimation_service"));
    if (client.call(srv))
    {
      ROS_INFO("[KMOVehicleExecutionNode] - pallet_estimation_service (started) - successfull");
    }
    else
    {
      ROS_ERROR("[KMOVehicleExecutionNode] - Failed to call service: pallet_estimation_service");
      return false;
    }
  }

  bool turnOffPalletEstimation()
  {
    orunav_msgs::ObjectPoseEstimation srv;
    srv.request.active = false;
    ros::ServiceClient client = nh_.serviceClient<orunav_msgs::ObjectPoseEstimation>(orunav_generic::getRobotTopicName(robot_id_, "/pallet_estimation_service"));
    if (client.call(srv))
    {
      ROS_INFO("[KMOVehicleExecutionNode] - pallet_estimation_service (stopped) - successfull");
    }
    else
    {
      ROS_ERROR("[KMOVehicleExecutionNode] - Failed to call service: pallet_estimation_service");
      return false;
    }
    return true;
  }

  void sendBrakeCommand(VehicleState::BrakeReason reason)
  {
    ROS_INFO("[KMOVehicleExecutionNode] RID:%d - sending brake command", robot_id_);
    orunav_msgs::ControllerCommand command;
    command.robot_id = robot_id_;
    command.command = command.COMMAND_BRAKE;
    command_pub_.publish(command);
    vehicle_state_.brakeSent(reason);
    usleep(5000);
  }

  void sendRecoverCommand(VehicleState::BrakeReason reason)
  {
    vehicle_state_.brakeClear(reason);
    if (!vehicle_state_.allBrakeReasonsCleared())
    {
      return;
    }
    if (!ebrake_id_set_.empty())
    {
      return;
    }

    ROS_INFO("[KMOVehicleExecutionNode] RID:%d - sending recover command", robot_id_);
    orunav_msgs::ControllerCommand command;
    command.robot_id = robot_id_;
    command.command = command.COMMAND_RECOVER;
    command_pub_.publish(command);
    usleep(5000);
  }

  void trackingErrorEBrake(double trackingError)
  {
    if (max_tracking_error_ < 0)
      return;
    if (trackingError > max_tracking_error_)
    {
      ROS_INFO("[KMOVehicleExecutionNode] RID:%d - tracking error too high (%f > %f) - BRAKE.", robot_id_, trackingError, max_tracking_error_);
      sendBrakeCommand(VehicleState::BrakeReason::TRACKING_ERROR);
    }
  }

  void waitForController()
  {

    ROS_INFO("[KMOVehicleExecutionNode] RID:%d - waiting for controller", robot_id_);
    // Wait in case the controller hasn't connected.
    while (trajectorychunk_pub_.getNumSubscribers() == 0)
    {
      usleep(1000);
    }

    while (command_pub_.getNumSubscribers() == 0)
    {
      usleep(1000);
    }
    ROS_INFO("[KMOVehicleExecutionNode] RID:%d - waiting for controller - done", robot_id_);
  }

  void sendActivateStartTimeCommand(const ros::Time &startTime)
  {

    if (vehicle_state_.canSendActivate())
    {
      orunav_msgs::ControllerCommand command;
      command.robot_id = robot_id_;
      command.traj_id = 0;
      command.command = command.COMMAND_ACTIVATE;
      command_pub_.publish(command);

      usleep(5000);

      command.command = command.COMMAND_STARTTIME;
      command.start_time = startTime;

      if (command.start_time < ros::Time::now())
      {
        ROS_WARN("[KMOVehicleExecutionNode] - command start time < current time : %f secs.", (ros::Time::now() - command.start_time).toSec());
        ROS_WARN_STREAM("[KMOVehicleExecutionNode] - current time: " << ros::Time::now().toSec()
                                                                     << " old command start time: " << command.start_time.toSec());
        //exit(-1); // Kill it! -> should not happen.
        command.start_time = ros::Time::now();
        ROS_WARN_STREAM("[KMOVehicleExecutionNode] - new command start time: " << command.start_time.toSec());
      }
      vehicle_state_.setTrajectoryChunksStartTime((command.start_time).toSec());
      current_start_time_ = command.start_time.toSec();

      ROS_INFO("[KMOVehicleExecutionNode] - command start time : %f", command.start_time.toSec());
      command_pub_.publish(command);

      usleep(5000);
    }
    ROS_INFO("[KMOVehicleExecutionNode] done sending controller command");
  }

  void sendTrajectoryChunks(const std::pair<unsigned int, orunav_generic::TrajectoryChunks> &chunks_data)
  {

    // Add the chunks
    inputs_mutex_.lock();
    vehicle_state_.appendTrajectoryChunks(chunks_data.first, chunks_data.second);
    vehicle_state_.saveCurrentTrajectoryChunks("current_chunks.txt");
    saveTrajectoryChunksTextFile(chunks_data.second, "chunks_data_to_be_added.txt");
    // Make sure that the ones that are in vehicle state are the ones that are sent to the controller....
    orunav_generic::TrajectoryChunks chunks = vehicle_state_.getTrajectoryChunks();
    ROS_INFO_STREAM("[KMOVehicleExecutionNode] - appended chunks, index: " << chunks_data.first);
    ROS_INFO("[KMOVehicleExecutionNode] - appended chunks, size : %lu", chunks.size());
    inputs_mutex_.unlock();

    // -----------------------------------------------------------------
    // Mutex off - only local variables from now on in the communication
    // -----------------------------------------------------------------
    waitForController();

    std::vector<double> A0, A1, b;

    orunav_msgs::ControllerTrajectoryChunkVec c_vec;
    // Send them off
    for (unsigned int i = chunks_data.first; i < chunks.size(); i++)
    {
      orunav_msgs::ControllerTrajectoryChunk c = orunav_conversions::createControllerTrajectoryChunkFromTrajectoryInterface(chunks[i]);

      c.robot_id = robot_id_;
      c.traj_id = 0;
      c.sequence_num = i;
      c.final = false;
      if (i == chunks.size() - 1)
        c.final = true;
      c_vec.chunks.push_back(c);
    }

    trajectorychunk_pub_.publish(c_vec);
    ROS_INFO("[KMOVehicleExecutionNode] - sending chunk size: %lu", c_vec.chunks.size());
  }

  // This takes care of computing/updating the reference trajectory followed by the controller node.
  void run()
  {

    while (!b_shutdown_)
    {
      //usleep(100000);
      ROS_INFO("[KMOVehicleExecutionNode] trajectory update thread - going to sleep : %s", vehicle_state_.getDebugString().c_str());

      boost::unique_lock<boost::mutex> uniqueLock(run_mutex_);
      cond_.wait(uniqueLock);

      ROS_INFO("[KMOVehicleExecutionNode] waking up, vehicle state : %s", vehicle_state_.getDebugString().c_str());
      // Need to be in WAIT state or ACTIVE state
      if (!vehicle_state_.canSendTrajectory())
      { // This handle the previous problem if many coordination times are sent at once (WAITING_TRAJECTORY_SENT)
        ROS_INFO("[KMOVehicleExecutionNode] - cannot send trajectory (wrong state)");
        usleep(100000);
        vehicle_state_.setResendTrajectory(true);
        continue;
      }

      if (vehicle_state_.getCriticalPointIdx() >= 0 && vehicle_state_.getCriticalPointIdx() < 2)
      {
        ROS_WARN_STREAM("[KMOVehicleExecutionNode] critical point is too close to the starting pose... - will NOT start");
        // This is perfecly ok, AT_CRITICAL_POINT and WAITING_FOR_TASK is the same state but with different names. Make sure that the controller is waiting.
        if (vehicle_state_.isWaiting())
        {
          vehicle_state_.setAtCriticalPoint();
        }
        continue;
      }
      if (vehicle_state_.getCurrentPathIdx() > 0)
      {
        if (vehicle_state_.getCriticalPointIdx() < vehicle_state_.getCurrentPathIdx() + 2)
        {
          ROS_WARN_STREAM("[KMOVehicleExecutionNode] critical point is too close to the starting pose... - will NOT start (path idx : " << vehicle_state_.getCurrentPathIdx() << ", crit point : " << vehicle_state_.getCriticalPointIdx() << ")");
          continue;
        }
      }

      orunav_msgs::Task task = vehicle_state_.getTask();
      orunav_generic::Path path = orunav_conversions::createPathFromPathMsg(task.path);

      orunav_generic::CoordinatedTimes cts;
      if (task.cts.ts.size() > 0)
        cts = orunav_conversions::getCoordinatedTimesFromCoordinatorTimeMsg(task.cts.ts[0]); // [0] -> this is still a vector (always lenght 1?) for old reasons
      if (cts_clear_first_entry_in_pairs_)
      {
        cts.clearFirstEntryInPairs();
        // for (size_t i = 0; i < cts.size(); i++) {
        //   std::cerr << "cts[" << i << "] : " << cts[i] << std::endl;
        // }
      }

      std::pair<unsigned int, orunav_generic::TrajectoryChunks> chunks_data;
      traj_params_.debugPrefix = std::string("coord_time/");

      ROS_INFO("--mutex lock--");
      inputs_mutex_.lock();
      current_constraints_path_ = path;

      // TODO - this will affect the order of the constraints, they are not currently used anyway.
      orunav_generic::makeValidPathForTrajectoryProcessing(path);

      if (!orunav_generic::validPathForTrajectoryProcessing(path))
      {
        ROS_ERROR("KMOVehicleExecution - INVALIDPATH quit(!) this SHOULD never happen");
        exit(-1);
      }
      else
      {
        ROS_INFO("VALIDPATH");
      }

      orunav_generic::savePathTextFile(path, "rid" + orunav_generic::toString(task.target.robot_id) + "_task" + orunav_generic::toString(task.target.task_id) + ".path");
      current_path_ = path;
      current_constraints_ = orunav_conversions::createPolygonConstraintsVecFromRobotConstraintsMsg(task.constraints);
      current_constraints_outer_ = orunav_conversions::createConvexOuterPolygonsFromRobotConstraintsMsg(task.constraints);
      current_target_ = task.target;
      current_cts_ = cts;
      inputs_mutex_.unlock();

      ///////////////////////////////////////////////////////////////////
      // CASE 1:
      ///////////////////////////////////////////////////////////////////
      if (vehicle_state_.vehicleStoppedAndTrajectoryNotCompleted() || (vehicle_state_.isWaitingTrajectoryEmpty() && vehicle_state_.hasActiveTaskCriticalPoint()))
      {
        // Waiting - need to ship a trajectory off from the current location with zero speed.
        ROS_INFO("[KMOVehicleExecutionNode] - CASE1");

        unsigned int path_idx;
        vehicle_state_.setPath(path);
        chunks_data = computeTrajectoryChunksCASE1(vehicle_state_, traj_params_, path_idx, use_ct_);
        if (chunks_data.second.empty())
        {
          ROS_WARN_STREAM("[KMOVehicleExecutionNode] CASE1 - couldn't compute trajectory, path size is to small... (path idx : " << path_idx << ", crit point : " << vehicle_state_.getCriticalPointIdx() << ")");
          // Move from "at critical point" to "waiting for task". Currently this is how the coordinator is aborting the current task.
          vehicle_state_.abortTask();
          continue;
        }
        vehicle_state_.setCurrentPathIdx(path_idx);

        // Important the chunks will need to be re-indexed from 0.
        // It is currently only CASE2 which use on-the-fly modification of an active trajectory.
        vehicle_state_.clearTrajectoryChunkIdx();
      }

      ///////////////////////////////////////////////////////////////////
      // CASE 2:
      ///////////////////////////////////////////////////////////////////
      else if (vehicle_state_.isActive())
      {
        ROS_INFO("[KMOVehicleExecutionNode] - CASE2, need to send an updated trajectory for goalID %d", current_target_.goal_id);
        unsigned int path_idx;
        // idx when we can safely connect
        unsigned int chunk_idx = vehicle_state_.getCurrentTrajectoryChunkIdx() + chunk_idx_connect_offset_; // This is the earliest we can connect to.
        unsigned int chunk_idx_to_end_margin = 5;                                                           // After we connect we need some chunks to drive before the end.

        bool new_path_is_shorter = (path.sizePath() < vehicle_state_.getPath().sizePath());
        if (new_path_is_shorter)
        {
          chunk_idx_to_end_margin = 1;
          ROS_INFO_STREAM("[KMOVehicleExecution] - SIZE of paths: new:" << path.sizePath() << " old:" << vehicle_state_.getPath().sizePath() << " curretn IDX:" << vehicle_state_.getCurrentPathIdx());
        }
        if (new_path_is_shorter || vehicle_state_.isChunkIdxValid(chunk_idx + chunk_idx_to_end_margin))
        {
          // Two options
          // 1) the new path start is the same as the prev path start
          // 2) the new path start is the same as the prev path goal
          // In case of 1), we're all fine to continue...
          // In case of 2), we need to connect the provided path with the current path.
          ROS_INFO_STREAM("[KMOVehicleExecution] - (+3) chunk_idx: valid : " << chunk_idx);
          double path_chunk_distance;
          vehicle_state_.updatePath(path);
          if (use_ct_)
          {
            ROS_INFO_STREAM("Adding cts: " << cts.size());
            ROS_INFO_STREAM("Path size : " << path.sizePath());
            vehicle_state_.setCoordinatedTimes(cts);
          }
          if (vehicle_state_.isDrivingSlowdown())
          {
            ROS_INFO_STREAM("[KMOVehicleExecution] - will drive in slowdown mode - ignoring CTs");
            vehicle_state_.clearCoordinatedTimes();
            bool valid = false;
            chunks_data = computeTrajectoryChunksCASE2(vehicle_state_, traj_slowdown_params_, chunk_idx, path_idx, path_chunk_distance, valid, use_ct_);
            if (!valid)
            {
              continue;
            }
            // Special case - clear the CT's -> however, should check that the CTS are not slower than the slowdown...
          }
          else if (vehicle_state_.newVelocityConstraints())
          {
            ROS_INFO_STREAM("[KMOVehicleExecution] - got new velocity constraints");
            TrajectoryProcessor::Params traj_params = traj_params_original_;
	    updateTrajParamsWithVelocityConstraints(traj_params, vehicle_state_);
            ROS_INFO_STREAM("new trajectory params: " << traj_params);

            // Overwrite the default velocity constratins
            if (overwrite_traj_params_with_velocity_constraints_)
            {
              traj_params_ = traj_params;
            }

            bool valid = false;
            chunks_data = computeTrajectoryChunksCASE2(vehicle_state_, traj_params, chunk_idx, path_idx, path_chunk_distance, valid, use_ct_);
            if (!valid)
            {
              continue;
            }
            vehicle_state_.resetNewVelocityConstraint();
          }
          else
          {
            bool valid;
            chunks_data = computeTrajectoryChunksCASE2(vehicle_state_, traj_params_, chunk_idx, path_idx, path_chunk_distance, valid, use_ct_);
            if (!valid)
            {
              continue;
            }
          }
	  #if 0
          if (use_ct_ && use_ahead_brake_)
          {
            double ahead_time;
            if (cts.isAhead(path_idx, ros::Time::now().toSec(), ahead_time))
            {
              ROS_WARN_STREAM("[VehicleExecutionNode] - is ahead : " << ahead_time);
              if (ahead_time > 3.)
              {
		//                sendBrakeCommand(false); // CTS
              }
              continue;
            }
          }
	  #endif
          ROS_INFO_STREAM("[VehicleExecutionNode] - distance between the connected path state and the chunk_idx used : " << path_chunk_distance);
          // Check the distance > threshold (could also be to check what control input is required to bring it from state chunk to state path and to check if this is reasonable rather then a simple distance check...).
          vehicle_state_.setCurrentPathIdx(path_idx);
        }
        else
        {
          // The vehicle is simply to close to the final goal...
          // Instead of stopping, simply retry until we get into another state.
          ROS_INFO_STREAM("[KMOVehicleExecutionNode] - too close to the goal, will try again");
          usleep(1000000);
          vehicle_state_.setResendTrajectory(true);
          continue;
        }
      }
      ///////////////////////////////////////////////////////////////////
      // CASE 3:
      ///////////////////////////////////////////////////////////////////
      else if (vehicle_state_.isWaitingTrajectoryEmpty())
      {
        ROS_INFO("[KMOVehicleExecutionNode] - CASE3, great - goalID is %d", current_target_.goal_id);
        // Is the start distance close enought with the current distance?
        double dist = orunav_generic::getDistBetween(vehicle_state_.getCurrentState2d().getPose2d(),
                                                     path.getPose2d(0));
        if (dist > 1.2)
        { // TODO param (this looks like a very magic number :-))
          ROS_WARN(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> [KMOVehicleExecutionNode] - really off start pose, will not try to connect to along a path in WAIT mode (%f)", dist);
          continue;
        }

        vehicle_state_.setPath(path);
        if (use_ct_)
        {
          vehicle_state_.setCoordinatedTimes(cts);
        }
        ROS_INFO("Computing trajectory");
	if (vehicle_state_.newVelocityConstraints()) {
	  ROS_INFO_STREAM("[KMOVehicleExecution] - got new velocity constraints");
	  TrajectoryProcessor::Params traj_params = traj_params_original_;
	  updateTrajParamsWithVelocityConstraints(traj_params, vehicle_state_);
	  
	  ROS_INFO_STREAM("new trajectory params: " << traj_params);
	  
	  // Overwrite the default velocity constratins
	  if (overwrite_traj_params_with_velocity_constraints_) {
	    traj_params_ = traj_params;
	  }
	  chunks_data = computeTrajectoryChunksCASE3(vehicle_state_, traj_params, use_ct_);
	  vehicle_state_.resetNewVelocityConstraint();
	}
	else {
	  chunks_data = computeTrajectoryChunksCASE3(vehicle_state_, traj_params_, use_ct_);
	}
        ROS_INFO("Computing trajectory - done");

        // Do we need to perform any operations at start? (note this is the only case when we should do any start operations...).
        if (vehicle_state_.performStartOperation())
        {
          usleep(100000);
          vehicle_state_.setResendTrajectory(true);
          continue;
        }
      }

      ///////////////////////////////////////////////////////////////////
      // CASE 4:
      ///////////////////////////////////////////////////////////////////
      else if (vehicle_state_.brakeSentUsingServiceCall()) {
	ROS_INFO("[KMOVehicleExecutionNode] - CASE4, goalID is %d", current_target_.goal_id);
	ROS_ERROR("[KMOVehicleExecutionNode] - waiting the recovery will be triggered.");
	continue;
      }
      
       ///////////////////////////////////////////////////////////////////
      // CASE 5:
      ///////////////////////////////////////////////////////////////////
      else
      {
        ROS_INFO("[KMOVehicleExecutionNode] - CASE5, goalID is %d", current_target_.goal_id);
	ROS_ERROR("[KMOVehicleExecutionNode] - in wrong STATE(!) - should never happen");
      }

      //-----------------------------------------------------------------
      vehicle_state_.activateTask();

      // Add a check whether to brake due to slow speeds.
      // If the speed after breaking gets high enough wake up / unbreak.

      ros::Time start_time;
      double start_time_d = vehicle_state_.getCoordinatedStartTime();
      if (start_time_d < 0)
      {
        start_time = ros::Time::now() + ros::Duration(0.5);
        if (use_ct_)
        {
          ROS_WARN("[KMOVehicleExecutionNode] - start time is not coordinated");
        }
      }
      else
      {
        start_time = ros::Time(start_time_d);
      }
      sendTrajectoryChunks(chunks_data);
      vehicle_state_.trajectorySent();
      if (!vehicle_state_.isActive())
      {
        sendActivateStartTimeCommand(start_time);
      }
    } // while

    ROS_ERROR("[KMOVehicleExecutionNode] trajectory update thread - died(!) : %s", vehicle_state_.getDebugString().c_str());
  }
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "orunav_vehicle_execution_node");
  ros::NodeHandle params("~");

  KMOVehicleExecutionNode ve(params);

  ros::spin();
}
