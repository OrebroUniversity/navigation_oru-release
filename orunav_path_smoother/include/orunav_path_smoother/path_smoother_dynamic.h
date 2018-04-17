#pragma once

#include <acado_toolkit.hpp>
#include <acado/bindings/acado_gnuplot/gnuplot_window.hpp>
#include <orunav_path_smoother/path_smoother.h>
#include <orunav_path_smoother/acado_tools.h>
#include <orunav_generic/interfaces.h>
#include <orunav_generic/path_utils.h>
#include <orunav_trajectory_processor/trajectory_processor_naive.h>
#include <orunav_constraint_extract/polygon_constraint.h>

//! Helper class to divide trajectories into subsections.
class SplitIndex
{
 public:
  class Params
  {
  public:
    Params()
      {
	max_nb_points = 10;
	nb_points_discard = 5;
      }
    int max_nb_points;
    int nb_points_discard;
    bool valid() {
      if (max_nb_points > nb_points_discard)
	return true;
      return false;
    }
    friend std::ostream& operator<<(std::ostream &os, const SplitIndex::Params &obj)
      {
	os << "\nmax_nb_points     : " << obj.max_nb_points;
	os << "\nnb_points_discard : " << obj.nb_points_discard;
	return os;
      }

  };
  
 SplitIndex(Params &params, const orunav_generic::Trajectory &traj) : _traj(traj) {

    assert(params.valid());
    _params = params;
    bool done = false;
    int start = 0;
    int size = traj.sizeTrajectory();
    while (!done) {
      
      int stop = start + _params.max_nb_points;
      if (stop >= size) {
	stop = size;
	done = true;
      }
      _indexes.push_back(this->getIncrementVec(start, stop));

      start += _params.max_nb_points - _params.nb_points_discard - 1;
    }
  }
  
  std::vector<int> getIncrementVec(int start, int stop) const {
    std::vector<int> ret;
    for (int i = start; i < stop; i++) {
      ret.push_back(i);
    }
    return ret;
  }

  std::vector<orunav_generic::Trajectory> getTrajectories(const orunav_generic::Trajectory &traj) const {
    std::vector<orunav_generic::Trajectory> ret;
    for (int i = 0; i < _indexes.size(); i++) {
      ret.push_back(orunav_generic::selectTrajectoryIndexes(traj, _indexes[i]));
    }
    return ret;
  }

  std::vector<constraint_extract::PolygonConstraintsVec> getConstraintsVec(const constraint_extract::PolygonConstraintsVec &cons) const {
    std::vector<constraint_extract::PolygonConstraintsVec> ret;
    for (int i = 0; i < _indexes.size(); i++) {
      ret.push_back(selectConstraintsVecIndexes(cons, _indexes[i]));
    }
    return ret;
  }

  void setTrajectory(int idx, const orunav_generic::Trajectory &traj) {
    bool last_entry = (idx == _indexes.size()-1);
    int stop = _indexes[idx].size();
    if (!last_entry) {
      stop = _indexes[idx].size()  - _params.nb_points_discard;
    }

    int offset = _indexes[idx][0];
    for (int i = 0; i < stop; i++) { 
      
      //      _traj.addTrajectoryPoint(traj.getPose2d(i), traj.getSteeringAngle(i), traj.getDriveVel(i), traj.getSteeringVel(i));
      _traj.setPose2d(traj.getPose2d(i), offset+i);
      _traj.setSteeringAngle(traj.getSteeringAngle(i), offset+i);
      _traj.setDriveVel(traj.getDriveVel(i), offset+i);
      _traj.setSteeringVel(traj.getSteeringVel(i), offset+i);
    }
  }

  orunav_generic::Trajectory getTrajectory(int idx) const {
    return orunav_generic::selectTrajectoryIndexes(this->_traj, _indexes[idx]);
  }

  orunav_generic::Trajectory getTrajectory() const {
    return _traj;
  }

  void printDebug() const {
    for (int i = 0; i < _indexes.size(); i++) {
      std::cout << "\n[vec:" << i << "]" << std::endl;
      for (int j = 0; j < _indexes[i].size(); j++) {
	std::cout << "(" << _indexes[i][j] << ")" << std::flush;
      }
    }
    std::cout << std::endl;
  }

  int size() const {
    return _indexes.size();
  }

 private:
  std::vector<std::vector<int> > _indexes;
  orunav_generic::Trajectory _traj;
  Params _params;
};


class PathSmootherDynamic : public PathSmootherInterface
{
 public:
  class Params
  {
  public:
    Params()
      {
	v_min = -1;
	v_max = 1;
	w_min = -1;
	w_max = 1;
	phi_min = -1.1;
	phi_max = 1.1;
	get_speed = true; //true;
	init_states = true;
	init_controls = false;
	max_nb_opt_points = 12;
	even_point_dist = true;	
	min_dist = -1.;
	use_v_constraints = true;//true;
	use_w_constraints = true;//true;
	use_total_time = true;
	nb_iter_steps = 100;
	visualize = true; //false;
	wheel_base = 0.68; // snowwhite
	update_v_w_bounds = true;
	keep_w_bounds = false;
	w_zero = false;
	minimize_phi_and_dist = false;
	use_th_constraints = true; // Note that this is only used if there is any constraints provided.
	use_xy_constraints = true; // Note that this is only used if there is any constraints provided.
	use_constraints_modulus = 1;
	kkt_tolerance = 0.001;
	integrator_tolerance = 0.0001;
	weight_steering_control = 1.;
	use_multiple_shooting = true;
	use_condensing = true;
	use_incremental = true;
	incr_max_nb_points = 20;
	incr_nb_points_discard = 5;
      }
    double v_min;
    double v_max;
    double w_min;
    double w_max;
    double phi_min;
    double phi_max;
    bool get_speed;
    bool init_states;
    bool init_controls;
    int max_nb_opt_points;
    bool even_point_dist;
    double min_dist;
    bool use_v_constraints;
    bool use_w_constraints;
    bool use_total_time;
    int nb_iter_steps;
    bool visualize;
    double wheel_base;
    bool update_v_w_bounds;
    bool keep_w_bounds;
    bool w_zero;
    bool minimize_phi_and_dist;
    bool use_th_constraints;
    bool use_xy_constraints;
    int use_constraints_modulus;
    double kkt_tolerance;
    double integrator_tolerance;
    double weight_steering_control;
    bool use_multiple_shooting;
    bool use_condensing;
    bool use_incremental;
    int incr_max_nb_points;
    int incr_nb_points_discard;

    friend std::ostream& operator<<(std::ostream &os, const PathSmootherDynamic::Params &obj)
      {
	os << "\nv_min             : " << obj.v_min;
	os << "\nv_max             : " << obj.v_max;
	os << "\nw_min             : " << obj.w_min;
	os << "\nw_max             : " << obj.w_max;
	os << "\nphi_min           : " << obj.phi_min;
	os << "\nphi_max           : " << obj.phi_max;
	os << "\nget_speed         : " << obj.get_speed;
	os << "\ninit_states       : " << obj.init_states;
	os << "\ninit_controls     : " << obj.init_controls;
	os << "\nmax_nb_opt_points : " << obj.max_nb_opt_points;
	os << "\neven_point_dist   : " << obj.even_point_dist;
	os << "\nmin_dist          : " << obj.min_dist;
	os << "\nuse_v_constraints : " << obj.use_v_constraints;
	os << "\nuse_w_constraints : " << obj.use_w_constraints;
	os << "\nuse_total_time    : " << obj.use_total_time;
	os << "\nnb_iter_steps     : " << obj.nb_iter_steps;
	os << "\nvisualize         : " << obj.visualize;
	os << "\nwheel_base        : " << obj.wheel_base;
	os << "\nupdate_v_w_bounds : " << obj.update_v_w_bounds;
	os << "\nkeep_w_bounds     : " << obj.keep_w_bounds;
	os << "\nw_zero            : " << obj.w_zero;
	os << "\nminimize_phi_dist : " << obj.minimize_phi_and_dist;
	os << "\nuse_th_constraints: " << obj.use_th_constraints;
	os << "\nuse_xy_constraints: " << obj.use_xy_constraints;
	os << "\nuse_constr_modulus: " << obj.use_constraints_modulus;
	os << "\nkkt_tolerance     : " << obj.kkt_tolerance;
	os << "\nintegr_tolerance  : " << obj.integrator_tolerance;
	os << "\nweight_steer_ctrl : " << obj.weight_steering_control;
	os << "\nuse_multiple_s... : " << obj.use_multiple_shooting;
	os << "\nuse_condensing    : " << obj.use_condensing;
	os << "\nuse_incremental   : " << obj.use_incremental;
	os << "\nincr_max_nb_points: " << obj.incr_max_nb_points;
	os << "\nincr_nb_points_dis: " << obj.incr_nb_points_discard;
	return os;
      }

  };

  void computeThBounds(const double &lb_orig, const double &ub_orig, const double &th, double &lb_new, double &ub_new) {
    // We know that the provided th should be the closest fit between the orignal bounds.
    // However, due to       orunav_generic::removeThNormalization(traj); the orientation angle are not between (0..2PI) here.
    lb_new = lb_orig;
    while (lb_new - th > 0) {
      lb_new -= 2*M_PI;
    }
    while (lb_new - th < -2*M_PI) {
      lb_new += 2*M_PI;
    }
    if (!(lb_new < th)) {
      std::cerr << "===ERROR===" << std::endl;
      std::cerr << "lb_new : " << lb_new << std::endl;
      std::cerr << "th : " << th << std::endl;
      assert(false);
    }
    assert(lb_new < th); // Check.

    ub_new = ub_orig;
    while (ub_new - th > 2*M_PI) {
      ub_new -= 2*M_PI;
    }
    while (ub_new - th < 0) {
      ub_new += 2*M_PI;
    }
    if (!(ub_new > th)) {
      std::cerr << "===ERROR===" << std::endl;
      std::cerr << "ub_new : " << ub_new << std::endl;
      std::cerr << "th : " << th << std::endl;
    }
    assert(ub_new > th); // Check.

  }

  // All ACADO calls goes here.
  orunav_generic::Trajectory smooth_(const orunav_generic::Trajectory &traj, const constraint_extract::PolygonConstraintsVec &constraints, double dt, double start_time, double stop_time, bool use_pose_constraints)
    {
      // Always always...
      ACADO_clearStaticCounters();

      std::cout << "Setting up constraints -- start" << std::endl;

      ACADO::VariablesGrid q_init = convertPathToACADOStateVariableGrid(traj, 0.0, dt);
      
      ACADO::VariablesGrid u_init = convertTrajectoryToACADOControlVariablesGrid(traj, 0.0, dt);
      if (params.w_zero) {
	//	u_init = convertTrajectoryToACADOControlVariablesGrid(orunav_generic::setFixedControlValuesW(traj, 0.), 0.0, dt);
	setFixedACADOControlVariablesGrid(u_init, 0., 0.);
      }

      ACADO::DifferentialEquation f(start_time, stop_time);
      ACADO::DifferentialState        x,y,th,phi;     // the differential states
      ACADO::Control                  v, w;     // the control input u

      f << dot(x) == cos(th)*v;
      f << dot(y) == sin(th)*v;
      f << dot(th) == tan(phi)*v/params.wheel_base; // 0.68 = L
      f << dot(phi) == w;
      
      ACADO::OCP ocp(q_init);
      if (params.minimize_phi_and_dist) {
	ocp.minimizeLagrangeTerm(v*v + params.weight_steering_control*w*w);
      }
      else {
	ocp.minimizeMayerTerm(1.);
      }
      ocp.subjectTo(f);
      // Enforce the the start / end pose.

      ocp.subjectTo(ACADO::AT_START, x == traj.getPose2d(0)(0));
      ocp.subjectTo(ACADO::AT_START, y == traj.getPose2d(0)(1));
      ocp.subjectTo(ACADO::AT_START, th == traj.getPose2d(0)(2));
      ocp.subjectTo(ACADO::AT_START, phi == traj.getSteeringAngle(0));
      //ocp.subjectTo(ACADO::AT_START, v == 0);
      //ocp.subjectTo(ACADO::AT_START, w == 0);
      
      ocp.subjectTo(ACADO::AT_END, x == traj.getPose2d(traj.sizePath()-1)(0));
      ocp.subjectTo(ACADO::AT_END, y == traj.getPose2d(traj.sizePath()-1)(1));
      ocp.subjectTo(ACADO::AT_END, th == traj.getPose2d(traj.sizePath()-1)(2));
      ocp.subjectTo(ACADO::AT_END, phi == traj.getSteeringAngle(traj.sizePath()-1));
      //ocp.subjectTo(ACADO::AT_END, v == 0);
      //ocp.subjectTo(ACADO::AT_END, w == 0);
            
      //      if (params.use_v_constraints)
      //	ocp.subjectTo( params.v_min <= v <= params.v_max );
      //      if (params.use_w_constraints)
      //	ocp.subjectTo( params.w_min <= w <= params.w_max );
      ocp.subjectTo( params.phi_min <= phi <= params.phi_max );
      
      if (use_pose_constraints) {
	assert(constraints.size() == traj.sizePath());
	for (size_t i = 0; i < constraints.size(); i++) {
	  if (i % params.use_constraints_modulus == 0 || i == constraints.size()-1) {
	    // Use this constraint
	    //	    std::cout << "Using constraint # : " << i << std::endl;
	  }
	  else {
	    continue;
	  }
	  // Orientation
	  if (params.use_th_constraints) {
	    // Check for normalization problems that could occur here... simply make sure that we have a bounds that the current th is within.
	    double lb_th, ub_th;
	    computeThBounds(constraints[i].getThBounds()[0], constraints[i].getThBounds()[1], traj.getPose2d(i)(2), lb_th, ub_th);
	    ocp.subjectTo(i, lb_th <= th <= ub_th);
	  }
	  // Position
	  if (params.use_xy_constraints) {
	    std::vector<double> A0, A1, b;
	    constraints[i].getInnerConstraint().getMatrixFormAsVectors(A0, A1, b);
	    assert(A0.size() == A1.size());
	    assert(A0.size() == b.size());
	    size_t size = A0.size();
	    for (size_t j = 0; j < size; j++)
	      ocp.subjectTo(i, A0[j]*x + A1[j]*y <= b[j]);
	  }
	}
      }
      
      std::cout << "Setting up constraints -- end" << std::endl;
      

      std::cout << "Optimization -- start" << std::endl;
      ACADO::OptimizationAlgorithm algorithm(ocp);
      // ACADO params -- 
      if (!params.use_multiple_shooting)
	algorithm.set( ACADO::DISCRETIZATION_TYPE, ACADO::SINGLE_SHOOTING ); // For the non-objective -> there is not any difference.
      else
	algorithm.set( ACADO::DISCRETIZATION_TYPE, ACADO::MULTIPLE_SHOOTING );
      {
	int ret;
	algorithm.get( ACADO::DISCRETIZATION_TYPE, ret);
	if (ret == ACADO::SINGLE_SHOOTING) {
	  std::cout << "SINGLE_SHOOTING will be used" << std::endl;
	}
	if (ret == ACADO::MULTIPLE_SHOOTING) {
	  std::cout << "MULTIPLE_SHOOTING will be used" << std::endl;
	}
      }

      algorithm.set( ACADO::MAX_NUM_INTEGRATOR_STEPS, 100 ); // For the integrator.
      algorithm.set( ACADO::MAX_NUM_ITERATIONS, params.nb_iter_steps ); 
      algorithm.set( ACADO::PRINTLEVEL, ACADO::HIGH );
      algorithm.set( ACADO::PRINT_SCP_METHOD_PROFILE, BT_TRUE );
      //      algorithm.set( ACADO::USE_REFERENCE_PREDICTION, ACADO::BT_FALSE );
      // if (params.use_condensing)
      //   algorithm.set( ACADO::USE_CONDENSING, ACADO::BT_TRUE ); // Important!
      // else 
      //   algorithm.set( ACADO::USE_CONDENSING, ACADO::BT_FALSE );
      // {
      //   int ret;
      //   algorithm.get( ACADO::USE_CONDENSING, ret);
      //   if (ret == ACADO::BT_TRUE) {
      //     std::cout << "CONDENSING will be used" << std::endl;
      //   }
      //   if (ret == ACADO::BT_FALSE) {
      //     std::cout << "CONDENSING will NOT be used" << std::endl;
      //   }
      // }

      algorithm.set( ACADO::USE_REALTIME_ITERATIONS, BT_FALSE ); // Important!
      algorithm.set( ACADO::KKT_TOLERANCE, params.kkt_tolerance );
      algorithm.set( ACADO::INTEGRATOR_TOLERANCE, params.integrator_tolerance );
            
      
      std::cout << "Initialize variables" << std::endl;
      if (params.init_states)
	algorithm.initializeDifferentialStates( q_init );
      if (params.init_controls)
	algorithm.initializeControls( u_init );
     
      std::cout << "Setting up states / control variables" << std::endl;
      ACADO::VariablesGrid states, controls;
      std::cout << "-1" << std::endl;
      algorithm.getDifferentialStates(states);
      std::cout << "-2" << std::endl;
      algorithm.getControls(controls);
      std::cout << "-3" << std::endl;
      
      algorithm.getDifferentialStates("states_init.txt");
      std::cout << "-4" << std::endl;
      algorithm.getControls("controls_init.txt");
      std::cout << "-5" << std::endl;
      

      if (params.visualize) {
	ACADO::GnuplotWindow window4(ACADO::PLOT_AT_START);
	window4.addSubplot( x, "x - init" );
	window4.addSubplot( y, "y - init" );
	window4.addSubplot( th, "th - init" );
	window4.addSubplot( phi, "phi - init" );
	window4.addSubplot( v, "v - init" );
	window4.addSubplot( w, "w - init" );
	
	ACADO::GnuplotWindow window2(ACADO::PLOT_AT_EACH_ITERATION);
	window2.addSubplot( x, "x - iter..." );
	window2.addSubplot( y, "y - iter..." );
	window2.addSubplot( th, "th - iter..." );
	window2.addSubplot( phi, "phi - iter..." );
	window2.addSubplot( v, "v - iter..." );
	window2.addSubplot( w, "w - iter..." );

	algorithm << window2;
	algorithm << window4;
      }

      std::cout << "Solve - running..." << std::endl;
      algorithm.solve();
      std::cout << "Optimization -- end" << std::endl;
      
      algorithm.getDifferentialStates(states);
      algorithm.getControls(controls);
      algorithm.getDifferentialStates("states_final.txt");
      algorithm.getControls("controls_final.txt");

      if (params.visualize) {
	ACADO::GnuplotWindow window;
	window.addSubplot( q_init(0), "x - provided" );
	window.addSubplot( q_init(1), "y - provided" );
	window.addSubplot( q_init(2), "th - provided" );
	window.addSubplot( q_init(3), "phi - povided" );
	window.addSubplot( u_init(0), "v - provided" );
	window.addSubplot( u_init(1), "w - provided" );
	window.plot();
	
        
	ACADO::GnuplotWindow window3;
	window3.addSubplot( states(0), "x - final" );
	window3.addSubplot( states(1), "y - final" );
	window3.addSubplot( states(2), "th - final" );
	window3.addSubplot( states(3), "phi - final" );
	window3.addSubplot( controls(0), "v - final" );
	window3.addSubplot( controls(1), "w - final" );
	window3.plot();
      }

      //      return convertACADOStateVariableGridToPath(states);
            return convertACADOStateControlVariableGridToTrajectory(states, controls);
      }



  orunav_generic::Trajectory smoothTraj(const orunav_generic::PathInterface &path_orig, const orunav_generic::State2dInterface& start, const orunav_generic::State2dInterface &goal, const constraint_extract::PolygonConstraintsVec &constraints)
//const std::vector<constraint_extract::PolygonConstraint, Eigen::aligned_allocator<PolygonConstraint> > &constraints)
    {
      // Always always...
      //     ACADO_clearStaticCounters();

      bool use_pose_constraints = (constraints.size() ==  path_orig.sizePath()); // Cannot subsample the path etc. from now on.

      std::cout << "PATH SMOOTHER : constraints.size() : " << constraints.size() << std::endl;
      std::cout << "PATH SMOOTHER : path_orig.sizePath() : " << path_orig.sizePath() << std::endl;

      if (use_pose_constraints) {
	std::cout << "----- will use spatial constraints -----" << std::endl;
      }

      // Use the traj processor to get a reasonable estimate of the T.
      double T = 0.;
      orunav_generic::Trajectory traj_gen;
      {
	TrajectoryProcessorNaive gen;
	TrajectoryProcessor::Params p;
	p.maxVel = 0.5;
	p.maxAcc = 0.2;
	p.wheelBaseX = params.wheel_base;
	gen.setParams(p);
	gen.addPathInterface(path_orig);
	traj_gen = gen.getTrajectory(); 
	T = orunav_generic::getTotalTime(gen);
      }
      // Get the min / max time from the trajectory
      
      std::cout << "--------- Estimated total time T : " << T << " -----------" << std::endl;
      unsigned int orig_size = path_orig.sizePath();
      int skip_points = orig_size / params.max_nb_opt_points - 1;
      if (skip_points < 0)
	skip_points = 0;
      double dt = 0.06 * (1 + skip_points);
      orunav_generic::Path path;
      if (params.even_point_dist) {
	double min_dist = orunav_generic::getTotalDistance(path_orig) / params.max_nb_opt_points;
	if (min_dist < params.min_dist)
	  min_dist = params.min_dist;
	path = orunav_generic::minIncrDistancePath(path_orig, min_dist);
      }
      else {
	path = orunav_generic::subSamplePath(path_orig, skip_points);
      }
      if (params.use_total_time) {
	dt = T / path.sizePath(); 
      }

      if (use_pose_constraints) {
	path = path_orig;
      }

      std::cout << "Used dt : " << dt << std::endl;

      //orunav_generic::removeThNormalization(path);
      orunav_generic::Trajectory traj = orunav_generic::convertPathToTrajectoryWithoutModel(path, dt);
      assert(orunav_generic::validPath(traj, M_PI));
      if (orunav_generic::validPath(traj, M_PI))
        std::cerr << "Non-normalized path(!) - should never happen" << std::endl;
      orunav_generic::removeThNormalization(traj);
      assert(orunav_generic::validPath(traj, M_PI));
      
      double start_time = 0.0;
      unsigned int size = traj.sizeTrajectory();
      double stop_time = (size - 1)*dt;
      
      if (params.update_v_w_bounds) {
	PathSmootherDynamic::Params params_orig = params;
	if (params.init_controls || params.get_speed) {
	  orunav_generic::getMinMaxVelocities(traj, params.v_min, params.v_max, params.w_min, params.w_max);
	}
	else {
	  // Using the generated trajectory (from the traj processsor)
	  orunav_generic::getMinMaxVelocities(traj_gen, params.v_min, params.v_max, params.w_min, params.w_max);
	}
	// This is used to smooth a straight path which otherwise will have a w_min = w_max = 0.
	if (params.keep_w_bounds) {
	  params.w_min = params_orig.w_min;
	  params.w_max = params_orig.w_max;
	}
	std::cout << "------ Updated v/w params : -------- " << params << std::endl;
      }
     
      // Make sure to normalize the start and end pose - this is done by updating the start and end trajectory state, this should be done after the velocities are computed.
      traj.setPose2d(start.getPose2d(), 0);
      traj.setSteeringAngle(start.getSteeringAngle(), 0);
      traj.setPose2d(goal.getPose2d(), traj.sizePath()-1);
      traj.setSteeringAngle(goal.getSteeringAngle(), traj.sizePath()-1);

      std::cout << "updating the start and end pose : " << std::endl;
      assert(orunav_generic::validPath(traj, M_PI));
      orunav_generic::removeThNormalization(traj); // Force the start point and end point to not be normalized.
      assert(orunav_generic::validPath(traj, M_PI));
      
      if (!params.use_incremental)
      {
	return smooth_(traj, constraints, dt, start_time, stop_time, use_pose_constraints); 
      }

      // Run the iterative approach
      // Keep the start and goal fixed (as previous) but divide the section to chunks and optimize over the cunks. The start of the next chunk is located in the previous chunk.
      SplitIndex::Params si_params;
      si_params.max_nb_points =  params.incr_max_nb_points;
      si_params.nb_points_discard = params.incr_nb_points_discard;
      int nb_points_discard;

      SplitIndex si(si_params, traj);

      std::vector<constraint_extract::PolygonConstraintsVec> constraints_vec = si.getConstraintsVec(constraints);

      for (int i = 0; i < si.size(); i++) { 
	orunav_generic::Trajectory t = si.getTrajectory(i);
	stop_time = dt * t.sizeTrajectory()-1;
	std::cout << "stop_time : " << stop_time << std::endl;
	std::cout << "================================================" << std::endl;
	std::cout << "t.sizeTrajectory() : " << t.sizeTrajectory() << std::endl;
	std::cout << "constraints_vec[i].size() : " << constraints_vec[i].size() << std::endl;
 	orunav_generic::Trajectory ts = smooth_(t, constraints_vec[i], dt, start_time, stop_time, use_pose_constraints); 
	std::cout << "ts.sizeTrajectory() : " << ts.sizeTrajectory() << std::endl;
	si.setTrajectory(i, ts);
      }
      return si.getTrajectory();
    }

  orunav_generic::Path smooth(const orunav_generic::PathInterface &path_orig, const orunav_generic::State2dInterface& start, const orunav_generic::State2dInterface &goal, const constraint_extract::PolygonConstraintsVec &constraints) {
    orunav_generic::Path p(smoothTraj(path_orig, start, goal, constraints));
    return p;
  }

  orunav_generic::Path smooth(const orunav_generic::PathInterface &path_orig, const orunav_generic::State2dInterface& start, const orunav_generic::State2dInterface &goal) {
    constraint_extract::PolygonConstraintsVec constraints;
    return this->smooth(path_orig, start, goal, constraints);
  }


  PathSmootherDynamic::Params params;
  // These variables can only be initated once within one program / node etc. therefore they are declared as private members. Note that this class can therefore only be initiated once per program / node.
  /* ACADO::DifferentialState        x,y,th,phi;     // the differential states */
  /* ACADO::Control                  v, w;     // the control input u */
  
};
