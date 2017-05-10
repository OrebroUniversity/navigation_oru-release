// This is built by CMakeLists.txt


#include <qpProblem.h>
#include <orunav_generic/io.h>
#include <orunav_generic/path_utils.h>

inline State convertGenericState2dToState(const orunav_generic::State2dInterface &s) {
   State ret;
   ret.set(s.getPose2d()[0], s.getPose2d()[1], s.getPose2d()[2], s.getSteeringAngle());
   return ret;
}

inline orunav_generic::State2d convertStateToGenericState2d(const State &s) {
  return orunav_generic::State2d(s.x(), s.y(), s.theta(), s.phi());
}

inline orunav_generic::Control convertControlToGenericControl(const Control &c) {
  return orunav_generic::Control(c.v(), c.w());
}


inline trajectoryCache fillTrajectoryCache(const orunav_generic::TrajectoryInterface &traj, unsigned int start_idx, const Constraints &constraints) {
  trajectoryCache traj_cache;
  trajectoryStep step;
  for (unsigned int i = start_idx; i < traj.sizeTrajectory(); i++) {
    step.state.set(traj.getPose2d(i)[0],
			traj.getPose2d(i)[1],
			traj.getPose2d(i)[2],
			traj.getSteeringAngle(i));

    step.control.set(traj.getDriveVel(i), traj.getSteeringVel(i));
    
    traj_cache.append(step, constraints);
  }

  // Fill the full preview window.
  step.control.v() = 0;
  step.control.w() = 0;
  for (unsigned int i = 0; i < SW_PREVIEW_WINDOW_LEN; ++i)
    {
      if (i == SW_PREVIEW_WINDOW_LEN - 1)
	step.last = true;
      traj_cache.append(step, constraints);
    }
  
  return traj_cache;
}


inline trajectoryChunk fillTrajectoryChunk(const orunav_generic::TrajectoryInterface &traj, const Constraints &constraints) {
  trajectoryChunk traj_chunks(constraints);
  for (unsigned int i = 0; i < traj.sizeTrajectory(); i++) {
    trajectoryStep step;
    
    step.state.set(traj.getPose2d(i)[0],
			traj.getPose2d(i)[1],
			traj.getPose2d(i)[2],
			traj.getSteeringAngle(i));

    step.control.set(traj.getDriveVel(i), traj.getSteeringVel(i));
    
    traj_chunks.steps.push_back(step);
  }
  traj_chunks.finalize();
  return traj_chunks;
}


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Main
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main(int argc, char** argv)
{
  cout << "Preview length used : " << SW_PREVIEW_WINDOW_LEN << std::endl;
  
  // Setup the parameters
  swParameters parameters;
  
  // ------------------------------
  // QP parameters
  parameters.qp_time_limit = 0.010;
  parameters.qp_as_change_limit = 500;
  parameters.qp_max_solution_reuse = 0;
  // ------------------------------
  
  // ------------------------------
  // Gains
  parameters.state_gains_mode1.x() = 100;
  parameters.state_gains_mode1.y() = 100;
  parameters.state_gains_mode1.theta() = 100;
  parameters.state_gains_mode1.phi() = 1;
  parameters.sqrt_state_gains_mode1 = parameters.state_gains_mode1;
  parameters.sqrt_state_gains_mode1.sqrt();

  parameters.control_gains_mode1.v() = 1;
  parameters.control_gains_mode1.w() = 1;

  parameters.state_gains_mode2.x() = 100;
  parameters.state_gains_mode2.y() = 100;
  parameters.state_gains_mode2.theta() = 100;
  parameters.state_gains_mode2.phi() = 1;
  parameters.sqrt_state_gains_mode2 = parameters.state_gains_mode2;
  parameters.sqrt_state_gains_mode2.sqrt();

  parameters.control_gains_mode2.v() = 1;
  parameters.control_gains_mode2.w() = 1;
  // ------------------------------
  
  
  // ------------------------------
  // State filtering
  parameters.error_feedback_gains.x() = 1;
  parameters.error_feedback_gains.y() = 1;
  parameters.error_feedback_gains.theta() = 1;
  parameters.error_feedback_gains.phi() = 1;
  // ------------------------------


  // ------------------------------
  // Contstraints
  parameters.max_tangential_velocity = 1;
  parameters.max_tangential_acceleration = 0.5;
  parameters.max_centripetal_acceleration =  1;
  
  parameters.max_steer_angle = SW_PI/3;
  parameters.max_steer_angular_velocity = SW_PI/2;
  
  parameters.max_orientation_angle = 2*SW_PI;
  // ------------------------------

  
  // ------------------------------
  // Safety
  parameters.max_steering_wheel_velocity_delta = 1.0;
  parameters.max_steer_angular_velocity_delta = 1.0;
  // ------------------------------


  // ------------------------------
  // Features
  parameters.enable_closed_loop = true;
  parameters.enable_command_execution = false; // true;
  parameters.enable_state_transformation =  false;//true;
  
  parameters.max_graceful_brake_steps = 0;
  parameters.max_graceful_emergency_brake_steps = 0;
  // ------------------------------
  
  
  // ------------------------------
  // Enable / disable constraints
  parameters.enable_position_constraints = true; // true;
  parameters.enable_cen_acc_constraints = true; // true;
  parameters.enable_tan_acc_constraints = true; // true;
  parameters.enable_orientation_constraints = true; //true;
  // ------------------------------
  
  
  // ------------------------------
  parameters.car_wheel_base = 1.190; // 0.680;
  // ------------------------------

  // Load a computed trajectory
  orunav_generic::Trajectory input_traj = orunav_generic::loadTrajectoryTextFile("test_08_data/test.traj");
  cout << "Loaded : " << input_traj.sizeTrajectory() << " trajectory points." << endl;
  std::cout << "First pose : " << input_traj.getPose2d(0) << std::endl;
  orunav_generic::setFirstPoseAsOrigin(input_traj);
  std::cout << "First pose (after state transformation) : " << input_traj.getPose2d(0) << std::endl;
  orunav_generic::Trajectory output_traj;
  
  // First point should match no matter what...
  output_traj.add(input_traj.getState2d(0), input_traj.getControl(0));

  
  qpProblem qp(parameters);
  // Use the same constraints all over... (TODO - this should instead be heavily exploited . (should, for example, be used later on to force the vehicle to not move backward / forward in the end... - or to change the state weights to increase towards the and).
  Constraints constraints(parameters);

  State current_state;
  current_state.set(input_traj.getPose2d(0)[0],
		    input_traj.getPose2d(0)[1],
		    input_traj.getPose2d(0)[2],
		    input_traj.getSteeringAngle(0));
  
  Control input_control;
  input_control.set(input_traj.getDriveVel(0), input_traj.getSteeringVel(0));

//  Control control_prev; // Control computed on the previous iteration.
//  control_prev.set(input_traj.getDriveVel(0), input_traj.getSteeringVel(0));

  for (unsigned int i = 0; i < input_traj.sizeTrajectory(); i++) {
    std::cout << "i : " << i << std::endl;
    // Fill the trajectory cache
    trajectoryCache traj_cache = fillTrajectoryCache(input_traj, i, constraints);
    qp.preview_win.form(traj_cache);

    State expected_state; // Output, taken from the preview window.
    Control control; // In / out param send the previous computed command here... and obtain the command to be applied to the current_state.

    control = input_control;
    //control.set(input_traj.getDriveVel(i), input_traj.getSteeringVel(i)); - makes no sence, this is already in the preview window.

    std::cout << "current_state : " << current_state << std::endl;
    std::cout << "control input : " << control << std::endl;

    qp.solve(parameters,
	     current_state,
	     expected_state,
	     control);


    double acc = (control.v() - input_control.v())/0.06;
    std::cout << "acc[" << i << "] : " << acc << std::endl;

    input_control = control; // To the next iter.
    std::cout << "control output : " << control << std::endl;

    // Apply the control to the state...
    //"current_state = current_state + control" -> use the generic params to do this.
    orunav_generic::State2d current_state_ = convertStateToGenericState2d(current_state);
    orunav_generic::Control control_ = convertControlToGenericControl(control);

    current_state_.addControlStep(control_, parameters.car_wheel_base, 0.06);

    //current_state = expected_state, note this is not what should be done, this basically will ignore all constraints.
    current_state = convertGenericState2dToState(current_state_);

    output_traj.add( convertStateToGenericState2d(current_state), 
		     convertControlToGenericControl(control));
    
  }
  
  orunav_generic::saveTrajectoryTextFile(input_traj, "test_08_data/original.traj");
  orunav_generic::saveTrajectoryTextFile(output_traj, "test_08_data/mpc_smooth.traj");
  std::cout << "done." << std::endl;

  return (0);
}
