#pragma once

#include <sstream>

#include "commonDefines.h"
#include "headers_ros.h"
#include "state.h"
#include "control.h"


#define SW_PARAM_PREFIX string("/robot") + robot_id_ss.str() + string("/params/controller/")
//#define SW_PARAM_PREFIX string("params/controller/")
//#define SW_TOPIC_PREFIX string("/robot") + robot_id_ss.str() + string("/controller/")
#define SW_TOPIC_PREFIX string("")


/**
 * @brief Container for controller parameters.
 */
class swParameters
{
    public:
        double qp_time_limit;
        int qp_as_change_limit;
        int qp_max_solution_reuse;
        int qp_delay_index_offset_v;
        int qp_delay_index_offset_w;


        State error_feedback_gains;

        State state_gains_mode1;
        State sqrt_state_gains_mode1;
        Control control_gains_mode1;


        State state_gains_mode2;
        State sqrt_state_gains_mode2;
        Control control_gains_mode2;
        
        
        double max_steering_wheel_velocity_delta;
        double max_steer_angular_velocity_delta;


        double max_tangential_velocity;
        double max_tangential_acceleration;
        double max_centripetal_acceleration;

        double max_steer_angle;
        double max_steer_angular_velocity;

        double max_orientation_angle;

        int delay_index_offset_v;
        int delay_index_offset_w;

        bool enable_closed_loop;
        bool enable_command_execution;
        bool enable_state_transformation;

        int max_graceful_brake_steps;
        int max_graceful_emergency_brake_steps;


        bool enable_position_constraints;
        bool enable_cen_acc_constraints;
        bool enable_tan_acc_constraints;
        bool enable_orientation_constraints;


        double car_wheel_base;
        int robot_id;


#ifdef SW_BUILD_SIMULATION
        string topic_simulation_command;
        string topic_simulation_sensor;
#endif
        string topic_report;
        string topic_trajectories;
        string topic_commands;
        string topic_active_robots;



        void init (ros::NodeHandle &ros_node)
        {
            if (!ros_node.hasParam("robot_id"))
            {
                SW_THROW_MSG("ID of the robot is not specified.");
            }


            ros_node.getParam("robot_id", robot_id);
            stringstream robot_id_ss;
            robot_id_ss << robot_id;

            // ------------------------------
            // QP parameters
            ros_node.param<double>(SW_PARAM_PREFIX + "qp_time_limit", qp_time_limit, 0.010);
            ros_node.param<int>(SW_PARAM_PREFIX + "qp_as_change_limit", qp_as_change_limit, 500);
            ros_node.param<int>(SW_PARAM_PREFIX + "qp_max_solution_reuse", qp_max_solution_reuse, 0);
            ros_node.param<int>(SW_PARAM_PREFIX + "qp_delay_index_offset_v", qp_delay_index_offset_v, 0);
            ros_node.param<int>(SW_PARAM_PREFIX + "qp_delay_index_offset_w", qp_delay_index_offset_w, 0);
            // ------------------------------


            // ------------------------------
            // Gains
            ros_node.param<double>(SW_PARAM_PREFIX + "state_gains_x_mode1", state_gains_mode1.x(), 1);
            ros_node.param<double>(SW_PARAM_PREFIX + "state_gains_y_mode1", state_gains_mode1.y(), 1);
            ros_node.param<double>(SW_PARAM_PREFIX + "state_gains_theta_mode1", state_gains_mode1.theta(), 1);
            ros_node.param<double>(SW_PARAM_PREFIX + "state_gains_phi_mode1", state_gains_mode1.phi(), 1);
            sqrt_state_gains_mode1 = state_gains_mode1;
            sqrt_state_gains_mode1.sqrt();

            ros_node.param<double>(SW_PARAM_PREFIX + "control_gains_v_mode1", control_gains_mode1.v(), 1);
            ros_node.param<double>(SW_PARAM_PREFIX + "control_gains_w_mode1", control_gains_mode1.w(), 1);

            ros_node.param<double>(SW_PARAM_PREFIX + "state_gains_x_mode2", state_gains_mode2.x(), 1);
            ros_node.param<double>(SW_PARAM_PREFIX + "state_gains_y_mode2", state_gains_mode2.y(), 1);
            ros_node.param<double>(SW_PARAM_PREFIX + "state_gains_theta_mode2", state_gains_mode2.theta(), 1);
            ros_node.param<double>(SW_PARAM_PREFIX + "state_gains_phi_mode2", state_gains_mode2.phi(), 1);
            sqrt_state_gains_mode2 = state_gains_mode2;
            sqrt_state_gains_mode2.sqrt();

            ros_node.param<double>(SW_PARAM_PREFIX + "control_gains_v_mode2", control_gains_mode2.v(), 1);
            ros_node.param<double>(SW_PARAM_PREFIX + "control_gains_w_mode2", control_gains_mode2.w(), 1);
            // ------------------------------


            // ------------------------------
            // State filtering
            ros_node.param<double>(SW_PARAM_PREFIX + "error_feedback_gains_x", error_feedback_gains.x(), 1);
            ros_node.param<double>(SW_PARAM_PREFIX + "error_feedback_gains_y", error_feedback_gains.y(), 1);
            ros_node.param<double>(SW_PARAM_PREFIX + "error_feedback_gains_theta", error_feedback_gains.theta(), 1);
            ros_node.param<double>(SW_PARAM_PREFIX + "error_feedback_gains_phi", error_feedback_gains.phi(), 1);
            if ((error_feedback_gains.x() > 1.0) || (error_feedback_gains.x() < 0.0) 
                || (error_feedback_gains.y() > 1.0) || (error_feedback_gains.y() < 0.0) 
                || (error_feedback_gains.theta() > 1.0) || (error_feedback_gains.theta() < 0.0) 
                || (error_feedback_gains.phi() > 1.0) || (error_feedback_gains.phi() < 0.0))
            {
                SW_THROW_MSG("Wrong value of a error feedback gains.");
            }
            // ------------------------------


            // ------------------------------
            // Contstraints
            ros_node.param<double>(SW_PARAM_PREFIX + "max_tangential_velocity", max_tangential_velocity, 1);
            ros_node.param<double>(SW_PARAM_PREFIX + "max_tangential_acceleration", max_tangential_acceleration, 15);
            ros_node.param<double>(SW_PARAM_PREFIX + "max_centripetal_acceleration", max_centripetal_acceleration, 15);

            ros_node.param<double>(SW_PARAM_PREFIX + "max_steer_angle", max_steer_angle, SW_PI/3);
            ros_node.param<double>(SW_PARAM_PREFIX + "max_steer_angular_velocity", max_steer_angular_velocity, SW_PI/2);

            ros_node.param<double>(SW_PARAM_PREFIX + "max_orientation_angle", max_orientation_angle, 2*SW_PI);
            // ------------------------------

            // ------------------------------
            // Safety
            ros_node.param<double>(SW_PARAM_PREFIX + "max_steering_wheel_velocity_delta", max_steering_wheel_velocity_delta, 1.0);
            ros_node.param<double>(SW_PARAM_PREFIX + "max_steer_angular_velocity_delta", max_steer_angular_velocity_delta, 1.0);
            // ------------------------------


            // ------------------------------
            // Features
            ros_node.param<bool>(SW_PARAM_PREFIX + "enable_closed_loop", enable_closed_loop, true);
            ros_node.param<bool>(SW_PARAM_PREFIX + "enable_command_execution", enable_command_execution, true);
            ros_node.param<bool>(SW_PARAM_PREFIX + "enable_state_transformation", enable_state_transformation, true);
            ros_node.param<int>(SW_PARAM_PREFIX + "max_graceful_brake_steps", max_graceful_brake_steps, 0);
            ros_node.param<int>(SW_PARAM_PREFIX + "max_graceful_emergency_brake_steps", max_graceful_emergency_brake_steps, 0);
            if (max_graceful_brake_steps < 0)
            {
                max_graceful_brake_steps = 0;
            }
            if (max_graceful_emergency_brake_steps < 0)
            {
                max_graceful_emergency_brake_steps = 0;
            }
            // ------------------------------


            // ------------------------------
            // Enable / disable constraints
            ros_node.param<bool>(SW_PARAM_PREFIX + "enable_position_constraints",    enable_position_constraints, true);
	    ros_node.param<bool>(SW_PARAM_PREFIX + "enable_cen_acc_constraints",     enable_cen_acc_constraints, true);
            ros_node.param<bool>(SW_PARAM_PREFIX + "enable_tan_acc_constraints",     enable_tan_acc_constraints, true);
            ros_node.param<bool>(SW_PARAM_PREFIX + "enable_orientation_constraints", enable_orientation_constraints, true);
            // ------------------------------

	    
            // ------------------------------
            // Distance between the wheel pairs. Physical parameter
	    //ros_node.param<double>("/params/physical/wheel_base", car_wheel_base, 0.680);
            ros_node.param<double>("wheel_base", car_wheel_base, 0.680);
            // ------------------------------

            //=======================================================

#ifdef SW_BUILD_SIMULATION
	    std::ostringstream os;
      os << "/robot" << robot_id << "/cmd_vel";
      //os << "cmd_vel";
	    topic_simulation_command = os.str();
	    //	    topic_simulation_command = "cmd_vel";
            topic_simulation_sensor = "/gazebo/link_states";
#endif
            topic_report = SW_TOPIC_PREFIX + "reports";
            topic_trajectories = SW_TOPIC_PREFIX + "trajectories";
            topic_commands = SW_TOPIC_PREFIX + "commands";
            /// @todo Topic names should be consistent.
            topic_active_robots = "/active_robots";
        }
};
