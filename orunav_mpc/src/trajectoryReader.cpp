#include "trajectoryReader.h"
#include "swLogger.h"



/**
 * @brief Reader, which runs in a separate thread.
 *
 * @param[in,out] thread_data thead data.
 */
threadReturn trajectoryReader::readLoop(threadData *thread_data)
{
    ros::Time last_hearbeat_time = ros::Time::now();

    thdata = thread_data;
    start_time = ros::Time(0);
    is_active = false;


    // trajectory chunks
    ros::CallbackQueue traj_queue;
    ros::Subscriber traj_subscriber;

    ros::SubscribeOptions traj_sub_opts = ros::SubscribeOptions::create<orunav_msgs::ControllerTrajectoryChunkVec>(
            thread_data->parameters.topic_trajectories,
            1000,
            boost::bind(&trajectoryReader::processTrajectoryChunkVec, this, _1),
            ros::VoidPtr(),
            &traj_queue);

    traj_subscriber = thread_data->ros_node.subscribe(traj_sub_opts);


    // commands
    ros::CallbackQueue comm_queue;
    ros::Subscriber comm_subscriber;

    ros::SubscribeOptions comm_sub_opts = ros::SubscribeOptions::create<orunav_msgs::ControllerCommand>(
            thread_data->parameters.topic_commands,
            100,
            boost::bind(&trajectoryReader::actOnCommand, this, _1),
            ros::VoidPtr(),
            &comm_queue);

    comm_subscriber = thread_data->ros_node.subscribe(comm_sub_opts);


    // active robots
    ros::CallbackQueue act_queue;
    ros::Subscriber act_subscriber;

    ros::SubscribeOptions act_sub_opts = ros::SubscribeOptions::create<orunav_msgs::ActiveRobots>(
            thread_data->parameters.topic_active_robots,
            100,
            boost::bind(&trajectoryReader::checkActivity, this, _1),
            ros::VoidPtr(),
            &act_queue);

    act_subscriber = thread_data->ros_node.subscribe(act_sub_opts);


    // process queues
    for( ;; )
    {
        if (start_time.sec != 0)
        {
            /* 
             * Waiting can be reimplemented in several different ways:
             *  - using sleep() method in a separate thread;
             *  - using ros::Timer and a dedicated event queue;
             *  - using the system time routines (if high precision is necessary).
             */
            if (ros::Time::now() >= start_time)
            {
                if (ros::Time::now() >= start_time + ros::Duration(SW_START_TIME_ALLOWED_ERROR_SEC))
                {
                    //thdata->status.set(SW_PROCESS_STATUS_FAIL);
                    ROS_WARN("Failed to start tracking at the specified time.");
                }
                //else
                {
                    // Active list - currently not used.
                    is_active = true; ////// <<<< -------
                    
                    if (is_active == true)
                    {
                        thdata->traj_pool.unblock();
                        ROS_INFO("Trajectory pool is unblocked. If there is enough data, tracking starts automatically.");
                    }
                    else
                    {
                        thdata->status.set(SW_PROCESS_STATUS_FAIL);
                        ROS_WARN("Cannot start tracking, since the robot is not in the active list.");
                    }
                }
                start_time = ros::Time(0);
            }
        }


        if (ros::Time::now() - last_hearbeat_time > ros::Duration(SW_HEARBEAT_TIMEOUT_SEC))
        {
          // TODO - what to do with the heartbeat? Keep it?
            // ROS_WARN("No heartbeat signal.");

            // swProcessStatus status = thdata->status.get();
            // if ( (status == SW_PROCESS_STATUS_ACTIVE) || (status == SW_PROCESS_STATUS_FINALIZE))
            // {
            //     thdata->status.set(SW_PROCESS_STATUS_FAIL);
            // }
        }


        while (comm_queue.isEmpty() == false)
        {
            if (thread_data->isProcessTerminated())
            {
                return (SW_THREAD_RETURN_STOPPED);
            }
            comm_queue.callOne ();
        }

        while (act_queue.isEmpty() == false)
        {
            if (thread_data->isProcessTerminated())
            {
                return (SW_THREAD_RETURN_STOPPED);
            }
            act_queue.callOne ();
            last_hearbeat_time = ros::Time::now();
        }

        traj_queue.callOne (ros::WallDuration(SW_TRAJECTORY_CALLBACK_WAIT_SEC));

        if (thread_data->isProcessTerminated())
        {
            return (SW_THREAD_RETURN_STOPPED);
        }
    }

    return (SW_THREAD_RETURN_DONE);
}


/**
 * @brief Process a vector of trajectory chunks.
 *
 * @param[in] msg a message containing a vector of trajectory chunks.
 *
 * @note See orunav_msgs::ControllerTrajectoryChunkVec message.
 */
void trajectoryReader::processTrajectoryChunkVec(
        const orunav_msgs::ControllerTrajectoryChunkVecConstPtr& msg)
{
    for (size_t i = 0; i < msg->chunks.size(); i++) 
    {
      processTrajectoryChunk_(msg->chunks[i]);
    }
}


/**
 * @brief Process trajectory chunk.
 *
 * @param[in] msg a message containing a trajectory chunk.
 *
 * @note See orunav_msgs::ControllerTrajectoryChunk message.
 */
void trajectoryReader::processTrajectoryChunk(
        const orunav_msgs::ControllerTrajectoryChunkConstPtr& msg)
{
  processTrajectoryChunk_(*msg);
}

/**
 * @brief Process trajectory chunk.
 *
 * @param[in] msg a message containing a trajectory chunk.
 *
 * @note See orunav_msgs::ControllerTrajectoryChunk message.
 */
void trajectoryReader::processTrajectoryChunk_(
        const orunav_msgs::ControllerTrajectoryChunk& msg)
{
    // Correctness
    if (msg.robot_id != thdata->parameters.robot_id)
    {
        return;
    }

    if (msg.steps.size() == 0)
    {
        ROS_WARN("Trajectory message parser: Empty trajectory chunk.");
        return;
    }


    try
    {
        SW_LOG_TRAJECTORY("ID = " << msg.traj_id << " | SEQ = " << msg.sequence_num);


        // Initialize constraints
        Constraints constraints(thdata->parameters);
        initStepConstraints_(msg, constraints);

        trajectoryChunk traj_chunk(constraints);
        traj_chunk.sequence_num = msg.sequence_num;
        traj_chunk.id = msg.traj_id;
        traj_chunk.steps.clear();


        // Add steps
        trajectoryStep step;
        for (unsigned int i = 0; i < msg.steps.size(); ++i)
        {
            if (msg.steps[i].mode == msg.steps[i].MODE_1)
            {
                step.mode = SW_STEP_MODE_1;
            }
            else if (msg.steps[i].mode == msg.steps[i].MODE_2)
            {
                step.mode = SW_STEP_MODE_2;
            }
            else
            {
                ROS_WARN("Trajectory message parser: Unknown step mode, default is used.");
                step.mode = SW_STEP_MODE_DEFAULT;
            }

            step.state.set(
                    msg.steps[i].state.position_x,
                    msg.steps[i].state.position_y,
                    msg.steps[i].state.orientation_angle,
                    msg.steps[i].state.steering_angle);

            step.control.set(
                    msg.steps[i].velocities.tangential,
                    msg.steps[i].velocities.steering);

            traj_chunk.steps.push_back(step);
        }

        // Finalize if necessary
        if (msg.final == true)
        {
            traj_chunk.finalize();
        }

        // Add 
        thdata->traj_pool.addTrajectoryChunk(traj_chunk);
    }
    catch (const exception &e)
    {  
        SW_LOG_TRAJECTORY(e.what());
        ROS_WARN("%s", e.what());
    }
}


void trajectoryReader::initStepConstraints(
        orunav_msgs::ControllerTrajectoryChunk::ConstPtr msg, 
        Constraints & constraints)
{
  initStepConstraints_(*msg, constraints);
}

/**
 * @brief Copy constraints from a message to a local class.
 *
 * @param[in] msg message.
 * @param[out] constraints constraints extracted from the message.
 */
void trajectoryReader::initStepConstraints_(
        const orunav_msgs::ControllerTrajectoryChunk &msg, 
        Constraints & constraints)
{
    constraints.a0 = msg.constraints.spatial_coef_a0;
    constraints.a1 = msg.constraints.spatial_coef_a1;
    constraints.b = msg.constraints.spatial_coef_b;
    constraints.pos_constraints_num = constraints.a0.size();


    // Orientation
    if ((msg.constraints.bounds_orientation.size() != 0)
        && (msg.constraints.bounds_orientation.size() != 2))
    {
        ROS_WARN("Trajectory message parser: Malformed bounds on orientation were skipped.");
    }
    else if (msg.constraints.bounds_orientation.size() == 2)
    {
        constraints.theta_min = msg.constraints.bounds_orientation[0];
        constraints.theta_max = msg.constraints.bounds_orientation[1];
    }


    // Steering velocity
    if ((msg.constraints.bounds_steering_velocity.size() != 0)
        && (msg.constraints.bounds_steering_velocity.size() != 2))
    {
        ROS_WARN("Trajectory message parser: Malformed bounds on steering velocity were skipped.");
    }
    else if (msg.constraints.bounds_steering_velocity.size() == 2)
    {
        constraints.w_min = msg.constraints.bounds_steering_velocity[0];
        constraints.w_max = msg.constraints.bounds_steering_velocity[1];
    }


    // Tangential velocity
    if ((msg.constraints.bounds_tangential_velocity.size() != 0)
        && (msg.constraints.bounds_tangential_velocity.size() != 2))
    {
        ROS_WARN("Trajectory message parser: Malformed bounds on tangential velocity were skipped.");
    }
    else if (msg.constraints.bounds_tangential_velocity.size() == 2)
    {
        constraints.v_min = msg.constraints.bounds_tangential_velocity[0];
        constraints.v_max = msg.constraints.bounds_tangential_velocity[1];
    }


    // Tangential acceleration
    if ((msg.constraints.bounds_tangential_acceleration.size() != 0)
        && (msg.constraints.bounds_tangential_acceleration.size() != 2))
    {
        ROS_WARN("Trajectory message parser: Malformed bounds on tangential acceleration were skipped.");
    }
    else if (msg.constraints.bounds_tangential_acceleration.size() == 2)
    {
        constraints.tanacc_min = msg.constraints.bounds_tangential_acceleration[0];
        constraints.tanacc_max = msg.constraints.bounds_tangential_acceleration[1];
    }


    // Centripetal acceleration
    if ((msg.constraints.max_centripetal_acceleration.size() != 0)
        && (msg.constraints.max_centripetal_acceleration.size() != 1))
    {
        ROS_WARN("Trajectory message parser: Malformed constraint on centripetal acceleration.");
    }
    else if (msg.constraints.max_centripetal_acceleration.size() == 1)
    {
        constraints.cenacc_max = msg.constraints.max_centripetal_acceleration[0];
    }
}



/**
 * @brief Execute received command.
 *
 * @param[in] cmd command message.
 *
 * @note See orunav_msgs::ControllerCommand message.
 */
void trajectoryReader::actOnCommand(const orunav_msgs::ControllerCommandConstPtr& cmd)
{
    if (cmd->robot_id != thdata->parameters.robot_id)
    {
        return;
    }

    try
    {
        SW_LOG_TRAJECTORY("Command: " << cmd->command);

        if (cmd->command == cmd->COMMAND_BRAKE)
        {
            thdata->status.set(SW_PROCESS_STATUS_FAIL);
            ROS_INFO("Command = brake: Stopping.");
        }
        else if (cmd->command == cmd->COMMAND_ACTIVATE)
        {
            thdata->traj_pool.setActive(cmd->traj_id);
            ROS_INFO("Command = activate: Trajectory [%d] is activated.", cmd->traj_id);
        }
        else if (cmd->command == cmd->COMMAND_RECOVER)
        {
            thdata->status.recover();
            ROS_INFO("Command = recover: Executed.");
        }
        else if (cmd->command == cmd->COMMAND_STARTTIME)
        {
            if (thdata->traj_pool.isBlocked() == false)
            {
                ROS_WARN("Command = starttime: Ignored, since the trajectory pool is already unblocked.");
            }
            else
            {
                start_time = cmd->start_time;
                double timeDiff = (ros::Time::now() - start_time).toNSec() * 1e-6;
                ROS_INFO("[orunav_mpc] Elapsed time since command: %3.3f",timeDiff);
                if (start_time < ros::Time::now() - ros::Duration(SW_START_TIME_ALLOWED_ERROR_SEC))
                {
                    start_time = ros::Time(0);
                    thdata->status.set(SW_PROCESS_STATUS_FAIL);
                    ROS_WARN("Command = starttime: The specified time is in the past.");
                }
                else
                {
                    ROS_INFO("Command = starttime: Time is set, waiting.");
                }
            }
        }
        else 
        {
            ROS_WARN("Command = ?: An unsupported command is received.");
        }
    }
    catch (const exception &e)
    {  
        SW_LOG_TRAJECTORY(e.what());
        ROS_WARN("%s", e.what());
    }
}




/**
 * @brief Checks if this robot is active
 *
 * @param[in] actrob a message, which contains list of active robots.
 */
void trajectoryReader::checkActivity(const orunav_msgs::ActiveRobotsConstPtr& actrob)
{
    // Not needed at the moment.
    is_active = true;

    
    // bool i_am_active = false;


    // for (unsigned int i = 0; i < actrob->robot_ids.size(); ++i)
    // {
    //     if (thdata->parameters.robot_id == actrob->robot_ids[i])
    //     {
    //         i_am_active = true;
    //     }
    // }


    // swProcessStatus status = thdata->status.get();
    // if ( (i_am_active == false) 
    //         && ((status == SW_PROCESS_STATUS_ACTIVE) || (status == SW_PROCESS_STATUS_FINALIZE)) )
    // {
    //     ROS_WARN("Tracking is active, while the robot is not in the active list.");
    //     thdata->status.set(SW_PROCESS_STATUS_FAIL);
    // }

    // is_active = i_am_active;
}
