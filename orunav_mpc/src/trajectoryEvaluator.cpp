#include "trajectoryEvaluator.h"
#include "qpProblem.h"
#include "swLogger.h"
#include "swTimer.h"

#include "headers_ros.h"


/**
 * @brief A loop, which runs in a separate thread.
 *
 * @param[in] thread_data shared data
 *
 * @return #threadReturn
 */
threadReturn trajectoryEvaluator::evaluatorLoop(threadData *thread_data)
{
    threadReturn retval = SW_THREAD_RETURN_DONE;

    ros_publisher = thread_data->ros_node.advertise<orunav_msgs::ControllerReport>(
            thread_data->parameters.topic_report, 1);


    vector<trajectoryCache> trajectories;

    State current_state;
    Control last_control;

    qpProblem qp(thread_data->parameters);


    for (;;)
    {
        if (thread_data->isProcessTerminated())
        {
            retval = SW_THREAD_RETURN_STOPPED;
            break;
        }

        thread_data->eval_sync.wait();

        if (thread_data->isProcessTerminated())
        {
            retval = SW_THREAD_RETURN_STOPPED;
            break;
        }


        swTimer timer;


        traj_value.clear();

        swProcessStatus status = thread_data->status.get();

        if (status == SW_PROCESS_STATUS_ACTIVE)
        {
            thread_data->traj_pool.getAllTrajectories (
                    trajectories,
                    current_state,
                    last_control);
            traj_value.resize(trajectories.size());


            for (unsigned int i = 0; i < trajectories.size(); ++i)
            {
                traj_value[i].id = trajectories[i].id;
                traj_value[i].active = trajectories[i].active;

                if (trajectories[i].getSize() < SW_PREVIEW_WINDOW_LEN_EXT)
                {
                    traj_value[i].status = SW_EVAL_NODATA;
                    traj_value[i].value = 0;
                    continue;
                }

                qp.preview_win.form(trajectories[i]);
                Control last_control_copy = last_control;
                qpReturn qp_return = qp.solve(
                        thread_data->parameters,
                        current_state,
                        last_control_copy,
                        traj_value[i].value);

                if (qp_return == SW_QP_OK)
                {
                    traj_value[i].status = SW_EVAL_OK;
                }
                else
                {
                    traj_value[i].status = SW_EVAL_FAIL;
                }
            }
        }

        report(thread_data, status);
        timer.stop();
        SW_LOG_EVALUATION("Duration of the iteration = " << timer);
    }

    return (retval);
}



/**
 * @brief Report results.
 *
 * @param[in] thread_data shared data.
 * @param[in] status status of the controller.
 *
 * One report includes (see orunav_msgs::ControllerReport):
 *  - ID of the car.
 *
 *  - The current status of the controller (#swProcessStatus) 
 *
 *  - The current state of a car (position, orientation, steering angle).
 *
 *  - When the status is #SW_PROCESS_STATUS_ACTIVE, results of trajectory evaluation are also 
 *  reported for each trajectory:
 *      - ID of the trajectory.
 *      - The result of attempt #swEvaluationResult.
 *      - The value of objective function (a positive floating point number), if the problem 
 *      is solved. The value of objective function is a weighted sum of deviations from the
 *      reference states and controls. The weights are essentially the gains of QP problem.
 */
void trajectoryEvaluator::report(
        threadData *thread_data, 
        const swProcessStatus status)
{
    orunav_msgs::ControllerReport ctrl_report;


    // ID
    ctrl_report.robot_id = thread_data->parameters.robot_id;


    // Status
    switch (status)
    {
        case SW_PROCESS_STATUS_ACTIVE:
            ctrl_report.status = ctrl_report.CONTROLLER_STATUS_ACTIVE;
            break;
        case SW_PROCESS_STATUS_WAIT:
            ctrl_report.status = ctrl_report.CONTROLLER_STATUS_WAIT;
            break;
        case SW_PROCESS_STATUS_FAIL:
            ctrl_report.status = ctrl_report.CONTROLLER_STATUS_FAIL;
            break;
        case SW_PROCESS_STATUS_FINALIZE:
            ctrl_report.status = ctrl_report.CONTROLLER_STATUS_FINALIZE;
            break;
        case SW_PROCESS_STATUS_TERMINATE:
            ctrl_report.status = ctrl_report.CONTROLLER_STATUS_TERMINATE;
            break;
    }


    // State
    State current_state;
    thread_data->state.get(current_state);

    ctrl_report.state.steering_angle = current_state.phi();
    ctrl_report.state.orientation_angle = current_state.theta();
    ctrl_report.state.position_x = current_state.x();
    ctrl_report.state.position_y = current_state.y();

    ctrl_report.stamp = ros::Time::now();


    // Values
    ctrl_report.traj_values.clear();
    for (unsigned int i = 0; i < traj_value.size(); ++i)
    {
        orunav_msgs::ControllerTrajectoryValue value;

        value.traj_id = traj_value[i].id;
        value.value = traj_value[i].value;
        value.active = traj_value[i].active;

        switch (traj_value[i].status)
        {
            case SW_EVAL_OK:
                value.status = value.VALUE_OK;
                break;
            case SW_EVAL_FAIL:
                value.status = value.VALUE_FAIL;
                break;
            case SW_EVAL_NODATA:
                value.status = value.VALUE_NODATA;
                break;
        }

        ctrl_report.traj_values.push_back(value);
    }

    // Trajectory indexes
    if (status == SW_PROCESS_STATUS_ACTIVE) {
        trajectoryPoolIndex indexes;
        thread_data->traj_pool.getActiveTrajectoryIndexes(indexes);
        ctrl_report.traj_chunk_sequence_num = indexes.chunk;
        ctrl_report.traj_step_sequence_num = indexes.step;
    }       
    else
    {
        ctrl_report.traj_chunk_sequence_num = 0;
        ctrl_report.traj_step_sequence_num = 0;
    }
    // publish
    ros_publisher.publish(ctrl_report);
}
