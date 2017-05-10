#include <cmath>

#include "controller.h"
#include "commandSender.h"
#include "swTimer.h"
#include "swLogger.h"


//============================================================
// State container
//============================================================
stateContainer::stateContainer()
{
    is_initialized = false;
}


void stateContainer::waitSensor(threadData *thread_data)
{
    thread_data->state.wait();
    thread_data->state.get(sensor_state);

    if (is_initialized == false)
    {
        estimated_state = sensor_state;
        is_initialized = true;
    }
    else
    {
        estimated_state.feedbackError(
                expected_state, 
                sensor_state,
                thread_data->parameters.error_feedback_gains);
    }
    thread_data->state.set(thread_data->parameters, estimated_state);
}


const State & stateContainer::sensor() const
{
    return (sensor_state);
}

const State & stateContainer::expected() const
{
    return (expected_state);
}

const State & stateContainer::estimated() const
{
    return (estimated_state);
}


void stateContainer::setExpected(const State &exp_state)
{
    expected_state = exp_state;
}


//============================================================
// Controller
//============================================================

/**
 * @brief Constructor.
 */
Controller::Controller()
{
    graceful_counter = 0;
    last_command_negligible = false;
}


/**
 * @brief A loop, which runs in a separate thread.
 *
 * @param[in] thread_data shared data
 *
 * @return #threadReturn
 */
threadReturn Controller::controlLoop(threadData *thread_data)
{
    threadReturn retval = SW_THREAD_RETURN_DONE;

    Command command;
    // An instance of command sender.
    commandSender command_sender(thread_data);
    qpProblem qp(thread_data->parameters);

    try
    {
        for (;;)
        {
            SW_LOG_CONTROL(">>> start");

            if (thread_data->isProcessTerminated())
            {
                computeStopCommand(command);
                command_sender.send(state.sensor(), command);
                retval = SW_THREAD_RETURN_STOPPED;
                break;
            }

            state.waitSensor(thread_data);

            if (thread_data->isProcessTerminated())
            {
                computeStopCommand(command);
                command_sender.send(state.sensor(), command);
                retval = SW_THREAD_RETURN_STOPPED;
                break;
            }

            SW_LOG_CONTROL("Sensor state: " << state.sensor());
            SW_LOG_CONTROL("Expected state: " << state.expected());
            SW_LOG_CONTROL("Estimated state: " << state.estimated());

            swTimer timer;
            swProcessStatus status = getCommand (thread_data, qp, command);
            timer.stop();
            SW_LOG_CONTROL("Duration of control iteration: " << timer);


            command_sender.send(state.sensor(), command);
            last_command_negligible = command_sender.isNegligible(command);

            SW_LOG_CONTROL("<<< end");
    
            if (status == SW_PROCESS_STATUS_FAIL)
            {
                thread_data->traj_pool.clear();
            }
            else
            {
                thread_data->traj_pool.prune();
            }

            thread_data->status.set(status);
            thread_data->eval_sync.signal();
        }
    }
    catch (const exception &e)
    {
        // Attempt to stop the car on exception
        SW_LOG_CONTROL("Exception caught: " << e.what());
        computeStopCommand(command);
        command_sender.send(state.sensor(), command);
        thread_data->terminateProcess();
        throw; // rethrow
    }

    return (retval);
}



/**
 * @brief Compute command.
 *
 * @param[in] thread_data shared data
 * @param[in] qp an instance of QP problem class
 * @param[in,out] command old command / computed command
 *
 * @return status to which the controller must switch (#swProcessStatus).
 */
swProcessStatus Controller::getCommand(
        threadData * thread_data,
        qpProblem & qp,
        Command & command)
{
    swProcessStatus retval;


    switch (thread_data->status.get())
    {
        case SW_PROCESS_STATUS_ACTIVE:
            retval = getCommandActive(thread_data, qp, command);
            break;

        case SW_PROCESS_STATUS_WAIT:
            retval = getCommandWait(thread_data, qp, command);
            break;

        case SW_PROCESS_STATUS_FINALIZE:
            retval = getCommandFinalize(thread_data, qp, command);
            break;

        case SW_PROCESS_STATUS_FAIL:
            retval = getCommandFail(thread_data->parameters, qp, command);
            break;

        default:
            retval = SW_PROCESS_STATUS_TERMINATE;
            computeStopCommand(command);
            break;
    }

    SW_LOG_CONTROL("Computed car control: " << command);


    if (thread_data->parameters.enable_closed_loop == false)
    {
        // Replace the command by the reference one, if closed loop is disabled.
        if (retval == SW_PROCESS_STATUS_ACTIVE)
        {
            qp.preview_win.getRefCommand(command);
        }
        else
        {
            computeStopCommand(command);
        }
    }

    return (retval);
}



/**
 * @brief Compute command (process status == SW_PROCESS_STATUS_FAIL)
 *
 * @param[in] parameters parameters of the controller
 * @param[in] qp an instance of QP problem class
 * @param[in,out] command old command / computed command
 *
 * @return status to which the controller must switch (#swProcessStatus).
 */
swProcessStatus Controller::getCommandFail(
        const swParameters &parameters,
        qpProblem & qp,
        Command & command)
{
    if ((graceful_counter < parameters.max_graceful_emergency_brake_steps)
            && (last_command_negligible == false))
    {
        ++graceful_counter;

        if (cache.fillBrake(parameters) == SW_CACHE_OK)
        {
            if (computeQPCommand(parameters, qp, command) == SW_QP_FAILED)
            {
                SW_LOG_CONTROL("Graceful brake: cannot solve QP. Stop.");
                computeStopCommand(command);
                cache.reset();
            }
            // else do not reset cache
        }
        else
        {
            SW_LOG_CONTROL("Graceful emergency brake: cannot form preview window. Stop.");
            computeStopCommand(command);
            cache.reset();
        }
    }
    else
    {
        SW_LOG_CONTROL("Brake: the car is stopped.");
        computeStopCommand(command);
        cache.reset();
    }

    return (SW_PROCESS_STATUS_FAIL);
}


/**
 * @brief Compute command (process status == SW_PROCESS_STATUS_FINALIZE)
 *
 * @param[in] thread_data shared data
 * @param[in] qp an instance of QP problem class
 * @param[in,out] command old command / computed command
 *
 * @return status to which the controller must switch (#swProcessStatus).
 */
swProcessStatus Controller::getCommandFinalize(
        threadData * thread_data,
        qpProblem & qp,
        Command & command)
{
    swProcessStatus retval;

    if ((graceful_counter < thread_data->parameters.max_graceful_brake_steps)
            && (last_command_negligible == false))
    {
        ++graceful_counter;

        if (cache.fillFinalize() == SW_CACHE_OK)
        {
            if (computeQPCommand(thread_data->parameters, qp, command) == SW_QP_FAILED)
            {
                SW_LOG_CONTROL("Car is stopped due to a failure.");
                retval = getCommandFail(thread_data->parameters, qp, command);
            }
            else
            {
                retval = SW_PROCESS_STATUS_FINALIZE;
            }
        }
        else
        {
            SW_LOG_CONTROL("Graceful brake: cannot form preview window. Stop.");
            retval = SW_PROCESS_STATUS_FAIL;
            computeStopCommand(command);
            cache.reset();
        }
    }
    else
    {
        SW_LOG_CONTROL("The goal is reached.");
        cache.reset();
        thread_data->traj_pool.clear();
        computeStopCommand(command);
        retval = SW_PROCESS_STATUS_WAIT;
    }

    return (retval);
}


/**
 * @brief Compute command (process status == SW_PROCESS_STATUS_WAIT)
 *
 * @param[in] thread_data shared data
 * @param[in] qp an instance of QP problem class
 * @param[in,out] command old command / computed command
 *
 * @return status to which the controller must switch (#swProcessStatus).
 */
swProcessStatus Controller::getCommandWait(
        threadData * thread_data,
        qpProblem & qp,
        Command & command)
{
    swProcessStatus retval;


    graceful_counter = 0;

    trajectoryCache tmp_cache = cache;
    thread_data->traj_pool.getActiveTrajectory(state.estimated(), control, cache);

    switch (cache.checkFill())
    {
        case SW_CACHE_OK:
            if (computeQPCommand(thread_data->parameters, qp, command) == SW_QP_FAILED)
            {
                SW_LOG_CONTROL("Car is stopped due to a failure.");
                retval = getCommandFail(thread_data->parameters, qp, command);
            }
            else
            {
                retval = SW_PROCESS_STATUS_ACTIVE;
            }
            break;

        case SW_CACHE_NODATA:
            SW_LOG_CONTROL("No data -- waiting.");
            computeStopCommand(command);
            retval = SW_PROCESS_STATUS_WAIT;
            break;

        default:
            cache = tmp_cache;
            SW_LOG_CONTROL("Car is stopped due to a failure.");
            retval = getCommandFail(thread_data->parameters, qp, command);
            break;
    }

    return (retval);
}



/**
 * @brief Compute command (process status == SW_PROCESS_STATUS_ACTIVE)
 *
 * @param[in] thread_data shared data
 * @param[in] qp an instance of QP problem class
 * @param[in,out] command old command / computed command
 *
 * @return status to which the controller must switch (#swProcessStatus).
 */
swProcessStatus Controller::getCommandActive(
        threadData * thread_data,
        qpProblem & qp,
        Command & command)
{
    swProcessStatus retval;

    graceful_counter = 0;

    trajectoryCache tmp_cache = cache;
    thread_data->traj_pool.getActiveTrajectory(state.estimated(), control, cache);

    switch (cache.checkFill())
    {
        case SW_CACHE_OK:
            if (computeQPCommand(thread_data->parameters, qp, command) == SW_QP_FAILED)
            {
                SW_LOG_CONTROL("Car is stopped due to a failure.");
                retval = getCommandFail(thread_data->parameters, qp, command);
            }
            else
            {
                retval = SW_PROCESS_STATUS_ACTIVE;
            }
            break;

        case SW_CACHE_END:
            SW_LOG_CONTROL("The end is reached -- finalizing.");
            retval = getCommandFinalize(thread_data, qp, command);
            break;

        default:
            cache = tmp_cache;
            SW_LOG_CONTROL("Car is stopped due to a failure.");
            retval = getCommandFail(thread_data->parameters, qp, command);
            break;
    }

    return (retval);
}



/**
 * @brief Compute command, which stops a car.
 *
 * @param[out] command command.
 */
void Controller::computeStopCommand(Command &command)
{
    control.set(0.0, 0.0);
    command.initStop(state.sensor());
    state.setExpected(state.sensor());
}



/**
 * @brief Solve QP in order to obtain command.
 *
 * @param[in] parameters parameters of the controller
 * @param[in] qp an instance of QP problem class
 * @param[in,out] command old command / computed command
 *
 * @return #qpReturn
 */
qpReturn Controller::computeQPCommand(
        const swParameters &parameters,
        qpProblem & qp,
        Command & command)
{
    qp.preview_win.form(cache);

#ifdef SW_ENABLE_LOGGING
    State reference_state;
    qp.preview_win.getRefState(reference_state);
    SW_LOG_CONTROL("Reference state: " << reference_state);

    Command reference_command;
    qp.preview_win.getRefCommand(reference_command);
    SW_LOG_CONTROL("Reference car control: " << reference_command);
#endif //SW_ENABLE_LOGGING


    State expected_state;
    swTimer timer;
    qpReturn qp_return = qp.solve(
            parameters,
            state.estimated(),
            expected_state,
            control);
    timer.stop();

    SW_LOG_CONTROL("QP time: " << timer);

    switch (qp_return)
    {
        case SW_QP_FAILED:
            SW_LOG_CONTROL("QP was failed, tracking must be stopped.");
            break;
        case SW_QP_REUSED:
            SW_LOG_CONTROL("QP was failed, reusing old solution.");
        default:
            state.setExpected(expected_state);
            command.set(control.v(), control.w(), state.estimated().phi());
            break;
    }

    return (qp_return);
}
