#include <pthread.h>
#include <unistd.h>

#include "threadData.h"


sharedStateData::sharedStateData()
{
    init_state_set = false;
}


void sharedStateData::get(State & state_out)
{
    swMutexScopedLock state_lock(*this);
    state_out = current_state;
}


void sharedStateData::set(
        const swParameters &parameters,
        const State &state_in)
{
    swMutexScopedLock state_lock(*this);


    if (parameters.enable_state_transformation)
    {
        if (!init_state_set)
        {
            init_state = state_in;
            sinT = sin(init_state.theta());
            cosT = cos(init_state.theta());

            init_state_set = true;
        }

        current_state = state_in;
        current_state.transform(init_state, sinT, cosT);        
    }
    else
    {
        current_state = state_in;
    }
}


//=====================================================================================
// sharedStatus
//=====================================================================================
/**
 * @brief Constructor
 */
sharedStatus::sharedStatus()
{
    swMutexScopedLock get_lock(mutex);
    status = SW_PROCESS_STATUS_WAIT;
}


/**
 * @brief Returns status of the controller.
 *
 * @return swProcessStatus
 */
swProcessStatus sharedStatus::get()
{
    swMutexScopedLock get_lock(mutex);
    return(status);
}


/**
 * @brief Sets status.
 *
 * @note Allowed status changes:
 *  - any status  ->  the same status;
 *  - any  ->  SW_PROCESS_STATUS_TERMINATE;
 *  - any except SW_PROCESS_STATUS_TERMINATE  ->  SW_PROCESS_STATUS_FAIL;
 *  - SW_PROCESS_STATUS_WAIT  ->  SW_PROCESS_STATUS_ACTIVE;
 *  - SW_PROCESS_STATUS_ACTIVE  ->  SW_PROCESS_STATUS_FINALIZE, SW_PROCESS_STATUS_WAIT;
 *  - SW_PROCESS_STATUS_FINALIZE  ->  SW_PROCESS_STATUS_WAIT.
 *
 * @param[in] status_new new status
 */
void sharedStatus::set(const swProcessStatus status_new)
{
    swMutexScopedLock get_lock(mutex);

    if (status == status_new)
    {
        // no need to do anything
        return;
    }

    if (status == SW_PROCESS_STATUS_TERMINATE)
    {
        // this status cannot be changed
        return;
    }

    switch (status_new)
    {
        case SW_PROCESS_STATUS_TERMINATE:
        case SW_PROCESS_STATUS_FAIL:
            status = status_new;
            break;
        case SW_PROCESS_STATUS_ACTIVE:
            if (status != SW_PROCESS_STATUS_WAIT) 
            {
                status = SW_PROCESS_STATUS_FAIL;
            }
            else
            {
                status = status_new;
            }
            break;
        case SW_PROCESS_STATUS_FINALIZE:
            if (status != SW_PROCESS_STATUS_ACTIVE)
            {
                status = SW_PROCESS_STATUS_FAIL;
            }
            else
            {
                status = status_new;
            }
            break;
        case SW_PROCESS_STATUS_WAIT:
            if ((status != SW_PROCESS_STATUS_ACTIVE)
                    && (status != SW_PROCESS_STATUS_FINALIZE))
            {
                status = SW_PROCESS_STATUS_FAIL;
            }
            else
            {
                status = status_new;
            }
            break;
        default:
            status = SW_PROCESS_STATUS_TERMINATE;
            break;
    }
}


/**
 * @brief Recover after failure. If the current status is not SW_PROCESS_STATUS_FAIL
 * does nothing, otherwise sets status to SW_PROCESS_STATUS_WAIT.
 */
void sharedStatus::recover()
{
    swMutexScopedLock get_lock(mutex);
    if (status == SW_PROCESS_STATUS_FAIL)
    {
        status = SW_PROCESS_STATUS_WAIT;
    }
    else
    {
        SW_THROW_MSG("Recover is impossible, since execution was not failed.");
    }
}


//=====================================================================================
// threadData
//=====================================================================================


/**
 * @brief Constructor.
 */
threadData::threadData() : ros_node("~")
{
}


/**
 * @brief Initialize.
 */
void threadData::init()
{
    parameters.init(ros_node);
}


/**
 * @brief Request termination of the process.
 */
void threadData::terminateProcess()
{
    // inform thread that we are going to stop
    status.set(SW_PROCESS_STATUS_TERMINATE);
    // wake up the controller thread
    state.signal();
    eval_sync.signal();
}


/**
 * @brief Returns true if process is terminated.
 *
 * @return true/false.
 */
bool threadData::isProcessTerminated()
{
    return (status.get() == SW_PROCESS_STATUS_TERMINATE);
}
