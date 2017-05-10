#pragma once

#include "commonDefines.h"
#include "sharedAccess.h"
#include "state.h"
#include "control.h"
#include "command.h"
#include "headers_ros.h"
#include "trajectoryPool.h"
#include "parameters.h"


/**
 * @brief Status of the whole process.
 */
enum swProcessStatus
{
    /// The process is waiting.
    SW_PROCESS_STATUS_WAIT,

    /// Tracking is failed. This status can be changed only using a dedicated function.
    SW_PROCESS_STATUS_FAIL,

    /// Tracking is active.
    SW_PROCESS_STATUS_ACTIVE,

    /// The end of trajectory is reached, controller performs final corrections.
    SW_PROCESS_STATUS_FINALIZE,

    /// Stopping of the process was requested. This status cannot be changed.
    SW_PROCESS_STATUS_TERMINATE
};


/**
 * @brief Return status of a thread.
 */
enum threadReturn
{
    /// No more work to do.
    SW_THREAD_RETURN_DONE,
    /// Thread was stopped by request due to changed status of the process.
    SW_THREAD_RETURN_STOPPED
};



class sharedStateData : public swCondition
{
    public:
        sharedStateData();
        void get(State &);
        void set(const swParameters &, const State &);

    private:
        State current_state;

        State init_state;
        bool init_state_set;
        double cosT;
        double sinT;
};



/**
 * @brief A thread-aware container for status.
 */
class sharedStatus
{
    public:
        sharedStatus();
        swProcessStatus get();
        void set(const swProcessStatus);
        void recover();

    private:
        swMutex mutex;
        swProcessStatus status;
};



/**
 * @brief Shared data and functions used by threads.
 */
class threadData
{
    public:
        /// ROS node handle.
        ros::NodeHandle ros_node;

        /// Parameters of the controller.
        swParameters parameters;

        /// Pool of trajectories
        trajectoryPool traj_pool;

        /// Current state.
        sharedStateData state;

        /// Synchronization for evaluator
        swCondition eval_sync;

        /// Status of the process.
        sharedStatus status;


        threadData();
        void init();

        void terminateProcess();
        bool isProcessTerminated();
};
