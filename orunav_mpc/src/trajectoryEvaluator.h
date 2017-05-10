#pragma once

#include <vector>

#include "commonDefines.h"
#include "threadData.h"
#include "headers_ros.h"


/**
 * @brief Result of trajectroy evaluation.
 */
enum swEvaluationResult
{
    /// Ok.
    SW_EVAL_OK,

    /// QP was failed
    SW_EVAL_FAIL,

    /// Not enough data to form problem.
    SW_EVAL_NODATA
};



/**
 * @brief Value of a trajectory.
 */
class trajectoryValue
{
    public:
        /// Status of the value
        swEvaluationResult status;

        /// Value of the trajectory
        double value;

        /// ID of the trajectory
        trajectoryID id;

        /// True if active
        bool active;
};



/**
 * @brief The main loop of this class runs in a separate thread and evaluates trajectories.
 */
class trajectoryEvaluator
{
    public:
        threadReturn evaluatorLoop(threadData *);

        void report(threadData *, const swProcessStatus);

    private:
        vector<trajectoryValue> traj_value;
        ros::Publisher ros_publisher;
};
