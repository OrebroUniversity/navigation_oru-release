#include <cstdlib>
#include <sys/types.h>
#include <sched.h>

#include <iostream>

#include <signal.h>


#include "headers_ros.h"
#include "threadData.h"
#include "threadPool.h"
#include "controller.h"
#include "trajectoryReader.h"
#include "trajectoryEvaluator.h"
#include "swLogger.h"


#define SW_FATAL(s) ROS_ERROR("%s", (s));
#define SW_DIE(s) SW_FATAL(s); exit (-1);


#ifdef SW_BUILD_SIMULATION

#include "ROSSensorSubscriber.h"

#else // SW_BUILD_SIMULATION

#ifdef SW_ENABLE_NAVSERVER_READER
#include "navserverConnection.h"
#else
#include "canSensorReader.h"
#endif

#endif // SW_BUILD_SIMULATION




/**
 * @brief Sensor reader.
 *
 * @param[in] thread_data_ptr thread data.
 *
 * @return Nothing.
 */
void* startSensorReader (void *thread_data_ptr)
{
    threadData *thread_data = (threadData *) thread_data_ptr;
    
    try
    {
#ifdef SW_BUILD_SIMULATION
        ROSSensorSubscriber reader;
#else // SW_BUILD_SIMULATION
#ifdef SW_ENABLE_NAVSERVER_READER
        navserverConnection reader;
#else
        canSensorReader reader;
#endif
#endif // SW_BUILD_SIMULATION

        reader.readLoop(thread_data);
    }
    catch (const exception &e)
    {
        SW_FATAL(e.what());
        if (kill (getpid(), SIGINT) != 0)
        {
            SW_DIE("Cannot send SIGINT to the process.");
        }
    }

    return NULL;
}


/**
 * @brief Thread for the trajectory reader.
 *
 * @param[in] thread_data_ptr thread data.
 *
 * @return Nothing.
 */
void* startTrajectoryReader (void *thread_data_ptr)
{
    threadData *thread_data = (threadData *) thread_data_ptr;
    
    try
    {
        trajectoryReader traj_reader;
        traj_reader.readLoop(thread_data);
    }
    catch (const exception &e)
    {
        SW_FATAL(e.what());
        if (kill (getpid(), SIGINT) != 0)
        {
            SW_DIE("Cannot send SIGINT to the process.");
        }
    }

    return NULL;
}


/**
 * @brief Thread for the controller.
 *
 * @param[in] thread_data_ptr thread data.
 *
 * @return Nothing.
 */
void* startController (void *thread_data_ptr)
{
    threadData *thread_data = (threadData *) thread_data_ptr;

    try
    {
        Controller controller;
        controller.controlLoop(thread_data);
    }
    catch (const exception &e)
    {
        SW_FATAL(e.what());
        if (kill (getpid(), SIGINT) != 0)
        {
            SW_DIE("Cannot send SIGINT to the process.");
        }
    }

    return NULL;
}


/**
 * @brief Thread of trajectory evaluator.
 *
 * @param[in] thread_data_ptr thread data.
 *
 * @return Nothing.
 */
void* startEvaluator (void *thread_data_ptr)
{
    threadData *thread_data = (threadData *) thread_data_ptr;

    try
    {
        trajectoryEvaluator evaluator;
        evaluator.evaluatorLoop(thread_data);
    }
    catch (const exception &e)
    {
        SW_FATAL(e.what());
        if (kill (getpid(), SIGINT) != 0)
        {
            SW_DIE("Cannot send SIGINT to the process.");
        }
    }

    return NULL;
}


#ifdef SW_ENABLE_LOGGING
/**
 * @brief Logger.
 *
 * @param[in] thread_data_ptr thread data.
 *
 * @return Nothing.
 */
void* startLogger (void *thread_data_ptr)
{
    threadData *thread_data = (threadData *) thread_data_ptr;
    sw_logger.open(thread_data->parameters.robot_id);
    try
    {
        sw_logger.flushLoop(thread_data);
    }
    catch (const exception &e)
    {
        SW_FATAL(e.what());
        if (kill (getpid(), SIGINT) != 0)
        {
            SW_DIE("Cannot send SIGINT to the process.");
        }
    }

    return NULL;
}
#endif // SW_ENABLE_LOGGING


/***************** Main *****************************/
int main(int argc, char** argv)
{
#ifdef SW_BUILD_SIMULATION
  std::cout << "SW_BUILD_SIMULATION == ON\n";
#else
    std::cout << "SW_BUILD_SIMULATION == OFF\n";
#endif
  
    ros::init (argc, argv, "sw_controller", ros::init_options::NoSigintHandler);

    threadData thread_data;
    threadPool thread_pool;

    try
    {
        thread_data.init();

        thread_pool.startTrajectoryThread(startTrajectoryReader, &thread_data);
        thread_pool.startSensorThread(startSensorReader, &thread_data);
        thread_pool.startControlThread(startController, &thread_data);
        thread_pool.startEvaluationThread(startEvaluator, &thread_data);
#ifdef SW_ENABLE_LOGGING
        thread_pool.startLoggerThread(startLogger, &thread_data);
#endif // SW_ENABLE_LOGGING


        sigset_t signals;
        int recv_signal;
        sigaddset(&signals, SIGINT);
        thread_pool.blockSigint();
        sigwait(&signals, &recv_signal);
        thread_data.terminateProcess();


        // Give threads some time to finish. In a normal operation
        // it is enough to wait one sampling period. If it is not 
        // enough, then something is wrong anyway.
        usleep (SW_THREAD_WAIT_TIME);
        thread_pool.cancelThreads();
    }
    catch (const exception &e)
    {
        SW_DIE(e.what());
    }

    exit (0);
}
