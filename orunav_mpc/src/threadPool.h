#pragma once

#include <pthread.h>
#include <signal.h>

#include "headers_ros.h"


/**
 * @brief Shared data and functions used by threads.
 */
class threadPool
{
    public:
// >>> PRIVATE
        /// Handle of a thread, which reads data from sensors.
        pthread_t sensor_thread;
        /// Handle of a thread, which generates control commands.
        pthread_t control_thread;
        /// Handle of a thread, which aquires trajectory.
        pthread_t trajectory_thread;
        /// Handle of a thread, which evaluates trajectories.
        pthread_t evaluation_thread;
        /// Handle of a logger thread.
        pthread_t logger_thread;


// >>> PUBLIC
        /**
         * @brief Start the sensor thread.
         *
         * @param[in] start_routine pointer to a thread function.
         * @param[in,out] data_ptr this pointer is passed to the created thread.
         */
        void startSensorThread(void *(*start_routine)(void *), void *data_ptr)
        {
            blockSigint(); // The new thread inherits signal mask
            if (pthread_create(&sensor_thread, NULL, start_routine, data_ptr) != 0)
            {
                SW_THROW_MSG("Call to pthread_create() failed.");
            }
            unblockSigint();

            struct sched_param param;
            param.sched_priority = SW_SCHED_PRIORITY;
            if (pthread_setschedparam(sensor_thread, SCHED_FIFO, &param) != 0)
            {
                ROS_WARN("Cannot change the priority of the sensor thread.");
            }
        }


        /**
         * @brief Start the control thread.
         *
         * @param[in] start_routine pointer to a thread function.
         * @param[in,out] data_ptr this pointer is passed to the created thread.
         */
        void startControlThread(void *(*start_routine)(void *), void *data_ptr)
        {
            blockSigint(); // The new thread inherits signal mask
            if (pthread_create(&control_thread, NULL, start_routine, data_ptr) != 0)
            {
                SW_THROW_MSG("Call to pthread_create() failed.");
            }
            unblockSigint();

            struct sched_param param;
            param.sched_priority = SW_SCHED_PRIORITY;
            if (pthread_setschedparam(control_thread, SCHED_FIFO, &param) != 0)
            {
                ROS_WARN("Cannot change the priority of the control thread.");
            }
        }


        /**
         * @brief Start the thread, which reads trajectories.
         *
         * @param[in] start_routine pointer to a thread function.
         * @param[in,out] data_ptr this pointer is passed to the created thread.
         */
        void startTrajectoryThread(void *(*start_routine)(void *), void *data_ptr)
        {
            blockSigint(); // The new thread inherits signal mask
            if (pthread_create(&trajectory_thread, NULL, start_routine, data_ptr) != 0)
            {
                SW_THROW_MSG("Call to pthread_create() failed.");
            }
            unblockSigint();
        }


        /**
         * @brief Start the thread, which evaluates trajectories.
         *
         * @param[in] start_routine pointer to a thread function.
         * @param[in,out] data_ptr this pointer is passed to the created thread.
         */
        void startEvaluationThread(void *(*start_routine)(void *), void *data_ptr)
        {
            blockSigint(); // The new thread inherits signal mask
            if (pthread_create(&evaluation_thread, NULL, start_routine, data_ptr) != 0)
            {
                SW_THROW_MSG("Call to pthread_create() failed.");
            }
            unblockSigint();
        }


        /**
         * @brief Start the logger thread.
         *
         * @param[in] start_routine pointer to a thread function.
         * @param[in,out] data_ptr this pointer is passed to the created thread.
         */
        void startLoggerThread(void *(*start_routine)(void *), void *data_ptr)
        {
            blockSigint(); // The new thread inherits signal mask
            if (pthread_create(&logger_thread, NULL, start_routine, data_ptr) != 0)
            {
                SW_THROW_MSG("Call to pthread_create() failed.");
            }
            unblockSigint();
        }


        /**
         * @brief Disable delivery of SIGINT signal to the caller thread.
         */
        void blockSigint()
        {
            sigset_t block_signals;
            sigaddset(&block_signals, SIGINT);
            if (pthread_sigmask(SIG_BLOCK, &block_signals, NULL) != 0)
            {
                SW_THROW_MSG("Call to pthread_sigmask() failed.");
            }
        }


        /**
         * @brief Enable delivery of SIGINT signal to the caller thread.
         */
        void unblockSigint()
        {
            sigset_t unblock_signals;
            sigaddset(&unblock_signals, SIGINT);
            if (pthread_sigmask(SIG_UNBLOCK, &unblock_signals, NULL) != 0)
            {
                SW_THROW_MSG("Call to pthread_sigmask() failed.");
            }
        }


        /**
         * @brief Force termination of the threads.
         */
        void cancelThreads()
        {
            pthread_cancel (sensor_thread);
            pthread_cancel (control_thread);
            pthread_cancel (trajectory_thread);
        }
};

