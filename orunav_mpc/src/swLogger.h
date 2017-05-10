#pragma once

#include "commonDefines.h"


#ifdef SW_ENABLE_LOGGING


#include <fstream>
#include <sstream>
#include <queue>
#include <string>

#include "threadData.h"
#include "sharedAccess.h"


enum swLoggerStreamID
{
    SW_LOG_CONTROL_ID = 0,
    SW_LOG_TRAJECTORY_ID = 1,
    SW_LOG_SENSOR_ID = 2,
    SW_LOG_EVALUATION_ID = 3,

    SW_LOG_STREAM_NUM = 4
};


#define SW_LOG(id, message)     { ostringstream log_stream12(ostringstream::out); \
                                  log_stream12 << message; \
                                  sw_logger.pushString(id, log_stream12.str()); }

#define SW_LOG_CONTROL(message)     SW_LOG(SW_LOG_CONTROL_ID, message << endl)
#define SW_LOG_TRAJECTORY(message)  SW_LOG(SW_LOG_TRAJECTORY_ID, message << endl)
#define SW_LOG_SENSOR(message)      SW_LOG(SW_LOG_SENSOR_ID, message << endl)
#define SW_LOG_EVALUATION(message)  SW_LOG(SW_LOG_EVALUATION_ID, message << endl)

#define SW_LOG_FAILEDQP_START       qpLogger qp_logger;
#define SW_LOG_FAILEDQP(message)    qp_logger.failed_qp_log << message;


/**
 * @brief Logger
 */
class swLogger
{
    public:
        swLogger ();
        ~swLogger ();
        void open(int robotId);
        void flushLoop(threadData *);
        void pushString(const swLoggerStreamID, const string &);

    private:
        queue<string> stream_queues[SW_LOG_STREAM_NUM];
        swMutex queue_locks[SW_LOG_STREAM_NUM];
        ofstream streams[SW_LOG_STREAM_NUM];

        void flushQueue(const swLoggerStreamID);
};


/**
 * @brief qpLogger
 */
class qpLogger
{
    public:
        /// Log streams
        ofstream failed_qp_log;

        qpLogger ();
        ~qpLogger ();
};

extern swLogger sw_logger;


#else // SW_ENABLE_LOGGING

#define SW_LOG_CONTROL(message)
#define SW_LOG_TRAJECTORY(message)
#define SW_LOG_SENSOR(message)
#define SW_LOG_EVALUATION(message)

#define SW_LOG_FAILEDQP_START
#define SW_LOG_FAILEDQP(message)

#endif // SW_ENABLE_LOGGING
