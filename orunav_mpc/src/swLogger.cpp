#include "swLogger.h"

#ifdef SW_ENABLE_LOGGING

#include <unistd.h> // usleep()
#include <limits>
#include <iomanip>
#include <iostream>



swLogger::swLogger ()
{
    // streams[SW_LOG_TRAJECTORY_ID].open(SW_TRAJECTORY_LOG_FILE);
    // streams[SW_LOG_SENSOR_ID].open(SW_SENSOR_LOG_FILE);
    // streams[SW_LOG_CONTROL_ID].open(SW_CONTROLLER_LOG_FILE);
    // streams[SW_LOG_EVALUATION_ID].open(SW_EVALUATION_LOG_FILE);
}

swLogger::~swLogger ()
{
    for (unsigned int i = 0; i < SW_LOG_STREAM_NUM; ++i)
    {
      if (streams[i].is_open()) {
        flushQueue(static_cast<swLoggerStreamID>(i));
        streams[i] << "Done" << endl;
        streams[i].close();
      }
    }
}

void swLogger::open(int robotId) {
    stringstream ss;//create a stringstream
    ss << robotId;//add number to the stream
    string nr = ss.str();
    string file = SW_ROBOT_ID_PREFIX_LOG_FILE+nr;
    streams[SW_LOG_TRAJECTORY_ID].open(std::string(file+SW_TRAJECTORY_LOG_FILE).c_str());
    streams[SW_LOG_SENSOR_ID].open(std::string(file+SW_SENSOR_LOG_FILE).c_str());
    streams[SW_LOG_CONTROL_ID].open(std::string(file+SW_CONTROLLER_LOG_FILE).c_str());
    streams[SW_LOG_EVALUATION_ID].open(std::string(file+SW_EVALUATION_LOG_FILE).c_str());
}

void swLogger::pushString(const swLoggerStreamID id, const string & str)
{
    queue_locks[id].lock();
    stream_queues[id].push(str);
    queue_locks[id].unlock();
}


void swLogger::flushQueue(const swLoggerStreamID id)
{
    const unsigned int num = stream_queues[id].size();

    for (unsigned int i = 0; i < num; ++i)
    {
        queue_locks[id].lock();
        const string str = stream_queues[id].front();
        stream_queues[id].pop();
        queue_locks[id].unlock();

        streams[id] << str;
    }
}


void swLogger::flushLoop(threadData *thdata)
{
    for (;;)
    {
        if (thdata->isProcessTerminated())
        {
            break;
        }

        usleep(SW_SAMPLING_PERIOD_MICROSEC);

        for (unsigned int i = 0; i < SW_LOG_STREAM_NUM; ++i)
        {
            flushQueue(static_cast<swLoggerStreamID>(i));
        }
    }
}



qpLogger::qpLogger()
{
    failed_qp_log.open(QP_LOG_FILE);
    failed_qp_log << setprecision(std::numeric_limits<double>::digits10);
}

qpLogger::~qpLogger()
{
    failed_qp_log.close();
}


swLogger sw_logger;
#endif //SW_ENABLE_LOGGING
