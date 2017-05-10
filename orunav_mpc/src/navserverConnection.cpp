#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <iostream>

#include <cstring>
#include <cstdlib>


#include "navserverConnection.h"
#include "swTimer.h"


/**
 * @brief Constructor.
 */
navserverConnection::navserverConnection()
{
    navserv_address.sin_addr.s_addr = inet_addr(SW_NAVSERVER_IP);
    navserv_address.sin_port = htons(SW_NAVSERVER_PORT);
    navserv_address.sin_family = AF_INET;
}


/**
 * @brief Open connection to Navserver and clear reception buffer.
 */
void navserverConnection::openConnection()
{
    navserv_conn = socket(AF_INET, SOCK_STREAM, 0);
    if (navserv_conn < 0)
    {
        SW_THROW_MSG("Call to socket() failed.");
    }
    
    if (connect(navserv_conn, (sockaddr*)&navserv_address, sizeof(navserv_address)) < 0)
    {
        SW_THROW_MSG("Call to connect() failed.");
    }


    if (send(navserv_conn, SW_NAVSERVER_SUBSCRIBE, sizeof(SW_NAVSERVER_SUBSCRIBE), 0) < 0)
    {
        SW_THROW_MSG("Call to send() failed.");
    }


    for (unsigned int counter = 0; counter < SW_NAVSERVER_BUF_CLEAR_ITER; ++counter)
    {
        struct timeval timeout;
        fd_set read_conns;
        char msg_buf[SW_NAVSERVER_READ_BUF_SIZE];

        FD_ZERO(&read_conns);
        FD_SET(navserv_conn, &read_conns);

        timeout.tv_sec = 0;
        timeout.tv_usec = SW_NAVSERVER_SELECT_TIMEOUT;

        if (select(navserv_conn+1, &read_conns, NULL, NULL, &timeout) <= 0)
        {
            SW_THROW_MSG("Call to select() failed.");
        }
        if (recv(navserv_conn, msg_buf, SW_NAVSERVER_READ_BUF_SIZE, MSG_DONTWAIT) < 0)
        {
            SW_THROW_MSG("Call to recv() failed.");
        }
    }
}


/**
 * @brief Close connection.
 */
void navserverConnection::closeConnection()
{
    if (send(navserv_conn, SW_NAVSERVER_STOP, sizeof(SW_NAVSERVER_STOP), 0) < 0)
    {
        SW_THROW_MSG("Call to send() failed.");
    }

    close(navserv_conn);
}


/**
 * @brief Receive one message.
 *
 * @param[in] buf_len length of the local buffer.
 * @param[out] buf local buffer.
 */
void navserverConnection::readMessage(const size_t buf_len, char * buf)
{
    struct timeval timeout;
    fd_set read_conns;

    FD_ZERO(&read_conns);
    FD_SET(navserv_conn, &read_conns);

    timeout.tv_sec = 0;
    timeout.tv_usec = SW_NAVSERVER_SELECT_TIMEOUT;

    int res = select(navserv_conn+1, &read_conns, NULL, NULL, &timeout);
    if (res < 0)
    {
        SW_THROW_MSG("Call to select() failed.");
    }
    else if (res == 0)
    {
        SW_LOG_SENSOR(swGetTime());
        SW_THROW_MSG("Call to select() failed: timeout expired.");
    }


    SW_LOG_SENSOR("Received a packet. " << swGetTime());

    ssize_t msg_len = recv(navserv_conn, buf, buf_len, MSG_DONTWAIT);
    if ((msg_len < 0) || ((size_t) msg_len >= buf_len))
    {
        SW_THROW_MSG("Call to recv() failed.");
    }
    buf[msg_len] = '\0';
}


/**
 * @brief Parse received message.
 *
 * @param[in] msg_buf local buffer containing a message.
 * @param[out] state the current state.
 */
void navserverConnection::parseMessage(const char *msg_buf, State &state) const
{
    char parse_buf[SW_NAVSERVER_READ_BUF_SIZE];
    double data_timestamp;
    const char * line_ptr;


    line_ptr = strstr(msg_buf, "enc");
    if (line_ptr != NULL)
    {
        double v;
        if (sscanf (line_ptr, "%3c %lf %lf %lf", 
                    parse_buf, &data_timestamp, &v, &state.phi()) != 4)
        {
            SW_THROW_MSG("Call to sscanf() failed.");
        }
        SW_LOG_SENSOR("WHEEL: time = " << data_timestamp << "   v = " << v << "   phi = " << state.phi());
    }
    else
    {
        SW_THROW_MSG("Call to strstr() failed.");
    }

    line_ptr = strstr(msg_buf, "state");
    if (line_ptr != NULL)
    {
        if (sscanf (line_ptr, "%5c %lf %lf %lf %lf", 
                    parse_buf, &data_timestamp, &state.x(), &state.y(), &state.theta()) != 5)
        {
            SW_THROW_MSG("Call to sscanf() failed.");
        }
        SW_LOG_SENSOR("POSTURE: time = " << data_timestamp 
                << "   x = " << state.x() 
                << "   y = " << state.y()
                << "   theta = " << state.theta());
    }
    else
    {
        SW_THROW_MSG("Call to strstr() failed.");
    }
}


/**
 * @brief The read loop, which runs in a separate thread.
 *
 * @param[in,out] thread_data thread data.
 */
threadReturn navserverConnection::readLoop(threadData *thread_data)
{
    threadReturn retval = SW_THREAD_RETURN_DONE;
    openConnection();


    for (;;)
    {
        char msg_buf[SW_NAVSERVER_READ_BUF_SIZE];
        State state;

        SW_LOG_SENSOR(">>>");
        readMessage(SW_NAVSERVER_READ_BUF_SIZE, msg_buf);
        parseMessage(msg_buf, state);
        // normalize theta
        state.normalizeTheta();

        SW_LOG_SENSOR(state);
        SW_LOG_SENSOR("<<<");

        thread_data->state.set(thread_data->parameters, state);
        thread_data->state.signal();
        if (thread_data->isProcessTerminated())
        {
            retval = SW_THREAD_RETURN_STOPPED;
            break;
        }
    }

    closeConnection();

    return (retval);
}
