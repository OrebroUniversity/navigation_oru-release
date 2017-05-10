#pragma once

#include <sys/socket.h>
#include <netinet/in.h>

#include "commonDefines.h"
#include "state.h"
#include "threadData.h"
#include "swLogger.h"


/**
 * @brief Connection to the Navserver (runs on board of a car). This connection
 * is used to obtain the current state of a car. Several strong assumptions are
 * made on parameters of the connection.
 *
 * @swAssume Assumes, that the state and data from encoders are sent in the same
 * TCP packet.
 *
 * @swAssume Assumes, that the delays in transmission of the TCP packets are
 * small (less than 20 ms).
 *
 * @swAssume Assumes, that no more than one message can be available in the reception 
 * buffer at the same time.
 */
class navserverConnection
{
    public:
        navserverConnection();
        threadReturn readLoop(threadData *);


    private:
        /// Socket connection.
        int navserv_conn;
        /// Adderess of the Navserver
        sockaddr_in navserv_address;


        void openConnection();
        void closeConnection();
        void readMessage(const size_t, char *);
        void parseMessage(const char *, State &) const;
};
