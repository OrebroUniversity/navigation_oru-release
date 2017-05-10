#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <unistd.h>

#include <iostream>
#include <cstdio>

#include <cstring>
#include <cstdlib>

#include <vector>


#define NAVSERVER_IP "192.168.100.100"
#define NAVSERVER_PORT 5432
#define NAVSERVER_SUBSCRIBE "subscribe state enc\r\n"
#define NAVSERVER_STOP "!\r\n"
#define BUFFER_SIZE 512

using namespace std;


int main ()
{
    int navserv_conn;
    sockaddr_in navserv_address;
    struct timeval timeout;
    struct timeval oldtime;
    struct timeval currtime;
    fd_set read_conns;
    vector<double> timediffs;
    char msg_buf[BUFFER_SIZE];
    char parse_buf[BUFFER_SIZE];



    navserv_address.sin_addr.s_addr = inet_addr(NAVSERVER_IP);
    navserv_address.sin_family = AF_INET;
    navserv_address.sin_port = htons(NAVSERVER_PORT);

    navserv_conn = socket(AF_INET, SOCK_STREAM, 0);


    if (connect(navserv_conn, (sockaddr*)&navserv_address, sizeof(navserv_address)) < 0)
    {
        cout << 1 << endl;
        throw string("Connection problem");
    }


    if (send(navserv_conn, NAVSERVER_SUBSCRIBE, sizeof(NAVSERVER_SUBSCRIBE), 0) < 0)
    {
        cout << 2 << endl;
        throw string("Sending problem");
    }


    for (int counter = 0; counter < 10; ++counter)
    {
        FD_ZERO(&read_conns);
        FD_SET(navserv_conn, &read_conns);

        timeout.tv_sec = 0;
        timeout.tv_usec = 80000;

        if (select(navserv_conn+1, &read_conns, NULL, NULL, &timeout) <= 0)
        {
            cout << 3 << endl;
            throw string("Fail");
        }
        int msg_len = recv(navserv_conn, msg_buf, BUFFER_SIZE, MSG_DONTWAIT);
        if (msg_len < 0)
        {
            cout << 4 << endl;
            throw string("Reception problem");
        }
//        write(STDOUT_FILENO, msg_buf, msg_len);
    }


    gettimeofday(&oldtime, 0);
    for (int counter = 0; counter < 1000; ++counter)
    {
        FD_ZERO(&read_conns);
        FD_SET(navserv_conn, &read_conns);

        timeout.tv_sec = 0;
        timeout.tv_usec = 80000;

        int res = select(navserv_conn+1, &read_conns, NULL, NULL, &timeout);
//        cout << res << endl;
//        printf ("selecttime = %f\n", (double) timeout.tv_sec + 0.000001 * timeout.tv_usec);
        gettimeofday(&currtime, 0);
        double timediff = currtime.tv_sec - oldtime.tv_sec + 0.000001 * (currtime.tv_usec - oldtime.tv_usec);
        timediffs.push_back(timediff);

        if (res <= 0)
        {
            cout << 5 << endl;
            break;
            throw string("Fail");
        }


        int msg_len = recv(navserv_conn, msg_buf, BUFFER_SIZE, MSG_DONTWAIT);
        if ((msg_len < 0) || (msg_len >= BUFFER_SIZE))
        {
            cout << 6 << endl;
            throw string("Reception problem");
        }
//        write(STDOUT_FILENO, msg_buf, msg_len);
        msg_buf[msg_len] = '\0';



//        printf ("timediff = %f\n", timediff);

        oldtime.tv_sec = currtime.tv_sec;
        oldtime.tv_usec = currtime.tv_usec;



        double data_timestamp;
        char * line_ptr;
        double V, phi;
        double X, Y, theta;

        line_ptr = strstr(msg_buf, "enc");
        if (line_ptr != NULL)
        {
            if (sscanf (line_ptr, "%3c %lf %lf %lf", parse_buf, &data_timestamp, &V, &phi) != 4)
            {
                cout << 9 << endl;
                throw string("Fail");
            }
        }
        else
        {
            cout << 7 << endl;
            throw string("Fail");
        }
        printf ("t = %f // V = %f   phi = %f\n", data_timestamp, V, phi);

        line_ptr = strstr(msg_buf, "state");
        if (line_ptr != NULL)
        {
            if (sscanf (line_ptr, "%5c %lf %lf %lf %lf", parse_buf, &data_timestamp, &X, &Y, &theta) != 5)
            {
                cout << 10 << endl;
                throw string("Fail");
            }
        }
        else
        {
            cout << 8 << endl;
            throw string("Fail");
        }
        printf ("t = %f // X = %f   Y = %f   theta = %f\n", data_timestamp, X, Y, theta);

        timediff = currtime.tv_sec + 0.000001 * (currtime.tv_usec) - data_timestamp;
        cout << "----------------" << endl;
//        timediffs.push_back(timediff);
    }

    if (send(navserv_conn, NAVSERVER_STOP, sizeof(NAVSERVER_STOP), 0) < 0)
    {
        throw string("Sending problem");
    }

    for (unsigned int i = 0; i < timediffs.size(); ++i)
    {
        printf ("timediff = %f\n", timediffs[i]);
    }

    close(navserv_conn);
}
