#pragma once

#include "commonDefines.h"
#include "command.h"
#include "threadData.h"
#include "state.h"


#ifdef SW_BUILD_SIMULATION
#include "headers_ros.h"
#else //SW_BUILD_SIMULATION
#include "canlibWrapper.h"
#endif // SW_BUILD_SIMULATION


/**
 * @brief Sends commands to a car.
 */
class commandSender
{
    public:
        commandSender (threadData *);
        void send(const State &, const Command &);
        bool isNegligible(const Command &);


    private:
        /// Velocity, which was sent on the previous invocation
        double prev_velocity;
        /// Maximal change of the velocity
        int max_car_velocity_change;
        /// Maximal change of the steering angle
        int max_car_steer_angle_change;


        int convertAngle(const double);
        int convertVelocity(const double);


#ifdef SW_BUILD_SIMULATION
        ros::Publisher ros_publisher;
#else //SW_BUILD_SIMULATION
        Canlib sw_can;
        unsigned char pdoCounter;
        /// Message flags
        char message_flags;

        void formWriteMSG(int, int, const char);
#endif // SW_BUILD_SIMULATION
};
