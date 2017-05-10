#pragma once

#include "commonDefines.h"

#include "threadData.h"
#include "state.h"
#include "headers_ros.h"


/**
 * @brief Obtain the current state from ROS messages (in simulation).
 */
class ROSSensorSubscriber
{
    public:
        threadReturn readLoop(threadData *);
        void sensorCallback (const gazebo_msgs::LinkStates::ConstPtr &);

    private:
        State current_state;
	int robot_id;

};
