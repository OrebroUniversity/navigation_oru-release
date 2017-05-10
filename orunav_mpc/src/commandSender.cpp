#include <unistd.h> // usleep()
#include <cstdlib> // abs()

#include "commandSender.h"


/**
 * @brief Constructor
 *
 * @param[in,out] thread_data thread data.
 */
commandSender::commandSender(threadData *thread_data) 
{
    prev_velocity = 0.0;
    max_car_velocity_change = convertVelocity(thread_data->parameters.max_steering_wheel_velocity_delta);
    max_car_steer_angle_change = convertAngle(thread_data->parameters.max_steer_angular_velocity_delta);


#ifdef SW_BUILD_SIMULATION
    ros_publisher = thread_data->ros_node.advertise<geometry_msgs::Twist>(
            thread_data->parameters.topic_simulation_command, 1);
#else // SW_BUILD_SIMULATION
    pdoCounter = 0;

    // clear timeout errors in plc
    // this is important!
    for(unsigned int i = 0; i < SW_CAN_CLEAR_ITER_NUM; ++i)
    {
        formWriteMSG (0, 0, 0);
        usleep(SW_CAN_CLEAR_DELAY);
    }

    if (thread_data->parameters.enable_command_execution == true)
    {
        message_flags = SW_CAN_FLAG_OVERRIDE | SW_CAN_FLAG_DRIVE_ON | SW_CAN_FLAG_STEER_ON;
    }
    else
    {
        message_flags = SW_CAN_FLAG_OVERRIDE;
    }
#endif // SW_BUILD_SIMULATION
}


/**
 * @brief Convert angle to centidegrees.
 *
 * @param[in] angle angle in radians
 *
 * @return angle in centidegrees as integer
 */
int commandSender::convertAngle(const double angle)
{
    return (static_cast<int> (SW_STEER_RESOLUTION_PI * angle));
}


/**
 * @brief Convert velocity to mm/s
 *
 * @param[in] velocity velocity in m/s
 *
 * @return velocity in mm/s as integer
 */
int commandSender::convertVelocity(const double velocity)
{
    return (static_cast<int> (SW_VELOCITY_RESOLUTION * velocity));
}


/**
 * @brief Send command.
 *
 * @param[in] current_state the current state.
 * @param[in] command command.
 *
 * In simulation command is sent in geometry_msgs::Twist message, which is published on 
 * ROS topic swParameters#topic_simulation_command. Message includes two velocities.
 */
void commandSender::send(const State & current_state, const Command & command)
{
    if (convertVelocity(abs(prev_velocity - command.getVelocity()))
            > max_car_velocity_change)
    {
        SW_THROW_MSG("Velocity change limit is violated.");
    }
    if (convertAngle(abs(current_state.phi() - command.getSteeringAngle()))
            > max_car_steer_angle_change)
    {
        SW_THROW_MSG("Steering angle change limit is violated.");
    }
    prev_velocity = command.getVelocity();


#ifdef SW_BUILD_SIMULATION
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = command.getVelocity();
    cmd_msg.angular.z = command.getSteeringVelocity();
    ros_publisher.publish(cmd_msg);
#else // SW_BUILD_SIMULATION
    if (command.isStop())
    {
        formWriteMSG (
                convertVelocity (command.getVelocity()),
                convertAngle (command.getSteeringAngle()),
                // Do not execute steering angle
                //message_flags ^ SW_CAN_FLAG_STEER_ON);
                // This would prevent all commands from execution
                // (The car will stay on brake).
                SW_CAN_FLAG_OVERRIDE);
    }
    else
    {
        formWriteMSG (
                convertVelocity (command.getVelocity()),
                convertAngle (command.getSteeringAngle()),
                message_flags);
    }
#endif // SW_BUILD_SIMULATION
}



/**
 * @brief Checks if given command can be neglected.
 *
 * @param[in] command command
 *
 * @return true if command is insignificant, false otherwise.
 */
bool commandSender::isNegligible(const Command &command)
{
    if ((fabs(command.getTangentialVelocity()) < SW_VELOCITY_COMMAND_TOL) &&
        (fabs(command.getSteeringVelocity()) < SW_VELOCITY_COMMAND_TOL))
    {
        return (true);
    }
    else
    {
        return (false);
    }
}



#ifndef SW_BUILD_SIMULATION

/**
 * @brief Forms and sends a CAN message containing command (is not used in simulation).
 *
 * @param[in] speed speed (mm/s)
 * @param[in] angle angle (centidegree)
 * @param[in] flags flags for the controller (not for canlib)
 */
void commandSender::formWriteMSG(int speed, int angle, const char flags)
{
    canMessage msg;

    msg.flag = 0;
    msg.dlc = 8;
    msg.time_stamp = 0;
    msg.id = SW_CAN_COMMAND_ID + SW_CAN_NODE_ID;


    msg.msg[0] = speed & 0xff;
    msg.msg[1] = speed >> 8 & 0xff;
    msg.msg[2] = angle & 0xff;
    msg.msg[3] = angle >> 8 & 0xff;

    msg.msg[4] = pdoCounter;
    ++pdoCounter;

    msg.msg[5] = flags;

    msg.msg[6] = 0;
    msg.msg[7] = 0;


    sw_can.canWrite(msg);
}

#endif // SW_BUILD_SIMULATION
