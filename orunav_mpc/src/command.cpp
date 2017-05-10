#include "command.h"


/**
 * @brief Constructor
 */
Command::Command()
{
    set (0.0, 0.0, 0.0);
}


/**
 * @brief Set variables.
 *
 * @param[in] v velocity of the reference point
 * @param[in] w angular steering velocity
 * @param[in] phi the current steering angle (w*dt is added to it!)
 */
void Command::set(const double v, const double w, const double phi)
{
    stop_command = false; 
    ref_point_vel = v;
    steering_vel = w;
    steering_angle = phi + w * SW_SAMPLING_PERIOD_SEC;
}


/**
 * @brief Initialize command, which stop a car.
 *
 * @param[in] current_state current state of a car.
 */
void Command::initStop(const State & current_state)
{
    stop_command = true;    
    ref_point_vel = 0.0;
    steering_vel = 0.0;
    steering_angle = current_state.phi();
}



/**
 * @brief Can be used to check if the command is a stop command.
 *
 * @return true/false.
 */
bool Command::isStop() const
{
    return (stop_command);
}


/**
 * @brief Returns velocity.
 *
 * @return Velocity of the steering wheel.
 */
double Command::getVelocity () const
{
    return (ref_point_vel / cos(steering_angle));
}


/**
 * @brief Returns angular steering velocity.
 *
 * @return Angular steering velocity.
 */
double Command::getSteeringVelocity () const
{
    return (steering_vel);
}


/**
 * @brief Returns the steering angle (centidegree)
 *
 * @return Steering angle.
 */
double Command::getSteeringAngle () const
{
    return (steering_angle);
}


/**
 * @brief Returns the tangential velocity.
 *
 * @return Tangential velocity.
 */
double Command::getTangentialVelocity() const
{
    return (ref_point_vel);
}


/**
 * @brief Output function.
 *
 * @param[in,out] out output stream.
 * @param[in] command command.
 *
 * @return Output stream.
 */
ostream& operator<< (ostream& out, const Command &command)
{
    out << setiosflags(ios::fixed) << setprecision(3)
        << "Velocity (tangential) = " << command.ref_point_vel
        << ", velocity (steering wheel) = " << (command.ref_point_vel / cos(command.steering_angle))
        << ", steering velocity = " << command.steering_vel
        << ", steering angle = " << command.steering_angle;
    return(out);
}
