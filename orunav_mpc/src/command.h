#pragma once


#include <iostream>
#include <iomanip>
#include <cmath>

#include "control.h"
#include "state.h"
#include "commonDefines.h"


/**
 * @brief Commands, which are sent to a car. They are not necessary the same
 * variables as the ones included in the control vector of the model.
 */
class Command
{
    public:
        Command();
        void set(const double, const double, const double);
        void initStop(const State &);
        bool isStop() const;
        double getVelocity() const;
        double getTangentialVelocity() const;
        double getSteeringVelocity() const;
        double getSteeringAngle() const;
        friend ostream& operator<< (ostream&, const Command &);

    private:
        /// Velocity of the reference point
        double ref_point_vel;
        /// Angular steering velocity
        double steering_vel;
        /// Steering angle
        double steering_angle;
        /// Stop flag.
        bool stop_command;
};
