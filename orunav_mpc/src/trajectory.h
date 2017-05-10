#pragma once

#include <vector>
#include <deque>


#include "commonDefines.h"
#include "state.h"
#include "control.h"
#include "parameters.h"


/**
 * @brief Step tracking mode, which indicates the set of gains to use for this step.
 */
enum swTrajectoryStepMode
{
    SW_STEP_MODE_1 = 0,
    SW_STEP_MODE_2 = 1,
    SW_STEP_MODE_DEFAULT = SW_STEP_MODE_1
};


/**
 * @brief Constraints for a particular step.
 */
class Constraints
{
    public:
        ///@{
        /// Bounds on tangential velocity
        double v_min;
        double v_max;
        ///@}

        ///@{
        /// Bounds on steering angular velocity
        double w_min;
        double w_max;
        ///@}
        
        ///@{
        /** 
         * Bounds on steering angle, the steering angle must stay within (-pi/2,pi/2).
         * The actual limit is sligtly tighter and is specified in configuration file.
         */ 
        double phi_min;
        double phi_max;
        ///@}

        ///@{
        /// Bounds on orientation
        double theta_min;
        double theta_max;
        ///@}

        ///@{
        /// Bounds on tangential acceleration
        double tanacc_min;
        double tanacc_max;
        ///@}

        /// Maximal centripetal acceleration
        double cenacc_max;


        ///@{
        /// Constraints on position in the form [a0 a1] * [x; y] <= b
        vector<double> a0;
        vector<double> a1;
        vector<double> b;
        unsigned int pos_constraints_num;
        ///@}


        Constraints(const swParameters &);
};



/**
 * @brief A step on a trajectory, includes a reference points, respective constraints and parameters.
 */
class trajectoryStep
{
    public:
        bool last;
        swTrajectoryStepMode mode;
        State state;
        Control control;


        trajectoryStep();
};
