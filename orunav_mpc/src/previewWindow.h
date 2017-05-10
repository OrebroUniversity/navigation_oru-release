#pragma once

#include <vector>

#include "commonDefines.h"
#include "state.h"
#include "command.h"
#include "control.h"
#include "trajectoryCache.h"


class previewWindow
{
    public:
        friend class qpProblem;
        friend class qpConstraints;


        /**
         * @brief Get the reference command for the next sampling period.
         *
         * @param[out] command command.
         */
        void getRefCommand(Command &command) const
        {
            command.set (v[1], w[1], phi[0]);
        }


        /**
         * @brief Get the reference control.
         *
         * @param[out] control control.
         */
        void getRefControl(Control &control) const
        {
            control.set (v[1], w[1]);
        }


        /**
         * @brief Returns the current reference state.
         *
         * @param[out] reference_state
         */
        void getRefState(State & reference_state) const
        {
            reference_state.set(x[0], y[0], theta[0], phi[0]);
        }


        /**
         * @brief Forms preview window using given trajectory.
         *
         * @param[in] traj trajectory
         */
        void form(const trajectoryCache &traj)
        {
            constraints.clear();
            constraints_num = 0;


            for (unsigned int i = 0; i < SW_PREVIEW_WINDOW_LEN_EXT; ++i)
            {
                const trajectoryStep & step = traj.getStep(i);
                const Constraints & constr = traj.getConstraints(i);

                x[i]     = step.state.x();
                y[i]     = step.state.y();
                theta[i] = step.state.theta();
                phi[i]   = step.state.phi();

                v[i] = step.control.v();
                w[i] = step.control.w();

                mode[i]  = step.mode;

                // Skip the constraints on the current state
                if (i != 0)
                {
                    constraints.push_back(constr);
                    constraints_num += constr.pos_constraints_num;
                }
            }
        }


    private:
        /// Reference x coordinate
        double x[SW_PREVIEW_WINDOW_LEN_EXT];
        /// Reference y coordinate
        double y[SW_PREVIEW_WINDOW_LEN_EXT];
        /// Reference orientation
        double theta[SW_PREVIEW_WINDOW_LEN_EXT];
        /// Reference steering angle
        double phi[SW_PREVIEW_WINDOW_LEN_EXT];

        /// Reference velocity
        double v[SW_PREVIEW_WINDOW_LEN_EXT];
        /// Reference steering velocity
        double w[SW_PREVIEW_WINDOW_LEN_EXT];

        /// Reference point mode.
        int mode[SW_PREVIEW_WINDOW_LEN_EXT];

        /// Vector of state constraints (SW_PREVIEW_WINDOW_LEN).
        vector<Constraints> constraints;
        /// Total number of state constraints.
        unsigned int constraints_num;
};
