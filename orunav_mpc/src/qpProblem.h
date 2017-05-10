#pragma once

/******************************************
 **************** INCLUDES ****************
 ******************************************/

#include "commonDefines.h"

#include "qpConstraints.h"
#include "previewWindow.h"
#include "parameters.h"


/**
 * @brief Return status of #qpProblem methods.
 */
enum qpReturn
{
    /// QP was failed (this is not a fatal error, we can wait for a new trajectory for example).
    SW_QP_FAILED,
    /// QP was failed, but the control is obtained from the last available solution.
    SW_QP_REUSED,
    /// Completed successfully.
    SW_QP_OK
};



/******************************************/

/**
 * @brief Quadratic optimization problem for Model Predictive control.
 */
class qpProblem
{
    public:
        previewWindow preview_win;

        qpProblem(const swParameters &);

        qpReturn solve(const swParameters &, const State &, State &, Control &);
        qpReturn solve(const swParameters &, const State &, const Control &, double &);


    private:
        /// QP constraints
        qpConstraints qp_constraints;

        /** 
         * The difference between the current and the reference state 
         * (Initial state of the linearized QP problem).
         */
        State qp_current_state;

        /// Array for internal use
        double out[SW_STATE_CONCAT_LEN*SW_CONTROL_CONCAT_LEN + SW_STATE_CONCAT_LEN];
        double out_copy[SW_STATE_CONCAT_LEN*SW_CONTROL_CONCAT_LEN + SW_STATE_CONCAT_LEN];

        /// 0.5*Hessian matrix of the objective function
        double P[SW_CONTROL_CONCAT_LEN*SW_CONTROL_CONCAT_LEN];

        /// Vector of the objective function
        concatControlFixed<SW_PREVIEW_WINDOW_LEN> p;

        /// Solution of the QP
        concatControlFixed<SW_PREVIEW_WINDOW_LEN> solution;

        /// Counter of failed QP's
        int failed_qp_counter;


        void form_p(const swParameters &, const double *, concatState &);
        void formHessian(const swParameters &, double *);
        qpReturn qpOASES(const swParameters &, double &);
        void computeExpectedState(const unsigned int, State &);

#ifdef SW_ENABLE_LOGGING
        void logFailedQP(const double *);
#endif //SW_ENABLE_LOGGING
};
