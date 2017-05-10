#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#include <qpOASES.hpp>
#pragma GCC diagnostic pop

#include "qpProblem.h"
#include "func_gen.h"
#include "headers_eigen.h"
#include "swTimer.h"
#include "swLogger.h"


using namespace Eigen;


/**
 * @brief Constructor.
 *
 * @param[in] parameters parameters of the controller
 */
qpProblem::qpProblem(const swParameters &parameters) : qp_constraints (parameters)
{
    // Cannot reuse solution on the first iteration
    failed_qp_counter = parameters.qp_max_solution_reuse;
}


/**
 * @brief Solves a QP MPC problem.
 *
 * @param[in] parameters parameters of the controller
 * @param[in] current_state the current state of the system.
 * @param[out] expected_state the state, which is supposed to be reached during the
 *                          following sampling period.
 * @param[in,out] control control, computed on the previous iteration [in],
 *                        control to be sent to a car [out].
 *
 * @return #qpReturn
 */
qpReturn qpProblem::solve(
        const swParameters &parameters,
        const State &current_state, 
        State &expected_state,
        Control &control)
{
    State ref_state;
    preview_win.getRefState(ref_state);

    qp_current_state = current_state - ref_state;


    form_car_T_Sx0(
            SW_SAMPLING_PERIOD_SEC, 
            parameters.car_wheel_base, 
            preview_win.theta + 1, 
            preview_win.phi + 1, 
            preview_win.v + 1, 
            qp_current_state.raw, 
            out);

    copy (out, out + SW_STATE_CONCAT_LEN*SW_CONTROL_CONCAT_LEN + SW_STATE_CONCAT_LEN, out_copy);

    // Matrix T (X = S*x0 + T*U)
    double *T = out;
    // Vector Sx0 (X = S*x0 + T*U)
    concatState Sx0(out + SW_STATE_CONCAT_LEN*SW_CONTROL_CONCAT_LEN);

    qp_constraints.form(Sx0, T, control, qp_current_state, preview_win);


    form_p(parameters, T, Sx0);
    formHessian(parameters, T);


    double obj_value;
    Control ref_control;
    preview_win.getRefControl(ref_control);
    if (qpOASES(parameters, obj_value) == SW_QP_FAILED)
    {
#ifdef SW_ENABLE_LOGGING
        logFailedQP(out_copy);
#endif // SW_ENABLE_LOGGING

        if ((failed_qp_counter < parameters.qp_max_solution_reuse)
                && (failed_qp_counter < (int) SW_PREVIEW_WINDOW_LEN - 1))
        {
            ++failed_qp_counter;
        
            computeExpectedState (failed_qp_counter, expected_state);

            control.set( 
                    ref_control.v() + solution.v(failed_qp_counter), 
                    ref_control.w() + solution.w(failed_qp_counter));

            return (SW_QP_REUSED);
        }
        else
        {
            return (SW_QP_FAILED);
        }
    }
    else
    {
        failed_qp_counter = 0;

        computeExpectedState (0, expected_state);

        control.set( 
                ref_control.v() + solution.v(failed_qp_counter+parameters.qp_delay_index_offset_v),
                ref_control.w() + solution.w(failed_qp_counter+parameters.qp_delay_index_offset_w));
        return (SW_QP_OK);
    }
}



/**
 * @brief Solves a QP MPC problem (does not return control, but returns value of
 * the objective function).
 *
 * @param[in] parameters parameters of the controller
 * @param[in] current_state the current state of the system.
 * @param[in] control control computed on the previous iteration [in],
 * @param[out] obj_value value of the objective function (0 on failure).
 *
 * @return #qpReturn
 */
qpReturn qpProblem::solve(
        const swParameters &parameters,
        const State &current_state, 
        const Control &control,
        double &obj_value)
{
    State ref_state;
    preview_win.getRefState(ref_state);

    qp_current_state = current_state - ref_state;


    form_car_T_Sx0(
            SW_SAMPLING_PERIOD_SEC, 
            parameters.car_wheel_base, 
            preview_win.theta + 1, 
            preview_win.phi + 1, 
            preview_win.v + 1, 
            qp_current_state.raw, 
            out);


    // Matrix T (X = S*x0 + T*U)
    double *T = out;
    // Vector Sx0 (X = S*x0 + T*U)
    concatState Sx0(out + SW_STATE_CONCAT_LEN*SW_CONTROL_CONCAT_LEN);

    qp_constraints.form(Sx0, T, control, qp_current_state, preview_win);


    // Compute constant term of the objective fucnction (which is usually dropped).
    double constant_term = 0;
    for (unsigned int i = 0; i < SW_PREVIEW_WINDOW_LEN; ++i)
    {
        if (preview_win.mode[i+1] == SW_STEP_MODE_2)
        {
            constant_term += 
                  Sx0.x(i)*Sx0.x(i) * parameters.state_gains_mode2.x()
                + Sx0.y(i)*Sx0.y(i) * parameters.state_gains_mode2.y()
                + Sx0.theta(i)*Sx0.theta(i) * parameters.state_gains_mode2.theta()
                + Sx0.phi(i)*Sx0.phi(i) * parameters.state_gains_mode2.phi();
        }
        else
        {
            constant_term += 
                  Sx0.x(i)*Sx0.x(i) * parameters.state_gains_mode1.x()
                + Sx0.y(i)*Sx0.y(i) * parameters.state_gains_mode1.y()
                + Sx0.theta(i)*Sx0.theta(i) * parameters.state_gains_mode1.theta()
                + Sx0.phi(i)*Sx0.phi(i) * parameters.state_gains_mode1.phi();
        }
    }

    form_p(parameters, T, Sx0);
    formHessian(parameters, T);

    qpReturn retval = qpOASES(parameters, obj_value);
    if (retval == SW_QP_OK)
    {
        obj_value += constant_term;
    }
    return (retval);
}



/**
 * @brief Calls qpOASES on the formed problem.
 *
 * @param[in] parameters parameters of the controller.
 * @param[out] obj_value value of the objective function (0 on failure).
 *
 * @return #qpReturn
 */
qpReturn qpProblem::qpOASES(
        const swParameters &parameters,
        double &obj_value)
{
    USING_NAMESPACE_QPOASES;

    // The class must be reinitialized on each iteration since
    // the number of constraints may vary
    QProblem qp_oases(
            SW_CONTROL_CONCAT_LEN, 
            qp_constraints.constraints_total_num,
            /// @swAssume Control gains are > tolerance (positive and not too small).
            HST_POSDEF);
    int nWSR = parameters.qp_as_change_limit;
    double cputime = parameters.qp_time_limit;


    Options qp_options;
    qp_options.setToDefault();

    // Hessian is positive definite
    qp_options.enableRegularisation = BT_FALSE;
    qp_options.numRegularisationSteps = 0;
    qp_options.enableFlippingBounds = BT_FALSE;
    qp_options.enableNZCTests = BT_FALSE;

    // All bounds are inactive at the first iteration
    qp_options.initialStatusBounds = ST_INACTIVE; // Flipping bounds require it to be ST_LOWER / ST_UPPER

    /// @swAssume Control inputs never have -/+Inf bounds.
    qp_options.enableFarBounds = BT_FALSE; // Flipping bounds require it to be BT_TRUE

    qp_oases.setOptions(qp_options);


    if (qp_oases.init(
            P,                      // Hessian
            p.raw,                  // linear term
            qp_constraints.A,       // A matrix of constraints (row-wise)
            qp_constraints.lb.raw,  // lower bounds 
            qp_constraints.ub.raw,  // upper bounds
            NULL,                   // lower bounds on A
            qp_constraints.b,       // upper bounds on A
            nWSR,
            &cputime) == SUCCESSFUL_RETURN)
    {
        if (qp_oases.getPrimalSolution (solution.raw) != SUCCESSFUL_RETURN)
        {
            SW_THROW_MSG("Cannot get the solution of the QP.");
        }
        obj_value = qp_oases.getObjVal();

        return (SW_QP_OK);
    }
    else
    {
        obj_value = 0;
        return (SW_QP_FAILED);
    }
}



/**
 * @brief Forms the linear term of objective function in QP.
 *
 * @param[in] parameters parameters of the controller
 * @param[in] T matrix T (see derivations)
 * @param[in,out] Sx0 vector S*x0 (see derivations)
 *
 * @attention Destroys vector Sx0.
 */
void qpProblem::form_p(
        const swParameters &parameters,
        const double * T, 
        concatState & Sx0)
{
    // first form Q*Sx0 and store it in Sx0
    unsigned int ind;
    for (ind = 0; ind < SW_PREVIEW_WINDOW_LEN; ++ind)
    {
        if (preview_win.mode[ind+1] == SW_STEP_MODE_2)
        {
            Sx0.multiply(ind, parameters.state_gains_mode2);
        }
        else
        {
            Sx0.multiply(ind, parameters.state_gains_mode1);
        }
    }


    // make sure that p = zeros(SW_PREVIEW_WINDOW_LEN*SW_CONTROL_VAR_N,1)
    fill_n(p.raw, SW_CONTROL_CONCAT_LEN, 0.0);

    // form transpose(Sx0)*T
    // loop over columns (handle Nu=2 columns at each loop)
    for (unsigned int i = 0; i < SW_PREVIEW_WINDOW_LEN; ++i)
    {
        ind = i*SW_CONTROL_VAR_N*SW_STATE_CONCAT_LEN;
        for (unsigned int j=i*SW_STATE_VAR_N; j<SW_STATE_CONCAT_LEN; ++j)
        {
            p.v(i) += T[ind+j] * Sx0.raw[j];
        }

        ind += SW_STATE_CONCAT_LEN;
        for (unsigned int j=i*SW_STATE_VAR_N; j<SW_STATE_CONCAT_LEN; ++j)
        {
            p.w(i) += T[ind+j] * Sx0.raw[j];
        }
    }
}


/**
 * @brief Forms Hessian in the objective function.
 *
 * @param[in] parameters parameters of the controller
 * @param[in,out] T matrix T (see derivations)
 *
 * @attention Destroys matrix T.
 */
void qpProblem::formHessian(
        const swParameters &parameters,
        double *T)
{
    int ind_u1 = 0, ind_u2 = 0;
    int ind_xu1 = 0, ind_xu2 = 0;

    // loop over columns (handle Nu=2 columns at each loop)
    State sqrt_state_gains;
    for (unsigned int i = 0, k = 0; i < SW_PREVIEW_WINDOW_LEN; ++i) 
    {
        ind_u1 = k*SW_STATE_CONCAT_LEN;
        ++k;
        ind_u2 = k*SW_STATE_CONCAT_LEN;
        ++k;
        // loop over rows (handle Nx=4 rows at each loop)
        for (unsigned int j = i; j < SW_PREVIEW_WINDOW_LEN; ++j) 
        {
            if (preview_win.mode[j + 1] == SW_STEP_MODE_2)
            {
                sqrt_state_gains = parameters.sqrt_state_gains_mode2;
            }
            else
            {
                sqrt_state_gains = parameters.sqrt_state_gains_mode1;
            }

            ind_xu1 = SW_STATE_VAR_N*j + ind_u1;
            ind_xu2 = SW_STATE_VAR_N*j + ind_u2;

            T[ind_xu1+0] *= sqrt_state_gains.x();
            T[ind_xu1+1] *= sqrt_state_gains.y();
            T[ind_xu1+2] *= sqrt_state_gains.theta();
            T[ind_xu1+3] *= sqrt_state_gains.phi();

            T[ind_xu2+0] *= sqrt_state_gains.x();
            T[ind_xu2+1] *= sqrt_state_gains.y();
            T[ind_xu2+2] *= sqrt_state_gains.theta();
            T[ind_xu2+3] *= sqrt_state_gains.phi();
        }
    }


    Map< Matrix<double, SW_STATE_CONCAT_LEN, SW_CONTROL_CONCAT_LEN> > T_map(T);
    Map< Matrix<double, SW_CONTROL_CONCAT_LEN, SW_CONTROL_CONCAT_LEN> > P_map(P);
    P_map = T_map.transpose()*T_map;

    Control control_gains;
    for (unsigned int i = 0; i < SW_CONTROL_CONCAT_LEN;)
    {
        if (preview_win.mode[i/SW_CONTROL_VAR_N + 1] == SW_STEP_MODE_2)
        {
            control_gains = parameters.control_gains_mode2;
        }
        else
        {
            control_gains = parameters.control_gains_mode1;
        }

        P[i*SW_CONTROL_CONCAT_LEN + i] += control_gains.v();
        ++i;
        P[i*SW_CONTROL_CONCAT_LEN + i] += control_gains.w();
        ++i;
    }
}



/**
 * @brief Compute expected state using solution of the system.
 *
 * @param[in] index index of the state in the preview window [0, N-1].
 * @param[out] expected_state expected state.
 */
void qpProblem::computeExpectedState(const unsigned int index, State &expected_state)
{
    expected_state.x() = preview_win.x[index+1];
    expected_state.y() = preview_win.y[index+1];
    expected_state.theta() = preview_win.theta[index+1];
    expected_state.phi() = preview_win.phi[index+1];

    Map< Matrix<double, SW_STATE_VAR_N, 1> > x(expected_state.raw);
    Map< Matrix<double, SW_CONTROL_CONCAT_LEN, 1> > u(solution.raw);

    Map< Matrix<double, SW_STATE_CONCAT_LEN, SW_CONTROL_CONCAT_LEN> > T(out_copy);
    Map< Matrix<double, SW_CONTROL_CONCAT_LEN, SW_CONTROL_CONCAT_LEN> > Sx0(out_copy + SW_STATE_CONCAT_LEN*SW_CONTROL_CONCAT_LEN);

    x += Sx0.block(index*SW_STATE_VAR_N, 0, SW_STATE_VAR_N, 1) 
        + T.block(index*SW_STATE_VAR_N, 0, SW_STATE_VAR_N, SW_CONTROL_CONCAT_LEN)*u;
}


#ifdef SW_ENABLE_LOGGING

/**
 * @brief Dump QP parameters and data to an Octave script for debug.
 *
 * @param[in] T matrix T (see derivations)
 */
void qpProblem::logFailedQP(const double *T)
{
    SW_LOG_FAILEDQP_START;


    unsigned int  i, j;

    // Hessian
    SW_LOG_FAILEDQP("P = [");
    for (i = 0; i < SW_CONTROL_CONCAT_LEN; ++i)
    {
         for (j = 0; j < SW_CONTROL_CONCAT_LEN; ++j)
         {
             SW_LOG_FAILEDQP(P[j + i*SW_CONTROL_CONCAT_LEN] << "  ");
         }
         SW_LOG_FAILEDQP(";" << endl);
    }
    SW_LOG_FAILEDQP("];" << endl << endl);


    // linear term
    SW_LOG_FAILEDQP("p = [");
    for (i = 0; i < SW_CONTROL_CONCAT_LEN; ++i)
    {
         SW_LOG_FAILEDQP(p.raw[i] << ";" << endl);
    }
    SW_LOG_FAILEDQP("];" << endl << endl);


    // constraints
    SW_LOG_FAILEDQP("A = [");
    for (i = 0; i < qp_constraints.constraints_total_num; ++i)
    {
         for (j = 0; j < SW_CONTROL_CONCAT_LEN; ++j)
         {
             SW_LOG_FAILEDQP(qp_constraints.A[j + i*SW_CONTROL_CONCAT_LEN] << "  ");
         }
         SW_LOG_FAILEDQP(";" << endl);
    }
    SW_LOG_FAILEDQP("];" << endl << endl);


    // lower bounds
    SW_LOG_FAILEDQP("lb = [");
    for (i = 0; i < SW_CONTROL_CONCAT_LEN; ++i)
    {
         SW_LOG_FAILEDQP(qp_constraints.lb.raw[i] << ";" << endl);
    }
    SW_LOG_FAILEDQP("];" << endl << endl);


    // upper bounds
    SW_LOG_FAILEDQP("ub = [");
    for (i = 0; i < SW_CONTROL_CONCAT_LEN; ++i)
    {
         SW_LOG_FAILEDQP(qp_constraints.ub.raw[i] << ";" << endl);
    }
    SW_LOG_FAILEDQP("];" << endl << endl);

    
    SW_LOG_FAILEDQP("b = [");
    for (i = 0; i < qp_constraints.constraints_total_num; ++i)
    {
         SW_LOG_FAILEDQP(qp_constraints.b[i] << ";" << endl);
    }
    SW_LOG_FAILEDQP("];" << endl << endl);


    SW_LOG_FAILEDQP(endl << endl);
    SW_LOG_FAILEDQP("[X, FVAL, OUTPUT, LAMBDA] = qp ([], P, p, [], [], lb, ub, [], A, b)" << endl);

    SW_LOG_FAILEDQP(endl << endl);

    SW_LOG_FAILEDQP("reference_state_control = [");
    for (i = 0; i < SW_PREVIEW_WINDOW_LEN_EXT; ++i)
    {
        SW_LOG_FAILEDQP(
               preview_win.x[i] << "  "
            << preview_win.y[i] << "  "
            << preview_win.theta[i] << "  "
            << preview_win.phi[i] << "  " 
            << preview_win.v[i] << "  " 
            << preview_win.w[i] << ";" << endl);
    }
    SW_LOG_FAILEDQP("];" << endl << endl);

    SW_LOG_FAILEDQP("qp_current_state = ["
        << qp_current_state.x() << "  "
        << qp_current_state.y() << "  "
        << qp_current_state.theta() << "  "
        << qp_current_state.phi() << "];" << endl);
    
    
    SW_LOG_FAILEDQP(endl << endl);
        
   
    for (i = 0; i < SW_PREVIEW_WINDOW_LEN; ++i)
    {
        SW_LOG_FAILEDQP("G" << i << " = [");
        for (j = 0; j < preview_win.constraints[i].pos_constraints_num; ++j)
        {
            SW_LOG_FAILEDQP(
                    preview_win.constraints[i].a0[j] << " "
                 << preview_win.constraints[i].a1[j] << " "
                 << 0 << " "
                 << 0 << ";" << endl);
        }
        SW_LOG_FAILEDQP("];" << endl << endl);
    }

    SW_LOG_FAILEDQP("G = blkdiag(");
    for (i = 0; i < SW_PREVIEW_WINDOW_LEN-1; ++i)
    {
        SW_LOG_FAILEDQP("G" << i << ", ");
    }
    SW_LOG_FAILEDQP("G" << i << ");" << endl << endl);

    SW_LOG_FAILEDQP("g = [");
    for (i = 0; i < SW_PREVIEW_WINDOW_LEN; ++i)
    {
        for (j = 0; j < preview_win.constraints[i].pos_constraints_num; ++j)
        {
             SW_LOG_FAILEDQP(preview_win.constraints[i].b[j] << ";" << endl);
        }
    }
    SW_LOG_FAILEDQP("];" << endl << endl);


    SW_LOG_FAILEDQP("T = [");
    for (i = 0; i < SW_CONTROL_CONCAT_LEN; ++i)
    {
        for (j = 0; j < SW_STATE_CONCAT_LEN; ++j)
        {
            SW_LOG_FAILEDQP(T[i*SW_STATE_CONCAT_LEN + j] << " ");
        }
        SW_LOG_FAILEDQP(";" << endl);
    }
    SW_LOG_FAILEDQP("]';" << endl << endl);
}

#endif //SW_ENABLE_LOGGING
