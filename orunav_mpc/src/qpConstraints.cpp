#include "qpConstraints.h"

namespace qp_constraints
{
  // Taken from angles ros-pkg
  static inline double normalize_angle_positive(double angle)
  {
    return fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
  }
  
  static inline double normalize_angle(double angle)
  {    
    double a = normalize_angle_positive(angle);
    if (a > M_PI)
      a -= 2.0 *M_PI;
    return a;
  }
}
/**
 * @brief Number of constraints on different variables.
 */
enum constraintNumber
{
    SW_QP_B_PHI_U_LEN = SW_PREVIEW_WINDOW_LEN,
    SW_QP_B_PHI_L_LEN = SW_PREVIEW_WINDOW_LEN,
    SW_QP_B_TANACC_U_LEN = SW_PREVIEW_WINDOW_LEN-1,
    SW_QP_B_TANACC_L_LEN = SW_PREVIEW_WINDOW_LEN-1,
    SW_QP_B_THETA_U_LEN = SW_PREVIEW_WINDOW_LEN,
    SW_QP_B_THETA_L_LEN = SW_PREVIEW_WINDOW_LEN
};



/**
 * @brief Constructor.
 *
 * @param[in] param parameters of the controller.
 */
qpConstraints::qpConstraints(const swParameters &param)
{
    A = NULL;
    b = NULL;


    enable_position_constraints =    param.enable_position_constraints;
    enable_cen_acc_constraints =     param.enable_cen_acc_constraints;
    enable_tan_acc_constraints =     param.enable_tan_acc_constraints;
    enable_orientation_constraints = param.enable_orientation_constraints;

    min_constraints_num = SW_QP_B_PHI_U_LEN + SW_QP_B_PHI_L_LEN;


    b_phi_u_ind = 0;
    b_phi_l_ind = b_phi_u_ind + SW_QP_B_PHI_U_LEN;
    int b_next_ind = b_phi_l_ind + SW_QP_B_PHI_L_LEN;

    b_phi_u = NULL;
    b_phi_l = NULL; 
    A_phi_u = NULL;
    A_phi_l = NULL;


    b_tanacc_u_ind = 0;
    b_tanacc_l_ind = 0;
    b_tanacc_u = A_tanacc_u = NULL;
    b_tanacc_l = A_tanacc_l = NULL;
    if (enable_tan_acc_constraints)
    {
        min_constraints_num += SW_QP_B_TANACC_U_LEN + SW_QP_B_TANACC_L_LEN;

        b_tanacc_u_ind = b_next_ind;
        b_tanacc_l_ind = b_tanacc_u_ind + SW_QP_B_TANACC_U_LEN;

        b_next_ind = b_tanacc_l_ind + SW_QP_B_TANACC_L_LEN;
    }

    b_theta_u_ind = 0;
    b_theta_l_ind = 0;
    b_theta_u = A_theta_u = NULL;
    b_theta_l = A_theta_l = NULL;
    if (enable_orientation_constraints)
    {
        min_constraints_num += SW_QP_B_THETA_U_LEN + SW_QP_B_THETA_L_LEN;

        b_theta_u_ind = b_next_ind;
        b_theta_l_ind = b_theta_u_ind + SW_QP_B_THETA_U_LEN;

        b_next_ind = b_theta_l_ind + SW_QP_B_THETA_L_LEN;
    }

    b_position_ind = 0;
    b_position = A_position = NULL;
    if (enable_position_constraints)
    {
        b_position_ind = b_next_ind;
    }
}        
        

/**
 * @brief Destructor.
 */
qpConstraints::~qpConstraints()
{
    if (A != NULL)
    {
        delete [] A;
    }
    if (b != NULL)
    {
        delete [] b;
    }
}



/**
 * @brief Forms constraints.
 *
 * @param[in] Sx0 vector S*x0 (see derivations)
 * @param[in] T matrix T (see derivations)
 * @param[in] old_control the control, which was computed on the previous iteration 
 * @param[in] qp_current_state the current state of the linearized problem
 * @param[in] prev_win preview window
 */
void qpConstraints::form(
        const concatState &Sx0, 
        const double * T,
        const Control &old_control,
        const State &qp_current_state,
        const previewWindow &prev_win)
{
    if (A != NULL)
    {
        delete [] A;
    }
    if (b != NULL)
    {
        delete [] b;
    }

    constraints_total_num = min_constraints_num;
    if (enable_position_constraints)
    {
        constraints_total_num += prev_win.constraints_num;
    }
  
    A = new double[SW_CONTROL_CONCAT_LEN * constraints_total_num]();
    b = new double[constraints_total_num]();
    initPointers ();

    form_b(Sx0, old_control, qp_current_state, prev_win);
    form_A(T, prev_win);
}



/**
 * @brief Initialize pointers to parts of A and b.
 */
void qpConstraints::initPointers()
{
    b_phi_u = &b[b_phi_u_ind];
    b_phi_l = &b[b_phi_l_ind];
    A_phi_u = &A[b_phi_u_ind * SW_CONTROL_CONCAT_LEN];
    A_phi_l = &A[b_phi_l_ind * SW_CONTROL_CONCAT_LEN];

    if (enable_tan_acc_constraints)
    {
        b_tanacc_u = &b[b_tanacc_u_ind];
        b_tanacc_l = &b[b_tanacc_l_ind];

        A_tanacc_u = &A[b_tanacc_u_ind * SW_CONTROL_CONCAT_LEN];
        A_tanacc_l = &A[b_tanacc_l_ind * SW_CONTROL_CONCAT_LEN];
    }

    if (enable_orientation_constraints)
    {
        b_theta_u = &b[b_theta_u_ind];
        b_theta_l = &b[b_theta_l_ind];

        A_theta_u = &A[b_theta_u_ind * SW_CONTROL_CONCAT_LEN];
        A_theta_l = &A[b_theta_l_ind * SW_CONTROL_CONCAT_LEN];
    }

    if (enable_position_constraints)
    {
        b_position = &b[b_position_ind];
        A_position = &A[b_position_ind * SW_CONTROL_CONCAT_LEN];
    }
}



/**
 * @brief Forms bounds 
 *
 * @param[in] Sx0 vector S*x0 (see derivations)
 * @param[in] old_control the control, which was computed on the previous iteration
 * @param[in] qp_current_state the current state of the linearized problem
 * @param[in] prev_win preview window
 */
void qpConstraints::form_b(
        const concatState &Sx0, 
        const Control &old_control,
        const State &qp_current_state,
        const previewWindow &prev_win)
{
    for (unsigned int i = 0; i < SW_PREVIEW_WINDOW_LEN; ++i) // rows
    {
        // bounds on v
        lb.v(i) = prev_win.constraints[i].v_min - prev_win.v[i + 1];
        ub.v(i) = prev_win.constraints[i].v_max - prev_win.v[i + 1];

        // bounds on w
        lb.w(i) = prev_win.constraints[i].w_min - prev_win.w[i + 1];
        ub.w(i) = prev_win.constraints[i].w_max - prev_win.w[i + 1];

        // bounds on A_phi
        b_phi_u[i] =   prev_win.constraints[i].phi_max - prev_win.phi[i + 1] - qp_current_state.phi();
        b_phi_l[i] = -(prev_win.constraints[i].phi_min - prev_win.phi[i + 1] - qp_current_state.phi());

        if (enable_orientation_constraints)
        {
            // bounds on A_theta
            b_theta_u[i] =   prev_win.constraints[i].theta_max - prev_win.theta[i + 1] - Sx0.theta(i);
            b_theta_l[i] = -(prev_win.constraints[i].theta_min - prev_win.theta[i + 1] - Sx0.theta(i));
        }
    }


    if (enable_tan_acc_constraints)
    {
        for (unsigned int i = 0; i < SW_PREVIEW_WINDOW_LEN - 1; ++i) 
        {
            // bounds on A_tanacc
            b_tanacc_u[i] =   SW_SAMPLING_PERIOD_SEC * prev_win.constraints[i+1].tanacc_max - prev_win.v[i+2] + prev_win.v[i+1];
            b_tanacc_l[i] = -(SW_SAMPLING_PERIOD_SEC * prev_win.constraints[i+1].tanacc_min - prev_win.v[i+2] + prev_win.v[i+1]);
        }
        
        double v0_min = SW_SAMPLING_PERIOD_SEC * prev_win.constraints[0].tanacc_min - prev_win.v[1] + old_control.v();
        double v0_max = SW_SAMPLING_PERIOD_SEC * prev_win.constraints[0].tanacc_max - prev_win.v[1] + old_control.v();
        if (lb.v(0) < v0_min)
        {
            lb.v(0) = v0_min;
        }
        if (ub.v(0) > v0_max)
        {
            ub.v(0) = v0_max;
        }
    }


    if (enable_cen_acc_constraints)
    {
        for (unsigned int i = 0; i < SW_PREVIEW_WINDOW_LEN; ++i) 
        {
	    //            const double abs_omega = abs ((prev_win.theta[i+1] - prev_win.theta[i]) / SW_SAMPLING_PERIOD_SEC);
	    const double abs_omega = abs (qp_constraints::normalize_angle(prev_win.theta[i+1] - prev_win.theta[i]) / SW_SAMPLING_PERIOD_SEC);
            if (abs_omega > SW_ANGULAR_VELOCITY_TOL)
            {
                const double R = abs (prev_win.v[i+1]) / abs_omega;
                double vi_max = sqrt(prev_win.constraints[i].cenacc_max * R);
                double vi_min = - vi_max;

                vi_max -= prev_win.v[i+1];
                vi_min -= prev_win.v[i+1];

                if (lb.v(i) < vi_min)
                {
                    lb.v(i) = vi_min;
                }
                if (ub.v(i) > vi_max)
                {
                    ub.v(i) = vi_max;
                }
            }
        }
    }


    if (enable_position_constraints)
    {
        unsigned int ind = 0;
        for (unsigned int i = 0; i < SW_PREVIEW_WINDOW_LEN; ++i)
        {
            for (unsigned int j = 0; j < prev_win.constraints[i].pos_constraints_num; ++j)
            {
                b_position[ind + j] =
                    prev_win.constraints[i].b[j]
                    - (prev_win.constraints[i].a0[j] * Sx0.x(i)             + prev_win.constraints[i].a1[j] * Sx0.y(i))
                    - (prev_win.constraints[i].a0[j] * prev_win.x[i + 1]    + prev_win.constraints[i].a1[j] * prev_win.y[i + 1]);
            }
            ind += prev_win.constraints[i].pos_constraints_num;
        }
    }
}



/**
 * @brief Forms A.
 *
 * @param[in] T matrix T (see derivations)
 * @param[in] prev_win preview window
 *
 * @attention A is row-wise.
 */
void qpConstraints::form_A(
        const double *T,
        const previewWindow &prev_win)
{
    // Part of A corresponding to the constraints on phi is constant
    for (unsigned int i = 0; i < SW_PREVIEW_WINDOW_LEN; ++i)
    {
        unsigned int column = SW_CONTROL_VAR_N * i + 1;
        for (unsigned int j = i; j < SW_PREVIEW_WINDOW_LEN; ++j)
        {
            const int ind = column + j*SW_CONTROL_CONCAT_LEN;
            A_phi_u[ind] =  SW_SAMPLING_PERIOD_SEC;
            A_phi_l[ind] = -SW_SAMPLING_PERIOD_SEC;
        }
    }


    if (enable_tan_acc_constraints)
    {
        // Part of A corresponding to the constraints on tangential acceleration is constant
        for (unsigned int i = 0; i < SW_QP_B_TANACC_U_LEN; ++i)
        {
            const int ind = i*SW_CONTROL_CONCAT_LEN + i*SW_CONTROL_VAR_N;

            A_tanacc_u[ind] = -1;
            A_tanacc_u[ind + 2] = 1;

            A_tanacc_l[ind] = -A_tanacc_u[ind];
            A_tanacc_l[ind + 2] = -A_tanacc_u[ind + 2];
        }
    }


    if (enable_orientation_constraints)
    {
        for (unsigned int i = 0; i < SW_PREVIEW_WINDOW_LEN; ++i) // column
        {
            for (unsigned int j = i; j < SW_QP_B_THETA_U_LEN; ++j) // row
            {
                const int ind = i + j*SW_CONTROL_CONCAT_LEN;

                A_theta_u[ind]     =  T[i*SW_STATE_CONCAT_LEN + j*SW_STATE_VAR_N + 2];
                A_theta_l[ind]     = -A_theta_u[ind];
                
                A_theta_u[ind + 1] =  T[(i+1)*SW_STATE_CONCAT_LEN + j*SW_STATE_VAR_N + 2];
                A_theta_l[ind + 1] = -A_theta_u[ind + 1];
            }   
        }   
    }


    if (enable_position_constraints)
    {
        // form G*T
        //
        // G =  ..00
        //      ..00      0
        //          ..00
        //          ..00
        //          ..00
        //        0     ..00
        //
        //
        // T =  ..
        //      ..   0
        //      ..
        //      ..
        //      ....
        //      ....
        //      ....
        //      ....
        //      ......
        //      ......
        //      ......
        //      ......
        //
        // GT = ..
        //      ..   0
        //      ....
        //      ....
        //      ....
        //      ......


        unsigned int ind = 0;
        for (unsigned int i = 0; i < SW_PREVIEW_WINDOW_LEN; ++i)
        {
            for (unsigned int j = 0; j < prev_win.constraints[i].pos_constraints_num; ++j) // row
            {
                for (unsigned int k = 0; k < i*SW_CONTROL_VAR_N + SW_CONTROL_VAR_N; ++k) // column
                {
                    const double *T_ptr = &T[k*SW_STATE_CONCAT_LEN + i*SW_STATE_VAR_N];


                    A_position[(ind + j)*SW_CONTROL_CONCAT_LEN + k] = 
                        prev_win.constraints[i].a0[j] * T_ptr[0] + prev_win.constraints[i].a1[j] * T_ptr[1];
                }
            }

            ind += prev_win.constraints[i].pos_constraints_num;
        }
    }
}
