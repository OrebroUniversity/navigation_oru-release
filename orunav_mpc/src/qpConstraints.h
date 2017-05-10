#pragma once

#include "commonDefines.h"
#include "threadData.h"
#include "previewWindow.h"
#include "state.h"
#include "control.h"



/**
 * @brief Constraints of the QP problem.
 */
class qpConstraints
{
    public:
        /// Matrix A of constraints (see derivations, A includes G*T and H).
        double *A;
        /// Vector of bounds on qpConstraints#A, such that   A * x <= b
        double *b;

        /// Upper bounds on control variables
        concatControlFixed<SW_PREVIEW_WINDOW_LEN> ub;
        /// Lower bounds on control variables
        concatControlFixed<SW_PREVIEW_WINDOW_LEN> lb;

        /// Total number of state constraints.
        unsigned int constraints_total_num;


        qpConstraints(const swParameters &);
        ~qpConstraints();

        void form (
                const concatState &, 
                const double *, 
                const Control &, 
                const State &,
                const previewWindow &);


    private:
        /// Minimal number of state constraints, which are always present
        unsigned int min_constraints_num;

        ///@{
        /// Pointers to parts of vector b.
        double *b_theta_u;
        double *b_theta_l;
        double *b_phi_u;
        double *b_phi_l;
        double *b_tanacc_u;
        double *b_tanacc_l;
        double *b_position;
        ///@}

        ///@{
        /// Pointers to parts of matrix A.
        double *A_phi_u;
        double *A_phi_l;
        double *A_tanacc_u;
        double *A_tanacc_l;
        double *A_theta_u;
        double *A_theta_l;
        double *A_position;
        ///@}

        ///@{
        /// Indicies of parts of b
        int b_phi_u_ind;
        int b_phi_l_ind;
        int b_tanacc_u_ind;
        int b_tanacc_l_ind;
        int b_theta_u_ind;
        int b_theta_l_ind;
        int b_position_ind;
        ///@}

        bool enable_position_constraints;
        bool enable_cen_acc_constraints;
        bool enable_tan_acc_constraints;
        bool enable_orientation_constraints;


        void initPointers();

        void form_b(const concatState &, const Control &, const State &, const previewWindow &);
        void form_A(const double *, const previewWindow &);
};
