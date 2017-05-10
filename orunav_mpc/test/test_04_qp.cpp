#include <sys/types.h>
#include <unistd.h>
#include <sched.h>
#include <stdlib.h>
#include <iostream>


#include "../src/swTimer.h"
#include "../src/state.h"
#include "../src/state.cpp"
#include "../src/control.h"

#include "defines.h"
#include "utility_qp.h"



// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// QP
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <eigen3/Eigen/Core>
#include "qpOASES.hpp"

#include "func_gen.h"

using namespace Eigen;


/******************************************/

/** \brief Definition of a preview window */
class qpProblem
{
    public:
        /** \brief 0.5*Hessian matrix of the objective function */
        double P[SW_CONTROL_CONCAT_LEN*SW_CONTROL_CONCAT_LEN];

        /** \brief Vector of the objective function */
        concatControlFixed<SW_PREVIEW_WINDOW_LEN> p;


        double ub[SW_CONTROL_CONCAT_LEN];
        double lb[SW_CONTROL_CONCAT_LEN];

        double solution[SW_CONTROL_CONCAT_LEN];
        double control_phi;
        double control_v;


        /** \brief Array for internal use */
        double out[SW_STATE_CONCAT_LEN*SW_CONTROL_CONCAT_LEN + SW_STATE_CONCAT_LEN];

        /** \brief Matrix T (X = S*x0 + T*U)

            \note points to the array #out
         */
        double *T;

        /** \brief Vector Sx0 (X = S*x0 + T*U)

            \note points to the array #out
         */
        double *Sx0;


        double phi_GT[SW_CONTROL_CONCAT_LEN*SW_PREVIEW_WINDOW_LEN];
        double phi_GT_lb[SW_PREVIEW_WINDOW_LEN];
        double phi_GT_ub[SW_PREVIEW_WINDOW_LEN];
        

        // -------------------------------------------------------------------------

// >>> PUBLIC       
        qpProblem()
        {
            T   = out;
            Sx0 = out + SW_STATE_CONCAT_LEN*SW_CONTROL_CONCAT_LEN;
            
            
            // XXX row-wise !!!
            fill(phi_GT, phi_GT+SW_CONTROL_CONCAT_LEN*SW_PREVIEW_WINDOW_LEN, 0.0);
            for (unsigned int i = 0; i < SW_PREVIEW_WINDOW_LEN; ++i)
            {
                for (unsigned int j = i; j < SW_PREVIEW_WINDOW_LEN; ++j)
                {
                    phi_GT[(2*i + 1) + j*SW_CONTROL_CONCAT_LEN] = SW_SAMPLING_PERIOD_SEC;
                }
            }
            //Matrix_print(SW_CONTROL_CONCAT_LEN, SW_PREVIEW_WINDOW_LEN, phi_GT, "phi_GT");
        }

        void solve(
                const referenceTrajectory &ref_traj,
                const Control& control_gains,
                const State& state_gains,
                const State& sqrt_state_gains,
                const State& current_state)
        {
            State qp_current_state;
            qp_current_state = current_state - ref_traj.init_state;


            formBounds(ref_traj.v, ref_traj.w, ref_traj.phi, qp_current_state);

            formObjective(
                    ref_traj,
                    control_gains,
                    state_gains,
                    sqrt_state_gains,
                    qp_current_state);

            USING_NAMESPACE_QPOASES;

            QProblem qp_oases(SW_CONTROL_CONCAT_LEN, SW_PREVIEW_WINDOW_LEN);
            int nWSR = SW_QP_AS_CHANGES_LIMIT;
            double cputime = SW_QP_TIME_LIMIT * 100;

            if (qp_oases.init(
                    P,          // Hessain
                    p.raw,      // linear term
                    phi_GT,     // A matrix of constraints
                    lb,         // lower bounds 
                    ub,         // upper bounds
                    phi_GT_lb,  // lower bounds on A
                    phi_GT_ub,  // upper bounds on A
                    nWSR,
                    &cputime) == SUCCESSFUL_RETURN)
            {

                if (qp_oases.getPrimalSolution (solution) != SUCCESSFUL_RETURN)
                {
                    cout << "Cannot get the solution of the QP." << endl;
                    exit (1);
                }
                control_v = solution[0] + ref_traj.v[0];
                control_phi = solution[1]*SW_SAMPLING_PERIOD_SEC + ref_traj.phi[0];
            }
            else
            {
                cout << "Cannot solve the QP. CPU = " << cputime << ", ITER = " << nWSR << endl;
                exit (1);
            }
        }

// >>> PRIVATE
        void formBounds(
                const double *reference_v, 
                const double *reference_w,
                const double *reference_phi,
                const State & qp_current_state)
        {
            for (unsigned int i = 0; i < SW_PREVIEW_WINDOW_LEN; ++i) // rows
            {
                // bounds on v
                lb[SW_CONTROL_VAR_N*i] = -SW_MAX_REF_POINT_VELOCITY - reference_v[i];
                ub[SW_CONTROL_VAR_N*i] =  SW_MAX_REF_POINT_VELOCITY - reference_v[i];

                // bounds on w
                lb[SW_CONTROL_VAR_N*i + 1] = -SW_MAX_STEER_ANGULAR_VELOCITY - reference_w[i];
                ub[SW_CONTROL_VAR_N*i + 1] =  SW_MAX_STEER_ANGULAR_VELOCITY - reference_w[i];

                // bounds on phi_GT
                phi_GT_lb[i] = -SW_MAX_STEER_ANGLE - reference_phi[i] - qp_current_state.phi();
                phi_GT_ub[i] =  SW_MAX_STEER_ANGLE - reference_phi[i] - qp_current_state.phi();
            }
        }


        void form_p(const State &state_gains)
        {
            concatState Sx0v (Sx0);

            // first form Q*Sx0 and store it in Sx0
            unsigned int ind;
            for (ind = 0; ind < SW_PREVIEW_WINDOW_LEN; ++ind)
            {
                Sx0v.multiply(ind, state_gains);
            }


            // make sure that p = zeros(SW_PREVIEW_WINDOW_LEN*SW_CONTROL_VAR_N,1)
            fill_n(p.raw, SW_CONTROL_CONCAT_LEN, 0.0);

            // form transpose(Sx0)*T
            for (unsigned int i = 0; i < SW_PREVIEW_WINDOW_LEN; ++i) // loop over columns (handle Nu=2 columns at each loop)
            {
                ind = i*SW_CONTROL_VAR_N*SW_STATE_CONCAT_LEN;
                for (unsigned int j=i*SW_STATE_VAR_N; j<SW_STATE_CONCAT_LEN; ++j)
                {
                    p.v(i) += T[ind+j] * Sx0v.raw[j];
                }

                ind += SW_STATE_CONCAT_LEN;
                for (unsigned int j=i*SW_STATE_VAR_N; j<SW_STATE_CONCAT_LEN; ++j)
                {
                    p.w(i) += T[ind+j] * Sx0v.raw[j];
                }
            }
        }


        void form_cholQT(const State &sqrt_state_gains)
        {
            unsigned int k = 0;
            unsigned int ind_u1 = 0, ind_u2 = 0;
            unsigned int ind_xu1 = 0, ind_xu2 = 0;

            for (unsigned int i = 0; i < SW_PREVIEW_WINDOW_LEN; ++i) // loop over columns (handle Nu=2 columns at each loop)
            {
                ind_u1 = k*SW_STATE_CONCAT_LEN;
                ++k;
                ind_u2 = k*SW_STATE_CONCAT_LEN;
                ++k;
                for (unsigned int j = i; j < SW_PREVIEW_WINDOW_LEN; ++j) // loop over rows (handle Nx=4 rows at each loop)
                {
                    ind_xu1 = SW_STATE_VAR_N*j + ind_u1;
                    ind_xu2 = SW_STATE_VAR_N*j + ind_u2;

                    T[ind_xu1+0] *= sqrt_state_gains.x();
                    T[ind_xu2+0] *= sqrt_state_gains.x();
                    T[ind_xu1+1] *= sqrt_state_gains.y();
                    T[ind_xu2+1] *= sqrt_state_gains.y();
                    T[ind_xu1+2] *= sqrt_state_gains.theta();
                    T[ind_xu2+2] *= sqrt_state_gains.theta();
                    T[ind_xu1+3] *= sqrt_state_gains.phi();
                    T[ind_xu2+3] *= sqrt_state_gains.phi();
                }
            }
        }


        void formObjective(
                const referenceTrajectory &ref_traj,
                const Control& control_gains,
                const State& state_gains,
                const State& sqrt_state_gains,
                const State& qp_current_state)
        {
            form_car_T_Sx0(
                    SW_SAMPLING_PERIOD_SEC, 
                    SW_WHEEL_BASE, 
                    (double *) ref_traj.theta, 
                    (double *) ref_traj.phi, 
                    (double *) ref_traj.v, 
                    (double *) qp_current_state.raw, 
                    out);

            form_p(state_gains);
            form_cholQT(sqrt_state_gains);

            Map< Matrix<double, SW_STATE_CONCAT_LEN, SW_CONTROL_CONCAT_LEN> > T_map(T);
            Map< Matrix<double, SW_CONTROL_CONCAT_LEN, SW_CONTROL_CONCAT_LEN> > P_map(P);
	    
	    // There is something weird with Eigen here - go figure...
	    // Don't work (this is how it's donw int he qpProblem.cpp file...)
	    //	    P_map = T_map.transpose()*T_map;
	    // Don't work:
	    //	    Matrix<double, SW_STATE_CONCAT_LEN, SW_CONTROL_CONCAT_LEN> T_map_transpose = T_map;
	    //	    T_map_transpose.transposeInPlace();
	    // Works(!)
	    Matrix<double, SW_CONTROL_CONCAT_LEN, SW_STATE_CONCAT_LEN> T_map_transpose = T_map.transpose();
	    P_map = T_map_transpose * T_map;

            for (unsigned int i = 0; i < SW_CONTROL_CONCAT_LEN;)
            {
                P[i*SW_CONTROL_CONCAT_LEN + i] += control_gains.v();
                ++i;
                P[i*SW_CONTROL_CONCAT_LEN + i] += control_gains.w();
                ++i;
            }
        }
};


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Main
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main(int argc, char** argv)
{
    if (SW_PREVIEW_WINDOW_LEN != 5)
    {
        cout << "This test requires preview window length to be 5." << endl;
        return (0);
    }

    referenceTrajectory ref_traj("test_04_data/trajectory.txt");
    State current_state;
    qpProblem qp;
    State init_state;


// Gains    
    Control control_gains;
    State state_gains;
    State sqrt_state_gains;

    state_gains.set (1.0, 1.0, 1.0, 1.0);
    sqrt_state_gains            = state_gains;
    control_gains.set(1.0, 1.0); 


// states from sensors
    vector<State> sens_states;
    ifstream ifs;
    ifs.open("test_04_data/sensors.txt");
    while (!ifs.eof())
    {
        State state;
        string line;

        getline(ifs, line);
        sscanf(line.c_str(), "%lf %lf %lf %lf", &state.x(), &state.y(), &state.theta(), &state.phi());
        state.normalizeTheta();
        sens_states.push_back(state);
    }
    ifs.close();


// ref_solution from Octave   
    vector<double> ref_solution;
    ifs.open("test_04_data/solution.txt");
    while (!ifs.eof())
    {
        double val;
        string line;

        getline(ifs, line);
        sscanf(line.c_str(), "%lf", &val);
        ref_solution.push_back(val);
    }
    ifs.close();


// control loop
    int sol_ind = 0;
    double max_err = 0.0;
    double sinT = 0.0, cosT = 0.0;
    for (int counter = 0 ;; ++counter)
    {             

        current_state = sens_states[counter];

        if (counter == 0)
        {
            init_state = current_state;
            sinT = sin(init_state.theta());
            cosT = cos(init_state.theta());
        }

        current_state.transform(init_state, sinT, cosT);

        swTimer timer;
        if (ref_traj.formPreview () == REF_TRAJECTORY_RETURN_STOP)
        {
            break;
        }

        qp.solve(
                ref_traj,
                control_gains,
                state_gains,
                sqrt_state_gains,
                current_state);
        timer.stop();
        printf("-------------\ntime = % f\n", timer.get());

        State state;
        Control control;

        state.x()     = ref_traj.x[0];
        state.y()     = ref_traj.y[0];
        state.theta() = ref_traj.theta[0];
        state.phi()   = ref_traj.phi[0];

        control.v()   = ref_traj.v[0];
        control.w()   = ref_traj.w[0];


        printf("-------------\n");
        for (unsigned int j = 0; j < SW_CONTROL_CONCAT_LEN; ++j, ++sol_ind)
        {
            double err = abs(qp.solution[j] - ref_solution[sol_ind]);
            printf("value: % 8e   ref: % 8e   err: % 8e\n", 
                    qp.solution[j], 
                    ref_solution[sol_ind], 
                    err);
            if (err > max_err)
            {
                max_err = err;
            }
        }
/*
        cout<< "CONTROL: phi = " << qp.control_phi << ", v = " << qp.control_v 
            << " // REFERENCE: phi = " << state.phi() << ", v = " << control.v() << endl;
*/            
    }
    printf("-------------\n");
    printf("Max error % 8e\n", max_err);

    return (0);
}
