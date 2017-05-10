#include <sys/types.h>
#include <unistd.h>
#include <sched.h>
#include <stdlib.h>
#include <iostream>


#include "qpOASES.hpp"


using namespace std;

/*
 * Finds a point, which satisfies constraints and is nearest to the given point.
 */
int main(int argc, char** argv)
{
    double x[2] = {0.0, 0.0};

    
    double A[2*3] = {   -1.0, 1.0,      //  -x + y <= 0
                         1.0, 0.0,      //   x <= 1.0
                         0.0, -1.0,  }; //  -y <= 0 
    double B[3] = {0.0, 1.0, 0.0};


    USING_NAMESPACE_QPOASES;

    QProblem qp_oases(2, 3, HST_IDENTITY);
    int nWSR = 500;
    double cputime = 1.0;


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


    //double P[4] = {1.0, 0.0, 0.0, 1.0};
    //double p[2] = {0.0, 0.0};
    double p[2] = {0.5, 1.0}; // negated initial state

    if (qp_oases.init(
            NULL,       // Hessian
            p,          // linear term
            A,          // A matrix of constraints
            NULL,       // lower bounds 
            NULL,       // upper bounds
            NULL,       // lower bounds on A
            B,          // upper bounds on A
            nWSR,
            &cputime) == SUCCESSFUL_RETURN)
    {
        if (qp_oases.getPrimalSolution (x) != SUCCESSFUL_RETURN)
        {
            cout << "Cannot get the solution of the QP." << endl;
            exit (1);
        }

        cout << "x = " << x[0] << "   y = " << x[1] << endl;
    }
    else
    {
        cout << "Cannot solve the QP. CPU = " << cputime << ", ITER = " << nWSR << endl;
        exit (1);
    }
}
