/*
  SINGLE STAGE MANEUVER - PARALLELIZABLE

  --------------------------------------------------------
  The following command line arguments should be provided:
  --------------------------------------------------------
  prhs[0]  - L1 (front axel -- joint)	(double)
  prhs[1]  - L2 (back axel -- joint)	(double)
 
  prhs[2]  - phi_max					(double)

  prhs[3]  - x     initial    			(double)
  prhs[4]  - y     initial    			(double)
  prhs[5]  - theta initial    			(double)
  prhs[6]  - phi   initial    			(double)

  prhs[7]  - x     desired    			(double)
  prhs[8]  - y     desired    			(double)
  prhs[9]  - theta desired    			(double)
  prhs[10] - phi   desired    			(double)

  prhs[11] - ObjFun           			(int)
  prhs[12] - Direction (1:Forw;-1:Back)	(int)
  --------------------------------------------------------
 */

#include "mex.h"
#include "../include/LHD_utility.h"
#include "time.h"

void mexFunction(int nlhs, mxArray *plhs[], /* Output variables */
        int nrhs, const mxArray *prhs[])  {/* Input variables */
        
    
    /* Macros for the ouput and input arguments */
    #define X_OUT       plhs[0]
    #define Y_OUT       plhs[1]
    #define THETA_OUT   plhs[2]
    #define PHI_OUT     plhs[3]
    
    if(nrhs != 13) { /* Check the number of arguments */
        mexErrMsgTxt("Wrong number of input arguments.");
    }
    
    /* input */
    double *l1          = mxGetPr(prhs[0]);
    double *l2          = mxGetPr(prhs[1]);
    double *phi_max     = mxGetPr(prhs[2]);
    double *x_start     = mxGetPr(prhs[3]);
    double *y_start     = mxGetPr(prhs[4]);
    double *theta_start = mxGetPr(prhs[5]);
    double *phi_start   = mxGetPr(prhs[6]);
    double *x_goal      = mxGetPr(prhs[7]);
    double *y_goal      = mxGetPr(prhs[8]);
    double *theta_goal  = mxGetPr(prhs[9]);
    double *phi_goal    = mxGetPr(prhs[10]);
    /* the following values will be cast as int */
    double *obj_fun     = mxGetPr(prhs[11]);
    double *direction   = mxGetPr(prhs[12]);
    
    mexPrintf("l1 : \t %f\n", *l1);
    mexPrintf("l2 : \t %f\n", *l2);
    mexPrintf("phi_max : \t %f\n", -*(phi_max));
    mexPrintf("x_start : \t %f\n", *x_start);
    mexPrintf("y_start : \t %f\n", *y_start);
    mexPrintf("theta_start : \t %f\n", *theta_start);
    mexPrintf("phi_start : \t %f\n", *phi_start);
    mexPrintf("x_goal : \t %f\n", *x_goal);
    mexPrintf("y_goal : \t %f\n", *y_goal);
    mexPrintf("theta_goal : \t %f\n", *theta_goal);
    mexPrintf("phi_goal : \t %f\n", *phi_goal);
    mexPrintf("obj_fun : \t %d\n", (int)*obj_fun);
    mexPrintf("direction : \t %d\n", (int)*direction);
      
    Maneuver m(*l1, *l2); // Define NM maneuvers
    
    m.set_options("ObjFun", (int)*obj_fun);
	//m.set_options("MaxNumbIter", 500);
	m.set_options("NumbPointsSim", 1000);
    m.set_options("DisplayFlag", 0);
    
	m.phiAllStages.set(-(*phi_max),*phi_max);
	m.set_N(1);

	// ------------------------------------------------------------

	m.set_initial_state(*x_start, *y_start, *theta_start, *phi_start);
	m.set_final_state  (*x_goal, *y_goal, *theta_goal, *phi_goal);

	// ------------------------------------------------------------

	m.sp[0].direction = (int)*direction;

	// set the stage parameters' bounds
	double a_low = -50;
	double a_up = 50;
	double k_low = 0.1;
	double k_up = 20;

	// max iteration counter & timeout
	int counter = 30;
    time_t start,end;
    time (&start);
    time (&end);
     
	// ----------------- Solve cycle -----------------
	while (m.status != 0 && counter > 0 && difftime (end,start) < 30) {
        mexPrintf("Solve iteration: %d\n", counter);
		m.set_bounds_sp(a_low, a_up, k_low, k_up);
		m.set_poly_default();
		m.solve(0); // Solve

		// change the stage parameters' bounds
		a_low -= 10;
		a_up += 10;
		k_up += 20;
		counter --;
        time (&end);
        mexPrintf("Time elapsed (s): %f\n", difftime (end,start));
	}
    
    mexPrintf("Final status: %d\n", m.status);
    
    if (m.status == 0) {
        X_OUT = mxCreateDoubleMatrix(m.out[0].NumbPointsSim, 1, mxREAL);
        Y_OUT = mxCreateDoubleMatrix(m.out[0].NumbPointsSim, 1, mxREAL);
        THETA_OUT = mxCreateDoubleMatrix(m.out[0].NumbPointsSim, 1, mxREAL);
        PHI_OUT = mxCreateDoubleMatrix(m.out[0].NumbPointsSim, 1, mxREAL);
        
        double* x       = mxGetPr(X_OUT);
        double* y       = mxGetPr(Y_OUT);
        double* theta   = mxGetPr(THETA_OUT);
        double* phi     = mxGetPr(PHI_OUT);
        
        for (int i = 0; i < m.out[0].NumbPointsSim; i++) {
            x[i]        = m.out[0].state[i].x;
            y[i]        = m.out[0].state[i].y,
            theta[i]    = m.out[0].state[i].theta,
            phi[i]      = m.out[0].state[i].phi;
        }
    } else {
        X_OUT = mxCreateDoubleMatrix(0, 0, mxREAL);
        Y_OUT = mxCreateDoubleMatrix(0, 0, mxREAL);
        THETA_OUT = mxCreateDoubleMatrix(0, 0, mxREAL);
        PHI_OUT = mxCreateDoubleMatrix(0, 0, mxREAL);
    }
    return;
}
        

