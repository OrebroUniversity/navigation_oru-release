// Time-stamp: <2012-06-27 14:58:55 (drdv)>

/* \brief Define in order to use OptimizationAlgorithm 

   \note If not defined ParameterEstimationAlgorithm is used instead. This requires the use of ocp.minimizeLSQ(h) (see Maneuver_ACADO.cpp)
*/
#define OPTIMIZATION_ALGORITHM

/** \biref Number of stage parameters

    \note We have 6 parameters for each stage: a0_x, a0_y, aT_x, aT_y, k_0, k_T
*/
#define NUM_PARAM 6

/** \biref Number of parameters associated to a via region

    \note We have 4 parameters for each via region: x, y, theta, phi
*/
#define NUM_VIA 4 

/** \biref MAX_NUMBER_OF_STAGES

    \note Largest nummber of stages for a single maneuver. I use it in ODE_F( double *, double *,
    void * ) to initialize f_tmp. Otherwise I either have to use new (but this is slower) or use
    templates which is slightly faster but a bit inconvenient. 
*/
#define MAX_NUMBER_OF_STAGES 5    

/* \brief Threshold for theta

   \verbatim
   There is a problem when I define theta=0 (for the boundary conditions). It works if theta = 0 + 1e-14 !

   If in ODE_OneStage_opt.cpp I declare theta_0 as a double it works!! So maybe this is a bug in
   ACADO. In my previous implementation when I was using c-function to define the ODE I did not have
   such problmes. So now I do

  if ((theta < THETA_ZERO) && (theta > -THETA_ZERO))
    {
      if (_theta>0)
    theta = THETA_ZERO;
      else
    theta = -THETA_ZERO;
    }

    See Maneuver::set_initial_state and Maneuver::set_final_state
   \endverbatim
*/
#define THETA_ZERO 0.00001 

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

extern "C" {

  /** \brief Computes the three coefficients of the ODE

      \param[in] L1 Distance from the front axis to the joint
      \param[in] L2 Distance from the rear axis to the joint
      \param[in] a0_x Parameter to be identified (x acceleration at time 0)
      \param[in] a0_y Parameter to be identified (y acceleration at time 0)
      \param[in] aT_x Parameter to be identified (x acceleration at time 1)
      \param[in] aT_y Parameter to be identified (y acceleration at time 1)
      \param[in] k_0 Parameter to be identified (parameter related to initial orientation)
      \param[in] k_T Parameter to be identified (parameter related to final orientation)
      \param[in] x_0 Initial x position of the fron part of the vehicle
      \param[in] y_0 Initial y position of the fron part of the vehicle
      \param[in] theta_0 Initial orientation of the front part of the vehicle
      \param[in] x_T Final x position of the fron part of the vehicle
      \param[in] y_T Final y position of the fron part of the vehicle
      \param[in] theta_T Final orientation of the front part of the vehicle
      \param[in] t time
      \param[out] out Coefficients of the ODE
      \return void

      \verbatim
      The ODE:
      \dot{phi} = out[0]*sin(phi) + out[1]*cos(phi) + out[2];
      \endverbatim
      
  */
  void get_abc(double L1, double L2,
	       double a0_x, double a0_y, double aT_x, double aT_y, double k_0, double k_T,
	       double x_0, double y_0, double theta_0,
	       double x_T, double y_T, double theta_T,
	       double t, double out[3]);

  /** \brief Computes the parameters defining the polynomials for x and y position

      \param[in] a0_x Parameter to be identified (x acceleration at time 0)
      \param[in] a0_y Parameter to be identified (y acceleration at time 0)
      \param[in] aT_x Parameter to be identified (x acceleration at time 1)
      \param[in] aT_y Parameter to be identified (y acceleration at time 1)
      \param[in] k_0 Parameter to be identified (parameter related to initial orientation)
      \param[in] k_T Parameter to be identified (parameter related to final orientation)
      \param[in] x_0 Initial x position of the fron part of the vehicle
      \param[in] y_0 Initial y position of the fron part of the vehicle
      \param[in] theta_0 Initial orientation of the front part of the vehicle
      \param[in] x_T Final x position of the fron part of the vehicle
      \param[in] y_T Final yposition of the fron part of the vehicle
      \param[in] theta_T Final orientation of the front part of the vehicle
      \param[out] out Parameters defining the polynomials for x and y position
      \return void    

      \note see Maple/get_parameters_xy.mpl
  */  
  void get_parameters_xy(double a0_x, double a0_y, double aT_x, double aT_y, double k_0, double k_T,
			 double x_0, double y_0, double theta_0,
			 double x_T, double y_T, double theta_T,
			 double out[12]);
}



