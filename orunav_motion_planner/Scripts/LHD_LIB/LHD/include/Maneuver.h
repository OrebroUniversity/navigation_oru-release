// Time-stamp: <2012-06-29 13:37:37 (drdv)>

#ifndef MANEUVER_H
#define MANEUVER_H

/******************************************
 **************** INCLUDES ****************
 ******************************************/

#include <acado_optimal_control.hpp>

#include "ViaRegion.h"
#include "common.h"
#include "Polynomial.h"
#include "StageOutput.h"
#include "StageParameters.h"
#include "OptionsLHD.h"

/******************************************/

/** \brief Defines all kind of constraints */
class Maneuver
{
 public:
  
  /** \brief Number of stages */
  int N;
  
  /** \brief Distance from the front axis to the joint */
  double L1;
  
  /** \brief Distance from the rear axis to the joint */
  double L2;

  /** \brief Initial state */
  State s0;

  /** \brief Final state */
  State s1;

  /** \brief Choose which element of f_tmp in ODE_F(double *i, double *, void *) to use in the integration process 

      ode_index = -1 means choose all

      \par

      ode_index = i means choose the i-th one (i >= 0)
  */
  int ode_index;

  /** \brief Indicates whether there was a problem when solving the OCP 

      status = -1: we still havent attempted to solve the OCP

      \par

      status = 0 : a solution was found
      
      \par

      status > 0 : there was a problem during solving the OCP
   */
  int status;

  /** \brief Simple bounds on phi (assumed to be the same for all stages) */
  SimpleBounds phiAllStages;

  /** \brief Vector of stage parameters (a0_x, a0_y, aT_x, aT_y, k_0, k_T) */
  std::vector<StageParameters> sp;

  /** \brief Simple bounds on the parameters associated to via regions */
  std::vector<ViaRegion> vr;

  /** \brief Vector of polynomials */
  std::vector<Polynomial> poly;

  /** \brief Vector of Stage Outputs */
  std::vector<StageOutput> out;

  /** \brief Options */
  OptionsLHD options;

  /** \brief Default constructor */
  Maneuver();

  /** \brief Default destructor */
  ~Maneuver();

  /** \brief Constructor

      \param[in] _L1 Distance from the front axis to the joint (#L1)
      \param[in] _L2 Distance from the rear axis to the joint (#L2)
  */
  Maneuver(double _L1, double _L2);
  
  /** \brief Constructor

      \param[in] _N Number of stages (#N).
  */
  Maneuver(int _N);

  /** \brief Sets #poly by default
      
      \return void

      \attention Do not modify options.NumbPointsSim after this function has been called
  */
  void set_poly_default();

  /** \brief Sets #poly
      
      \param[in] k set poly[k]
      \param[in] t0 Initial time
      \param[in] t1 Final time
      \param[in] dp_i Initial velocity
      \param[in] dp_f Final velocity
      \return void

      \note p_i = 0 (initial position) and p_f = 1 (final position) are assumed

      \attention Do not modify options.NumbPointsSim after this function has been called

  */
  void set_poly(int k, double t0, double t1, double dp_i, double dp_f);

  /** \brief Set bounds on the stage parameters

      \param[in] lb_a Lower bound for the "a parameters"
      \param[in] ub_a Upper bound for the "a parameters"
      \param[in] lb_k Lower bound for the "k parameters"
      \param[in] ub_k Upper bound for the "k parameters"
      \return void
  */
  void set_bounds_sp(double lb_a, double ub_a, double lb_k, double ub_k);

  /** \brief Set number of stages

      \param[in] _N Number of stages (#N). N>0 is assumed (note the resize(N-1)) 
      \return void
  */
  void set_N(int _N);

  /** \brief Set #status
      
      \param[in] _status Value to be assigned to #status
      \return void
  */
  void set_status(int _status);
  
  /** \brief Set #ode_index 
      
      \param[in] _ode_index #ode_index
      \return void
  */
  void choose_ode(int _ode_index);
  
  /** \brief Set dimensions
      
      \param[in] _L1 Distance from the front axis to the joint (#L1)
      \param[in] _L2 Distance from the rear axis to the joint (#L2)
      \return void
  */
  void set_dimensions(double _L1, double _L2);

 /** \brief Set initial state
      
      \param[in] _x Initial x position
      \param[in] _y Initial y position
      \param[in] _theta Initial orientation
      \param[in] _phi Initial searing angle      
      \return void
  */  
  void set_initial_state(double _x, double _y, double _theta, double _phi);

  /** \brief Set final state
      
      \param[in] _x Final x position
      \param[in] _y Final y position
      \param[in] _theta Final orientation
      \param[in] _phi Final searing angle
      \return void
  */  
  void set_final_state(double _x, double _y, double _theta, double _phi);

  /** \brief Print constraints info

      \return void
  */   
  void print();

  /** \brief Print solution

      \return void
  */   
  void print_solution();

  /** \brief Output solution to file

      \param[in] output_file Name of file
      \return void
  */   
  void output_solution2file(const char *output_file);

  /** \brief Generates a Matlab function that visualizes s0, s1 and via regions

      \param[in] counter Counter ... to explain (this method will change)
      \return void

      \note A file with name load_basic_variables.m is created in the Matlab directory
  */   
  void operator >> (int counter);
  
  /** \brief Sets options
      
      \param[in] option_name Name of an option
      \param[in] value Value of the option with name option_name
      \return void
  */
  void set_options(const char *option_name, int value);

  /** \brief Sets options
      
      \param[in] option_name Name of an option
      \param[in] value Value of the option with name option_name
      \return void
  */   
  void set_options(const char *option_name, double value);

  /*********************************************************************************
   *********************************************************************************
   ********************************************************************************/
  
  /** \brief Form and Solvse the OCP (find the parameters)
      
      \return void
  */
  void plan();
  
  /** \brief Integrate ODE_F(double *, double *, void *) with fixed parameters and generates a profile for phi
      
      \return void
  */
  void integrate();
  
  /** \brief Generates x, y, theta profiles (corresponding to the profile of phi, which is generated in 
      integrate_F()
      
      \return void
  */
  void form_pose();

  /** \brief Execute plan_F(), integrate_F() and form_pose_F()
      
      \return void
  */
  void solve(int exit_flag = 1);

};

#endif /*MANEUVER_H*/
