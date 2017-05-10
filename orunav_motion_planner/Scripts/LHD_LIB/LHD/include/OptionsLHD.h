// Time-stamp: <2012-06-27 15:29:58 (drdv)>

#ifndef OPTIONS_H
#define OPTIONS_H

/******************************************
 **************** INCLUDES ****************
 ******************************************/

#include <stdio.h>
#include <cstdlib>
#include <cstring>

/******************************************/

/** \brief Defines option flags */
class OptionsLHD
{
 public:

  friend class Maneuver;

  /** \brief Default constructor */
  OptionsLHD();
  
  /** \brief Default destructor */
  ~OptionsLHD();
  
  /** \brief Constructor

      \param[in] _NumbIntervalsOpt Number of sampling intervals for optimization (#NumbIntervalsOpt)
      \param[in] _NumbPointsSim Number of sampling intervals for simulation (#NumbPointsSim)
      \param[in] _InitialGuess Initial guess flag (#InitialGuess)
      \param[in] _KKT_tol KKT tolerance (#KKT_tol)
      \param[in] _ObjFun Objective function flag (#ObjFun)
      \param[in] _MaxNumbIter Max number of iterations for the SQP method (#MaxNumbIter)
      \param[in] _DisplayFlag Dismplay flag (#DisplayFlag)
  */
  OptionsLHD(int _NumbIntervalsOpt, int _NumbPointsSim, int _InitialGuess, double _KKT_tol, int _ObjFun, int _MaxNumbIter, int _DisplayFlag);
  
  /** \brief Print options 
      
      \return void
  */  
  void print();

  /** \brief Sets options

      \param[in] option_name Name of an option
      \param[in] value Value of the option with name option_name
      \return void

      \verbatim
      Possible names of options:
      --------------------------
      NumbIntervalsOpt 
      NumbPointsSim 
      InitialGuess 
      ObjFun
      MaxNumbIter 
      DisplayFlag
      \endverbatim
  */
  void set(const char *option_name, int value);

  /** \brief Sets options

      \param[in] option_name Name of an option
      \param[in] value Value of the option with name option_name
      \return void

      \verbatim
      Possible names of options:
      --------------------------
      KKT_tol
      \endverbatim
  */
  void set(const char *option_name, double value);

 protected:

  /** \brief Number of (sampling) intervals for optimization

      \note If NumbIntervalsOpt = 5, the optimization is done at {0, 1, 2, 3, 4, 5}. Hence, there are
      NumbIntervalsOpt+1 points at which we obtain the profile of phi
   */
  int NumbIntervalsOpt;

  /** \brief Number of (sampling) points for optimization (computed internaly)

      \note NumbPointsOpt = NumbIntervalsOpt + 1.
   */
  int NumbPointsOpt;

  /** \brief Number of points at which to return the simulation results

      If NumbPointsSim = 5, the simulation is done at 5 points between (and including) t_start and
      t_end. For example if t_start = 1 and t_end = 5, we obtain the simulated profile of the state
      at times {1, 2, 3, 4, 5}. See Polynomial::set_EquidistantGrid(...).
   */
  int NumbPointsSim;

  /** \brief Initial guess flag

      InitialGuess = 0 (using auto initialization)
      \par

      InitialGuess = 1 (initial guess from user)
  */
  int InitialGuess;

  /** \brief KKT tolerance. */
  double KKT_tol;

  /** \brief Objective function flag

      ObjFun = 0 (do not use objective function)
      \par

      ObjFun = 1 (use objective function)

      \note see OPTIMIZATION_ALGORITHM in common.h
  */
  int ObjFun;

  /** \brief Max number of iterations for the SQP method

      \note MaxNumbIter > 0
  */
  int MaxNumbIter;

  /** \brief Dismplay flag

      DisplayFlag = 0 (do not display results)
      \par

      DisplayFlag = 1 (display results)
  */
  int DisplayFlag;
  
};

#endif /*OPTIONS_H*/
