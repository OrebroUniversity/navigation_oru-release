// Time-stamp: <2012-06-29 09:56:08 (drdv)>

#ifndef STAGE_PARAMETERS_H
#define STAGE_PARAMETERS_H

/******************************************
 **************** INCLUDES ****************
 ******************************************/

#include "SimpleBounds.h"

/******************************************/


/** \brief Defines stage parameters (a0_x, a0_y, aT_x, aT_y, k_0, k_T) */
class StageParameters
{
 public:

  /** \brief Direction of motion (1: forward, -1: backward) 

      \note By default direction = 1
   */
  int direction;

  /** \brief Simple bounds on a0_x */
  SimpleBounds a0_x;

  /** \brief Simple bounds on a0_y */
  SimpleBounds a0_y;

  /** \brief Simple bounds on aT_x */
  SimpleBounds aT_x;

  /** \brief Simple bounds on aT_y */
  SimpleBounds aT_y;

  /** \brief Simple bounds on k_0 */
  SimpleBounds k_0;

  /** \brief Simple bounds on k_T */
  SimpleBounds k_T;

  /** \brief Default constructor */
  StageParameters();
  
  /** \brief Default destructor */
  ~StageParameters();
  
  /** \brief Set guess for the stage parameters

      \param[in] _a0_x #a0_x 
      \param[in] _a0_y #a0_y 
      \param[in] _aT_x #aT_x 
      \param[in] _aT_y #aT_y 
      \param[in] _k_0  #k_0 
      \param[in] _k_T  #k_T 
      \return void
  */
  void set_guess(double _a0_x, double _a0_y, double _aT_x, double _aT_y, double _k_0, double _k_T); 

  /** \brief Set some of the bounds

      \param[in] lb_a Lower bound for the "a parameters"
      \param[in] ub_a Upper bound for the "a parameters"
      \param[in] lb_k Lower bound for the "k parameters"
      \param[in] ub_k Upper bound for the "k parameters"
      \return void
  */
  void set_bounds(double lb_a, double ub_a, double lb_k, double ub_k);

  /** \brief Set the direction of motion 

      \param[in] _direction Direction of motion
      \return void
  */
  void set_direction(int _direction);

  /** \brief Print stage parameters

      \return void
  */  
  void print();
  
};

#endif /*STAGE_PARAMETERS_H*/
