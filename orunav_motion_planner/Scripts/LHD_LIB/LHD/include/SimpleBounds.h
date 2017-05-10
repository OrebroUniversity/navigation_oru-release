// Time-stamp: <2012-06-01 22:52:09 (drdv)>

#ifndef SIMPLE_BOUNDS_H
#define SIMPLE_BOUNDS_H

/******************************************
 **************** INCLUDES ****************
 ******************************************/

#include <stdio.h>

/******************************************/


/** \brief Defines simple bounds (and initial guess) for a single variable */
class SimpleBounds
{
 public:

  /** \brief Lower bound */
  double lb;
  
  /** \brief Upper bound */
  double ub;
  
  /** \brief Initial guess

      \note: The default initial guess is in the middle of the bounds
  */
  double guess;
  
  /** \brief Solution */
  double solution;
  
  /** \brief Default constructor */
  SimpleBounds();
  
  /** \brief Default destructor */
  ~SimpleBounds();
  
  /** \brief Constructor

      \param[in] _lb Lower bound
      \param[in] _ub Upper bound
  */
  SimpleBounds(double _lb, double _ub);

  /** \brief Set bounds

      \param[in] _lb Lower bound
      \param[in] _ub Upper bound
      \return void
  */  
  void set(double _lb, double _ub);

  /** \brief Set initial guess

      \param[in] _guess Guess
      \return void
  */  
  void set_guess(double _guess);

  /** \brief Set bounds and guess

      \param[in] _lb Lower bound
      \param[in] _ub Upper bound
      \param[in] _guess Initial guess
      \return void
  */  
  void set(double _lb, double _ub, double _guess);

  /** \brief Set solution

      \param[in] _solution solution
      \return void
  */  
  void set_solution(double _solution);

  /** \brief Print bounds

      \return void
  */  
  void print();
  
};

#endif /*SIMPLE_BOUNDS_H*/
