// Time-stamp: <2012-06-01 22:52:15 (drdv)>

#ifndef STATE_H
#define STATE_H

/******************************************
 **************** INCLUDES ****************
 ******************************************/

#include <stdio.h>

/******************************************/


/** \brief Defines the state of LHD vehicle */
class State
{
 public:

  /** \brief Position x */
  double x;

  /** \brief Position y */
  double y;

  /** \brief Orientation [rad.] */
  double theta;

  /** \brief Steering angle [rad] */
  double phi;

  /** \brief Default constructor */
  State();
  
  /** \brief Default destructor */
  ~State();
  
  /** \brief Constructor
      
      \param[in] _x x position
      \param[in] _y y position
      \param[in] _theta Orientation
      \param[in] _phi Steering angle
  */
  State(double _x, double _y, double _theta, double _phi);

  /** \brief Set the state
      
      \param[in] _x x position
      \param[in] _y y position
      \param[in] _theta Orientation
      \param[in] _phi Steering angle
      \return void
  */
  void set(double _x, double _y, double _theta, double _phi);
  
  /** \brief Print state 

      \param[in] description Description of the state (e.g., "Initial state")
      \return void
  */  
  void print(const char * description = "");
  
};

#endif /*STATE_H*/
