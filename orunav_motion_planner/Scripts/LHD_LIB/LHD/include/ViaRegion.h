// Time-stamp: <2012-06-03 17:44:46 (drdv)>

#ifndef VIA_REGION_H
#define VIA_REGION_H

/******************************************
 **************** INCLUDES ****************
 ******************************************/

#include "SimpleBounds.h"
#include "math.h"

/******************************************/


/** \brief Defines a via region */
class ViaRegion
{
 public:

  /** \brief Simple bounds on x */
  SimpleBounds x;

  /** \brief Simple bounds on y */
  SimpleBounds y;

  /** \brief Simple bounds on theta */
  SimpleBounds theta;

  /** \brief Simple bounds on phi for the via regions

      \note phi.lb and phi.ub for the via regions are assumed to be equal to
      phi.lb and phi.ub for the stages (see Maneuver.h)
      
      \note phi.guess and phi.solution are used
  */
  SimpleBounds phi;

  /** \brief Default constructor */
  ViaRegion();
  
  /** \brief Default destructor */
  ~ViaRegion();
  
  /** \brief Set bounds on x and y (for via region)

      \param[in] x_lb Lower bound x
      \param[in] x_ub Upper bound x
      \param[in] y_lb Lower bound y
      \param[in] y_ub Upper bound y
      \return void
  */
  void set_xy(double x_lb, double x_ub, double y_lb, double y_ub); 

  /** \brief Set bounds on x and y (for via region)

      \param[in] x_center x center of square
      \param[in] y_center y center of square
      \param[in] relax how much to relax in each direction 
      \return void
  */
  void set_xy(double x_center, double y_center, double relax); 

  /** \brief Set bounds on theta

      \param[in] theta_center center of bounds on theta
      \param[in] relax how much to relax in each direction 
      \return void
  */
  void set_theta(double theta_center, double relax);

  /** \brief Print via region

      \return void
  */  
  void print();
  
};

#endif /*VIA_REGION_H*/
