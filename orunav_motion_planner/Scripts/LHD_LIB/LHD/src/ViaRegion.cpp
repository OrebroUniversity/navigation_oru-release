// Time-stamp: <2012-06-04 11:40:09 (drdv)>

#include "../include/ViaRegion.h"

ViaRegion::ViaRegion()
{
  // default bounds on phi
  phi.set(-M_PI/4,M_PI/4,0); 
  
  // default bounds on theta (do not use with via points)
  theta.set(-M_PI,M_PI,0); 
}

ViaRegion::~ViaRegion(){}
  
void ViaRegion::set_xy(double x_lb, double x_ub, double y_lb, double y_ub)
{
  x.set(x_lb,x_ub,(x_lb+x_ub)/2);
  y.set(y_lb,y_ub,(y_lb+y_ub)/2);
}

void ViaRegion::set_xy(double x_center, double y_center, double relax)
{
  if (relax < 0)
    {
      printf("WARNING (in function %s): relax < 0 \n",__FUNCTION__);
      relax = -relax;
    }

  x.set(x_center - relax, x_center + relax, x_center);
  y.set(y_center - relax, y_center + relax, y_center);
}

void ViaRegion::set_theta(double theta_center, double relax)
{
  if (relax < 0)
    {
      printf("WARNING (in function %s): relax < 0 \n",__FUNCTION__);
      relax = -relax;
    }

  theta.set(theta_center - relax, theta_center + relax, theta_center);
}

void ViaRegion::print()
{
  x.print();
  y.print();
  theta.print();
  phi.print();
}

//EOF
