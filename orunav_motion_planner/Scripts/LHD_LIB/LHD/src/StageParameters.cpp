#include "../include/StageParameters.h"

StageParameters::StageParameters()
{
  set_direction(1);
}

StageParameters::~StageParameters(){}

void StageParameters::set_guess(double _a0_x, double _a0_y, double _aT_x, double _aT_y, double _k_0, double _k_T)
{
  a0_x.set_guess(_a0_x);
  a0_y.set_guess(_a0_y);
  aT_x.set_guess(_aT_x);
  aT_y.set_guess(_aT_y);
  k_0.set_guess(_k_0);
  k_T.set_guess(_k_T);
}

void StageParameters::set_bounds(double lb_a, double ub_a, double lb_k, double ub_k)
{
  a0_x.set(lb_a, ub_a);
  a0_y.set(lb_a, ub_a);
  aT_x.set(lb_a, ub_a);
  aT_y.set(lb_a, ub_a);
  // k_0.set(lb_k, ub_k, lb_k);
  // k_T.set(lb_k, ub_k, lb_k);
  k_0.set(lb_k, ub_k);
  k_T.set(lb_k, ub_k);
}

void StageParameters::set_direction(int _direction)
{
  direction = _direction;
}

void StageParameters::print()
{
  a0_x.print();
  a0_y.print();
  aT_x.print();
  aT_y.print();
  k_0.print();
  k_T.print();  
}
