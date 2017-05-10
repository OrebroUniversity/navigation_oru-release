// Time-stamp: <2012-06-01 23:21:05 (drdv)>

#include "../include/SimpleBounds.h"

SimpleBounds::SimpleBounds()
{
  // set to something "large"
  lb = -1000;
  ub =  1000;
  set_guess(0);
  solution = 0;
}
  
SimpleBounds::~SimpleBounds(){}
  
SimpleBounds::SimpleBounds(double _lb, double _ub)
{
  set(_lb, _ub);
  set_guess((_lb+_ub)/2);
}

void SimpleBounds::set(double _lb, double _ub)
{
  lb = _lb;
  ub = _ub;
  set_guess((_lb+_ub)/2);
}

void SimpleBounds::set_guess(double _guess)
{
  guess = _guess;
}

void SimpleBounds::set(double _lb, double _ub, double _guess)
{
  lb = _lb;
  ub = _ub;
  set_guess(_guess);
}

void SimpleBounds::set_solution(double _solution)
{
  solution = _solution;
} 

void SimpleBounds::print()
{
  printf(" [% f, % f] (%f) \n", lb, ub, guess);
}

//EOF
