// Time-stamp: <2012-06-02 09:42:58 (drdv)>

#include "../include/State.h"

State::State()
{
  set(0,0,0,0);
}
  
State::~State(){}
  
State::State(double _x, double _y, double _theta, double _phi)
{
  set(_x, _y, _theta, _phi);
}

void State::set(double _x, double _y, double _theta, double _phi)
{
  x = _x;
  y = _y;
  theta = _theta;
  phi = _phi;
}
  
void State::print(const char *description)
{
  printf(" {%s} [% f, % f, % f, % f] \n", description, x, y, theta, phi);
}

//EOF
