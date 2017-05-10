// Time-stamp: <2012-07-10 08:35:10 (drdv)>

#include "../include/Maneuver.h"

Maneuver::Maneuver()
{
  set_status(-1);
  choose_ode(-1);
  phiAllStages.set(-M_PI/4,M_PI/4); // default bounds on phi
}
  
Maneuver::~Maneuver(){}

Maneuver::Maneuver(double _L1, double _L2)
{
  set_status(-1);
  choose_ode(-1);
  phiAllStages.set(-M_PI/4,M_PI/4); // default bounds on phi
  set_dimensions(_L1, _L2);
}
  
Maneuver::Maneuver(int _N)
{
  set_status(-1);
  set_N(_N);
  choose_ode(-1);
  phiAllStages.set(-M_PI/4,M_PI/4); // default bounds on phi
  
  // default bounds on the parameters
  set_bounds_sp(-50, 50, 0.1, 20);
}

void Maneuver::set_poly_default()
{
  for (int i=0; i<N; i++)
    {
      poly[i].set_EquidistantGrid( 0, 1, options.NumbPointsSim);
      //poly[i].poly3_get_coefficients(0, 1, 0, 0);
      poly[i].poly1_get_coefficients(0, 1);
      poly[i].eval();
    }
}

void Maneuver::set_poly(int k, double t0, double t1, double dp_i, double dp_f)
{
  poly[k].set_EquidistantGrid( t0, t1, options.NumbPointsSim);

  if (sp[k].direction == 1) // forward motion
    poly[k].poly3_get_coefficients(0, 1, dp_i, dp_f);
  else // backward motion
    poly[k].poly3_get_coefficients(0, 1, dp_f, dp_i);
  
  poly[k].eval();
}

void Maneuver::set_bounds_sp(double lb_a, double ub_a, double lb_k, double ub_k)
{ 
  for (int i=0; i<N; i++)
    sp[i].set_bounds(lb_a, ub_a, lb_k, ub_k);
}

void Maneuver::set_status(int _status)
{
  status = _status;
}

void Maneuver::set_N(int _N)
{
  N = _N;
  sp.resize(N);
  vr.resize(N-1);
  poly.resize(N, Polynomial(3)); // use third order polynomials
}

void Maneuver::choose_ode(int _ode_index)
{
  ode_index = _ode_index;
}

void Maneuver::set_dimensions(double _L1, double _L2)
{
  L1 = _L1;
  L2 = _L2;
}

void Maneuver::set_initial_state(double _x, double _y, double _theta, double _phi)
{
  if ((_theta < THETA_ZERO) && (_theta > -THETA_ZERO))
  {
    if (_theta>0)
      _theta = THETA_ZERO;
    else
      _theta = -THETA_ZERO;
  }
  s0.set(_x, _y, _theta, _phi);
}

void Maneuver::set_final_state(double _x, double _y, double _theta, double _phi)
{  
  if ((_theta < THETA_ZERO) && (_theta > -THETA_ZERO))
  {
    if (_theta>0)
      _theta = THETA_ZERO;
    else
      _theta = -THETA_ZERO;
  }
  s1.set(_x, _y, _theta, _phi);
}

void Maneuver::print()
{
  printf("======================================= \n");
  printf("phiAllStages bounds \n");
  printf("======================================= \n");
  phiAllStages.print();
  
  printf("======================================= \n");
  printf("Parameter bounds \n");
  printf("======================================= \n");
  for (int i=0; i<N; i++)
    {
      printf("--------> stage %d <--------\n",i);
      sp[i].print();
      printf("\n");
    }
  printf("======================================= \n");
  printf("Via regions constraints \n");
  printf("======================================= \n");
  for (int i=0; i<N-1; i++)
    {
      printf("--------> vr %d <--------\n",i);
      vr[i].print();
      printf("\n");
    }
  printf("======================================= \n");
  printf(" L1 = %f, L2 = %f \n", L1, L2);
  s0.print("state 0");
  s1.print("state 1");
  printf("======================================= \n");
}

void Maneuver::print_solution()
{
  printf("======================================= \n");
  printf("status = %d \n",status);
  printf("--------------------------------------- \n");
  printf("Parameters solution\n");
  printf("--------------------------------------- \n");
  printf(" User guess is depicted as well         \n");
  printf(" NOTE: In case of auto-initialization a \n");
  printf(" different guess is generated internaly \n");
  printf("======================================= \n");
  for (int i=0; i<N; i++)
    {
      printf("--------> stage %d <--------\n",i);
      printf(" a0_x = % f (guess = % f) \n",sp[i].a0_x.solution, sp[i].a0_x.guess);
      printf(" a0_y = % f (guess = % f) \n",sp[i].a0_y.solution, sp[i].a0_y.guess);
      printf(" aT_x = % f (guess = % f) \n",sp[i].aT_x.solution, sp[i].aT_x.guess);
      printf(" aT_y = % f (guess = % f) \n",sp[i].aT_y.solution, sp[i].aT_y.guess);
      printf(" k_0  = % f (guess = % f) \n",sp[i].k_0.solution , sp[i].k_0.guess);
      printf(" k_T  = % f (guess = % f) \n",sp[i].k_T.solution , sp[i].k_T.guess);
      printf("\n");
    }
  printf("======================================= \n");
  printf("Via regions solution \n");
  printf("======================================= \n");
  for (int i=0; i<N-1; i++)
    {
      printf("--------> stage %d <--------\n",i);
      printf(" x     = % f (guess = % f) \n",vr[i].x.solution    , vr[i].x.guess);
      printf(" y     = % f (guess = % f) \n",vr[i].y.solution    , vr[i].y.guess);
      printf(" theta = % f (guess = % f) \n",vr[i].theta.solution, vr[i].theta.guess);
      printf(" phi   = % f (guess = % f) \n",vr[i].phi.solution  , vr[i].phi.guess);
      printf("\n");
    }
  printf("======================================= \n");
}

void Maneuver::output_solution2file(const char *output_file)
{    
  FILE *file_op = fopen(output_file, "w");
    
  if(!file_op)
    {
      std::cerr << "Cannot open file (for writing) " << output_file << std::endl;
      std::exit(1);
    }
    
  fprintf(file_op, "#NumberOfStages \n");
  fprintf(file_op, "%d \n", N);
  fprintf(file_op, "\n");

  fprintf(file_op, "#StageParameters \n");
  for (int i=0; i<N; i++)
    {
      fprintf(file_op, "%4.20f \n", sp[i].a0_x.solution);
      fprintf(file_op, "%4.20f \n", sp[i].a0_y.solution);
      fprintf(file_op, "%4.20f \n", sp[i].aT_x.solution);
      fprintf(file_op, "%4.20f \n", sp[i].aT_y.solution);
      fprintf(file_op, "%4.20f \n", sp[i].k_0.solution);
      fprintf(file_op, "%4.20f \n", sp[i].k_T.solution);
    }
  fprintf(file_op, "\n");

  fprintf(file_op, "#ViaParameters \n");
  for (int i=0; i<N-1; i++)
    {
      fprintf(file_op, "%4.20f \n", vr[i].x.solution);
      fprintf(file_op, "%4.20f \n", vr[i].y.solution);
      fprintf(file_op, "%4.20f \n", vr[i].theta.solution);
      fprintf(file_op, "%4.20f \n", vr[i].phi.solution);
    }
  fclose(file_op);
}


void Maneuver::set_options(const char *option_name, int value)
{
  options.set(option_name,value);
}

void Maneuver::set_options(const char *option_name, double value)
{
  options.set(option_name,value);
}

//EOF
