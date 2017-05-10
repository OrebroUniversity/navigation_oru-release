// Time-stamp: <2012-06-02 10:10:39 (drdv)>

#include "../include/Polynomial.h"

Polynomial::Polynomial()
{
  set_N(-1);
}
  
Polynomial::~Polynomial(){}

Polynomial::Polynomial(int _N)
{
  set_N(_N);
}

Polynomial::Polynomial(int _N, double *coefficients)
{
  set_N(_N);
  set_a(coefficients);
}

void Polynomial::set_GridSize(int GridSize)
{
  grid.resize(GridSize,0);
}

void Polynomial::set_GridPoint(int k, double x)
{
  grid[k] = x;
}

void Polynomial::append_GridPoint(double x)
{
  grid.push_back(x);
}

double Polynomial::get_FirstGridPoint()
{
  return grid.front();
}

double Polynomial::get_LastGridPoint()
{
  return grid.back();
}

void Polynomial::eval()
{
  eval_p();
  eval_dp();
}

void Polynomial::eval_p()
{
  int GridSize = grid.size();
  
  if (GridSize == 0)
    printf("WARNING: grid_size = 0\n");
  else
    {
      p.resize(GridSize,0);	
      for (int i=0; i<GridSize; i++)
	{
	  /*
	    p[i] = 0;
	    for (int j=0; j<N+1; j++)
	    p[i] += a[j]*pow(grid[i],j); // inefficient
	  */
	  
	  p[i] = a[N-1] + a[N]*grid[i];
	  for (int j=N-1; j>0; j--)
	    p[i] = a[j-1] + p[i]*grid[i];
	}
    }
}

void Polynomial::eval_dp()
{
  int GridSize = grid.size();
  
  if (GridSize == 0)
    printf("WARNING: grid_size = 0\n");
  else
    {
      dp.resize(GridSize,0);	
      for (int i=0; i<GridSize; i++)
	{
	  /*
	    dp[i] = 0;
	    for (int j=1; j<N+1; j++)
	    dp[i] += j*a[j]*pow(grid[i],j-1); // inefficient
	  */
	  
	  dp[i] = (N-1)*a[N-1] + N*a[N]*grid[i];
	  for (int j=N-1; j>1; j--)
	    dp[i] = (j-1)*a[j-1] + dp[i]*grid[i];
	}
    }
}

void Polynomial::set_EquidistantGrid(double t0, double t1, int GridSize)
{
  grid.resize(GridSize);
  
  double step = (t1-t0)/(GridSize-1);    
  grid[0] = t0;
  for (int i=1; i<GridSize; i++)
    grid[i] = grid[i-1] + step;
}

void Polynomial::set_a(double *coefficients)
{
  for (int i=0; i<N+1; i++)
    a[i] = coefficients[i];
}

void Polynomial::set_N(int _N)
{
  N = _N;
  a.resize(N+1,0);
}

void Polynomial::print_a(const char *output_file)
{
  if (output_file == NULL)
    {
      printf("  (N=%d)   a = { ",N);
      for (int i=0; i<N+1; i++)
	printf("%4.20f ",a[i]);
      printf("}\n");
    }
  else
    {
      write_file(&a[0], N+1, 1, output_file, "w"); 	
    }
}

void Polynomial::print_grid(const char *output_file)
{
  int GridSize = grid.size();
  if (output_file == NULL)
    {
      printf(" grid[0..%d] = { ",GridSize-1);
      printf("%f ... %f",grid.front(), grid.back());
      printf("}\n");
    }
  else
    {
      write_file(&grid[0], GridSize, 1, output_file, "w"); 
    }
}

void Polynomial::print_p(const char *output_file)
{
  if (output_file == NULL)
    {
      printf("          p = { ");
      printf("%f ... %f",p.front(), p.back());
      printf("}\n");
    }
  else
    {
      int pSize = p.size();
      write_file(&p[0], pSize, 1, output_file, "w"); 
    }
}

void Polynomial::print_dp(const char *output_file)
{
  if (output_file == NULL)
    {
      printf("         dp = { ");
      printf("%f ... %f",dp.front(), dp.back());
      printf("}\n");
    }
  else
    {
      int dpSize = dp.size();
      write_file(&dp[0], dpSize, 1, output_file, "w"); 
    }
}

void Polynomial::poly3_get_coefficients(double p_i, double p_f, double dp_i, double dp_f)
{
  double t_i = get_FirstGridPoint();
  double t_f = get_LastGridPoint();
  
  if (t_i>= t_f)
    {
      printf("ERROR: t_i>= t_f \n");
      exit(0);
    }
  
  if (N < 3)
    {   
      printf("ERROR: this function can be used only with a third or higher order polynomial \n");
      exit(0);
    }
  else
    {
      double t1;
      double t11;
      double t12;
      double t13;
      double t18;
      double t21;
      double t24;
      double t31;
      double t35;
      double t39;
      double t6;
      double t7;
      double t9;
      
      t1 = t_f*t_f;
      t6 = 1/(t_f-t_i);
      t7 = t_i*t_f;
      t9 = t_i*t_i;
      t11 = 1/(-2.0*t7+t9+t1);
      t12 = t6*t11;
      t13 = t12*p_i;
      t18 = t12*p_f;
      t21 = t11*dp_i;
      t24 = t11*dp_f;
      a[0] = t1*(-3.0*t_i+t_f)*t13+t9*(3.0*t_f-t_i)*t18-t1*t_i*t21-t9*t_f*t24;
      t31 = t_f+2.0*t_i;
      t35 = 2.0*t_f+t_i;
      a[1] = 6.0*t7*t13-6.0*t7*t18+t31*t_f*t21+t35*t_i*t24;
      t39 = (t_f+t_i)*t6;
      a[2] = -3.0*t39*t11*p_i+3.0*t39*t11*p_f-t35*t11*dp_i-t31*t11*dp_f;
      a[3] = 2.0*t13-2.0*t18+t21+t24;
    }
}

void Polynomial::poly1_get_coefficients(double p_i, double p_f)
{
  double t_i = get_FirstGridPoint();
  double t_f = get_LastGridPoint();
  
  if (t_i>= t_f)
    {
      printf("ERROR: t_i>= t_f \n");
      exit(0);
    }
  
  if (N < 1)
    {   
      printf("ERROR: this function can be used only with a first or higher order polynomial \n");
      exit(0);
    }
  else
    {
      double t2;
      t2 = 1/(t_f-t_i);
      a[0] = t_f*t2*p_i-t_i*t2*p_f;
      a[1] = -t2*p_i+t2*p_f;
    }
}

//EOF
