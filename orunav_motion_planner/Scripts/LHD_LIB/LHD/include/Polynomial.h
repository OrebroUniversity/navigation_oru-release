// Time-stamp: <2012-06-04 09:17:52 (drdv)>

#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

/******************************************
 **************** INCLUDES ****************
 ******************************************/

#include <vector>
#include <cstdlib>
#include <stdio.h>

void write_file(double *A, int row, int col, const char *output_file, const char * mode);
void Matrix_print(int m, int n, double * A, const char * description);

/******************************************/


/** \brief Defines a polynomial */
class Polynomial
{
 public:

  /** \brief Order of the polynomial

      \note N = -1 indicates that N is not initialized
  */
  int N;

  /** \brief Coefficients of the polynomial

      \verbatim
      p(t) = a[0]*t^0 + a[1]*t^1 + ... a[N]*t^N
      \endverbatim
  */
  std::vector<double> a;

  /** \brief Grid at which to evaluate the polynomail */
  std::vector<double> grid;

  /** \brief Values of the polynomail at the grid points */
  std::vector<double> p;

  /** \brief Values of the derivative of the polynomail at the grid points */
  std::vector<double> dp;

  /** \brief Default constructor */
  Polynomial();

  /** \brief Default destructor */
  ~Polynomial();

  /** \brief Constructor

      \param[in] _N Order of polynomial
  */
  Polynomial(int _N);

  /** \brief Constructor

      \param[in] _N Order of polynomial
      \param[in] coefficients Coefficients of the polynomial
  */
  Polynomial(int _N, double *coefficients);

  /** \brief Set the size of the grid

      \param[in] GridSize Grid size (number of elements in the grid)
      \return void
  */
  void set_GridSize(int GridSize);

  /** \brief Set one grid point

      \param[in] k Index of grid point
      \param[in] x Value to assign
      \return void
  */
  void set_GridPoint(int k, double x);

  /** \brief Append one grid point

      \param[in] x Value to assign
      \return void
  */
  void append_GridPoint(double x);

  /** \brief Get the first grid point

      \return The first grid point
  */
  double get_FirstGridPoint();

  /** \brief Get the last grid point

      \return The last grid point
  */
  double get_LastGridPoint();

  /** \brief Calls eval_p() and eval_dp()

      \return void
  */
  void eval();

  /** \brief Evaluates the polynomial at the grid

      \verbatim
      p(t) = a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t 
           = a0 + (a1 + a2*t + a3*t*t + a4*t*t*t)*t 
           = a0 + (a1 + (a2 + a3*t + a4*t*t)*t)*t 
           = a0 + (a1 + (a2 + (a3 + a4*t)*t)*t)*t 
      \endverbatim

      \return void
  */
  void eval_p();

  /** \brief Evaluates the derivative of the polynomial at the grid

      \verbatim
      p(t) = a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t 
           = a0 + (a1 + a2*t + a3*t*t + a4*t*t*t)*t 
           = a0 + (a1 + (a2 + a3*t + a4*t*t)*t)*t 
           = a0 + (a1 + (a2 + (a3 + a4*t)*t)*t)*t 

     dp(t) = a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t 
           = a1 + (2*a2 + 3*a3*t + 4*a4*t*t)*t
           = a1 + (2*a2 + (3*a3 + 4*a4*t)*t)*t
      \endverbatim

      \return void
  */
  void eval_dp();
    
  /** \brief Generates M equaly spaced points between (and including) t0 and t1

      \param[in] t0 Initial time
      \param[in] t1 Final time
      \param[in] GridSize Grid size (number of elements in the grid)
      \return void

      \note t1 >= t0 and M>0 are assumed.

      \verbatim
      For example if t = 1, t1 = 5 and GridSize = 5 grid = {1,2,3,4,5}
      \endverbatim
  */
  void set_EquidistantGrid(double t0, double t1, int GridSize);
    
  /** \brief Sets the coefficients of the polynomial
      
      \param[in] coefficients Coefficients of the polynomial
      \return void      
  */
  void set_a(double *coefficients);

  /** \brief Sets the order of the polynomial

      \param[in] _N Order of the polynomial
      \return void      
  */
  void set_N(int _N);

  /** \brief Prints the coefficients of the polynomial

      \param[in] output_file Output file
      \return void

      \verbatim
      example: output_file = "../data/a.txt"
      \endverbatim
  */
  void print_a(const char *output_file = NULL);

  /** \brief Prints the grid points
    
      \param[in] output_file Output file
      \return void
  */
  void print_grid(const char *output_file = NULL);

  /** \brief Prints the polynomial values at the grid points

      \param[in] output_file Output file
      \return void      
  */
  void print_p(const char *output_file = NULL);

  /** \brief Prints the values of the derivative of the polynomial at the grid points

      \param[in] output_file Output file
      \return void      
  */
  void print_dp(const char *output_file = NULL);

  // ---------------------------------------------------
  // functions for predefined polynomial order ...
  // ---------------------------------------------------

  /** \brief Find the coefficients of a third order polynomial

      \param[in] p_i Initial position
      \param[in] p_f Final position
      \param[in] dp_i Initial velocity
      \param[in] dp_f Final velocity
      \return void      

      \verbatim
      A = [ 1, t_i,   t_i^2,   t_i^3;
            1, t_f,   t_f^2,   t_f^3;
            0,   1, 2*t_i  , 3*t_i^2;
            0,   1, 2*t_f  , 3*t_f^2];

      b = [p_i;p_f;dp_i;dp_f]

      a = A\b
      \endverbatim

      \note generated using codegen (2012-05-23, 15:01:32) - see poly3_coefficients.mpl     
  */
  void poly3_get_coefficients(double p_i, double p_f, double dp_i, double dp_f);

  /** \brief Find the coefficients of a first order polynomial

      \param[in] p_i Initial position
      \param[in] p_f Final position
      \return void      

      \verbatim
      A = [ 1, t_i;
            1, t_f];

      b = [p_i;p_f]

      a = A\b
      \endverbatim

      \note generated using codegen (2012-05-23, 21:29:26) - see poly1_coefficients.mpl
  */
  void poly1_get_coefficients(double p_i, double p_f);
  
};

#endif /*POLYNOMIAL_H*/
