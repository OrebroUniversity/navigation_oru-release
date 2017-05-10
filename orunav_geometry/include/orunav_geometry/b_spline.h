#pragma once

#include <vector>
#include <map>
#include <cassert>
#include <math.h>
#include <iostream>
#include <angles/angles.h>


inline bool eval_spline_gnuplot(double &x_val, const double &t_val, const std::vector<double> &x, const std::vector<double> &t)
{
     if (t_val < t[1] || t_val > t[2])
	  return false;

//     double u = t_val / (t[2] - t[1]);
     double u = t_val - t[1];
     std::cout << "t_val : " << t_val << std::endl;

     std::cout << "u : " << u << std::endl;
     
     // The cubic Monomial basis function.
     std::vector<double> m(4);
     m[0] = 1;
     m[1] = u;
     m[2] = u*u;
     m[3] = u*u*u;
     
     // The cubic Hermite basic functions.
     std::vector<double> h(4);
     h[0] = u*u*(2*u-3)+1;
     h[1] = -u*u*(2*u-3);
     h[2] = u*(u-1)*(u-1);
     h[3] = u*u*(u-1);

     // The cubic uniform basic functions.
     std::vector<double> bsp(4);
     bsp[0] = (1-3*u+3*u*u - u*u*u) / 6.;
     bsp[1] = (4-6*u*u+ 3*u*u*u) / 6.;
     bsp[2] = (1+3*u + 3*u*u - 3*u*u*u) / 6.;
     bsp[3] = u*u*u / 6.;

     x_val = bsp[0]*x[0] + bsp[1]*x[1] + bsp[2]*x[2] + bsp[3]*x[3];
     return true;
     }

inline bool eval_spline(double &x_val, const double &t_val, const std::vector<double> &x, const std::vector<double> &t)
{
     assert(x.size() == 4 && t.size() == 4);
     assert(t_val >= t[1] && t_val <= t[2]);

     std::vector<double> m(4); // Contain the matrix vector evaluation (eq. 1.3) from Uniform B-spline curves, Dahaner motion document.
     m[0] = -1 * x[0] + 3 * x[1] - 3 * x[2] + x[3];
     m[1] =  3 * x[0] - 6 * x[1] + 3 * x[2];
     m[2] = -3 * x[0]            + 3 * x[2];
     m[3] =      x[0] + 4 * x[1] +     x[2];

     // Normalize the value with respect to t, meaning that the inteval that u should lie between is [0-1[.
     double u = (t_val - t[1]) / (t[2] -t[1]);

     x_val = 1/6.*(pow(u,3)*m[0] + pow(u,2)*m[1] + u*m[2] + m[3]);
     return true;
}

inline bool eval_spline_angle(double &x_val, const double &t_val, const std::vector<double> &x_angles, const std::vector<double> &t)
{
     // Make sure that the x poses do not contain any trun-arounds [2_MPI <-> 0] (assuming they contain angles).
     assert(x_angles.size() == 4 && t.size() == 4);
     std::vector<double> x = x_angles;

     for (int i = 0; i < 3; i++)
     {
	  while (x[i+1] - x[i] > M_PI)
	  {
	       x[i+1] -= 2*M_PI;
	  }
	  while (x[i+1] - x[i] < -M_PI)
	  {
	       x[i+1] += 2*M_PI;
	  }
     }
     bool ret = eval_spline(x_val, t_val, x, t);
     x_val = angles::normalize_angle(x_val);
     return ret;
}

inline bool eval_spline_first_der(double &x_val, const double &t_val, const std::vector<double> &x, const std::vector<double> &t)
{
     assert(x.size() == 4 && t.size() == 4);
     assert(t_val >= t[1] && t_val <= t[2]);

     std::vector<double> m(3); // Contain the matrix vector evaluation (eq. A. 12) from Uniform B-spline curves, Dahaner motion document.
     m[0] = -1 * x[0] + 3 * x[1] - 3 * x[2] + x[3];
     m[1] =  3 * x[0] - 6 * x[1] + 3 * x[2];
     m[2] = -3 * x[0]            + 3 * x[2];
     
     // Normalize the value with respect to t, meaning that the inteval that u should lie between is [0-1[.
     double u = (t_val - t[1]) / (t[2] -t[1]);

     x_val = 1/6.*(pow(u,2)*3*m[0] + u*2*m[1] + m[2]);
     return true;
}

inline bool eval_spline_angle_first_der(double &x_val, const double &t_val, const std::vector<double> &x_angles, const std::vector<double> &t)
{
     assert(false); // This has not been checked properly.
     // Make sure that the x poses do not contain any trun-arounds [2_MPI <-> 0] (assuming they contain angles).
     assert(x_angles.size() == 4 && t.size() == 4);
     std::vector<double> x = x_angles;

     for (int i = 0; i < 3; i++)
     {
	  while (x[i+1] - x[i] > M_PI)
	  {
	       x[i+1] -= 2*M_PI;
	  }
	  while (x[i+1] - x[i] < -M_PI)
	  {
	       x[i+1] += 2*M_PI;
	  }
     }
     bool ret = eval_spline_first_der(x_val, t_val, x, t);
     x_val = angles::normalize_angle(x_val);
     return ret;
}

inline bool eval_spline_second_der(double &x_val, const double &t_val, const std::vector<double> &x, const std::vector<double> &t)
{
     assert(x.size() == 4 && t.size() == 4);
     assert(t_val >= t[1] && t_val <= t[2]);

     std::vector<double> m(2);
     m[0] = -1 * x[0] + 3 * x[1] - 3 * x[2] + x[3];
     m[1] =  3 * x[0] - 6 * x[1] + 3 * x[2];
          
     // Normalize the value with respect to t, meaning that the inteval that u should lie between is [0-1[.
     double u = (t_val - t[1]) / (t[2] -t[1]);

     x_val = 1/6.*(u*6*m[0] + 2*m[1]);
     return true;
}




//! Class to handle spline in 1 dimensions.
class BSpline
{
public:
     //! Constructor, angle if angles are to be interpolated (will handle flips over 0 <-> 2*M_PI).
     BSpline(bool angle = false);

     //! Destructor
     ~BSpline();

     //! Set if angles should be used in the interpolation.
     void setAngle(bool angle) { _angle = angle; }

     //! Add a point into the spline
     void addPoint(double x, double t);

     //! Return the value of the spline in val, returns false if the t value was out of bounds.
     bool getValue(double &val, double t) const;

     //! Return the first derivative of the spline in val, false if t value was out of bounds.
     bool getValueFirstDer(double &val, double t) const;

     //! Return the second derivative of the spline in val, false if t value was out of bounds.
     bool getValueSecondDer(double &val, double t) const;

     //! Return all values / derrivatives at once.
     bool getAllValues(double &val, double &valDot, double &valDotDot, double t) const;
     
     //! Return an approximation (not an analytical solution).
     bool getValueFirstDerApprox(double &x, double t) const;

     //! Remove first point.
     void removeFirstPoint();

     //! Clear all points.
     void clear();

     //! Return the number of points used.
     unsigned int size() const { return _xt.size(); }

     //! Return value at idx.
     double getValue(unsigned int idx) const;

     //! Return t value at idx.
     double getValueT(unsigned int idx) const;
private:
     bool selectKnots(std::vector<double> &x_vec, std::vector<double> &t_vec, double t) const;
     std::map<double, double> _xt;
     bool _angle;
};
