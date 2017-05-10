#include <orunav_geometry/b_spline.h>
#include <iostream>

BSpline::BSpline(bool angle) : _angle(angle)
{

}

BSpline::~BSpline()
{

}

void
BSpline::clear()
{
     _xt.clear();
}

void 
BSpline::addPoint(double x, double t)
{
     _xt[t] = x;
}

bool
BSpline::selectKnots(std::vector<double> &x_vec, std::vector<double> &t_vec, double t) const
{
  if (_xt.empty())
    return false;
  
  std::vector<std::pair<double,double> > p(4); // [0] is the oldest ... [3] the latest.
  // In the danaher documentation p[0] -> p_{i-1} etc.
  
  std::map<double,double>::const_iterator it = _xt.upper_bound(t); // Should be [2].
  
  double max_T = getValueT(this->size() - 2);

  if (t == getValueT(this->size() - 2))
    {
      // The it is [3] in this case (need to simply move the iter to [2].
      it--;
    }

  if (it == _xt.end())
    return false;
  
  p[2] = *it;
  it++;
  
  if (it == _xt.end())
    {
      // This might happen if the requrested t is exactly the same as knot p[1].
      it--; it--;
      if (t != (*it).first)
	return false;
      //assert(false); // Cannot send back x directly here need to do something...
    }
  
  p[3] = *it;
  
  it--; it--;
  
  if (it == _xt.begin())
    return false;
  
  p[1] = *it;
  it--;
  p[0] = *it;
  
  x_vec.resize(4);
  t_vec.resize(4);
  
  for (int i = 0; i < 4; i++)
    {
      t_vec[i] = p[i].first;
      x_vec[i] = p[i].second;
    }
  return true;
}

bool 
BSpline::getValue(double &x, double t) const
{
     x = 0.;
     std::vector<double> tmp_x(4);
     std::vector<double> tmp_t(4);
     
     if (!selectKnots(tmp_x, tmp_t, t))
       return false;
     if (_angle)
	  return eval_spline_angle(x, t, tmp_x, tmp_t);
     else
	  return eval_spline(x, t, tmp_x, tmp_t);
}

bool
BSpline::getValueFirstDerApprox(double &x, double t) const
{
     assert(!_angle);
     const double epsilon = 0.001;
     double x1, x2;
     if (!this->getValue(x1, t))
	  return false;
     if (this->getValue(x2, t+epsilon))
     {
	  x = (x2-x1) / epsilon;
     }
     else
     {
	  assert(this->getValue(x2, t-epsilon));
	  x = (x1-x2) / epsilon;
     }  
     return true;
}

bool 
BSpline::getValueFirstDer(double &x, double t) const
{
     x = 0.;
     std::vector<double> tmp_x(4);
     std::vector<double> tmp_t(4);
     
     if (!selectKnots(tmp_x, tmp_t, t))
       return false;
     if (_angle)
	  return eval_spline_angle_first_der(x, t, tmp_x, tmp_t);
     else
	  return eval_spline_first_der(x, t, tmp_x, tmp_t);
}

bool
BSpline::getValueSecondDer(double &x, double t) const
{
     x = 0.;
     std::vector<double> tmp_x(4);
     std::vector<double> tmp_t(4);
     
     if (!selectKnots(tmp_x, tmp_t, t))
       return false;
     if (_angle)
       {
	 assert(false);
         return false;
       }
     return eval_spline_second_der(x, t, tmp_x, tmp_t);
}

bool 
BSpline::getAllValues(double &val, double &valDot, double &valDotDot, double t) const
{
  std::vector<double> tmp_x(4);
  std::vector<double> tmp_t(4);
  
  if (!selectKnots(tmp_x, tmp_t, t))
    return false;
  if (_angle)
    {
      assert(false);
      return false;
    }
  if (!eval_spline(val, t, tmp_x, tmp_t))
    return false;
  if (!eval_spline_first_der(valDot, t, tmp_x, tmp_t))
    return false;
  if (!eval_spline_second_der(valDotDot, t, tmp_x, tmp_t))
    return false;
  return true;
}

void 
BSpline::removeFirstPoint()
{
     assert(_xt.empty());
     _xt.erase(_xt.begin());
}

double 
BSpline::getValue(unsigned int idx) const
{
     assert(idx < _xt.size());

     std::map<double, double>::const_iterator it = _xt.begin();

     for (uint i = 0; i < idx; i++)
	  it++;

     return (*it).second;
}

double 
BSpline::getValueT(unsigned int idx) const
{
     assert(idx < _xt.size());

     std::map<double, double>::const_iterator it = _xt.begin();

     for (uint i = 0; i < idx; i++)
	  it++;

     return (*it).first;
}
