#pragma once

#include <string>
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <sstream>	
#include <sys/time.h>

namespace orunav_generic
{

  void getVectorMeanStdev(const std::vector<double> &v, double &mean, double &stdev) {

    mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();

    std::vector<double> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(),
		   std::bind2nd(std::minus<double>(), mean));
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    stdev = std::sqrt(sq_sum / v.size());
  }

  void getVectorMinMax(const std::vector<double> &v, double &min, double &max) {

    min = *std::min_element(v.begin(), v.end());
    max = *std::max_element(v.begin(), v.end());
  }

  void getVectorQuartiles(const std::vector<double> &vec, double &q1, double &median, double &q3) {
    std::vector<double> v(vec);
    std::sort(v.begin(), v.end());
    q1 = v[v.size()*1/4];
    median = v[v.size()*2/4];
    q3 = v[v.size()*3/4];
  }

void normalizeVector(std::vector<double> &v) {
  double weight_factor = std::accumulate(v.begin(), v.end(), 0.);
  std::vector<double>::iterator it;
  for(it = v.begin(); it != v.end(); it++)
    *it = *it * weight_factor;
}

template<class T> std::string toString (const T& x)
{
     std::ostringstream o;

     if (!(o << x))
	  throw std::runtime_error ("::toString()");

     return o.str ();
}

template <class T> T 
fromString(const std::string &s) {
     T t;
     std::istringstream iss(s);
     iss >> t;
     return t;
}

  std::string getVectorStatisticStr(const std::vector<double> &data) {
    double mean, stdev, min, max, q1, median, q3;
    orunav_generic::getVectorMeanStdev(data, mean, stdev);
    orunav_generic::getVectorMinMax(data, min, max);
    orunav_generic::getVectorQuartiles(data, q1, median, q3);

    std::string ret("[mean]: " + toString(mean) + "\n[stdev]:" + toString(stdev) + "\n[min]: " + toString(min) + "\n[max] :" + toString(max) + "\n[q1]: " + toString(q1) + "\n[median]: " + toString(median) + "\n[q3]: " + toString(q3));
    return ret;
  }

  std::string getVectorStatisticGnuplotStr(const std::vector<double> &data) {
    double mean, stdev, min, max, q1, median, q3;
    orunav_generic::getVectorMeanStdev(data, mean, stdev);
    orunav_generic::getVectorMinMax(data, min, max);
    orunav_generic::getVectorQuartiles(data, q1, median, q3);
    
    std::string ret(toString(mean) + "\t" + toString(stdev) + "\t" + toString(min) + "\t" + toString(max) + "\t" + toString(q1) + "\t" + toString(median) + "\t" + toString(q3));
    return ret;
  }

 std::string getRobotTopicName(int robot_id, const std::string &topic) {
   return "/robot" + toString(robot_id) + topic;
 }

  std::string getRobotTF(int robot_id, const std::string &link) {
    return getRobotTopicName(robot_id, link);
  }

 std::string getRobotBaseLinkTF(int robot_id)
   {
     std::string ret = getRobotTopicName(robot_id, std::string("/base_link"));
     return ret;
   }

 std::string getRobotSteerLinkTF(int robot_id)
   {
     std::string ret = getRobotTopicName(robot_id, std::string("/steer_link"));
     return ret;
   }

 double getDoubleTime()
 {
   struct timeval time;
   gettimeofday(&time,NULL);
   return time.tv_sec + time.tv_usec * 1e-6;
 }



} // namespace
