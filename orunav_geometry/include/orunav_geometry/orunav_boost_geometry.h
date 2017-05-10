#pragma once

#include <orunav_geometry/geometry.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/geometry/multi/multi.hpp>
#include <boost/geometry/algorithms/union.hpp>

BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(boost::geometry::cs::cartesian)
//BOOST_GEOMETRY_REGISTER_POINT_2D(Eigen::Vector2d, double, boost::geometry::cs::cartesian, Eigen::Vector2d::x(), Eigen::Vector2d::y())

typedef boost::tuple<double, double> point;
typedef boost::geometry::model::polygon<point> polygon;

namespace orunav_geometry {

  //! Return a boost polygon given a set of 2d points.
  inline polygon getBoostPolygon(const orunav_generic::Point2dContainerInterface &pts) {
    polygon poly;
    for (unsigned int i = 0; i < pts.sizePoint2d(); i++)
      boost::geometry::append(poly, boost::geometry::make<point>(pts.getPoint2d(i)[0], pts.getPoint2d(i)[1]));
    boost::geometry::correct(poly);
    return poly;
  }

  //! Given a Boost polygon return a set of points.
  inline orunav_generic::Point2dVec getPoint2dVec(const polygon &poly) {
    orunav_generic::Point2dVec pts;
    std::vector<point> const& points = poly.outer();
    for (std::vector<point>::size_type i = 0; i < points.size(); ++i)
      {
	pts.push_back(Eigen::Vector2d(points[i].get<0>(), points[i].get<1>()));
      }
    return pts;
  }

} // namespace



