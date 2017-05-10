#include <orunav_generic/interfaces.h>
#include <orunav_geometry/orunav_boost_geometry.h>
#include <iostream>
#include <ctime>

using namespace std;

std::string boolstr(bool v)
{
    return v ? "true" : "false";
}

//BOOST_GEOMETRY_REGISTER_POINT_2D(Eigen::Vector2d, double, boost::geometry::cs::cartesian, Eigen::Vector2d::x(), Eigen::Vector2d::y())

 //Eigen::Vector2d::x, Eigen::Vector2d::y, Eigen::Vector2d::x, Eigen::Vector2d::y)

int main()
{
  using namespace boost::geometry;

  typedef boost::tuple<double, double> point;
  typedef boost::geometry::model::polygon<point> polygon;
  typedef boost::geometry::model::box<point> box;

  //  typedef boost::geometry::model::polygon<Eigen::Vector2d> eigen_polygon;
  
  orunav_generic::Point2dVec pts;
  pts.push_back(Eigen::Vector2d(10, 10));
  pts.push_back(Eigen::Vector2d(0, 10));
  pts.push_back(Eigen::Vector2d(0, 0));
  pts.push_back(Eigen::Vector2d(10, 0));

  // Convert that to a polygon...
  polygon poly_eigen;
  for (unsigned int i = 0; i < pts.sizePoint2d(); i++)
    append(poly_eigen, make<point>(pts.getPoint2d(i)[0], pts.getPoint2d(i)[1]));

  correct(poly_eigen);
  std::cout << "area: " << area(poly_eigen) << std::endl;


  polygon poly;
  boost::geometry::read_wkt("polygon((2.0 1.3, 2.4 1.7, 2.8 1.8, 3.4 1.2, 3.7 1.6,3.4 2.0, 4.1 3.0"
			    ", 5.3 2.6, 5.4 1.2, 4.9 0.8, 2.9 0.7,2.0 1.3))", poly);

  polygon poly2;
  //  boost::geometry::read_wkt("polygon((12.0 1.3, 12.4 1.7, 12.8 1.8, 13.4 1.2, 13.7 1.6,13.4 2.0, 14.1 3.0"
  //			    ", 15.3 2.6, 15.4 1.2, 14.9 0.8, 12.9 0.7,12.0 1.3))", poly2);
  boost::geometry::read_wkt("polygon((3.0 1.3, 3.4 1.7, 3.8 1.8, 4.4 1.2, 4.7 1.6,4.4 2.0, 5.1 3.0"
			    ", 6.3 2.6, 6.4 1.2, 5.9 0.8, 3.9 0.7,3.0 1.3))", poly2);

  correct(poly);
  correct(poly2);

  // As with lines, bounding box of polygons can be calculated
   box b;
   envelope(poly, b);
   std::cout << dsv(b) << std::endl;
  
  // The area of the polygon can be calulated
  std::cout << "area: " << area(poly) << std::endl;
  std::cout << "area2: " << area(poly2) << std::endl;

  // And the centroid, which is the center of gravity
  point cent;
  centroid(poly, cent);
  std::cout << "centroid: " << dsv(cent) << std::endl; 
  centroid(poly2, cent);
  std::cout << "centroid2: " << dsv(cent) << std::endl; 
  
  // You can test whether points are within a polygon
  std::cout << "point in polygon:"
	    << " p1: "  << boolstr(within(make<point>(3.0, 2.0), poly))
	    << " p2: "  << boolstr(within(make<point>(3.7, 2.0), poly))
	    << " p3: "  << boolstr(within(make<point>(4.4, 2.0), poly))
	    << std::endl;

  std::cout << "point in polygon2:"
	    << " p1: "  << boolstr(within(make<point>(3.0, 2.0), poly2))
	    << " p2: "  << boolstr(within(make<point>(3.7, 2.0), poly2))
	    << " p3: "  << boolstr(within(make<point>(4.4, 2.0), poly2))
	    << std::endl;

  // Do we have any overlap?
  std::cout << "disjoint : " << boolstr(disjoint(poly, poly2)) << std::endl;

  std::vector<polygon> polys;
  boost::geometry::union_(poly2, poly, polys);
  std::cout << "union size: " << polys.size() << std::endl;


  polygon hull, hull_union;
  boost::geometry::convex_hull(poly, hull);

  
  using boost::geometry::dsv;
  std::cout
    << "polygon: " << dsv(poly) << std::endl
    << "hull: " << dsv(hull) << std::endl
    ;

  if (polys.size() == 1) {
    boost::geometry::convex_hull(polys[0], hull_union);
    
    std::cout
      << "polygon_union: " << dsv(polys[0]) << std::endl
      << "hull_union: " << dsv(hull_union) << std::endl
      ;
  }
  
  
  return 0;
};
