#include <orunav_geometry/polygon.h>
#include <orunav_geometry/orunav_boost_geometry.h>

using namespace orunav_geometry;


Polygon::Polygon() {
  
}

Polygon::Polygon(const Point2dContainerInterface& pts) : points(pts) {
  
}

bool 
Polygon::collisionPoint2d(const Eigen::Vector2d &pos) const {

  polygon poly = getBoostPolygon(this->points);
  
  return boost::geometry::within(boost::geometry::make<point>(pos(0), pos(1)), poly);
}

bool
Polygon::addPolygon(const Polygon &p) {
  // If the "this" polygon is empty, simpy copy the provided polygon directly
  if (this->points.size() == 0) {
    this->points = p.points;
    return true;
  }

  if (p.points.size() == 0) {
    return false;
  }

  // Computes the union. Note that the polygons have to overlap in order to add them.
  polygon p1 = getBoostPolygon(this->points);
  polygon p2 = getBoostPolygon(p);
  
  if (boost::geometry::disjoint(p1,p2))
    return false;

  // The union will return all intersecting polygons. Here we assume that the one we're intressted in is the larges one. - TODO check this.
  std::vector<polygon> polys;
  
  try { // Since we know that the sets is not disjoint this union call should really work but sometimes doesn't... (a 'Boost.Geometry Overlay invalid input exception' is thrown)
    boost::geometry::union_(p1, p2, polys);
  }
  catch (...) {
    return false;
  }

  double area = -1.;
  for (std::vector<polygon>::size_type i = 0; i < polys.size(); ++i)
    {
      boost::geometry::correct(polys[i]);
      double tmp = boost::geometry::area(polys[i]); 
      if (area < tmp) {
	area = tmp;
	this->points = getPoint2dVec(polys[i]);
      }
    }
  return true;
}

void
Polygon::convexHull()
{
   polygon poly = getBoostPolygon(this->points);
   polygon hull;
   boost::geometry::convex_hull(poly, hull);
   this->points = getPoint2dVec(hull);
}

double
Polygon::getArea() const
{
   polygon poly = getBoostPolygon(this->points);
   return boost::geometry::area(poly);
}

void
Polygon::getMatrixForm(Eigen::MatrixXd &A, Eigen::VectorXd &b) const
{
  calculateMatrixForm(*this, A, b);
}

void
Polygon::getMatrixFormAsVectors(std::vector<double> &A0, std::vector<double> &A1, std::vector<double> &b) const {
    Eigen::MatrixXd A_;
    Eigen::VectorXd b_; 		 
    this->getMatrixForm(A_, b_);
    
    double *dt;
    dt = A_.col(0).data();
    A0 = std::vector<double>(dt,dt+A_.rows());
    dt = A_.col(1).data();
    A1 = std::vector<double>(dt,dt+A_.rows());
    dt = b_.data();
    b = std::vector<double>(dt,dt+b_.rows());
 }

bool
Polygon::intersection(const Polygon &p) const {
  polygon poly = getBoostPolygon(this->points);
  polygon poly2 = getBoostPolygon(p.points);

  return boost::geometry::within(poly, poly2);
}

void Polygon::clear() {
  points.clear();
}
