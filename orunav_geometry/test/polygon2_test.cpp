#include <orunav_geometry/geometry.h>
#include <orunav_geometry/serialization.h>
#include <orunav_geometry/polygon.h>

#include <fstream>
#include <iostream>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp> // for std::pair

int main()
{
  {
    orunav_generic::Point2dVec pts, pts2, pts3;
    pts.push_back(Eigen::Vector2d(10, 10));
    pts.push_back(Eigen::Vector2d(0, 10));
    pts.push_back(Eigen::Vector2d(0, 0));
    pts.push_back(Eigen::Vector2d(10, 0));
    
    pts2.push_back(Eigen::Vector2d(15, 10));
    pts2.push_back(Eigen::Vector2d(5, 10));
    pts2.push_back(Eigen::Vector2d(5, 9));
    pts2.push_back(Eigen::Vector2d(5, 8));
    pts2.push_back(Eigen::Vector2d(5, 7));
    pts2.push_back(Eigen::Vector2d(5, 6));
    pts2.push_back(Eigen::Vector2d(5, 5));
    pts2.push_back(Eigen::Vector2d(5, 4));
    pts2.push_back(Eigen::Vector2d(5, 3));
    pts2.push_back(Eigen::Vector2d(5, 2));
    pts2.push_back(Eigen::Vector2d(5, 1));
    pts2.push_back(Eigen::Vector2d(5, 0));
    pts2.push_back(Eigen::Vector2d(15, 0));


    pts3.push_back(Eigen::Vector2d(10, 10));
    pts3.push_back(Eigen::Vector2d(0, 10));
    pts3.push_back(Eigen::Vector2d(5,5));
    pts3.push_back(Eigen::Vector2d(0, 0));
    pts3.push_back(Eigen::Vector2d(5,5));
    pts3.push_back(Eigen::Vector2d(10, 0));

    
    orunav_geometry::Polygon p(pts);
    orunav_geometry::Polygon p2(pts2);
		   
    
    std::cout << "area - p : " << p.getArea() << " nb_points : " << p.sizePoint2d() << std::endl;
    std::cout << "area -p2 : " << p2.getArea() << " nb_points : " << p2.sizePoint2d() << std::endl;
    
    p.addPolygon(p2);
    
    std::cout << "area - p + p2 : " << p.getArea() << " nb_points : " << p.sizePoint2d() << std::endl;
    
    for (unsigned int i = 0; i < p.sizePoint2d(); i++)
      std::cout << "pts : " << p.getPoint2d(i) << std::endl;
    
    p.convexHull();
    std::cout << "area - convex hull (p + p2) : " << p.getArea() << " nb_points : " << p.sizePoint2d() << std::endl;
    
    orunav_geometry::Polygon p3(pts3);
    std::cout << "area - p3 : " << p3.getArea() << " nb_points : " << p3.sizePoint2d() << std::endl;
    for (unsigned int i = 0; i < p3.points.size(); i++)
      std::cout << p3.points[i] << std::endl;
    p3.convexHull();
    std::cout << "area (convex hull) - p3 : " << p3.getArea() << " nb_points : " << p3.sizePoint2d() << std::endl;
    for (unsigned int i = 0; i < p3.points.size(); i++)
      std::cout << p3.points[i] << std::endl;


    orunav_geometry::Polygon p4;
    p4.addPolygon(p);
    p4.addPolygon(p2);
    std::cout << "area - p4 (should be p + p2) : " << p4.getArea() << " nb_points : " << p4.sizePoint2d() << std::endl;


    std::cout << "---------------------------------------------------" << std::endl;

    for (unsigned int i = 0; i < p4.points.size(); i++)
      std::cout << p4.points[i] << "," << std::endl;
    
    Eigen::MatrixXd A; Eigen::VectorXd b;
    p4.getMatrixForm(A, b);

    std::cout << "A: " << A << std::endl;
    std::cout << "b: " << b << std::endl;

    std::cout << "---------------------------------------------------" << std::endl;
    std::string file_name("polygon.dat");
    // Save
    {
        std::cout << "saving..." << std::endl;
        std::ofstream ofs(file_name.c_str());
        boost::archive::text_oarchive ar(ofs);
        ar & p4;
    }
    // Load
    {    orunav_geometry::Polygon p4_restored;
        std::cout << "loading... " << std::endl;
        
        std::ifstream ifs(file_name.c_str());
        boost::archive::text_iarchive ar(ifs);
        ar & p4_restored;
        
        std::cout << "restored polygon points:" << std::endl;
        for (unsigned int i = 0; i < p4_restored.points.size(); i++)
            std::cout << p4_restored.points[i] << "," << std::endl;
    }
  }
  
  return 0;
};
