#include <orunav_constraint_extract/serialization.h>

#include <fstream>
#include <iostream>
#include <ctime>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp> // for std::pair

using namespace std;

int main()
{

    std::cout << "Try some serializations..." << std::endl;
    {
        constraint_extract::PolygonConstraint pc(Eigen::Vector2d(1.,1.), Eigen::Vector2d(-2., -3.), Eigen::Vector2d(-0.1, 0.1));
        std::string file_name = "polygonconstraint.dat";

        // Save
        {
            std::ofstream ofs(file_name.c_str());
            boost::archive::text_oarchive ar(ofs);
            ar & pc;
        }
        
        // Load
        constraint_extract::PolygonConstraint pc_restored;
        {
            std::ifstream ifs(file_name.c_str());
            boost::archive::text_iarchive ar(ifs);
            ar & pc_restored;
        }
        std::cout << "pc.getInnerConstraint().getArea() : " << pc.getInnerConstraint().getArea() << std::endl;
        std::cout << "pc_restored.getInnerConstraint().getArea() : " << pc_restored.getInnerConstraint().getArea() << std::endl;
        std::cout << "pc.getThBounds() : " << pc.getThBounds() << std::endl;
        std::cout << "pc_restored.getThBounds() : " << pc_restored.getThBounds() << std::endl;
    }

    // PolygonConstraintsVec
    {
        constraint_extract::PolygonConstraintsVec pcs;
        for (int i = 0; i < 10; i++) {
            constraint_extract::PolygonConstraint pc(Eigen::Vector2d(1.,1.), Eigen::Vector2d(-2., -3.), Eigen::Vector2d(-0.1, 0.1));
            pcs.push_back(pc);
        }
        std::string file_name = "polygonconstraintsvec.dat";

        // Save
        {
            std::ofstream ofs(file_name.c_str());
            boost::archive::text_oarchive ar(ofs);
            ar & pcs;
        }
        
        // Load
        constraint_extract::PolygonConstraintsVec pcs_restored;
        {
            std::ifstream ifs(file_name.c_str());
            boost::archive::text_iarchive ar(ifs);
            ar & pcs_restored;
        }

        std::cout << "pcs[0].getThBounds() : " << pcs[0].getThBounds() << std::endl;
        std::cout << "pcs_restored[0].getThBounds() : " << pcs_restored[0].getThBounds() << std::endl;

    }


    // Lookup params
    {
        constraint_extract::PolygonConstraintsLookup::Params p;
        p.x_left[0] = -1023.;
        std::string file_name = "polygonconstraints_params.dat";

        // Save
        {
            std::ofstream ofs(file_name.c_str());
            boost::archive::text_oarchive ar(ofs);
            ar & p;
        }
        // Load
        constraint_extract::PolygonConstraintsLookup::Params p_restored;
        {
            std::ifstream ifs(file_name.c_str());
            boost::archive::text_iarchive ar(ifs);
            ar & p_restored;
        }
        std::cout << "p : " << p << std::endl;
        std::cout << "p_restored : " << p_restored << std::endl;
    }

    // Occupancy Grid -> could be saved as an image instead?
    {
        nav_msgs::OccupancyGrid o;
        o.info.width = 10;
        std::string file_name = "occupancy_grid.dat";
        
        // Save
        {
            std::ofstream ofs(file_name.c_str());
            boost::archive::text_oarchive ar(ofs);
            ar & o;
        }
        // Load
        nav_msgs::OccupancyGrid o_restored;
        {
            std::ifstream ifs(file_name.c_str());
            boost::archive::text_iarchive ar(ifs);
            ar & o_restored;
        }
        std::cout << "o.info.width : " << o.info.width << std::endl;
        std::cout << "o_restored.info.width : " << o_restored.info.width << std::endl;
        
    }

    // Vector of OccupancyGrids
    std::cout << "------ Vector of OccupancyGrids ------" << std::endl;
    {
        std::vector<nav_msgs::OccupancyGrid> ovec;
        nav_msgs::OccupancyGrid o;
        for (int i = 0; i < 10; i++) {
            o.info.width = i+1;
            o.info.height = i+1;
            o.data.resize(o.info.width*o.info.height);
            std::fill(o.data.begin(), o.data.end(), 1);
            ovec.push_back(o);
        }
                
        std::string file_name = "occupancy_grids.dat";
        // Save
        {
            std::ofstream ofs(file_name.c_str());
            boost::archive::text_oarchive ar(ofs);
            ar &ovec;
        }
        std::vector<nav_msgs::OccupancyGrid> ovec_restored;
        // Load
        nav_msgs::OccupancyGrid o_restored;
        {
            std::ifstream ifs(file_name.c_str());
            boost::archive::text_iarchive ar(ifs);
            ar & ovec_restored;
        }
        std::cout << "ovec[5].info.width : " << ovec[5].info.width << std::endl;
        std::cout << "o_restored[5].info.width : " << ovec_restored[5].info.width << std::endl;
        
    }
}
