#include <orunav_generic/serialization.h>
#include <orunav_generic/io.h>
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
        orunav_generic::Control c1(0.1, 0.2);
        orunav_generic::Control c2(0.3, 0.4);
        orunav_generic::Pose2d p(1., 2., 3.);
        std::string file_name = "output.dat";
        // Save
        {
            std::ofstream ofs(file_name.c_str());
            boost::archive::text_oarchive ar(ofs);
            ar & c1 & c2 & p;
        }
        
        // Load
        orunav_generic::Control c1_restored, c2_restored;
        orunav_generic::Pose2d p_restored;
        {
            std::ifstream ifs(file_name.c_str());
            boost::archive::text_iarchive ar(ifs);
            ar & c1_restored & c2_restored & p_restored;
        }
        
        assert(c1.v == c1_restored.v);
        assert(c2.v == c2_restored.v);
        
        std::cout << "c1.v : " << c1.v << std::endl;
        std::cout << "c1_restored.v : " << c1_restored.v << std::endl;
        std::cout << "p_restored[0] : " << p_restored[0] << std::endl;
    }
    {
        std::string txt_file_name = "path.txt";
        std::cout << "loading old path file : " << txt_file_name << std::endl;
        orunav_generic::Path path = orunav_generic::loadPathTextFile(txt_file_name.c_str());
        std::cout << "loaded nb states : " << path.sizePath() << std::endl;
        if (path.sizePath() == 0) {
            std::cout << "no points - exiting" << std::endl;
            exit(-1);
        }

        // Save as posevec
        std::string file_name = "posevec.dat";
        orunav_generic::Pose2dVec pvec(path);
        {
            std::cout << " saving : " << file_name << std::endl;
            std::ofstream ofs(file_name.c_str());
            boost::archive::text_oarchive ar(ofs);
            ar & pvec;
        }
        // Load
        orunav_generic::Pose2dVec pvec_restored;
        {
            std::cout << " loading : " << file_name << std::endl;
            std::ifstream ifs(file_name.c_str());
            boost::archive::text_iarchive ar(ifs);
            ar & pvec_restored;
        }
        std::cout << "pvec.getPose2d(10) = " << pvec.getPose2d(10) << std::endl;
        std::cout << "pvec_restored.getPose2d(10) = " << pvec_restored.getPose2d(10) << std::endl;

        file_name = "path.dat";
        // Save
        {
            std::cout << " saving : " << file_name << std::endl;
            std::ofstream ofs(file_name.c_str());
            boost::archive::text_oarchive ar(ofs);
            ar & path;
        }
        
        // Load
        orunav_generic::Path path_restored;
        {
            std::cout << " loading : " << file_name << std::endl;
            std::ifstream ifs(file_name.c_str());
            boost::archive::text_iarchive ar(ifs);
            ar & path_restored;
        }
        std::cout << "loaded (restored) nb states : " << path_restored.sizePath() << std::endl;

        std::cout << "path.getPose2d(10) = " << path.getPose2d(10) << std::endl;
        std::cout << "path.getSteeringAngle(10) = " << path.getSteeringAngle(10) << std::endl;

        std::cout << "path_restored.getPose2d(10) = " << path_restored.getPose2d(10) << std::endl;
        std::cout << "path_restored.getSteeringAngle(10) = " << path_restored.getSteeringAngle(10) << std::endl;

    }

    {
        orunav_generic::CoordinatedTimes ct(10);
        ct[0] = 10.;
        std::string file_name = "ct.txt";
        // Save
        {
            std::cout << " saving : " << file_name << std::endl;
            std::ofstream ofs(file_name.c_str());
            boost::archive::text_oarchive ar(ofs);
            ar & ct;
        }
        
        // Load
        orunav_generic::CoordinatedTimes ct_restored;
        {
            std::cout << " loading : " << file_name << std::endl;
            std::ifstream ifs(file_name.c_str());
            boost::archive::text_iarchive ar(ifs);
            ar & ct_restored;
        }
        std::cout << "ct[0] : " << ct[0] << std::endl;
        std::cout << "ct_restored[0] : " << ct_restored[0] << std::endl;
    }

    {
        std::string file_name = "point2dvec.txt";
        orunav_generic::Point2dVec pv;
        pv.push_back(Eigen::Vector2d(1.,1.));
        pv.push_back(Eigen::Vector2d(2.,1.));

        // Save
        {
            std::cout << " saving : " << file_name << std::endl;
            std::ofstream ofs(file_name.c_str());
            boost::archive::text_oarchive ar(ofs);
            ar & pv;            
        }
        // Load
        orunav_generic::Point2dVec pv_restored;
        {
            std::cout << " loading : " << file_name << std::endl;
            std::ifstream ifs(file_name.c_str());
            boost::archive::text_iarchive ar(ifs);
            ar & pv_restored;
        }
        std::cout << "pv[0] : " << pv[0] << std::endl;
        std::cout << "pv_restored[0] : " << pv_restored[0] << std::endl;
    }
}
