#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

namespace po = boost::program_options;
using namespace std;
using namespace boost::filesystem;


Eigen::Affine3d getTf( const nav_msgs::Odometry odo){
		tf::Quaternion b(odo.pose.pose.orientation.x,odo.pose.pose.orientation.y, odo.pose.pose.orientation.z, odo.pose.pose.orientation.w);
		tf::Vector3 v(odo.pose.pose.position.x,odo.pose.pose.position.y,0);			
		Eigen::Affine3d out;
		tf::transformTFToEigen (tf::Transform(b,v), out);
		return out;
}

int main(int ac, char **av){
	ros::Time::init();

        po::options_description desc("Allowed options");
        std::string tf_frame, bag_dir;
        int start_idx;
        int bag_cnt;

        desc.add_options()
          ("help", "produce help message")
          ("tf_frame", po::value<std::string>(&tf_frame)->default_value(std::string("/kmo_vmc_navserver/state")), "/tf frame that the traversed distance should be computed on")
          ("bag_dir", po::value<std::string>(&bag_dir)->required(), "directory that contains one or several bag files that should be used")
          ("start_idx", po::value<int>(&start_idx)->default_value(0), "start index of the first file to use")
          ("debug", "debug flag")
    ;

        po::variables_map vm;
        po::store(po::parse_command_line(ac, av, desc), vm);
        
        if (vm.count("help")) {
          cout << desc << "\n";
          return 1;
        }
        
        po::notify(vm);    
        
        // Extract all files
        std::cout << "Loading directory : " << bag_dir << std::endl;
        path p(bag_dir);
        if (!is_directory(p)) {
          std::cerr << "[" << bag_dir << "] - is not a directory quitting..." << std::endl;
          return -1;
        }
        
        // The order is a bit off, sort them first...
        vector<path> pv;
        copy(directory_iterator(p), directory_iterator(), back_inserter(pv));
        sort(pv.begin(), pv.end());
	
        double tot_odo_dist = 0.;

        for (vector<path>::const_iterator it(pv.begin()+start_idx), it_end(pv.end()); it != it_end; ++it)
        {
                rosbag::Bag bag;
                bag.open(it->c_str(), rosbag::bagmode::Read);
                std::cout << "Reading : " << *it << std::endl;
                std::vector<std::string> topics;
		
		topics.push_back(tf_frame); 
		rosbag::View view(bag, rosbag::TopicQuery(topics));
		
		Eigen::Affine3d Told;
		double odo_dist = 0;
		bag_cnt = 0;
		
		BOOST_FOREACH(rosbag::MessageInstance const m, view)
		{
			/////////////////////////////////////////////////////////////////////////////////////////////
			nav_msgs::Odometry::ConstPtr odo = m.instantiate<nav_msgs::Odometry>();
			if (odo != NULL){
				
				if(m.getTopic() == tf_frame){
					
					Eigen::Affine3d Tnow = getTf(*odo);										
					
					if(bag_cnt == 0) {
                                            Told = Tnow;
                                        }
					bag_cnt++;
					Eigen::Affine3d Tmotion = Told.inverse() * Tnow;
					Told = Tnow;
					odo_dist += Tmotion.translation().norm();
				}
			}
		}
		bag.close();
                std::cout << "Distance travelled (in this bag) = " << odo_dist << "  (" << odo_dist/1000.0 << "km)" << std::endl;
                tot_odo_dist += odo_dist;
                std::cout << "Distance travelled (total) = " << tot_odo_dist << "  (" << tot_odo_dist/1000.0 << "km)" << std::endl;
	}
        std::cout << "Done." << std::endl;
}
