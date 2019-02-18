#include <stdio.h>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <orunav_msgs/ControllerTrajectoryChunkVec.h>


FILE * pFile;

void callbac(const orunav_msgs::ControllerTrajectoryChunkVec::ConstPtr& msg_ptr) {

  for(const auto& chunk : msg_ptr->chunks) {
    for(const auto& step : chunk.steps) {
      fprintf(pFile, "%lf %lf %lf %lf\n", step.state.position_x, step.state.position_y, step.state.orientation_angle, step.state.steering_angle);
    }
  }

  ROS_INFO_STREAM ("Success.");
  exit(0);
}


int main(int argn, char* args[]) {
	if (argn != 3) {
		std::cout << "Exactly two arguments for topic name and filename (in that order) please..." << std::endl;
		exit(0);
	}

	std::string filename = args[2];
	std::string topic_name = args[1];

	pFile = fopen (filename.c_str(),"w");

	if(!pFile) {
	  std::cout << "Opening the file sucked for some reason. Bad name? Did you 'touch' filename?" << std::endl;
	  exit(0);
	}

	std::cout << "Waiting for messages on the topic. Send a goal and the generated trajectory will be written to the specified file." << std::endl;

	ros::init(argn, args, "traj_ex");
	ros::NodeHandle nh;
	ros::Subscriber controller_traj_sub = nh.subscribe(topic_name, 1, &callbac);

	ros::spin();
}
