#include <CameraPoseCalibNode.hh>
#include <ros/ros.h>
using namespace std;

int main(int argc, char** argv) {

    ros::init(argc,argv,"camera_pose_calib_node");
    ros::NodeHandle params ("~");

    CameraPoseCalibNode calib(params);

    ros::spin();

}
