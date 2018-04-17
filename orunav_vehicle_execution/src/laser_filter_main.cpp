#include <ros/ros.h>
#include <orunav_generic/utils.h>
#include <orunav_msgs/ComputeTask.h>
#include <orunav_msgs/SetTask.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <orunav_vehicle_execution/io.h>
#include <orunav_vehicle_execution/laser_filter.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <orunav_geometry/robot_model_2d.h>

#include <boost/program_options.hpp>


namespace po = boost::program_options;

//! Simple node to record the background laser data.
class LaserFilterNode
{
private:
  ros::Subscriber laser_sub_;
  ros::NodeHandle nh_;

  orunav_geometry::Polygon boundary_;
  orunav_vehicle_execution::LaserFilter laser_filter_;
  std::string output_file_;
  tf::TransformListener tf_listener_;
  bool invalidate_close_idx_;
  laser_geometry::LaserProjection laser_projection_;
  std::string frame_id_;

  boost::shared_ptr<orunav_geometry::RobotModel2dInterface> model_;
  orunav_geometry::Polygon poly_;
public:
  LaserFilterNode(ros::NodeHandle paramHandle)
  {

    std::string model_name;
    paramHandle.param<std::string>("model_name",model_name,std::string("cititruck"));
    paramHandle.param<std::string>("laser_topic",model_name,std::string("/robot1/laserscan0"));
    paramHandle.param<std::string>("output_file", output_file_, std::string("laser_background.dat"));
    paramHandle.param<std::string>("frame_id", frame_id_, std::string("/robot1/base_link"));
    paramHandle.param<bool>("invalidate_close_idx", invalidate_close_idx_, false);


    orunav_geometry::RobotModelTypeFactory model_factory;
    model_ = model_factory.create(model_name);

    orunav_generic::RobotInternalState2d state;
    poly_ = model_->getBoundingRegion(state);

  }

  void process_laserscan(const sensor_msgs::LaserScanConstPtr &msg) {

    ROS_INFO_STREAM("got laser");

    if (!laser_filter_.isInitialized()) {
      laser_filter_.resize(msg->ranges.size());
    }

    // Force all measurement to have a valid reading, that is we want to be able to filter on the laser index directly.
    sensor_msgs::LaserScan scan = *msg;


    for (size_t i = 0; i < scan.ranges.size(); i++) {
      if (scan.ranges[i] <= scan.range_min || scan.ranges[i] >= scan.range_max)  {
        // Mark these as invalid...
        laser_filter_.setValid(i, false);
        if (invalidate_close_idx_) {
          laser_filter_.setValid(i-i, false);
          laser_filter_.setValid(i+i, false);
        }
        // Force these to be valid (to be able to use the laser to point conversion.
        scan.ranges[i] = scan.range_max-0.01;
      }
    }


    if(!tf_listener_.waitForTransform(
         scan.header.frame_id,
         frame_id_,
         scan.header.stamp + ros::Duration().fromSec(scan.ranges.size()*scan.time_increment),
         ros::Duration(1.0))) {
      return;
    }

    sensor_msgs::PointCloud cloud;
    laser_projection_.transformLaserScanToPointCloud(frame_id_,scan,
                                                     cloud,tf_listener_);

    assert(cloud.size() == scan.ranges.size());

    // Do the collision checking
    for (size_t i = 0; i < cloud.points.size(); i++) {
      if (boundary_.collisionPoint2d(Eigen::Vector2d(cloud.points[i].x,
                                                     cloud.points[i].y))) {
        laser_filter_.setValid(i, false);
        if (invalidate_close_idx_) {
          laser_filter_.setValid(i-i, false);
          laser_filter_.setValid(i+i, false);
        }

      }
    }

  laser_filter_.save(output_file_);
  }


 };


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_filter_node");
  ros::NodeHandle params ("~");

  LaserFilterNode l(params);

  ros::spin();
}


// Only valid if only one laser scaner is used.
void process_laserscan0(const sensor_msgs::LaserScanConstPtr &msg) {

    // Filter the scans
    //sensor_msgs::LaserScan scan = orunav_vehicle_execution::filter_laser_scan(msg, laserscan0_filter_);
    //process_laserscan(msg);
}

void process_laserscan1(const sensor_msgs::LaserScanConstPtr &msg) {

    // Filter the scans
    //sensor_msgs::LaserScan scan = orunav_vehicle_execution::filter_laser_scan(msg, laserscan1_filter_);
    //process_laserscan(msg);
  }

