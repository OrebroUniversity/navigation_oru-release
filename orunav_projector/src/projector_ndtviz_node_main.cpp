// Node used to generate images to the onboard mounted projector.
// Will use the controller feedback / tf and the visualization markers (/robotX/visualization_marker topic)

#include <orunav_projector/projector_viz.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <visualization_msgs/Marker.h>

#include <pupil_ros/surface_position.h>
#include <orunav_generic/types.h>
#include <orunav_generic/utils.h>
#include <orunav_generic/pose2d.h>
#include <orunav_msgs/ControllerReport.h>
#include <orunav_msgs/ForkReport.h>

#include <orunav_conversions/conversions.h>
#include <laser_geometry/laser_geometry.h>

#include <orunav_geometry/polygon_arrow.h>

#include <pcl_ros/point_cloud.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class ProjectorNDTVizNode {

private:
  ros::NodeHandle nh_;

  ros::Subscriber control_report_sub_;
  ros::Subscriber fork_report_sub_;
  ros::Subscriber marker_sub_;
  ros::Subscriber laserscan_sub_;
  ros::Subscriber surface_position_sub_;

  bool use_forks_;
  int robot_id_;
  bool movable_camera_;
  ProjectorViz ndt_viz_;

  tf::TransformListener tf_listener_;
  std::string projector_frame_id_;
  std::string tf_base_link_;
  std::string controller_report_topic;

  Eigen::Vector3d pos, fp;
  double aspect_ratio;
  laser_geometry::LaserProjection laser_projection_;
  bool draw_sweep_area_, draw_path_, draw_arrow_, blink_{false}; // defined for blinking action
  orunav_geometry::PolygonArrow poly_arrow_;

  double secs; // defined for blinking action

  float prev_blink_{0.0};
  float curr_blink_{0.0};
public:
  ProjectorNDTVizNode(ros::NodeHandle &paramHandle)
  {
    double grid_size;

    // Parameters
    paramHandle.param<int>("robot_id", robot_id_, 1);
    paramHandle.param<bool>("use_forks", use_forks_, true);
    paramHandle.param<bool>("draw_sweep_area", draw_sweep_area_, false);
    paramHandle.param<bool>("draw_arrow", draw_arrow_, true);
    paramHandle.param<bool>("draw_path", draw_path_, false);

    // paramHandle.param<double>("pos_x", pos(0), -0.325782);
    // paramHandle.param<double>("pos_y", pos(1), -0.0142168);
    // paramHandle.param<double>("pos_z", pos(2), 1.06297);
    // paramHandle.param<double>("fp_x", fp(0), -0.866651);
    // paramHandle.param<double>("fp_y", fp(1), -0.0096648);
    // paramHandle.param<double>("fp_z", fp(2), 0);

    paramHandle.param<double>("pos_x", pos(0), -0.340318);
    paramHandle.param<double>("pos_y", pos(1), 0.0268098);
    paramHandle.param<double>("pos_z", pos(2), 1.22858);
    paramHandle.param<double>("fp_x", fp(0), -1.21756);
    paramHandle.param<double>("fp_y", fp(1), 0.0341927);

    paramHandle.param<double>("fp_z", fp(2), 0);
    paramHandle.param<double>("aspect_ratio", aspect_ratio, 0.67);
    paramHandle.param<double>("grid_size", grid_size, 0.3);
    paramHandle.param<bool>("movable_camera", movable_camera_, false);
    paramHandle.param<std::string>("projector_frame_id", projector_frame_id_, orunav_generic::getRobotTF(robot_id_, std::string("/projector_link")));
    paramHandle.param<std::string>("tf_base_link", tf_base_link_, orunav_generic::getRobotBaseLinkTF(robot_id_));
    paramHandle.param<std::string>("controller_report_topic", controller_report_topic, orunav_generic::getRobotTopicName(robot_id_, "/controller/report"));

    ROS_INFO("The controller report topic is currently: %s", controller_report_topic.c_str() );
    secs = ros::Time::now().toSec(); //defined for blinking action

    std::string slowdown_texture;
    paramHandle.param<std::string>("slowdown_texture", slowdown_texture, "zebra.png");

    control_report_sub_ = nh_.subscribe<orunav_msgs::ControllerReport>(controller_report_topic, 10,&ProjectorNDTVizNode::process_report, this);
    if (use_forks_) {
      fork_report_sub_ = nh_.subscribe<orunav_msgs::ForkReport>(orunav_generic::getRobotTopicName(robot_id_, "/fork/report"), 10, &ProjectorNDTVizNode::process_fork_report,this);
    }
    marker_sub_ = nh_.subscribe<visualization_msgs::Marker>(orunav_generic::getRobotTopicName(robot_id_, "/visualization_marker"), 10, &ProjectorNDTVizNode::process_marker, this);

    surface_position_sub_ = nh_.subscribe<pupil_ros::surface_position>("/pupil_capture/surface", 1, &ProjectorNDTVizNode::surfacePositionCallback, this);
    // laserscan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(orunav_generic::getRobotTopicName(robot_id_, "/laser_forkdir_scan"), 10,&ProjectorNDTVizNode::process_laserscan, this);


    ndt_viz_.win3D->setFullScreen(true); // full screen setting default - true
    ndt_viz_.win3D->setMotionBlurFrames(0);
    ndt_viz_.win3D->setAspectRatioFactor(aspect_ratio);
    if (!movable_camera_)
      ndt_viz_.win3D->switchCamera(std::string("fixed"));
    if (grid_size > 0)
      ndt_viz_.addGrid(100, grid_size);
    ndt_viz_.win3D->start_main_loop_own_thread();

    ndt_viz_.setSlowdownRegionTexture(slowdown_texture);

  }

  void surfacePositionCallback(const pupil_ros::surface_positionConstPtr& surface_position) {
    curr_blink_ = (float) not surface_position->onsrf;
    float final = 0.8 * prev_blink_  + 0.2 * curr_blink_;
    prev_blink_ = final;
    if(final > 0.5) this->blink_ = true;
    else this->blink_ = false;
    //this->blink_ = not surface_position->onsrf;
  }

  void process_marker(const visualization_msgs::MarkerConstPtr &msg) {
    // We're intressted in:
    // * the path
    // * the sweep area
    // * the end pose
    // * the ebrake and slowdown area

    draw_sweep_area_ = false;
    if (msg->ns == std::string("sweep2") && draw_sweep_area_) {
      // Assume that we have a line strip - type = 4
      if (msg->type == 4) {
        // Parse the points.
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > pts;
        for (size_t i = 0; i < msg->points.size(); i++) {
          pts.push_back(Eigen::Vector2d(msg->points[i].x, msg->points[i].y));
        }
        ndt_viz_.addPolygon(pts, 1., 1., 1.);
      }
    }
    if (msg->ns == std::string("path_points1") && draw_path_) {
      if (msg->type == 4) {
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > pts;
        for (size_t i = 0; i < msg->points.size(); i++) {
          pts.push_back(Eigen::Vector2d(msg->points[i].x, msg->points[i].y));
        }
        ndt_viz_.addPathLine(pts, 1., 1., 1.);//0.3, 1., 0.2);
      }
    }
    if (msg->ns == std::string("ebrake0")) {
      if (msg->type == 4) {
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > pts;
        for (size_t i = 0; i < msg->points.size(); i++) {
          pts.push_back(Eigen::Vector2d(msg->points[i].x, msg->points[i].y));
        }
        ndt_viz_.addEBrakeRegion(pts, 0.9, 0.3, 0.3);
      }
    }
    if (msg->ns == std::string("slowdown0")) {
      if (msg->type == 4) {
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > pts;
        for (size_t i = 0; i < msg->points.size(); i++) {
          pts.push_back(Eigen::Vector2d(msg->points[i].x, msg->points[i].y));
        }
        ndt_viz_.addSlowdownRegion(pts, 0.3, 0.3, 0.9);
      }
    }
    if (msg->ns == std::string("laserscan_cloud0")) {
      if (msg->type == 8) {
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > pts;
        for (size_t i = 0; i < msg->points.size(); i++) {
          pts.push_back(Eigen::Vector2d(msg->points[i].x, msg->points[i].y));
        }
        ndt_viz_.addLaserCloud(pts, 0.9, 0.3, 0.9);
      }

    }

  }

// OLD function for drawing the arrow

//  void draw_arrow() {
//    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > pts;
//    const orunav_geometry::Polygon& poly = poly_arrow_.getPosePolygon();
//    for (size_t i = 0; i < poly.sizePoint2d(); i++) {
//      pts.push_back(poly.getPoint2d(i));
//    }
//    ndt_viz_.addPolygon(pts, 1., 1., 1.);
//  }

// NEW function for drawing the color filled arrow and blinking
// Also, note the changes made in the file polygon_arrow.h in orunav_geometry

  void draw_arrow() {
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > pts;
    const orunav_geometry::Polygon& poly = poly_arrow_.getPosePolygon();
    for (size_t i = 0; i < poly.sizePoint2d(); i++) {
      pts.push_back(poly.getPoint2d(i));
    }
    //ndt_viz_.addPolygon(pts, 0., 0., 1.);

    //std::cout<<"Time   "<<secs<<std::endl;

    double localSecs = ros::Time::now().toSec();
    double diff = localSecs-secs;

    static double changeCol = 1.;


    if(diff > 0.2 &&  blink_) {
      secs = ros::Time::now().toSec();
      if(changeCol == 0.)
        changeCol = 1.;
      else
        changeCol = 0.;
    }
    if(!blink_)
      changeCol = 1.;
    //ndt_viz_.addColorArrow(pts, changeCol, 0., 0.);
    ndt_viz_.addColorArrow(pts, changeCol, changeCol, changeCol);	 // RAVI - following the filling style of eBrake region and Slowdown regions
    // the color changing of the arrow is not working when changing the parameters here, but the color changes only when its changed in the
    // projector_viz.h --> test_region.setColor4(1.,1.,1.,1.);

  }





// may be also useful for troubleshooting

// ndt_viz_.win3D->setFullScreen(false);




  void update_arrow(const orunav_generic::State2d &state) {
    orunav_generic::Pose2d pose = state.getPose2d(); // The current location
    // Add some offset, note that what's drawn here assumes that the vehicle is driving backwards - forks forward.
    pose = orunav_generic::addPose2d(pose, orunav_generic::Pose2d(-1.0, 0., M_PI));

    // Add some turning (from the steering wheel to the arrow)
    pose[2] -= state.getSteeringAngle(); //Steering angle input to the arrow -- this needs to be replaced by the new steering wheel inout of the BT Truck.

    poly_arrow_.update(pose);
  }

  void process_report(const orunav_msgs::ControllerReportConstPtr &msg) {


    orunav_generic::State2d state = orunav_conversions::createState2dFromControllerStateMsg(msg->state);
    if (draw_arrow_) {
      update_arrow(state);
      draw_arrow();
    }

    orunav_generic::Pose2d pose = state.getPose2d();
    Eigen::Vector3d proj_pos, proj_fp;
    {
      orunav_generic::Pose2d rel_pose(pos(0), pos(1), 0);
      orunav_generic::Pose2d glb_pose = orunav_generic::addPose2d(pose, rel_pose);
      proj_pos = Eigen::Vector3d(glb_pose(0), glb_pose(1), pos(2));
    }

    {
      orunav_generic::Pose2d rel_pose(fp(0), fp(1), 0);
      orunav_generic::Pose2d glb_pose = orunav_generic::addPose2d(pose, rel_pose);
      proj_fp = Eigen::Vector3d(glb_pose(0), glb_pose(1), fp(2));
    }

    if (movable_camera_)
      return;

    ndt_viz_.win3D->setCameraPosition(proj_pos(0),
                                      proj_pos(1),
                                      proj_pos(2));

    ndt_viz_.win3D->setCameraPointingToPoint(proj_fp(0),
                                             proj_fp(1),
                                             proj_fp(2));


    // if (msg->status == msg->CONTROLLER_STATUS_ACTIVE ||
    //     msg->status == msg->CONTROLLER_STATUS_FINALIZE) {
    //   ndt_viz_.addTrajectoryPoint(state.getPose2d()[0],
    //                               state.getPose2d()[1],
    //                               0.0,1.0,0,0);
    //   ndt_viz_.displayTrajectory();
    // }
    // else {
    //   ndt_viz_.clearTrajectoryPoints();
    // }


    // Eigen::Affine3d Tcam;
    // if (!projector_frame_id_.empty()) {

    //   tf::StampedTransform transform;
    //   tf_listener_.waitForTransform("/world", projector_frame_id_, msg->stamp, ros::Duration(1.0));
    //   try{
    //     tf_listener_.lookupTransform("/world", projector_frame_id_, msg->stamp, transform);

    //     tf::poseTFToEigen(transform, Tcam);
    //   }
    //   catch (tf::TransformException ex){
    //     ROS_ERROR("%s",ex.what());
    //     return;
    //   }
    // }

    // ndt_viz_.win3D->setCameraPosition(Tcam.translation()(0),
    //                                   Tcam.translation()(1),
    //                                   Tcam.translation()(2));

    // // Compute a forward point ("to look at"), here we assume that the X-axis gives the forward direction.
    // Eigen::Affine3d Tlook_at_offset = Eigen::Affine3d::Identity();
    // Tlook_at_offset.translation() = Eigen::Vector3d(1., 0., 0.);
    // Eigen::Affine3d Tlook_at = Tcam * Tlook_at_offset;

    // ndt_viz_.win3D->setCameraPointingToPoint(Tlook_at.translation()(0),
    //                                          Tlook_at.translation()(1),
    //                                          Tlook_at.translation()(2));

    // ndt_viz_.addTrajectoryPoint(state.getPose2d()[0],
    //                             state.getPose2d()[1],
    //                             0.0,1.0,0,0);
    // ndt_viz_.displayTrajectory();



  }

  void process_fork_report(const orunav_msgs::ForkReportConstPtr &msg) {

  }

  // void process_laserscan(const sensor_msgs::LaserScanConstPtr &msg) {

  //   ROS_INFO_STREAM("process_laserscan");
  //   if(!tf_listener_.waitForTransform(
  //                                     msg->header.frame_id,
  //                                     "/world",
  //                                     msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),
  //                                     ros::Duration(1.0))) {
  //     return;
  //   }
  //   tf::StampedTransform transform;
  //   try{
  //     tf_listener_.lookupTransform(msg->header.frame_id, "/world",
  //                                 msg->header.stamp, transform);

  //     }
  //     catch (tf::TransformException ex){
  //       ROS_ERROR("%s",ex.what());
  //     }

  //   sensor_msgs::PointCloud2 cloud;
  //   laser_projection_.projectLaser(*msg, cloud);

  //   pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  //   pcl::fromROSMsg<pcl::PointXYZ> (cloud, pcl_cloud);

  //   Eigen::Vector3d laser_pose(transform.getOrigin().x(),
  //                              transform.getOrigin().y(),
  //                              tf::getYaw(transform.getRotation()));

  //   ndt_viz_.addScan(laser_pose, pcl_cloud);
  // }

};

int main(int argc, char** argv) {

  ros::init(argc,argv,"projector_ndtviz_node");
  ros::NodeHandle params ("~");

  ProjectorNDTVizNode p(params);

  sleep(1);
  glClearColor(.0f,.0f,.0f,1.0f);

  ros::spin();

}
