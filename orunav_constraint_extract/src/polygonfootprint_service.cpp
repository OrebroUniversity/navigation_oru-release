#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <orunav_msgs/PolygonConstraint.h>
#include <orunav_msgs/GetPolygonFootPrint.h>
#include <orunav_rviz/orunav_rviz.h>
#include <orunav_generic/io.h>
#include <orunav_constraint_extract/polygon_constraint.h>
#include <orunav_constraint_extract/utils.h>
#include <orunav_constraint_extract/serialization.h>
#include <orunav_constraint_extract/conversions.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <boost/serialization/vector.hpp>

bool visualize;
ros::Publisher marker_pub;
double distance;

bool getPolygonFootPrintCallback(orunav_msgs::GetPolygonFootPrint::Request  &req,
                                 orunav_msgs::GetPolygonFootPrint::Response &res )
{
  ROS_INFO("[PolygonFootPrintService]: Obtained a request for a footprint given a path and target");

  // Step through the path
  bool valid = true;
//  orunav_generic::Path path = orunav_conversions::createPathFromPathMsgUsingTargetsAsFirstLast(req.path);
  orunav_generic::Path path = orunav_conversions::createPathFromPathMsg(req.path);

  int constraint_idx;
  constraint_extract::PolygonConstraintsVec constraints;
  
  std::vector<orunav_generic::Path> paths = orunav_generic::splitOnDistance(path, distance);
  std::vector<int> centers;
  std::vector<int> startPoints;
  std::vector<int> endPoints;

  orunav_geometry::RobotModelTypeFactory model_type_factory;
  orunav_geometry::RobotModel2dInterface* model = model_type_factory.getModel(req.target.type_id);
  orunav_generic::RobotInternalState2d internal_state2d;

  int acc = 0;
  ros::Time t_1 = ros::Time::now();
  for (size_t i = 0; i < paths.size(); i++) {
    // Compute the footprint for each path snippet.
    // TODO - check how the index should be keept (if needed).
    constraint_extract::PolygonConstraint constraint;
    orunav_geometry::Polygon sweep_area = constraint_extract::computeSweepArea(paths[i], *model, internal_state2d);

    sweep_area.convexHull();

    constraint.setOuterConstraint(sweep_area);
    constraint.start_point_idx = acc;
    constraint.end_point_idx = acc+paths[i].sizePath()-1;

    //constraint.outer_constraint_points = sweep_area.points;
    for (int k = 0; k < sweep_area.points.size(); k++) {
      geometry_msgs::Point p;
      p.x = sweep_area.points[k](0,0);
      p.y = sweep_area.points[k](1,0);
      constraint.outer_constraint_points.push_back(p);
    }

    constraints.push_back(constraint);

    /**
    std::vector<double> A0, A1, b;
    constraint.getOuterConstraint().getMatrixFormAsVectors(A0, A1, b);
    if (A0.size() == 19) {
      for (size_t k = 0; k < A0.size(); k++)
        ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>> WARNING!!! DEGENERATE CONSTRAINT!!! AB[%lu]: %f %f %f", k, A0[k], A1[k], b[k]);
    }
    **/

    //    startPoints.push_back(acc);
    //    endPoints.push_back(acc+paths[i].sizePath());
    centers.push_back(acc + paths[i].sizePath() / 2);
    acc += paths[i].sizePath();

    if (visualize) {
      orunav_rviz::drawPoint2dContainerAsConnectedLine(sweep_area, "sweep_footprint", i, 0, marker_pub);
    }
  }
  res.constraints = orunav_conversions::createRobotConstraintsFromPolygonConstraintsVec(constraints);

  for (size_t i = 0; i < centers.size(); i++) {
    res.constraints.points.push_back(centers[i]);
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "polygonfootprint_service");
  ros::NodeHandle n;
  ros::NodeHandle nh = ros::NodeHandle("~");

  int model_type = 1;
  int load_type = 1;
  std::string lookuptables_file;
  bool save_lookup = false;
  
  constraint_extract::PolygonConstraintsLookup::Params lookup_params;

  nh.param("visualize",visualize,false);
  nh.param("distance",distance, 4.);

  if (visualize)
    {
        ROS_INFO("[PolygonFootPrintService]: The output is visualized using visualization_markers (in rviz).");
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }

  
  ros::ServiceServer service = n.advertiseService("polygonfootprint_service", getPolygonFootPrintCallback);
  ros::spin();
  return 0;
}

