#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <orunav_msgs/PolygonConstraint.h>
#include <orunav_msgs/GetPolygonConstraints.h>
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
#include <boost/filesystem.hpp>

constraint_extract::PolygonConstraintsLookup *lookup;
bool visualize;
bool save_constraints_and_path;
ros::Publisher marker_pub;
bool visualize_outer_constraints;
bool visualize_only_invalid;
bool debug;

const char blue[] = {0x1b, '[', '1', ';', '3', '4', 'm', 0};

bool getPolygonConstraintsCallback(orunav_msgs::GetPolygonConstraints::Request  &req,
         orunav_msgs::GetPolygonConstraints::Response &res )
{
  ROS_INFO("[PolygonConstraintService]: Obtained a request for constraints given a path and a map");

  // Step through the path
  bool valid = true;
//  orunav_generic::Path path = orunav_conversions::createPathFromPathMsgUsingTargetsAsFirstLast(req.path);
  orunav_generic::Path path = orunav_conversions::createPathFromPathMsg(req.path);
  orunav_generic::Pose2d target_start_pose = orunav_conversions::createPose2dFromMsg(req.path.target_start.pose);
  orunav_generic::Pose2d target_goal_pose = orunav_conversions::createPose2dFromMsg(req.path.target_goal.pose);

  if (debug) {
    ROS_INFO_STREAM("[PolygonConstraintService]: path length : " << path.sizePath());
    ROS_INFO_STREAM("[PolygonConstraintService]: map : " << req.map.info);
    ROS_INFO_STREAM("[PolygonConstraintServcie]: map data size : " << req.map.data.size());
  }

  if (path.sizePath() == 0) {
    ROS_WARN("[PolygonConstraintService]: empty path provided(!)");
    return false;
  }

  constraint_extract::PolygonConstraint constraint;
  int constraint_idx;
  constraint_extract::PolygonConstraintsVec constraints;
  
  ros::Time t_1 = ros::Time::now();
  for (size_t i = 0; i < path.sizePath(); i++) {
      bool found = false;
      if (i < path.sizePath() - 1) {
          found = lookup->findConstraint(req.map, path.getPose2d(i), path.getPose2d(i+1), constraint, constraint_idx);
//          found = lookup->findConstraint(req.map, path.getPose2d(i), constraint, constraint_idx);
      }
      else {
          found = lookup->findConstraint(req.map, path.getPose2d(i), constraint, constraint_idx);
      }

      if (found) {
          // Check the constraint
          bool valid = constraint_extract::validConstraint(constraint);
          if (!valid) {
              ROS_WARN_STREAM("[PolygonConstraintService]: " << "found a constraint that are not valid " << i << " - should never happen(!)");
          }
          orunav_geometry::Polygon outer_constraint = lookup->computeConstraintOuterRegion(constraint);
          constraint.setOuterConstraint(outer_constraint);
          constraints.push_back(constraint);
          
          if (visualize) {
              if (visualize_only_invalid && valid) {
                  continue;
              }
              orunav_generic::Pose2d pose = path.getPose2d(i);
              //orunav_rviz::drawPoint2dContainerAsConnectedLine(constraint.getInnerConstraint(), "global_best_constraint", i, 1, marker_pub);
              orunav_rviz::drawPose2d(orunav_generic::Pose2d(pose(0), pose(1), constraint.getThBounds()(0)), i, 2, 0.5, "global_th_left", marker_pub);
              orunav_rviz::drawPose2d(orunav_generic::Pose2d(pose(0), pose(1), constraint.getThBounds()(1)), i, 2, 0.5, "global_th_right", marker_pub);
              if (visualize_outer_constraints) {
                //orunav_geometry::Polygon outer_constraint = lookup->computeConstraintOuterRegion(constraint);
                  orunav_rviz::drawPoint2dContainerAsConnectedLine(outer_constraint, "global_best_outer_constraint", i, 0, marker_pub);
              }
          }
      }
      else {
          ROS_WARN_STREAM("[PolygonConstraintService]: " << "FAILED to find constraint at idx : " << i );
          //assert(false);
          return false;
      }
  }


  ros::Time t_2 = ros::Time::now();
  ROS_INFO_STREAM("[PolygonConstraintService]: done computing constraints, time : " << (t_2 - t_1).toSec());

  std::vector<int> invalid_idx = getInvalidConstraintPathOverlap(constraints, path);
  if (!invalid_idx.empty()) {
      ROS_WARN_STREAM("[PolygonConstraintService]: " << "there is " << invalid_idx.size() << " missing overlap between consecutive points - should never happen(!)");
      for (size_t i = 0; i < invalid_idx.size(); i++) {
          ROS_WARN_STREAM("\t" << invalid_idx[i] << " <-> " << invalid_idx[i]+1);
          if (visualize) {
              orunav_rviz::drawPoint2dContainerAsConnectedLine(constraints[invalid_idx[i]].getInnerConstraint(), "invalid_path_overlap", invalid_idx[i], 1, marker_pub);
              orunav_rviz::drawPoint2dContainerAsConnectedLine(constraints[invalid_idx[i]+1].getInnerConstraint(), "invalid_path_overlap", invalid_idx[i]+1, 1, marker_pub);
          }
      }
  }
  else {
      ROS_INFO("[PolygonConstraintService]: no missing overlaps in the constraints.");
  }

  // Check that the start and end constraints contains the target and goal poses(!).
  if (!constraints.front().isFeasible(target_start_pose)) {
      ROS_WARN("[PolygonConstraintService]: First constraint cannot hold the target start pose(!) - MAJOR problem");
  }
  if (!constraints.back().isFeasible(target_goal_pose)) {
      ROS_WARN("[PolygonConstraintService]: Last constraint cannot hold the target goal pose(!) - MAJOR problem");
  }

  // if (!constraint_extract::validConstraintWithPose(constraints.front(), target_start_pose)) {
  //     ROS_WARN("[PolygonConstraintService]: First constraint cannot hold the target start pose(!) - MAJOR problem");
  // }
    
  if (save_constraints_and_path) {
      // Save the path (standard text file).
      {
          std::stringstream st;
          st << "polygonconstraint_service_path_" << req.path.robot_id << "-" << req.path.goal_id << ".path";
          std::string fn = st.str();
          orunav_generic::savePathTextFile(path, fn);
      }

      // Save the constraints
      {
          std::stringstream st;
          st << "polygonconstraint_service_constraints_" << req.path.robot_id << "-" << req.path.goal_id << ".dat";
          std::string fn = st.str();
          std::ofstream ofs(fn.c_str());
          boost::archive::text_oarchive ar(ofs);
          ar & constraints;
      }

      // Save the outer constraints (as a text file)
      {
          // For debugging / figure generation only.
          std::stringstream st;
          st << "polygonconstraint_service_outer_constraint_" << req.path.robot_id << "-" << req.path.goal_id << ".dat";
          std::string fn = st.str();
          std::ofstream ofs(fn.c_str());
          for (size_t i = 0; i < constraints.size(); i++) {
              orunav_geometry::Polygon outer_constraint = lookup->computeConstraintOuterRegion(constraints[i]);
              ofs << orunav_generic::getPoint2dContainerInterfaceGnuplotString(outer_constraint, true);
              ofs << "\n";
          }
          ofs.close();
      }
      // Save the inner position constraints and angular constraints (as text files) useful for gnuplotting.
      {
        std::stringstream st;
        st << "polygonconstraint_service_all_" << req.path.robot_id << "-" << req.path.goal_id;
        constraint_extract::saveConstraintsTextFiles(constraints, path, st.str());
      }
  }
  
  Eigen::MatrixXd A;
  Eigen::VectorXd b; 		 
  double *dt;

  std::cout << "constraints.size() : " << constraints.size() << std::endl;
  std::cout << "path.sizePath() : " << path.sizePath() << std::endl;

  assert(constraints.size() == path.sizePath());

  res.constraints = orunav_conversions::createRobotConstraintsFromPolygonConstraintsVec(constraints);
  
  //res.constraints.robot_id = req.path.robot_id;
  //res.constraints.goal_id = req.path.goal_id;
  
#if 0
  for(size_t i = 0; i< constraints.size(); i++) {
      orunav_msgs::PolygonConstraint poly;
      // Inner
      poly.constraint_id = i;
      constraints[i].getInnerConstraint().getMatrixForm(A,b);
      dt = A.col(0).data();
      poly.A0 = std::vector<double>(dt,dt+A.rows());//sizeof(dt)/sizeof(double));
      dt = A.col(1).data();
      poly.A1 = std::vector<double>(dt,dt+A.rows());//sizeof(dt)/sizeof(double));
      dt = b.data();
      poly.b = std::vector<double>(dt,dt+b.rows());//+sizeof(dt)/sizeof(double));
      poly.theta_min = constraints[i].getThBounds()(0);
      poly.theta_max = constraints[i].getThBounds()(1);
      poly.feasible_x = path.getPose2d(i)(0);
      poly.feasible_y = path.getPose2d(i)(1);
      poly.feasible_th = path.getPose2d(i)(2);
      res.constraints.constraints.push_back(poly);

      // Outer
      constraints[i].getOuterConstraint().getMatrixForm(A,b);
      dt = A.col(0).data();
      poly.A0 = std::vector<double>(dt,dt+A.rows());//sizeof(dt)/sizeof(double));
      dt = A.col(1).data();
      poly.A1 = std::vector<double>(dt,dt+A.rows());//sizeof(dt)/sizeof(double));
      dt = b.data();
      poly.b = std::vector<double>(dt,dt+b.rows());//+sizeof(dt)/sizeof(double));
      poly.theta_min = constraints[i].getThBounds()(0);
      poly.theta_max = constraints[i].getThBounds()(1);
      poly.feasible_x = path.getPose2d(i)(0);
      poly.feasible_y = path.getPose2d(i)(1);
      poly.feasible_th = path.getPose2d(i)(2);
      res.constraints.constraints_outer.push_back(poly);

      // Depriciated below.
      res.constraints.points.push_back(i); // There is no subsampling done here... for all path points there will be a constraint computed.
  }
#endif

  if (visualize) {
      ROS_DEBUG("[PolygonConstraintService]: VISUALIZING!!!");
      orunav_rviz::drawConstraints(res.constraints, marker_pub);
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "polygonconstraint_service");
  ros::NodeHandle n;
  ros::NodeHandle nh = ros::NodeHandle("~");

  int model_type = 1;
  int load_type = 1;
  std::string lookuptables_file;
  bool save_lookup = false;
  
  constraint_extract::PolygonConstraintsLookup::Params lookup_params;

  nh.param("visualize",visualize,false);
  nh.param("model_type", model_type, 1);
  nh.param("load_type", load_type, 1);
  nh.param("save_constraints_and_path", save_constraints_and_path, false);
  nh.param("lookuptables_file", lookuptables_file, std::string(""));
  nh.param("save_lookuptables", save_lookup, false);
  nh.param("visualize_outer_constraints", visualize_outer_constraints, true);
  nh.param("visualize_only_invalid", visualize_only_invalid, false);
  nh.param("skip_overlap", lookup_params.skip_overlap, false);
  nh.param("resolution", lookup_params.resolution, 0.1);
  nh.param("debug", debug, false);
  
  if (visualize)
    {
        ROS_INFO("[PolygonConstraintService]: The output is visualized using visualization_markers (in rviz).");
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }


  orunav_geometry::RobotModelTypeFactory model_type_factory;
  orunav_geometry::RobotModel2dInterface* model = model_type_factory.getModel(model_type);
  
  orunav_geometry::RobotModel2dWithState robot(*model);
  orunav_generic::RobotInternalState2d s;
  
  switch (load_type) {
  case 1:
      s.loadType = orunav_generic::RobotInternalState2d::NO_LOAD;
      break;
  case 2:
      s.loadType = orunav_generic::RobotInternalState2d::EUR_PALLET;
      break;
  case 3:
      s.loadType = orunav_generic::RobotInternalState2d::HALF_PALLET;
      break;
  };
    
  lookup = new constraint_extract::PolygonConstraintsLookup(*model, s.loadType, lookup_params);

  if ( !boost::filesystem::exists( lookuptables_file ) )
  {
    save_lookup = true;
    ROS_WARN_STREAM("Lookup tables files doesn't exist!");
  }

  if (!lookuptables_file.empty() && !save_lookup) {
      ROS_INFO_STREAM("[PolygonConstraintService]: loading lookup tables : " << lookuptables_file);
      ros::Time t_1 = ros::Time::now();
      std::ifstream ifs(lookuptables_file.c_str());
      boost::archive::binary_iarchive ar(ifs);
      
      std::vector<nav_msgs::OccupancyGrid> grids;
      ar & grids;
//      ar & lookup->params_;
      lookup->setOccupancyGrids(grids);
      ros::Time t_2 = ros::Time::now();
      ROS_INFO_STREAM("[PolygonConstraintService]: done (nb grids: " << grids.size() << ") - loading time : " << (t_2 - t_1).toSec());
  }
  ROS_INFO("[PolygonConstraintService]: ---> Computing constraints");
  ros::Time t_3 = ros::Time::now();
  lookup->compute();
  ros::Time t_4 = ros::Time::now();
  ROS_INFO_STREAM("[PolygonConstraintService]: computation time : " << (t_4 - t_3).toSec() << " <--- done.");

  if (save_lookup) {
    ROS_INFO_STREAM(blue << "save_lookup IS true");
      std::cout << "[PolygonConstraintService]: saving the lookup tables to: " << lookuptables_file << std::endl;
      std::ofstream ofs(lookuptables_file.c_str());
//      boost::archive::text_oarchive ar(ofs);
      boost::archive::binary_oarchive ar(ofs);
      ar & lookup->getOccupancyGrids();
//      ar & lookup->params_;
      std::cout << "[PolygonConstraintService]: done saving." << std::endl;
  }
  
  ros::ServiceServer service = n.advertiseService("polygonconstraint_service", getPolygonConstraintsCallback);
  ros::spin();
  delete lookup;
  return 0;
}

