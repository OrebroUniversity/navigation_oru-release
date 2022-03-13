#include <ros/ros.h>

#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>

#include <orunav_msgs/GetPath.h>

#include <orunav_motion_planner/PathFinder.h>
#include <orunav_motion_planner/VehicleMission.h>

#include <orunav_motion_planner/DualSteerModel.h>
#include <orunav_motion_planner/DualSteerConfiguration.h>

#include <iostream>
#include <string>
#include <fstream>
#include <math.h>
#include <chrono>
#include <ctime>



#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <visualization_msgs/Marker.h>
#include <orunav_rviz/orunav_rviz.h>

#include <orunav_motion_planner/nav_msgs_occupancy_grid_to_planner_map.h>

/**
 * Responds to GetPath requests
 */
class GetPathService {

private:
  std::string motion_prim_dir_;
  std::string lookup_tables_dir_;
  std::string maps_dir_;

  VehicleModel* vehicle_model_;
  bool visualize_;
  bool BS;
  int sets; 
  int mission = 0;
  
  ros::Publisher marker_pub_;
  
  double min_incr_path_dist_;
  bool save_paths_;

  ros::NodeHandle nh_;
  ros::ServiceServer service_;


public:
  GetPathService(ros::NodeHandle param_nh, std::string name) {
      std::time_t t = std::time(NULL);

      // read parameters
      param_nh.param<std::string>("motion_primitives_directory", motion_prim_dir_, "./Primitives/");
      param_nh.param<std::string>("lookup_tables_directory", lookup_tables_dir_, "./LookupTables/");
      param_nh.param<std::string>("maps_directory", maps_dir_, "./");
      std::string logfile; param_nh.param<std::string>("log_file", logfile, WP::LOG_FILE);
      int loglevel; param_nh.param<int>("log_level", loglevel, WP::LOG_LEVEL);
      std::string auxfile; param_nh.param<std::string>("aux_file", auxfile, WP::AUX_FILE);
      param_nh.param<double>("min_incr_path_dist", min_incr_path_dist_, 0.001);
      param_nh.param<bool>("save_paths", save_paths_, false);
      param_nh.param<bool>("biSteering", BS, false);

      std::string model, model2, model3, model4, model5;
      param_nh.param<std::string>("model", model, "");
      param_nh.param<std::string>("model2", model2, "");
      param_nh.param<std::string>("model3", model3, "");
      param_nh.param<std::string>("model4", model4, "");
      param_nh.param<std::string>("model5", model5, "");
      param_nh.param<int>("num_of_sets", sets, 1);
      
      WP::setPrimitivesDir(motion_prim_dir_);
      WP::setTablesDir(lookup_tables_dir_);
      WP::setMapsDir(maps_dir_);
      WP::setExpansionMethod(WP::NodeExpansionMethod::NAIVE);
      WP::setLogFile(logfile);
      WP::setAuxFile(auxfile);
      WP::setLogLevel(loglevel);
      
      if(WP::LOG_LEVEL == WP::LogLevels::DEBUG) {
          writeLogLine("\n=================================\n", "GetPathService", WP::AUX_FILE, std::ios::trunc);
          char date_time[100];
          if (std::strftime(date_time, 100, "%d/%m/%Y %T", std::localtime(&t))) 
              writeLogLine(std::string("date: ") + date_time, WP::AUX_FILE);
          writeLogLine(std::string("double steering mode: ") + std::to_string(BS), WP::AUX_FILE);
          writeLogLine(std::string("sets of primitives: "), WP::AUX_FILE);
      }

      if (BS) { 
        //Dual steer start
        std::array<std::string,5> models{model, model2, model3, model4, model5}; //Cecchi_add
        vehicle_model_ = new DualSteerModel(models, sets);//Cecchi_add
        WP::setExpansionMethod(WP::NodeExpansionMethod::NAIVE);
        WP::setVehicleType(WP::VehicleType::XA_4WS);
        for(int i = 0; i < sets; i++){
          std::cout << " vehicle type:" << WP::VEHICLE_TYPE << " " << WP::NODE_EXPANSION_METHOD << std::endl;
          ROS_INFO_STREAM("\x1B[33m[GetPathService] - Using model : \033[0m" << models[i] << "\n"); //Cecchi_add
        }
      }
      else { 
        vehicle_model_ = new CarModel(model);
        std::string msg = std::string("total primitives: ").append(std::to_string(vehicle_model_->getTotalPrimitives()));
        writeLogLine(msg, "GetPathService", WP::AUX_FILE);
      }
      
      if (WP::LOG_LEVEL == WP::LogLevels::DEBUG) 
          writeLogLine(std::string("total primitives: ") + std::to_string(vehicle_model_->getTotalPrimitives()), WP::AUX_FILE);

      param_nh.param<bool>("visualize",visualize_,true);
      if (visualize_) {
        ROS_INFO("[GetPathService] -  The output is visualized using /visualization_markers (in rviz).");
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
      }
      service_ = nh_.advertiseService("get_path", &GetPathService::getPathCB, this);
  }


  ~GetPathService() {
      delete vehicle_model_;
      ROS_INFO_STREAM("[GetPathService] - shutting down\n");
    }

  
  bool getPathCB(orunav_msgs::GetPath::Request &req,
		 orunav_msgs::GetPath::Response &res)
  {
    orunav_msgs::RobotTarget tgt = req.target;

    ROS_INFO("[GetPathService] - start : [%f,%f,%f](%f)", tgt.start.pose.position.x, tgt.start.pose.position.y,tf::getYaw(tgt.start.pose.orientation), tgt.start.steering);
    ROS_INFO("[GetPathService] - goal :  [%f,%f,%f](%f)", tgt.goal.pose.position.x, tgt.goal.pose.position.y,tf::getYaw(tgt.goal.pose.orientation), tgt.goal.steering);
    
    mission += 1;
    if (WP::LOG_LEVEL == WP::LogLevels::DEBUG) {
        std::ofstream f;
        f.open(WP::AUX_FILE);
        f << "\n -- mission number " << mission << " --" << std::endl;
        f << "goalID:  " << tgt.task_id << std::endl;
        f << "start : [" << tgt.start.pose.position.x << "," << tgt.start.pose.position.y << "," << tf::getYaw(tgt.start.pose.orientation) << "]" << std::endl;
        f << "goal :  [" << tgt.goal.pose.position.x  << "," << tgt.goal.pose.position.y  << "," << tf::getYaw(tgt.goal.pose.orientation)  << "]" << std::endl;
        f.close();
    }

    drawFootPrint("start", tgt.start.pose.position.x, tgt.start.pose.position.y, tf::getYaw(tgt.start.pose.orientation));
    drawFootPrint("goal", tgt.goal.pose.position.x, tgt.goal.pose.position.y, tf::getYaw(tgt.goal.pose.orientation));
    WorldOccupancyMap planner_map;
    convertNavMsgsOccupancyGridToWorldOccupancyMapRef(req.map, planner_map);

    orunav_generic::Pose2d map_pose_offset = getNavMsgsOccupancyGridOffsetRef(req.map);
    double map_offset_x = map_pose_offset(0);
    double map_offset_y = map_pose_offset(1);
    double start_orientation = tf::getYaw(tgt.start.pose.orientation);
    double goal_orientation = tf::getYaw(tgt.goal.pose.orientation);
 
    assert(fabs(map_pose_offset(2) < 0.001)); // The orientation should be the same... implement this whenever needed. 
    
    if (planner_map.getMap().empty()) {
      ROS_ERROR("[GetPathService] - error in the provided map / conversion");
      return false;
    }

    PathFinder* pf;
    if (planner_map.getMap().empty())
      pf = new PathFinder(20, 20);
    else
      pf = new PathFinder(planner_map);
    
    ROS_INFO("[CHECK START] - start : [%f,%f] ", tgt.start.pose.position.x-map_offset_x, tgt.start.pose.position.y-map_offset_y);
    VehicleMission vm(vehicle_model_,
                      tgt.start.pose.position.x-map_offset_x, tgt.start.pose.position.y-map_offset_y, start_orientation, tgt.start.steering,
                      tgt.goal.pose.position.x-map_offset_x, tgt.goal.pose.position.y-map_offset_y, goal_orientation, tgt.goal.steering);

    pf->addMission(&vm);
    req.max_planning_time = 0;
    if (req.max_planning_time > 0)
      pf->setTimeBound(req.max_planning_time);
    
    ROS_INFO("[GetPathService] - Starting to solve the path planning problem ... ");
    ros::Time start_time = ros::Time::now();
    std::vector<std::vector<Configuration*> > solution = pf->solve(false);
    ros::Time stop_time = ros::Time::now();
    ROS_INFO("[GetPathService] - Starting to solve the path planning problem - done");
    ROS_INFO("[GetPathService] - PATHPLANNER_PROCESSING_TIME: %f", (stop_time-start_time).toSec());
    
    assert(!solution.empty());
    bool solution_found = (solution[0].size() != 0);
    ROS_INFO_STREAM("[GetPathService] - solution_found : " << solution_found);
    ROS_INFO_STREAM("[GetPathService] - solution[0].size() : " << solution[0].size());
    
    orunav_generic::Path path;
    
    for (std::vector<std::vector<Configuration*> >::iterator it = solution.begin(); it != solution.end(); it++)
    {
      for (std::vector<Configuration*>::iterator confit = (*it).begin(); confit != (*it).end(); confit++) {
        std::vector<vehicleSimplePoint> path_local = (*confit)->getTrajectory();
        
        for (std::vector<vehicleSimplePoint>::iterator it2 = path_local.begin(); it2 != path_local.end(); it2++) {
          double orientation = it2->orient;  
          
          orunav_generic::State2d state(orunav_generic::Pose2d(it2->x+map_offset_x,
                                                               it2->y+map_offset_y,
                                                               orientation), it2->steering, it2->steeringRear); //Cecchi_add
          
          //s << "[PathService] pose" << it2->steering << " " << it2->steeringRear <<" ";
          path.addState2dInterface(state);  
        }       
       
      }
    }
    // Cleanup
    delete pf;
    for (std::vector<std::vector<Configuration*> >::iterator it = solution.begin(); it != solution.end(); it++) {
      std::vector<Configuration*> confs = (*it);
      for (std::vector<Configuration*>::iterator confit = confs.begin(); confit != confs.end(); confit++) {
        delete *confit;
      }
      confs.clear();
    }
    solution.clear();
    // Cleanup - end
    
    if (path.sizePath() == 0){
      return false;
      //solution_found = false;
      }

    ROS_INFO_STREAM("[GetPathService] - Nb of path points : " << path.sizePath());
    
    double dist = sqrt(pow(path.getPose2d(0)(0)- tgt.start.pose.position.x,2) + pow(path.getPose2d(0)(1)- tgt.start.pose.position.y,2));
    double orient = abs(start_orientation- path.getPose2d(0)(2));    
    orient = abs(goal_orientation- path.getPose2d(path.sizePath()-1)(2));
    dist = sqrt(pow(path.getPose2d(path.sizePath()-1)(0)- tgt.goal.pose.position.x,2) + pow(path.getPose2d(path.sizePath()-1)(1)- tgt.goal.pose.position.y,2));
    
    if (WP::LOG_LEVEL == 0) {
        std::ofstream f;
        f.open(WP::AUX_FILE);
        f << "path start:  [" << path.getPose2d(0)(0) << ", " << path.getPose2d(0)(1)<< ", " << path.getPose2d(0)(2) << "]  - dist real:" << dist  << " orient error: "<< orient << std::endl;
        f << "path goal:  [" << path.getPose2d(path.sizePath()-1)(0) << ", " << path.getPose2d(path.sizePath()-1)(1) << ", "<< path.getPose2d(path.sizePath()-1)(2) <<  "]  - dist real:" << dist << " orient error: "<< orient << std::endl;
        f << "total length: " << path.getLength() << std::endl;
        f << "cuspidi: " << path.cuspidi(4) << std::endl;
        f << "path: "<< std::endl;
        for (unsigned int i = 0; i < path.sizePath(); i++) {
            f << path.getPose2d(i)(0) << " "
            << path.getPose2d(i)(1) << " "
            << path.getPose2d(i)(2) << " "
            << path.getSteeringAngle(i) << " "
            << path.getSteeringAngleRear(i) << std::endl; //Cecchi_add
        }	  
        f.close();
    }
  
    

    res.valid = solution_found;
    if (solution_found) {
      // First requirement (that the points are separated by a minimum distance).
      orunav_generic::Path path_min_dist = orunav_generic::minIncrementalDistancePath(path, min_incr_path_dist_);
      // Second requirment (path states are not allowed to change direction of motion without any intermediate points).
      orunav_generic::Path path_dir_change = orunav_generic::minIntermediateDirPathPoints(path_min_dist);
      res.path = orunav_conversions::createPathMsgFromPathInterface(path_dir_change);
      res.path.target_start = tgt.start;
      res.path.target_goal = tgt.goal;
     

      if (visualize_) {
        orunav_generic::Pose2d start_pose(tgt.start.pose.position.x,
                                          tgt.start.pose.position.y,
                                          tf::getYaw(tgt.start.pose.orientation));
        orunav_generic::Pose2d goal_pose(tgt.goal.pose.position.x,
                                         tgt.goal.pose.position.y,
                                         tf::getYaw(tgt.goal.pose.orientation));
        orunav_rviz::drawPose2d(start_pose, 0, 0, 1., "start_pose2d", marker_pub_);
        orunav_rviz::drawPose2d(goal_pose, 0, 2, 1., "goal_pose2d", marker_pub_);
        orunav_rviz::drawPose2dContainer(orunav_generic::minIncrementalDistancePath(path_dir_change, 0.2), "path_subsampled", 1, marker_pub_);
      }

      if (save_paths_) {
        orunav_generic::Path path = orunav_conversions::createPathFromPathMsg(res.path);
        std::stringstream st;
        st << "path_" << res.path.robot_id << "-" << res.path.goal_id << ".path";       
        orunav_generic::savePathTextFile(path, st.str());
      }
      return true;
    }
    
    return false;
    
  }

  void drawFootPrint(std::string name, double x, double y, double th) {
        vehicleSimplePoint p;
        double x_min = std::numeric_limits<double>::max(), x_max = std::numeric_limits<double>::min(), 
            y_min = std::numeric_limits<double>::max(), y_max = std::numeric_limits<double>::min();

        p.initVehicleSimplePoint(x, y, th, 0.0);
        std::vector<cellPosition*> cells;
        cells = vehicle_model_->getCellsOccupiedInPosition(&p);
        
        //Find the four vertices
        for (std::vector<cellPosition*>::iterator it = cells.begin(); it != cells.end(); it++ ) {
              if ( (*it)->y_cell > y_max ) y_max = (*it)->y_cell;
              if ( (*it)->y_cell < y_min ) y_min = (*it)->y_cell;
              if ( (*it)->x_cell > x_max ) x_max = (*it)->x_cell;
              if ( (*it)->x_cell < x_min ) x_min = (*it)->x_cell;
        }
        
        //Create the polygon and publish it
        visualization_msgs::Marker line_strip;
        line_strip.id = 0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.header.frame_id = "map";
        line_strip.header.stamp = ros::Time::now();
        line_strip.scale.x = 0.1;
        line_strip.color.g = 1.0;
        line_strip.color.a = 1.0;
        
        geometry_msgs::Point ptr, pbr, pbl, ptl;
        ptr.x = x_max;
        ptr.y = y_max;
        line_strip.points.push_back(ptr);
        
        pbr.x = x_max;
        pbr.y = y_min;
        line_strip.points.push_back(pbr);
        
        pbl.x = x_min;
        pbl.y = y_min;
        line_strip.points.push_back(pbl);

        ptl.x = x_min;
        ptl.y = y_max;
        line_strip.points.push_back(ptl);
        line_strip.points.push_back(ptr);
        
        marker_pub_.publish(line_strip);
  }

};



int main(int argc, char** argv) {

 ros::init(argc, argv, "get_path_service");
 ros::NodeHandle parameters("~");
 GetPathService gps(parameters, ros::this_node::getName());

 ros::spin();
}
