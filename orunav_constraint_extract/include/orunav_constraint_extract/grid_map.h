#pragma once

#include <ros/console.h>
#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>
#include <orunav_conversions/conversions.h>
#include <orunav_generic/pose2d.h>
#include <orunav_geometry/robot_model_2d.h>

namespace constraint_extract {

  class GridPatchIdx : public std::vector<Eigen::Vector2i> {

  };

  
 bool grid_path_idx_sort_incr_predicate(const Eigen::Vector2i &p1, const Eigen::Vector2i &p2)
  {
    return p1.squaredNorm() < p2.squaredNorm();
  }

inline void metricToPixelOccupancyGrid(const nav_msgs::OccupancyGrid &msg, const Eigen::Vector2d &position, Eigen::Vector2i &pixel) {
  
  double granularity = msg.info.resolution;
  orunav_generic::Pose2d map_offset = orunav_conversions::createPose2dFromMsg(msg.info.origin);
  
  pixel(0) = static_cast<int>((-map_offset(0) + position(0)) / granularity);
  pixel(1) = static_cast<int>((-map_offset(1) + position(1)) / granularity);
}
  
inline void pixelToMetricOccupancyGrid(const nav_msgs::OccupancyGrid &msg, const Eigen::Vector2i &pixel, Eigen::Vector2d &position ) {
  double granularity = msg.info.resolution;
  orunav_generic::Pose2d map_offset = orunav_conversions::createPose2dFromMsg(msg.info.origin);
  
  position(0) = (pixel(0)+0.5)*granularity + map_offset(0);
  position(1) = (pixel(1)+0.5)*granularity + map_offset(1);
}

 inline int distanceToPixel(const nav_msgs::OccupancyGrid &msg, double distance) {
   
   return static_cast<int>(distance/msg.info.resolution+0.5);
 }

inline bool validPixelOccupancyGrid(const nav_msgs::OccupancyGrid &map, const Eigen::Vector2i &pixel) {
  if (pixel(0) < 0 || pixel(0) >= (int)map.info.width) {
    ROS_INFO_STREAM_THROTTLE(1, "WE found a spurious x_pixel, (" << pixel(0) << ","<< pixel(1)<< ")");
    return false;
  }
  if (pixel(1) < 0 || pixel(1) >= (int)map.info.height) {
    ROS_INFO_STREAM_THROTTLE(1, "WE found a spurious y_pixel, (" << pixel(0) << ","<< pixel(1)<< ")");
    return false;
  }
  return true;
}

 inline bool validPixelOffsetOccupancyGrid(const nav_msgs::OccupancyGrid &map, const Eigen::Vector2i &pixel, const Eigen::Vector2i &offset) {
   return validPixelOccupancyGrid(map, Eigen::Vector2i(pixel(0)+offset(0),pixel(1)+offset(1)));
}

inline long getOccupancyGridIdx(const nav_msgs::OccupancyGrid &map, const Eigen::Vector2i &pixel) {
  return pixel(1)*map.info.width + pixel(0);
}

 inline long getOccupancyGridIdxOffset(const nav_msgs::OccupancyGrid &map, const Eigen::Vector2i &pixel, const Eigen::Vector2i &offset) {
   return (pixel(1)+offset(1))*map.info.width + pixel(0) + offset(0);
 }

 inline void setValueOnOccupancyGrid(nav_msgs::OccupancyGrid &map, int value) {
   for (size_t i = 0; i < map.info.width*map.info.height; i++) {
     map.data[i] = value;
   }
 }

 inline void clearOccupancyValueOnGrid(nav_msgs::OccupancyGrid &map) {
   setValueOnOccupancyGrid(map, 0);
 }


inline bool isOccupied(const nav_msgs::OccupancyGrid &map, const Eigen::Vector2i &pixel) {
  //assert(validPixelOccupancyGrid(map, pixel));
  if (!validPixelOccupancyGrid(map, pixel))
    return true;
  if (map.data[getOccupancyGridIdx(map, pixel)] >= 99 || map.data[getOccupancyGridIdx(map, pixel)] == -1) //was 51
    return true;
  return false;
}

 inline bool isValidAndOccupied(const nav_msgs::OccupancyGrid &map, const Eigen::Vector2i &pixel) {
   if (validPixelOccupancyGrid(map,pixel)) {
     if (map.data[getOccupancyGridIdx(map, pixel)] >= 99 || map.data[getOccupancyGridIdx(map, pixel)] == -1) { // was 51
       return true;
     }
   }
   return false;
 }

 inline bool isOccupiedGridPatchIdx(const nav_msgs::OccupancyGrid &map, const GridPatchIdx &idx) {
   for (unsigned int i = 0; i < idx.size(); i++) {
     if (isOccupied(map, idx[i]))
       return true;
   }
   return false;
 }

 inline GridPatchIdx getOccupiedGridPatchIdxFromPatchIdx(const nav_msgs::OccupancyGrid &map, const GridPatchIdx &idx) {
   GridPatchIdx ret;
   for (unsigned int i = 0; i < idx.size(); i++) {
     if (isOccupied(map, idx[i]))
       ret.push_back(idx[i]);
   }
   return ret;
 }

inline bool isOccupiedCollisionInterfaceWithBoundingBox(const nav_msgs::OccupancyGrid &map, const orunav_generic::Point2dCollisionCheckInterface &collision, const Eigen::Vector2d &bottomLeft, const Eigen::Vector2d &topRight) {
  Eigen::Vector2i bottom_left, top_right;
  metricToPixelOccupancyGrid(map, bottomLeft, bottom_left);
  metricToPixelOccupancyGrid(map, topRight, top_right);
  
  if (!validPixelOccupancyGrid(map, bottom_left))
    return true;
  if (!validPixelOccupancyGrid(map, top_right))
    return true;
  
  for (int x = bottom_left(0); x < top_right(0); x++) {
    for (int y = bottom_left(1); y < top_right(1); y++) {
      if (isOccupied(map, Eigen::Vector2i(x,y))) {
	// Check if we have a collision...
	Eigen::Vector2d tmp;
	
	pixelToMetricOccupancyGrid(map, Eigen::Vector2i(x,y), tmp);
	if (collision.collisionPoint2d(tmp))
	  return true;
      }
    }
  }
  return false;
}


// TODO, fix the erasing(!)
 inline void removeGridPatchIdx(GridPatchIdx &idx, unsigned int removeIdx) {
   //idx.erase(idx.begin() + removeIdx);
 }

 inline GridPatchIdx getOccupiedGridPatch(const nav_msgs::OccupancyGrid &map) {
   GridPatchIdx ret;
   for(unsigned int  x=0; x<map.info.width; x++) {
     for(unsigned int y=0; y<map.info.height; y++) {
       if(isOccupied(map, Eigen::Vector2i(x,y))) {
	 ret.push_back(Eigen::Vector2i(x,y));
       }
     }
   }
   return ret;
 }
 
 inline orunav_generic::Point2dVec getPoint2dVecFromGridPatchIdx(const nav_msgs::OccupancyGrid &map, const GridPatchIdx &idx) {
   orunav_generic::Point2dVec ret;
   for (unsigned int i = 0; i < idx.size(); i++) {
     
     if (validPixelOccupancyGrid(map, idx[i])) {
       Eigen::Vector2d pos;
       pixelToMetricOccupancyGrid(map, idx[i], pos);
       ret.push_back(pos);
     }
   }
   return ret;
 }

 inline GridPatchIdx getGridPatchIdxFromPoint2dContainerInterface(const nav_msgs::OccupancyGrid &map, const orunav_generic::Point2dContainerInterface &pts) {
   GridPatchIdx ret;
   for (unsigned int i = 0; i < pts.sizePoint2d(); i++) {
     Eigen::Vector2i pixel;
     metricToPixelOccupancyGrid(map, pts.getPoint2d(i), pixel);
     if (validPixelOccupancyGrid(map, pixel)) {
       ret.push_back(pixel);
     }
   }
   return ret;
 }


 inline void addOffsetToGridPatchIdx(const nav_msgs::OccupancyGrid &map, GridPatchIdx &idx, const Eigen::Vector2i &offset) {
   for (unsigned int i = 0; i < idx.size(); i++) {
     idx[i]+=offset;
     if (!validPixelOccupancyGrid(map, idx[i]))
       removeGridPatchIdx(idx, i);
   }
 }

 GridPatchIdx getGridPatchIdxOffset(const nav_msgs::OccupancyGrid &map, const GridPatchIdx &idx, const Eigen::Vector2i &offset) {
   GridPatchIdx ret = idx;
   addOffsetToGridPatchIdx(map, ret, offset);
   return ret;
}
 
 inline bool isOccupiedRobotModel2dWithState(const nav_msgs::OccupancyGrid &map, const orunav_geometry::RobotModel2dWithState &model) {
  Eigen::Vector2d bottom_left, top_right;
  orunav_geometry::getBoundingBox2d(model, bottom_left, top_right);
  
  return isOccupiedCollisionInterfaceWithBoundingBox(map, model, bottom_left, top_right);
}


 inline GridPatchIdx getCollisionInterfaceWithBoundingBoxGridPatchIdx(const nav_msgs::OccupancyGrid &map, const orunav_generic::Point2dCollisionCheckInterface &collision, const Eigen::Vector2d &bottomLeft, const Eigen::Vector2d &topRight) {
   GridPatchIdx ret;

  Eigen::Vector2i bottom_left, top_right;
  metricToPixelOccupancyGrid(map, bottomLeft, bottom_left);
  metricToPixelOccupancyGrid(map, topRight, top_right);
  
  for (int x = bottom_left(0); x < top_right(0); x++) {
    for (int y = bottom_left(1); y < top_right(1); y++) {
      Eigen::Vector2i pixel(x,y);
      
      Eigen::Vector2d tmp;
      pixelToMetricOccupancyGrid(map, pixel, tmp);
      if (collision.collisionPoint2d(tmp))
	ret.push_back(pixel);
    }
  }
  return ret;
}

 inline GridPatchIdx getPolygonGridPatchIdx(const nav_msgs::OccupancyGrid &map, const orunav_geometry::Polygon &poly) {
   GridPatchIdx ret;

   Eigen::Vector2i bottom_left(0,0);
   Eigen::Vector2i top_right(map.info.width,map.info.height);
   
  for (int x = bottom_left(0); x < top_right(0); x++) {
    for (int y = bottom_left(1); y < top_right(1); y++) {
      Eigen::Vector2i pixel(x,y);
      
      Eigen::Vector2d tmp;
      pixelToMetricOccupancyGrid(map, pixel, tmp);
      if (poly.collisionPoint2d(tmp))
	ret.push_back(pixel);
    }
  }
  return ret;
}



 inline GridPatchIdx getRobotModel2dWithStateGridPatchIdx(const nav_msgs::OccupancyGrid &map, const orunav_geometry::RobotModel2dWithState &model) {
   Eigen::Vector2d bottom_left, top_right;
   orunav_geometry::getBoundingBox2d(model, bottom_left, top_right);
   
   return getCollisionInterfaceWithBoundingBoxGridPatchIdx(map, model, bottom_left, top_right);
 }

// Note: the center of the circle is at pixel (0,0)
 inline GridPatchIdx getRadiusGridPatchIdx(const nav_msgs::OccupancyGrid &map, double radius) {
   int pixel_radius = distanceToPixel(map, radius);
   
   GridPatchIdx ret;

   for (int x = -pixel_radius; x <= pixel_radius; x++) {
     for (int y = -pixel_radius; y <= pixel_radius; y++) {
       if (x*x + y*y <= pixel_radius*pixel_radius) {
	 ret.push_back(Eigen::Vector2i(x,y));
       }
     }
   }
   return ret;
 }
 

 inline void drawGridPatchIdxOnOccupancyMap(const GridPatchIdx &idx, nav_msgs::OccupancyGrid &map, int occupancyValue = 100) {
   for (unsigned int i = 0; i < idx.size(); i++) {
     if (validPixelOccupancyGrid(map, idx[i]))
       map.data[getOccupancyGridIdx(map, idx[i])] = occupancyValue;
   }
 }

 inline void clearAllowedStates(const nav_msgs::OccupancyGrid &map, Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> &allowedStates, const GridPatchIdx &idx) {
   for (unsigned int i = 0; i < idx.size(); i++) {
     if (validPixelOccupancyGrid(map, idx[i]))
       allowedStates(idx[i](0), idx[i](1)) = false;
   }
 }

 inline void drawAllowedStatesOnOccupancyMap(const Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> &allowedStates, nav_msgs::OccupancyGrid &map, int occupancyValue = 100) {
   for(unsigned int  x=0; x<map.info.width; x++) {
     for(unsigned int y=0; y<map.info.height; y++) {
       if (allowedStates(x,y)) {
	 map.data[getOccupancyGridIdx(map, Eigen::Vector2i(x,y))] = 0;
	 continue;
       }
       map.data[getOccupancyGridIdx(map, Eigen::Vector2i(x,y))] = occupancyValue;
     }
   }
 }


#define N_THETA_INCREMENTS 180
#define N_PHI_INCREMENTS 1
struct StateInterval {
	bool allowed_theta_phi[N_THETA_INCREMENTS][N_PHI_INCREMENTS];
};


 inline bool traceLinePixelThIdxOnGrid(const Eigen::Matrix<StateInterval,Eigen::Dynamic,Eigen::Dynamic> &stateIntervals, const Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> &allowedStates, const Eigen::Vector2i &p1, const Eigen::Vector2i& p2, const Eigen::Vector2i &thIdxBounds) {
   int x = p1(0);
   int y = p1(1);
   int dx;
   int dy;

   while(x != p2(0) || y != p2(1)) {
	
	if(x >= (size_t)allowedStates.cols() || y>= (size_t)allowedStates.rows()) {
	  std::cout<<"traceLine : goes out of map at "<<x<<" "<<y<<"\n";
	    return false;
	}
	if(!allowedStates(x,y)) {
	  std::cout<<"traceLine : PASSES THROUGH OBST\n";
	    return false;
	}
	for(int i=thIdxBounds(0); i<thIdxBounds(1); i++) {
	  if(!stateIntervals(x,y).allowed_theta_phi[i][0]) {
	    std::cout<<"traceLine : PASSES THROUGH WRONG THETA [" << x << "," << y << "](" << thIdxBounds(0) << ", " << thIdxBounds(1) << ") : " << i << "\n";
	    return false;
	    }
	}
	
	dx = p2(0)-x;
	dy = p2(1)-y;
	dx = dx > 0 ? 1 : dx < 0 ? -1 : 0; 
	dy = dy > 0 ? 1 : dy < 0 ? -1 : 0; 
	x = x + dx;
	y = y + dy;
    }	
    return true;
}


 
 void computeGridThetaPatchIdx(const orunav_generic::Pose2d &origin, const nav_msgs::OccupancyGrid &map, const orunav_geometry::RobotModel2dInterface &model, const orunav_generic::RobotInternalState2d &internalState, std::vector<GridPatchIdx> &idx) {
   
   idx.resize(N_THETA_INCREMENTS);
   
   // The patches has the center in pixel coord (0,0).
   double theta_increment = 2*M_PI / (double) N_THETA_INCREMENTS;
   
   orunav_geometry::RobotModel2dWithState robot(model);
   
   orunav_generic::Pose2d pose = origin;
   for (unsigned int i = 0; i < N_THETA_INCREMENTS; i++) {
     pose(2) = origin(2) + i*theta_increment;
     robot.update(pose, internalState);
     idx[i] = getRobotModel2dWithStateGridPatchIdx(map, robot);
   }
 }

 inline void drawStateIntervalsOnOccupancyMap(const Eigen::Matrix<StateInterval,Eigen::Dynamic,Eigen::Dynamic> &stateIntervals,const Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> &allowedStates,unsigned int thetaIdx, unsigned int phiIdx, nav_msgs::OccupancyGrid &map, int occupancyValue = 100) {

   assert(thetaIdx < N_THETA_INCREMENTS);
   assert(phiIdx == 0);
   for(unsigned int  x=0; x<map.info.width; x++) {
     for(size_t y=0; y<map.info.height; y++) {
       if (allowedStates(x,y) == false) {
	 map.data[getOccupancyGridIdx(map, Eigen::Vector2i(x,y))] = occupancyValue;
	 continue;
       }
       if (stateIntervals(x,y).allowed_theta_phi[thetaIdx][phiIdx]) {
	 map.data[getOccupancyGridIdx(map, Eigen::Vector2i(x,y))] = 0;
       } else {
	 map.data[getOccupancyGridIdx(map, Eigen::Vector2i(x,y))] = occupancyValue;
       }
     }
   }
 }


// Note this do not currently handle different steering angles (PHI), for waste actuated vehicles the footprint will change based on the steering angle.
 void precomputeAllowedPositions(const nav_msgs::OccupancyGrid &map,
				 const orunav_geometry::RobotModel2dInterface &model,
				 const orunav_generic::RobotInternalState2d &internalState,
				 Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> &allowedStates, 
				 Eigen::Matrix<StateInterval,Eigen::Dynamic,Eigen::Dynamic> &stateIntervals) {
   
   
   allowedStates = Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> (map.info.width, map.info.height);
   stateIntervals = Eigen::Matrix<StateInterval,Eigen::Dynamic,Eigen::Dynamic> (map.info.width, map.info.height);

   allowedStates.setOnes();
   //double cell_diag = sqrt(2)*map.info.resolution;
   double robot_min_radius = orunav_geometry::getMinDistanceFromOrigin(model.getBoundingRegion(internalState));
   double robot_max_radius = orunav_geometry::getMaxDistanceFromOrigin(model.getBoundingRegion(internalState));

   GridPatchIdx robot_min_radius_patch_idx = getRadiusGridPatchIdx(map, robot_min_radius);
   GridPatchIdx robot_max_radius_patch_idx = getRadiusGridPatchIdx(map, robot_max_radius);
   std::vector<GridPatchIdx> robot_theta_patch_idx(N_THETA_INCREMENTS);

   orunav_generic::Pose2d map_offset = orunav_conversions::createPose2dFromMsg(map.info.origin);
   computeGridThetaPatchIdx(map_offset, map, model, internalState, robot_theta_patch_idx); // Map offset - this will compute the idx in pixel coords starting with (0,0) as its center.
   
   // First ignore the rotation.
   for(unsigned int  x=0; x<map.info.width; x++) {
     for(unsigned int y=0; y<map.info.height; y++) {
       if(isOccupied(map, Eigen::Vector2i(x,y))) {
	 allowedStates(x,y) = false;
	 // Use the minimum radius to cancel out all allowed states
	 clearAllowedStates(map, allowedStates, getGridPatchIdxOffset(map, robot_min_radius_patch_idx, Eigen::Vector2i(x,y)));
	 continue;
       }
       // If its not possible to have any state at all simply don't do anything here.
       if (!allowedStates(x,y)) {
       	 continue;
       }
       // Check if this can holds all rotations
       if (!isOccupiedGridPatchIdx(map, getGridPatchIdxOffset(map, robot_max_radius_patch_idx, Eigen::Vector2i(x,y)))) {
       	 memset(stateIntervals(x,y).allowed_theta_phi,true,sizeof(bool)*N_THETA_INCREMENTS*N_PHI_INCREMENTS);
       	 continue;
       }

       memset(stateIntervals(x,y).allowed_theta_phi,false,sizeof(bool)*N_THETA_INCREMENTS*N_PHI_INCREMENTS);
	 
       // Compute the different orientations - start by assuming no orientation is valid.
       for(int p=0; p<N_THETA_INCREMENTS; p++) {
	 for(int q=0; q<N_PHI_INCREMENTS; q++) {
	   if (isOccupiedGridPatchIdx(map, getGridPatchIdxOffset(map, robot_theta_patch_idx[p], Eigen::Vector2i(x,y)))) {
	     stateIntervals(x,y).allowed_theta_phi[p][q] = false;
	   } else {
	     stateIntervals(x,y).allowed_theta_phi[p][q] = true;
	   }
	 }
       }
     }
   }

 }


// TODO - these are somewhat redundant with the metricToPixelOccupancyGrid abouve.
inline Eigen::Vector2d metricToPixelOccupancyMap(const nav_msgs::OccupancyGrid &msg, const Eigen::Vector2d &position) {
  Eigen::Vector2d pixel;
  double granularity = msg.info.resolution;
  orunav_generic::Pose2d map_offset = orunav_conversions::createPose2dFromMsg(msg.info.origin);
  
  pixel(0) = static_cast<int>((-map_offset(0) + position(0)) / granularity + 0.5);
  pixel(1) = static_cast<int>((-map_offset(1) + position(1)) / granularity + 0.5);

  return pixel;
}

inline Eigen::Vector2d pixelToMetricOccupancyMap(const nav_msgs::OccupancyGrid &msg, const Eigen::Vector2d &pixel) {
  Eigen::Vector2d position;
  double granularity = msg.info.resolution;
  orunav_generic::Pose2d map_offset = orunav_conversions::createPose2dFromMsg(msg.info.origin);
  
  position(0) = pixel(0)*granularity + map_offset(0);
  position(1) = pixel(1)*granularity + map_offset(1);

  return position;
}

inline long getOccupancyMapIdx(const nav_msgs::OccupancyGrid &map, int pixelX, int pixelY) {
  assert(pixelX >= 0);
  assert(pixelX < (int)map.info.width);
  assert(pixelY >= 0);
  assert(pixelY < (int)map.info.height);
  return pixelY*map.info.width + pixelX;
}

inline void addPolygonToOccupancyMap(const orunav_geometry::Polygon &poly, nav_msgs::OccupancyGrid &msg, int occupancyValue) {
  // Compute the bounding box.
  Eigen::Vector2d bottom_left;
  Eigen::Vector2d top_right;
  orunav_geometry::getBoundingBox2d(poly, bottom_left, top_right);
  // Get the corresponding pixel values.
  
  Eigen::Vector2d bottom_left_pixel = metricToPixelOccupancyMap(msg, bottom_left);
  Eigen::Vector2d top_right_pixel = metricToPixelOccupancyMap(msg, top_right);
    
  for (int x = bottom_left_pixel(0); x <= top_right_pixel(0); x++) {
    for (int y = bottom_left_pixel(1); y <= top_right_pixel(1); y++) {
      // Back to metric again...
      Eigen::Vector2d pixel(x,y);
      if (poly.collisionPoint2d(pixelToMetricOccupancyMap(msg, pixel))) {
	msg.data[getOccupancyMapIdx(msg, x, y)] = occupancyValue;
      }
    }
  }
}



class GridMap {

 private:
    nav_msgs::OccupancyGrid map;

  

    public:
 	void assignMsg(const nav_msgs::OccupancyGridConstPtr &msg);

	//        void precomputeAllowedPositions(Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> &allowedState, 
	//		Eigen::Matrix<StateInterval,Eigen::Dynamic,Eigen::Dynamic> &stateIntervals) ;
	void printDebug() const;
 public:
	 EIGEN_MAKE_ALIGNED_OPERATOR_NEW	
};


 class ValidStatesGrid {
 private:
 public:
   

 };

} // namespace
