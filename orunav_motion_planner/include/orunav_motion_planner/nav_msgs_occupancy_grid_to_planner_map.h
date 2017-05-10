#pragma once

#include <orunav_motion_planner/PathFinder.h>
#include <nav_msgs/OccupancyGrid.h>
#include <orunav_generic/types.h>
#include <orunav_conversions/conversions.h>

orunav_generic::Pose2d getNavMsgsOccupancyGridOffset(const nav_msgs::OccupancyGridConstPtr &msg) {
  return orunav_conversions::createPose2dFromMsg(msg->info.origin);
}

orunav_generic::Pose2d getNavMsgsOccupancyGridOffsetRef(const nav_msgs::OccupancyGrid &msg) {
  return orunav_conversions::createPose2dFromMsg(msg.info.origin);
}

// void convertNavMsgsOccupancyGridToWorldOccupancyMap(const nav_msgs::OccupancyGridConstPtr& msg, WorldOccupancyMap &map)
// {
//   double granularity = msg->info.resolution;
//   int xcells = msg->info.width;
//   int ycells = msg->info.height;


  
//   std::vector<std::vector<double> > occupancyMap;
//   occupancyMap.resize(msg->info.height);
  
//   unsigned int k = 0;
//   for (unsigned int i = 0; i < msg->info.height; i++)
//     {
//       occupancyMap[i].resize(msg->info.width);
//       for (unsigned int j = 0; j < msg->info.width; j++)
// 	{
// 	  occupancyMap[i][j] = msg->data[k]*0.01;
// 	  k++;
// 	}
//     }
//   map.initialize(xcells, ycells, granularity, occupancyMap);
// }

void convertNavMsgsOccupancyGridToWorldOccupancyMapRef(const nav_msgs::OccupancyGrid& msg, WorldOccupancyMap &map)
{
  double granularity = msg.info.resolution;

  int xcells = msg.info.width;
  int ycells = msg.info.height;
  
  std::vector<std::vector<double> > occupancyMap;
  occupancyMap.resize(msg.info.height);
  
  unsigned int k = 0;
  for (unsigned int i = 0; i < msg.info.height; i++)
    {
      occupancyMap[i].resize(msg.info.width);
      for (unsigned int j = 0; j < msg.info.width; j++)
	{
	  occupancyMap[i][j] = msg.data[k]*0.01;
	  k++;
	}
    }
  
  map.initialize(xcells, ycells, granularity, occupancyMap);
}
