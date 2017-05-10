#pragma once

#include <visualization_msgs/Marker.h>
#include <orunav_vehicle_execution/vehicle_state.h>
#include <orunav_rviz/orunav_rviz.h>

// Simple container class handle visualization of vehicle states, trajectories, etc. simply to clearn up the vehicle execution node.
class VehicleStateRviz() {

 public:
  void clear() { trajectory.clear(); pathTime.clear(); }
  orunav_generic::Trajectory trajectory;
  orunav_generic::PositionVec pathTime;
};


void drawDebugText(const VehicleState &vs, ros::Publisher &pub) {
  orunav_rviz::drawText(vs.getCurrentState2d().getPose2d(), 
                        vs.getDebugStringExtended(),
                        "state", 1, 1, 0.8, 0.3, marker_pub_);

}

void drawChunkIdx(const VehicleState &vs, size_t idx, std::string &name, ros::Publisher &pub) {
  orunav_generic::TrajectoryChunks chunks = vehicle_state_.getTrajectoryChunks();
  if (!chunks.empty()) {
    if (idx < chunks.size()) {
      orunav_generic::Pose2d p = vs.getTrajectoryChunks().getChunk(idx);
      orunav_rviz::drawPose2d(p, 1, 1, 2., name, marker_pub_);
    }
  }
      
  
  }
    orunav_generic::Pose p = vehicle_state_.getTrajectoryChunks().getChunk(vehicle_state_.getCurrentTrajectoryChunkIdx());
    
}

void drawEBrakeAreaForkSide(const VehicleState &vs, ros::Publisher &pub) {
  // Should be a polygon based on the trajectory the next 2 secounds.
  

}

void drawSlowDownAreaForkSide(const VehicleState &vs, ros::Publisher &pub) {

}
