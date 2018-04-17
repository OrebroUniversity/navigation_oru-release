#pragma once

#include <orunav_trajectory_processor/trajectory_processor_naive_ct.h>


// No previous trajectory.
std::pair<unsigned int, orunav_generic::TrajectoryChunks> computeTrajectoryChunksCASE3(const VehicleState &vs, const TrajectoryProcessor::Params &params, bool useCts) {
  
  orunav_generic::Path path = vs.getPath();
  
  TrajectoryProcessorNaiveCT gen;
  gen.addPathInterface(path);
  if (useCts) {
    orunav_generic::CoordinatedTimes cts = vs.getCoordinatedTimes();
    gen.addCoordinatedTimes(cts);
  }
  gen.setParams(params);
  orunav_generic::Trajectory traj = gen.getTrajectory();
  gen.printDebug();
  std::pair<unsigned int, orunav_generic::TrajectoryChunks> ret;
  ret.first = 0;
  ret.second = splitToTrajectoryChunks(traj, 10);
  
  return ret;
}
    

// Vehicle stopped and trajectory not completed (recover after a break)
std::pair<unsigned int, orunav_generic::TrajectoryChunks> computeTrajectoryChunksCASE1(VehicleState &vs, const TrajectoryProcessor::Params &params, unsigned int &pathIdx, bool useCts) {

  std::pair<unsigned int, orunav_generic::TrajectoryChunks> ret;
  orunav_generic::Path path = vs.getPath();

  // Cannot assume that the path is exactly the same as previous.
  // 1), need to find the path index to fit the current pose
  // 2), add the current pose as the first pose in the path
  // 3), we're really only into the trajectory -> simply truncate the path.
  // 4), recompute the trajectory.
  {
    orunav_generic::State2d state2d = vs.getCurrentState2d();
    std::vector<std::pair<size_t, double> > res = orunav_generic::findClosestStatesVec(path, state2d); 
    // Check that the path is in the right direction (maybe the second closest is better).
    assert(res.size() > 2);
    
    size_t max_nb_checks = 2;
    size_t iter = 0;
    while (iter < max_nb_checks) {
      if (checkDirection(state2d, path, res[iter].first)) {
        break;
      }
      iter++;
    }
    if (iter == max_nb_checks)
      iter = 0;
    pathIdx = res[iter].first;

    // Great - simply add the state2d to the path and generate the trajectory.
    path.setState2d(state2d, res[iter].first);

    // TODO - need to update and add CT.
    
    if (useCts) {
      orunav_generic::CoordinatedTimes cts = orunav_generic::truncateCts(vs.getCoordinatedTimes(), res[iter].first);
      vs.setCoordinatedTimes(cts);
    }
    
    path = truncatePath(path, res[iter].first);
  }

  // Check that the path is still valid...
  if (path.sizePath() < 3) {
    return ret;
  }
  TrajectoryProcessorNaiveCT gen;
  gen.addPathInterface(path);
  gen.setParams(params);
  orunav_generic::Trajectory traj = gen.getTrajectory();

  ret.first = 0;
  ret.second = splitToTrajectoryChunks(traj, 10);
  
  return ret;
}

// Vehicle is driving.
std::pair<unsigned int, orunav_generic::TrajectoryChunks> computeTrajectoryChunksCASE2(const VehicleState &vs, const TrajectoryProcessor::Params &params, const unsigned int &chunkIdx, unsigned int &pathIdx, double &pathChunkDistance, bool &valid, bool useCts) {

  valid = true;
  orunav_generic::Path path = vs.getPath();
  orunav_generic::CoordinatedTimes cts;
  if (useCts) {
    cts = vs.getCoordinatedTimes();
  }

  const orunav_generic::TrajectoryChunks& current_chunks = vs.getTrajectoryChunksRef();

  std::cout << "chunkIdx : " << chunkIdx << std::endl;
  std::cout << "current_chunks.size() : " << current_chunks.size() << std::endl;

  // 1), need to find the path index to fit the chunk_idx
  // 2), add the current pose as the first pose in the path
  // 3), we're really only into the trajectory -> simply truncate the path.
  // 4), recompute the trajectory.
  {
    orunav_generic::State2d state2d(current_chunks.getChunk(chunkIdx), 0);
    std::vector<std::pair<size_t, double> > res = orunav_generic::findClosestStatesVec(path, state2d); 
    // Check that the path is in the right direction (maybe the second closest is better).
    assert(res.size() > 2);
    
    size_t max_nb_checks = 2;
    size_t iter = 0;
    while (iter < max_nb_checks) {
      if (checkDirection(state2d, path, res[iter].first)) {
        break;
      }
      iter++;
    }
    if (iter == max_nb_checks)
      iter = 0;
    pathIdx = res[iter].first;
    pathChunkDistance = res[iter].second;
    
    // TODO add a sanity check here.
    // Need atleast the path to contain 2 points to continue from here... (this needs to be indenpentent on the chunk idx which due to slow speed could have much higher density)
    ROS_INFO_STREAM("pathIdx : " << pathIdx << " (" << path.sizePath() << ")");
    if ((pathIdx+3)> path.sizePath()) {
      valid = false;
      ROS_WARN_STREAM("To close to the end (not enough path points left in the path(!))");
      // return something empty...
      return std::pair<unsigned int, orunav_generic::TrajectoryChunks>();
    }
    ROS_INFO_STREAM("pathChunkDistance : " << pathChunkDistance);
    
    // CTS flipover check, this means that we have got a path index that is past the cts. 
    // Set back the path idx by keeping the cts and path point as second point, the current state2d will be set as the first entry below.
    if (!cts.empty()) {
      if (pathIdx > 1 && pathIdx < cts.size()-3) {
	if (cts[pathIdx] > 0.) {
	  ROS_ERROR_STREAM("================ CTS flipover check ==============");
	  pathIdx = pathIdx -1;
	}
      }
    }

    // Great - simply add the state2d to the path and generate the trajectory.
    path.setState2d(state2d, pathIdx);

    path = truncatePath(path, pathIdx);
    // need to update and add CT.
    if (!cts.empty()) {
      ROS_INFO_STREAM("BEFORE TRUNC cts.size() : " << cts.size());
      cts = truncateCts(cts, pathIdx);
      ROS_INFO_STREAM("AFTER TRUNC cts.size() : " << cts.size());
    }
    
    orunav_generic::makeValidPathCoordinatedTimesForTrajectoryProcessing(path,cts);
    ROS_INFO_STREAM("AFTER VALID cts.size() : " << cts.size());
  }

  TrajectoryProcessorNaiveCT gen;
  gen.addPathInterface(path);

  //gen.setStartIdxTime(ros::Time::now().toSec() + 0.5); // cts[0]
  // cts[0] -> the time to reach the chunkIdx.
  
  if (!cts.empty()) {
    cts[0] = vs.timeWhenChunkIdxIsReached(chunkIdx);
    ROS_INFO_STREAM("Time to reach chunkIdx " << chunkIdx << " : " << cts[0]);
    if (cts[0] < 0.) {
      ROS_ERROR("Invalid time to reach chunk.");
    }
  }
  gen.addCoordinatedTimes(cts);

  TrajectoryProcessor::Params params_local = params;
  params_local.debug = true; // Will generate a set of gnuplot files if enabled, check the gnuplot folder.
  params_local.debugPrefix = std::string("trajgen_CASE2/");
  gen.setParams(params_local);
  {
    orunav_generic::Control c;
    c.v = current_chunks.getChunk(chunkIdx).getDriveVel(0);
    c.w = current_chunks.getChunk(chunkIdx).getSteeringVel(0);
    gen.addControlConstraintPointAsStart(0, c);
  }

  gen.printDebug();
  orunav_generic::Trajectory traj = gen.getTrajectory();

  std::pair<unsigned int, orunav_generic::TrajectoryChunks> ret;
  ret.first = chunkIdx;
  ret.second = splitToTrajectoryChunks(traj, 10);
  
  return ret;
}

double getTargetStartGoalDistance(const orunav_msgs::RobotTarget &target) {
  orunav_generic::State2d start = orunav_conversions::createState2dFromPoseSteeringMsg(target.start);
  orunav_generic::State2d goal = orunav_conversions::createState2dFromPoseSteeringMsg(target.goal);
  return orunav_generic::getDistBetween(start.getPose2d(), goal.getPose2d());
}

bool getRepositioningPathUsingDrivenPath(orunav_generic::Path &path, const orunav_generic::PathInterface &drivenPath, double repositioningDistance) {
  
  if (drivenPath.sizePath() == 0)
    return false;
  
  double used_dist;
  path = orunav_generic::getRepositioningPath(drivenPath, repositioningDistance, used_dist);
  
  if (path.sizePath() == 0) {
    ROS_WARN("PathPlannerMRNode: Failed to find reposition path");
    return false;
  }
  return true;
}

bool getRepositioningPathMsgUsingDrivenPath(orunav_msgs::Path &path, const orunav_msgs::RobotTarget &target, const orunav_generic::PathInterface &drivenPath, double repositioningDistance) {
  orunav_generic::Path p;
  if (!getRepositioningPathUsingDrivenPath(p, drivenPath, repositioningDistance)) {
    return false;
  }
  path = orunav_conversions::createPathMsgFromPathInterface(p);
  path.target_start = target.start;
  path.target_goal = target.goal;
  return true;
}

