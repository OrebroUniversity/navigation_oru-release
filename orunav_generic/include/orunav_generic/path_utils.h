#pragma once

#include <orunav_generic/interfaces.h>
#include <orunav_generic/functions.h>
#include <angles/angles.h>
#include <cassert>
//#include <limits>       // std::numeric_limits

namespace orunav_generic
{
  
inline bool forwardPath(const PathInterface &path)
{
  assert(path.sizePath() > 1);
  Pose2d p1 = path.getPose2d(0);

  int fwd_counter = 0;
  int bwd_counter = 0;
  for (size_t i = 0; i < path.sizePath(); i++)
  {
    Pose2d p2 = path.getPose2d(i);
    if (getDistBetween(p1, p2) > 0.01)
    {
      if (getDirection(p1, p2) > 0)
        fwd_counter++;
      else
        bwd_counter++;
    }
  }
  if (fwd_counter > bwd_counter)
    return true;
  return false;
}

inline void getMinMaxSteeringAngle(const PathInterface &path, double &min, double &max) 
{
  max = -M_PI;
  min = M_PI;
  for (size_t i = 1; i < path.sizePath()-1; i++)
  {
    double steering = path.getSteeringAngle(i);
    if (steering < min)
      min = steering;
    if (steering > max)
      max = steering;
  }
}

inline void getIncrementalPathDiff(const PathInterface &path, double &maxDistIncr, double &maxThIncr, double &maxSteeringAngleIncr) {
  maxDistIncr = 0.;
  maxThIncr = 0.;
  maxSteeringAngleIncr = 0.;

  for (size_t i = 0; i < path.sizePath()-1; i++)
  {
    maxDistIncr = std::max(getDistBetween(path.getPose2d(i), path.getPose2d(i+1)), maxDistIncr);
    maxThIncr = std::max(fabs(getAngularNormDist(path.getPose2d(i), path.getPose2d(i+1))), maxThIncr);
    maxSteeringAngleIncr = std::max(fabs(path.getSteeringAngle(i) - path.getSteeringAngle(i+1)), maxSteeringAngleIncr);
  }
}
 
inline Path minIncrementalDistancePathIdx(const PathInterface &path, double minDist, std::vector<size_t> &idx)
{
  idx.clear();
  Path ret;
  if (path.sizePath() == 0)
    return ret;
  ret.addPathPoint(path.getPose2d(0), path.getSteeringAngle(0));
  idx.push_back(0);
  Pose2d back = path.getPose2d(path.sizePath()-1);
  for (size_t i = 1; i < path.sizePath()-1; i++)
  {
    double d1 = getDistBetween(ret.poses.back(), path.getPose2d(i));
    double d2 = getDistBetween(back, path.getPose2d(i));
    if (d1 > minDist && d2 > minDist) {
      ret.addPathPoint(path.getPose2d(i), path.getSteeringAngle(i));
      idx.push_back(i);
    }
  }
  size_t tmp = static_cast<size_t> (path.sizePath()-1);
  ret.addPathPoint(path.getPose2d(tmp), path.getSteeringAngle(tmp));
  idx.push_back(tmp);
  return ret;
}

inline Path minIncrementalDistancePath(const PathInterface &path, double minDist) {
  std::vector<size_t> idx;
  return minIncrementalDistancePathIdx(path, minDist, idx);
}

inline Path subSamplePath(const PathInterface &path, int skipNbPoints) 
{
  size_t last_i = 0;
  size_t size = path.sizePath();
  Path ret;
  if (size == 0)
    return ret;

  for (size_t i = 0; i < size; i += skipNbPoints) 
  {
    ret.addPathPoint(path.getPose2d(i), path.getSteeringAngle(i));
    last_i = i;
  }
  // Always make sure that the last point is added.
  if (last_i != size - 1 && size > 0) {
    ret.addPathPoint(path.getPose2d(size-1), path.getSteeringAngle(size-1));
  }
  return ret;
}


inline double getPathDistanceBetweenIndexes(const PathInterface &path, size_t start, size_t end) {
  if (path.sizePath() < 1)
    return 0.;
  if (end <= start)
    return 0.;
  assert(end < path.sizePath());
  double dist = 0.;
  for (size_t i = start; i < end-1; i++) {
    dist += getDistBetween(path.getPose2d(i), path.getPose2d(i+1));
  }
  return dist;
}

inline double getTotalDistance(const PathInterface &path)
{
  if (path.sizePath() < 1)
    return 0.;
  double dist = 0.;
  for (size_t i = 0; i < path.sizePath()-1; i++)
  {
    dist += getDistBetween(path.getPose2d(i), path.getPose2d(i+1));
  }
  return dist;
  //   return getPathDistanceBetweenIndexes(path, 0, path.sizePath()-1);
}

inline Path minIncrDistancePath(const PathInterface &path, double minDist)
{
  return  minIncrementalDistancePath(path, minDist);
}   

inline Path minIncrementalDistanceMinNbPointsPath(const PathInterface &path, double minDist, int minNbPoints) {
  double dist = getTotalDistance(path);
  double min_dist = dist / (1.*minNbPoints);
  if (min_dist > minDist)
    min_dist = minDist;
  return minIncrementalDistancePath(path, min_dist);
}



inline std::vector<size_t> calculateRequiredPathPointsIdx(const PathInterface &path)
{
  // Some trajectory points should always be keept. This holds for the starting and end but also if the direction changes along the path. Note that the direction calculations are needs to have at least som min distance traveled.
  std::vector<size_t> required_idx;
  assert(path.sizePath() >= 2);
  required_idx.push_back(0);
   
  bool dir = forwardDirection(path.getPose2d(0),path.getPose2d(1));
   
  for (size_t i = 1; i < path.sizePath()-1; i++)
  {
    if (dir != forwardDirection(path.getPose2d(i), path.getPose2d(i+1)))
    {
      required_idx.push_back(i);
      dir = !dir;
    }
  }
  required_idx.push_back(path.sizePath()-1);
  return required_idx;
}

inline bool getIdxFromEndMinDistance(const PathInterface &path, const double &minDist, size_t &idx, double &usedDistance) {
  std::vector<size_t> req_path_points = calculateRequiredPathPointsIdx(path);
   
  assert(req_path_points.size() > 1);
   
  size_t last_idx = req_path_points[req_path_points.size()-1];
  size_t second_req_idx = req_path_points[req_path_points.size()-2];

  idx = path.sizePath()-2;
  while (idx >= second_req_idx && idx >= 0) {
    usedDistance = getPathDistanceBetweenIndexes(path, idx, last_idx);
    if (usedDistance > minDist) {
      return true;
    }
    idx--;
  }
  return false;
}


inline int calculateNumberOfTravelDirectionalChanges(const PathInterface &path)
{
  std::vector<size_t> req_pts = calculateRequiredPathPointsIdx(path);
  return req_pts.size()-2; // 2 -> the start / end.
}

inline Path subSamplePathIncludingRequiredPathPoints(const PathInterface &path, int skipNbPoints) 
{
  size_t last_i = 0;
  size_t size = path.sizePath();
  Path ret;
  if (size == 0)
    return ret;

  std::vector<size_t> req_idx = calculateRequiredPathPointsIdx(path);
   
  size_t it = 0;
  for (size_t i = 0; i < size; i += skipNbPoints) 
  {
    if (it < req_idx.size()) {
      if (req_idx[it] == i) {
        it++;
      }
    }
    while (it < req_idx.size() && req_idx[it] < i) {
      ret.addPathPoint(path.getPose2d(i), path.getSteeringAngle(i));
      req_idx[it];
      i = it;
      last_i = i;
      continue;
    }

    ret.addPathPoint(path.getPose2d(i), path.getSteeringAngle(i));
    last_i = i;
  }
  // Always make sure that the last point is added.
  if (last_i != size - 1 && size > 0) {
    ret.addPathPoint(path.getPose2d(size-1), path.getSteeringAngle(size-1));
  }
  return ret;
}



inline Path calculateRequiredPathPoints(const PathInterface &path)
{
  // Some trajectory points should always be keept. This holds for the starting and end but also if the direction changes along the path. Note that the direction calculations needs to have at least som min distance traveled.
  Path ret;
  std::vector<size_t> required_idx = calculateRequiredPathPointsIdx(path);
  for (size_t i = 0; i < required_idx.size(); i++)
  {
    ret.addPathPoint(path.getPose2d(required_idx[i]), path.getSteeringAngle(required_idx[i]));
  }
  return ret;
}

inline Path getGoalStatesFromPaths(const std::vector<Path> &paths)
{
  Path ret;
  for (size_t i = 0; i < paths.size(); i++)
  {
    if (paths[i].sizePath() != 0) {
      size_t last = paths[i].sizePath() - 1;
      ret.addPathPoint(paths[i].getPose2d(last), paths[i].getSteeringAngle(last));
    }
  }
  return ret;
}

inline Path getStartStatesFromPaths(const std::vector<Path> &paths)
{
  Path ret;
  for (size_t i = 0; i < paths.size(); i++)
  {
    if (paths[i].sizePath() != 0) {
      ret.addPathPoint(paths[i].getPose2d(0), paths[i].getSteeringAngle(0));
    }
  }
  return ret;
}

inline Path getReversePath(const PathInterface& path)
{
  Path ret;
  for (size_t i = 0; i < path.sizePath(); i++)
  {
    Pose2d p = path.getPose2d(i);
    p(2) += M_PI;
    p(2) = angles::normalize_angle(p(2));
    ret.addPathPoint(p, -path.getSteeringAngle(i));
  }
  return ret;
}

inline Path getReversePathWithoutChangingDirection(const PathInterface &path)
{
  Path ret;
  size_t size = path.sizePath();
  for (size_t i = 0; i < size; i++)
  {
    Pose2d p = path.getPose2d(size-i-1);
    ret.addPathPoint(p, path.getSteeringAngle(size-i-1));
  }
  return ret;
}

// Return the last index if there is no distance left.
inline size_t getPathIdxWithGreaterDistance(const PathInterface &path, double distance,
                                     size_t start_idx) {
    double dist = 0.;
    size_t i = start_idx;
    while (dist < distance && i < path.sizePath()-1) {
      dist += getDistBetween(path.getPose2d(i), path.getPose2d(i+1));
      i++;
    }
    return i;
}

inline Path truncatePath(const PathInterface &path, size_t idx) {
  Path ret;
  for (size_t i = idx; i < path.sizePath(); i++) {
    ret.addPathPoint(path.getPose2d(i), path.getSteeringAngle(i));
  }
  return ret;
}

inline Path truncatePathEnd(const PathInterface &path, size_t idx) {
  Path ret;
  for (size_t i = 0; i < idx; i++) {
    ret.addPathPoint(path.getPose2d(i), path.getSteeringAngle(i));
  }
  return ret;
}

// The intermediate points exisit twice (when the change in direction occurs)
inline Path concatenateDirectionalPaths(const std::vector<Path> &paths)
{
  Path ret = paths[0];
  for (size_t i = 1; i < paths.size(); i++)
  {
    ret.poses.pop_back();
    ret.steeringAngles.pop_back();
    ret.poses.insert(ret.poses.end(), paths[i].poses.begin(), paths[i].poses.end());
    ret.steeringAngles.insert(ret.steeringAngles.end(), paths[i].steeringAngles.begin(), paths[i].steeringAngles.end());
  }
  return ret;
}


inline void removeThNormalization(PathInterface &path)
{
  double offset = 0.;
  for (size_t i = 1; i < path.sizePath(); i++)
  {
    double th_diff = path.getPose2d(i-1)(2) - path.getPose2d(i)(2) - offset;
    if (th_diff > M_PI) {
      offset += 2*M_PI;
    }
    if (th_diff < -M_PI) {
      offset -= 2*M_PI;
    }
    orunav_generic::Pose2d pose = path.getPose2d(i);
    pose(2) += offset;
    path.setPose2d(pose, i);
  }
}

inline orunav_generic::Path getThNormalizationFreePath(const PathInterface &path)
{
  orunav_generic::Path ret(path);
  removeThNormalization(ret);
  return ret;
}

inline Trajectory convertPathToTrajectoryWithoutModel(const PathInterface &path, double dt) 
{
  Trajectory traj;
  size_t size = path.sizePath();
  for (size_t i = 0; i < path.sizePath()-1; i++) 
  {
    const Pose2d &pose_current = path.getPose2d(i);
    const Pose2d &pose_next    = path.getPose2d(i+1);
    double v = getDistBetween(pose_current, pose_next)/dt;
    if (!forwardDirection(pose_current, pose_next)) {
      v = -v;
    }
    double w = (path.getSteeringAngle(i+1) - path.getSteeringAngle(i))/dt;
    traj.addTrajectoryPoint(pose_current, path.getSteeringAngle(i), v, w);
  }
  traj.addTrajectoryPoint(path.getPose2d(size-1), path.getSteeringAngle(size-1), 0., 0.);
  return traj;
}

inline void getMinMaxVelocities(const TrajectoryInterface &traj, double &minDriveVel, double &maxDriveVel, double &minSteeringVel, double &maxSteeringVel) {
  minDriveVel = 0.; maxDriveVel = 0.; minSteeringVel = 0.; maxSteeringVel = 0.;

  for (size_t i = 0; i < traj.sizeTrajectory(); i++) 
  {
    double v = traj.getDriveVel(i);
    double w = traj.getSteeringVel(i);

    if (v < minDriveVel)
      minDriveVel = v;
    if (w < minSteeringVel)
      minSteeringVel = w;
    if (v > maxDriveVel)
      maxDriveVel = v;
    if (w > maxSteeringVel)
      maxSteeringVel = w;
  }
}

//! Return a forward simulated path based on the trajectory, where the initial pose / steering angle is taken from the first trajectory point. (len is the length of the vehicle and dt the fixed time step).
inline orunav_generic::Path forwardSimulation(const TrajectoryInterface &traj, double len, double dt) {
  orunav_generic::Path path;
  assert(traj.sizeTrajectory() > 0);
  assert(len != 0);
  assert(dt > 0);
  orunav_generic::State2d current_state(traj, 0);
  
  path.addPathPoint(traj.getPose2d(0), traj.getSteeringAngle(0));

  for (size_t i = 0; i < traj.sizeTrajectory(); i++) {
    orunav_generic::Control ctrl(traj.getDriveVel(i), traj.getSteeringVel(i));;

    current_state.addControlStep(ctrl, len, dt);

    path.addState2dInterface(current_state);
  }
  assert(path.sizePath() == traj.sizePath()+1);

  return path;
}

inline void addPathToPath(Path &p, const PathInterface &p2) {
  for (size_t i = 0; i < p2.sizePath(); i++) {
    p.addPathPoint(p2.getPose2d(i), p2.getSteeringAngle(i));
  }
}

inline orunav_generic::Path selectPathIntervall(const orunav_generic::PathInterface &path, 
                                         size_t startIdx, 
                                         size_t stopIdx) {
  assert(stopIdx <= path.sizePath());
  assert(startIdx < stopIdx);
  orunav_generic::Path ret;
  for (size_t i = startIdx; i < stopIdx; i++) {
    ret.addPathPoint(path.getPose2d(i),path.getSteeringAngle(i));
  }
  return ret;
}

inline orunav_generic::Trajectory selectTrajectoryInterval(const TrajectoryInterface &traj,
						     size_t startIdx,
						     size_t stopIdx) {
  assert(stopIdx <= traj.sizePath());
  assert(startIdx < stopIdx);
  orunav_generic::Trajectory ret;
  for (size_t i = startIdx; i < stopIdx; i++) {
    ret.addTrajectoryPoint(traj.getPose2d(i),traj.getSteeringAngle(i), traj.getDriveVel(i), traj.getSteeringVel(i));
  }
  return ret;

}

inline orunav_generic::Trajectory selectTrajectoryIndexes(const TrajectoryInterface &traj,
						    std::vector<int> indexes) {
  orunav_generic::Trajectory ret;
  for (size_t i = 0; i < indexes.size(); i++) {
    ret.addTrajectoryPoint(traj.getPose2d(indexes[i]),traj.getSteeringAngle(indexes[i]), traj.getDriveVel(indexes[i]), traj.getSteeringVel(indexes[i]));
  }
  return ret;

 }

 
inline std::vector<Path> splitOnDistance(const PathInterface &path, double distance) {
  
  std::vector<Path> paths;
  size_t start_idx = 0;
  while (start_idx < path.sizePath()) {
 
    size_t end_idx = getPathIdxWithGreaterDistance(path, distance, start_idx);
    
    Path i_path = selectPathIntervall(path, start_idx, end_idx+1);
    paths.push_back(i_path);
    start_idx = end_idx+1;
  }
  return paths;
}

inline std::vector<Path> splitToDirectionalPaths(const PathInterface &path)
{
  assert(path.sizePath() > 2);
  std::vector<size_t> required_idx = calculateRequiredPathPointsIdx(path);
  std::vector<orunav_generic::Path> ret;
  for (size_t i = 0; i < required_idx.size()-1; i++)
  {
    size_t idx_start = required_idx[i];
    size_t idx_stop = required_idx[i+1];
      
    orunav_generic::Path sub_path;
    for (size_t j = idx_start; j <= idx_stop; j++)
    {
      sub_path.addPathPoint(path.getPose2d(j), path.getSteeringAngle(j));
    }
    ret.push_back(sub_path);
      
  }
  return ret;
}

inline int findState(const PathInterface &states, orunav_generic::State2dInterface &query, double maxDistOffset, double maxAngularOffset, double maxSteeringAngleOffset)
{
  int ret = -1;
  for (size_t i = 0; i < states.sizePath(); i++) {
    if (getDistBetween(query.getPose2d(), states.getPose2d(i)) < maxDistOffset) {
      if (fabs(getAngularNormDist(query.getPose2d(), states.getPose2d(i))) < maxAngularOffset) {
        if (fabs(angles::normalize_angle(query.getSteeringAngle() - states.getSteeringAngle(i))) < maxSteeringAngleOffset)
          ret = i;
      }
    }
  }
  return ret;
}

inline std::vector<std::pair<size_t, double> > getDistancesTo(const PathInterface &states, orunav_generic::State2dInterface &query) {
  std::vector<std::pair<size_t, double> > ret;
  for (size_t i = 0; i < states.sizePath(); i++) {
    ret.push_back(std::pair<size_t, double>(i, getDistBetween(query.getPose2d(), states.getPose2d(i)) + fabs(angles::normalize_angle(query.getSteeringAngle() - states.getSteeringAngle(i)))));
  }
  return ret;
}

inline bool sort_distance_pairs (std::pair<size_t, double> i, std::pair<size_t, double> j) { return (i.second <j.second); }

inline std::vector<std::pair<size_t, double> > findClosestStatesVec(const PathInterface &states, orunav_generic::State2dInterface &query) {
  std::vector<std::pair<size_t, double> > distances = getDistancesTo(states, query);
  // Sort them
  std::sort(distances.begin(), distances.end(), sort_distance_pairs);
  return distances;
}

inline void addUniqueStates(Path &states, const PathInterface &addStates, double maxDistOffset, double maxAngularOffset, double maxSteeringAngleOffset)
{
  for (size_t i = 0; i < addStates.sizePath(); i++) {
    State2d s(addStates, i);
    if (findState(states, s, maxDistOffset, maxAngularOffset, maxSteeringAngleOffset) < 0) { // Not found add it. 
      states.addState2dInterface(s);
    }
  }
}

inline double avgSqrDistancePathError(const PathInterface &p1, const PathInterface &p2) 
{
  size_t size = p1.sizePath();
  if (p2.sizePath() < size)
    size = p2.sizePath();
  
  double sqr_dist = 0.;
  for (size_t i = 0; i < size; i++)
  {
    double dist = orunav_generic::getDistBetween(p1.getPose2d(i), p2.getPose2d(i));
    std::cout << "[" << i << "] dist : " << dist << std::endl;
    sqr_dist += dist*dist;
  }
  return  sqr_dist / ((double)size);
}

inline double calcTrajectoryErrorFixedDt(const orunav_generic::TrajectoryInterface &traj, double len, double dt) {
  orunav_generic::Path p = forwardSimulation(traj, len, dt);
  //  orunav_generic::saveTrajectoryTextFile(traj, "dbg_t.txt");
  //  orunav_generic::savePathTextFile(p, "dbg_p.txt");
      
  return avgSqrDistancePathError(p, traj);
}

inline Trajectory setFixedControlValues(const orunav_generic::Path &path, double v, double w) {
  Trajectory traj;
  for (size_t i = 0; i < path.sizePath(); i++) 
  {
    traj.addTrajectoryPoint(path.getPose2d(i), path.getSteeringAngle(i), v, w);
  }
  return traj;
}

inline Trajectory setFixedControlValuesW(const orunav_generic::TrajectoryInterface &trajectory, double w) {
  Trajectory traj;
  for (size_t i = 0; i < traj.sizeTrajectory(); i++) 
  {
    traj.addTrajectoryPoint(trajectory.getPose2d(i), trajectory.getSteeringAngle(i), trajectory.getDriveVel(i), w);
  }
  return traj;
}

inline Path minIntermediateDirPathPointsIdx(const PathInterface &path, std::vector<size_t> &inter_idx) {
  // Need to compute the direction of change.
  inter_idx.clear();
  std::vector<bool> dir(path.sizePath()-1);
  for (size_t i = 0; i < path.sizePath()-1; i++) {
    dir[i] = forwardDirection(path.getPose2d(i), path.getPose2d(i+1));
  }
   
  // Check if we have any intermediate differences (e.g. false, false, true, false, false) - true is only occuring once.
  // Need to pick up (false, true, true...) as well as (..., true, true, false) etc.
  bool d = false;
  size_t last_change = 0;
  //  std::vector<size_t> inter_idx;
  for (size_t i = 0; i < dir.size(); i++) {
    if (i == 0)
      d = dir[0];
    else {
      if (d == dir[i]) {
        last_change++;
      }
      else {
        // Change occured
        if (last_change == 0) {
          // Here is one problem
          inter_idx.push_back(i);
          //	   std::cout << "found no intermp at : " << i << std::endl;
        }
        last_change = 0;
        d = dir[i];
      }
    }
  }
  // Sort out the last case (..., true, true, false).
  if (dir.size() > 1) {
    if (dir[dir.size()-1] != dir[dir.size()-2])
      inter_idx.push_back(dir.size());
  }

  if (inter_idx.empty()) {
    return Path(path);
  }

  Path ret;
  // Need to add in 'interpolated' points.
  size_t j = 0;
  for (size_t i = 0; i < path.sizePath(); i++) {
    if (i == inter_idx[j]) {
      // Add an interpolated one between i and the last path point.
      State2d last(path, i-1);
      State2d curr(path, i);
      State2d ds = orunav_generic::subState2d(last, curr);
      State2d interp_s = addState2d(last, ds.scale(0.5)); // 0.5 - the middle
      ret.addState2dInterface(interp_s);
      j++;
    }
    ret.addPathPoint(path.getPose2d(i), path.getSteeringAngle(i));
  }
  return ret;
  
}

inline Path minIntermediateDirPathPoints(const PathInterface &path) {
  std::vector<size_t> inter_idx;
  return minIntermediateDirPathPointsIdx(path, inter_idx);
}

inline bool validPathForTrajectoryProcessing(const orunav_generic::PathInterface &path) {
  orunav_generic::Path p = orunav_generic::minIntermediateDirPathPoints(path);
  return (p.sizePath() == path.sizePath());
}

inline TrajectoryChunks splitToTrajectoryChunks(const orunav_generic::TrajectoryInterface &traj, int splitStep)
{
  TrajectoryChunks ret;
  for (size_t i = 0; i < traj.sizeTrajectory(); i++)
  {
    if ((i % splitStep) == 0)
    {
      orunav_generic::Trajectory tr;
      ret.push_back(tr);
    }
    ret.back().addTrajectoryPoint(traj.getPose2d(i), traj.getSteeringAngle(i), traj.getDriveVel(i), traj.getSteeringVel(i));
  }
  return ret;
}

// Since the vehicle might go back and forth - we cannot simply use a closest distance metric.
// Here we instead assume that we call this function frequently, and update the idx when a the next
// chunks first pose is closest. This makes the current active chunk to be 0.5*60 ms * the length of each chunk ahead.
inline size_t getCurrentChunkIdx(const TrajectoryChunksInterface &chunks, const orunav_generic::Pose2d &pose, size_t previousIdx) 
{
  if (chunks.sizeChunks() == 0)
    return 0;

  if (!(chunks.sizeChunks() > previousIdx))
    return 0;

  double d1 = orunav_generic::getDistBetween(pose, chunks.getChunk(previousIdx).getPose2d(0));
  double d2 = d1+1.;

  if (previousIdx+1 < chunks.sizeChunks()) {
    d2 = orunav_generic::getDistBetween(pose, chunks.getChunk(previousIdx+1).getPose2d(0));
  }
  if (d2 < d1) {
    return previousIdx+1;
  }
  if (previousIdx+2 < chunks.sizeChunks()) {
    d2 = orunav_generic::getDistBetween(pose, chunks.getChunk(previousIdx+2).getPose2d(0));
  }
  if (d2 < d1) {
    return previousIdx+2;
  }

  return previousIdx;
}

// Same but for the path idx.
inline size_t getCurrentIdxSingleStep(const PathInterface &path, const orunav_generic::Pose2d &pose, size_t previousIdx) {
  if (path.sizePath() == 0)
    return 0;
  if (!(path.sizePath() > previousIdx))
    return 0;

  double d1 = orunav_generic::getDistBetween(pose, path.getPose2d(previousIdx));
  double d2 = d1+1.;
  //double d3 = d2+1.;
  // double d4 = d3+1.;
  // double d5 = d4+1.;

  if (previousIdx+1 < path.sizePath()) {
    d2 = orunav_generic::getDistBetween(pose, path.getPose2d(previousIdx+1));
  }
  if (d2 < d1) {
    return previousIdx+1;
  }
  // In-case we change direction.
  if (previousIdx+2 < path.sizePath()) {
    d2 = orunav_generic::getDistBetween(pose, path.getPose2d(previousIdx+2));
    if (d2 < d1) {
      return previousIdx+2;
    }
  }
  // These shouldn't be needed unless the path is e.g. moving back and forth in a "jerky" way
  // if (previousIdx+3 < path.sizePath()) {
  //   d3 = orunav_generic::getDistBetween(pose, path.getPose2d(previousIdx+3));
  //   if (d3 < d1) {
  //     return previousIdx+3;
  //   }
  // }
  // if (previousIdx+4 < path.sizePath()) {
  //   d4 = orunav_generic::getDistBetween(pose, path.getPose2d(previousIdx+4));
  //   if (d4 < d1) {
  //     return previousIdx+4;
  //   }
  // }
  // if (previousIdx+5 < path.sizePath()) {
  //   d5 = orunav_generic::getDistBetween(pose, path.getPose2d(previousIdx+5));
  //   if (d5 < d1) {
  //     return previousIdx+5;
  //   }
  // }

  return previousIdx;
}

// The path can be very dense. Make sure that we move until the distance starts to increase.
inline size_t getCurrentIdx(const PathInterface &path, const orunav_generic::Pose2d &pose, size_t previousIdx) {

  if (path.sizePath() == 0)
    return 0;
  if (!(path.sizePath() > previousIdx))
    return 0;
   
  size_t previous_idx = previousIdx;
  bool move_forward = true;
  while (move_forward) {
    size_t new_idx = getCurrentIdxSingleStep(path, pose, previous_idx);
    if (new_idx > previous_idx) {
      // Continue to move.
      previous_idx = new_idx;
    }
    else {
      // Stop
      return new_idx;
    }
  }
  assert(false);
  return 0;
}
 
// This function assumes that the chunks and path starts from the path stepIdx...
inline int getPathIdxUsingFirstPointInChunk(const PathInterface &path, const TrajectoryChunksInterface &chunks, size_t chunkIdx, double minValidDistance, size_t stepIdx) {
  // This uses the accumulated distance
  if (path.sizePath() == 0)
    return -1;
  if (chunks.sizeChunks() == 0)
    return -1;
  if (chunkIdx >= chunks.sizeChunks())
    return -1;
  double acc_chunk_dist = 0.;
  double acc_path_dist = 0.;
  for (size_t i = 0; i < chunkIdx; i++) {
    acc_chunk_dist += getTotalDistance(chunks.getChunk(i));
  }
  // Distance between the chunks...
  for (int i = 0; i < (int)(chunkIdx-1); i++) {
    // Last pose of the i chunk to the first pose of the i + chunk
    int last =chunks.getChunk(i).sizeTrajectory() -1;
    acc_chunk_dist += orunav_generic::getDistBetween(chunks.getChunk(i).getPose2d(last), chunks.getChunk(i+1).getPose2d(0));
  }
  for (int i = stepIdx; i < (int)(path.sizePath()-1); i++) {
    acc_path_dist += orunav_generic::getDistBetween(path.getPose2d(i), path.getPose2d(i+1));
    
    if (acc_path_dist/* + minValidDistance*/ > acc_chunk_dist)
      return i;
  }
  return -1;
}

//! Update the current chunk idx to the corresponding index in the future, given dt and future time.
inline void updateChunkIdxStepIdxGivenFutureTime(unsigned int &currentChunkIdx, unsigned int &currentStepIdx, unsigned int stepsPerChunk, double dt, double futureTime) {
  
  int step_inc = trunc(futureTime/(dt));
  int chunk_inc = trunc(step_inc / stepsPerChunk);

  step_inc = step_inc - chunk_inc*stepsPerChunk;

  currentChunkIdx += chunk_inc;
  currentStepIdx += step_inc;

  while (currentStepIdx >= stepsPerChunk) {
    currentChunkIdx++;
    currentStepIdx -= stepsPerChunk;
  }
}

//! Return a trajectory, if chunkIdxStop or stepIdxStop is bigger than the provided chunk data it will return the biggest trajectory up to this point.
inline orunav_generic::Trajectory trajectoryChunksInterfaceToTrajectory(const TrajectoryChunksInterface &chunks,
                                                           size_t chunkIdxStart, size_t stepIdxStart,
                                                           size_t chunkIdxStop, size_t stepIdxStop)
{
  assert(chunkIdxStart < chunks.sizeChunks());
  assert(chunkIdxStart <= chunkIdxStop);
  
  if (chunkIdxStop >= chunks.sizeChunks()) {
    chunkIdxStop = chunks.sizeChunks()-1;
    stepIdxStop = chunks.getChunk(chunkIdxStop).sizeTrajectory()-1;
  }

  orunav_generic::Trajectory ret;
  if (chunkIdxStart == chunkIdxStop) {
    const TrajectoryInterface &t = chunks.getChunk(chunkIdxStart);
    for (size_t i = stepIdxStart; i <= stepIdxStop; i++) {
      ret.addTrajectoryPoint(t.getPose2d(i), t.getSteeringAngle(i), t.getDriveVel(i), t.getSteeringVel(i)); 
    }
    return ret;
  }

  for (size_t i = chunkIdxStart; i <= chunkIdxStop; i++) {
    const TrajectoryInterface &t = chunks.getChunk(i);
    size_t step_start = 0;
    size_t step_end = t.sizeTrajectory()-1;
    if (i == chunkIdxStart)
      step_start = stepIdxStart;
    if (i == chunkIdxStop)
      step_end = stepIdxStop;
    
    for (size_t j = step_start; j <= step_end; j++) {
      ret.addTrajectoryPoint(t.getPose2d(j), t.getSteeringAngle(j), t.getDriveVel(j), t.getSteeringVel(j));
    }
  }
  return ret;
}

inline int getPathIdxUsingFirstPointInChunk2(const PathInterface &path, const TrajectoryChunksInterface &chunks, size_t chunkIdx, double minValidDistance, size_t stepIdx) {

  if (path.sizePath() == 0)
    return -1;
  if (chunks.sizeChunks() == 0)
    return -1;
  if (chunkIdx >= chunks.sizeChunks())
    return -1;

  State2d chunk_state(chunks.getChunk(chunkIdx), 0);
  int closest_idx = orunav_generic::findState(path, chunk_state, 0.01, 0.01, 0.01);
  // return cosest_idx;

  // Additional check using the accumulated distance
  orunav_generic::Path chunk_path;
  // Push back all the states until we reach chunkIdx.
  for (size_t i = 0; i < chunkIdx; i++) {
    orunav_generic::addPathToPath(chunk_path, chunks.getChunk(i));
  }
  // Add the last point (firt point in chunkIdx).
  chunk_path.addState2dInterface(chunk_state);
  
  // Compute the distance.
  double acc_chunk_dist = getTotalDistance(chunk_path);
  double acc_path_dist = 0.;

  for (int i = stepIdx; i < (int)(path.sizePath()-1); i++) {
    acc_path_dist += orunav_generic::getDistBetween(path.getPose2d(i), path.getPose2d(i+1));
    
    if (acc_path_dist + minValidDistance >= acc_chunk_dist) {
      
      // Ok, we're close to the real target... step 5 steps up / down and take the closest deal.
      if (abs(i - closest_idx) < 5) {
        return closest_idx;
      }
    }
  }
  return -1;
}

inline bool checkDirection(const orunav_generic::State2d &state2d, const orunav_generic::PathInterface &path, size_t idx) {
  if (!(idx+1 < path.sizePath()))
    return false;
  bool dir1 = orunav_generic::forwardDirection(state2d.getPose2d(), 
                                               path.getPose2d(idx));
  bool dir2 = orunav_generic::forwardDirection(path.getPose2d(idx),
                                               path.getPose2d(idx+1));
  if (dir1 == dir2)
    return true;
  return false;
}                                                                      

// This function assumes that the chunks and path starts from the path stepIdx...
inline int getPathIdxUsingFirstPointInChunk3(const PathInterface &path, const TrajectoryChunksInterface &chunks, size_t chunkIdx, double &distance) {
  // This uses the accumulated distance
  if (path.sizePath() == 0) {
    return -1;
  }
  if (chunks.sizeChunks() == 0) {
    return -1;
  }
  if (chunkIdx >= chunks.sizeChunks()) {
    return -1;
  }

  State2d chunk_state(chunks.getChunk(chunkIdx), 0);
  std::vector<std::pair<size_t, double> > res = findClosestStatesVec(path, chunk_state);

  // Check that the path is in the right direction (maybe the second closest is better).
  assert(res.size() > 2);
  
  size_t max_nb_checks = 2;
  size_t iter = 0;
  while (iter < max_nb_checks) {
    if (checkDirection(chunk_state, path, res[iter].first)) {
      break;
    }
    iter++;
  }
  if (iter == max_nb_checks)
    iter = 0;

  distance = res[iter].second;

  return (int)res[iter].first;
}



inline TrajectoryChunks appendChunks(const TrajectoryChunks &orig, size_t startIdx, const TrajectoryChunks &add) {
  TrajectoryChunks ret;
  for (size_t i = 0; i < startIdx; i++) {
    ret.push_back(orig[i]);
  }
  for (size_t i = 0; i < add.sizeChunks(); i++) {
    ret.push_back(add[i]);
  }
  return ret;
}

inline void addPose2dOffset(Pose2dContainerInterface &poses, orunav_generic::Pose2d &offset) {
  if (poses.sizePose2d() == 0)
    return;
  
  for (size_t i = 0; i < poses.sizePose2d(); i++) {
    Pose2d tmp = poses.getPose2d(i);
    poses.setPose2d(addPose2d(tmp, offset), i);
  }
}

inline void moveToOrigin(Pose2dContainerInterface &poses, const orunav_generic::Pose2d &origin) {
  if (poses.sizePose2d() == 0)
    return;
  
  for (size_t i = 0; i < poses.sizePose2d(); i++) {
    Pose2d tmp = poses.getPose2d(i);
    poses.setPose2d(addPose2d(origin, tmp), i);
  }
}

//! Move the path to make the first path entry be at the origin (0,0,0) - this will affect x, y, theta.
inline void setFirstPoseAsOrigin(Pose2dContainerInterface &poses) {
  if (poses.sizePose2d() == 0)
    return;

  Pose2d origin = poses.getPose2d(0);

  for (size_t i = 0; i < poses.sizePose2d(); i++) {
    Pose2d tmp = poses.getPose2d(i);
    poses.setPose2d(subPose2d(origin, tmp), i);
  }
}

inline int getMinDistIdxToPath(const PathInterface &path, const Pose2d &pose) {
  double min_dist = std::numeric_limits<double>::max();
  int min_dist_idx = -1;
  for (size_t i = 0; i < path.sizePath(); i++) {
    double dist = orunav_generic::getDistBetween(path.getPose2d(i), pose);
    if (min_dist > dist) {
      min_dist = dist;
      min_dist_idx = i;
    }
  }
  return min_dist_idx;
}


//! Return the time to reach chunk_idx given the current state
inline double timeToReachChunkIdx(const orunav_generic::TrajectoryChunks &chunks,
                           int chunkIdx,
                           const orunav_generic::State2d &state)
{
  // Start from the chunk idx and iterate backwards and take the minimum distance between the chunks to the states (each chunk step is 0.60 ms).
  // Here we assume that the chunkIdx is in the future(!)
  if (chunkIdx -1 < 0)
    return 0.; // Shouldn't happen.

  double min_dist = std::numeric_limits<double>::max();
  int min_dist_idx = -1;
  int min_chunk_idx = chunkIdx;
  int chunk_idx = chunkIdx;
   
  orunav_generic::Pose2d pose = state.getPose2d();

  while (chunk_idx > 0) {
    chunk_idx--;

    for (size_t i = 0; i < chunks.getChunk(chunk_idx).sizeTrajectory(); i++) {
      double dist = orunav_generic::getDistBetween(chunks.getChunk(chunk_idx).getPose2d(i), pose);
      if (min_dist > dist) {
        min_dist = dist;
        min_dist_idx = i;
        min_chunk_idx = chunk_idx;
      }
       
    } 
  }

  // Ok, closest distance found att chunk: chunkIdx and index : min_dist_idx
  std::cout << "offset : chunkIdx :" << chunkIdx << std::endl;
  std::cout << "offset : min_chunk_idx : " << min_chunk_idx << std::endl;
  std::cout << "offset : min_dist_idx :"  << min_dist_idx << std::endl;
  double time = (chunkIdx - min_chunk_idx) * 0.6 - min_dist_idx * 0.06;
  return time;
}

//! This is currently used for fixing straight segments in the wef file - yes it looks a bit funny but don't change it.
inline orunav_generic::Path createStraightPath(const orunav_generic::Pose2d &start, 
                                        const orunav_generic::Pose2d &goal,
                                        double resolution)
{
  // Compute a straight line path between the points.
  orunav_generic::Path path;

  int steps = static_cast<int>(1 / resolution);
  
  // TODO, need to check that the poses are connected through a line.

  Eigen::Vector3d inc = (goal-start)*resolution;
  steps = steps - 2;
  
  path.addPathPoint(start, 0.); // Assume zero steering angle here.
  for (int i = 0; i < steps; i++) {
    path.addPathPoint(start + i*inc, 0.);
  }
  path.addPathPoint(goal, 0.);

  return path;
}

//! This really improves the convergence rate of the ACADO optimization finding small path snippets (currently used for computing the path to final pallet pickup / lift location).
inline orunav_generic::Path createStraightPathFromStartPose(const orunav_generic::Pose2d &start,
                                                     const orunav_generic::Pose2d &goal,
                                                     double resolution) {
  // Compute a straight line path between the points.
  orunav_generic::Path path;
   
  int steps = static_cast<int>(1 / resolution);
   
  double distance = orunav_generic::getDistBetween(start, goal);
  if (!orunav_generic::forwardDirection(start, goal))
    distance *= -1;

  orunav_generic::Pose2d offset(distance, 0, 0);
  orunav_generic::Pose2d new_goal = orunav_generic::addPose2d(start, offset);
  Eigen::Vector3d diff = subPose2d(start, new_goal);
  Eigen::Vector3d inc = diff*resolution;
      
  path.addPathPoint(start, 0.);
  for (int i = 1; i <= steps; i++) {
    orunav_generic::Pose2d p = orunav_generic::addPose2d(start, i*inc);
    path.addPathPoint(p, 0.);
  }
  path.addPathPoint(new_goal, 0.);
   
  return path;
}


inline orunav_generic::Control computeAvgSqrControlDifference(const orunav_generic::TrajectoryInterface &ref, 
                                                       const orunav_generic::TrajectoryInterface &exec) {
  double dv_sum = 0;
  double sv_sum = 0;
  for (size_t i = 0; i < ref.sizeTrajectory(); i++) {
    double ddv = ref.getDriveVel(i) - exec.getDriveVel(i);
    double dsv = ref.getSteeringVel(i) - exec.getSteeringVel(i);
     
    dv_sum += ddv*ddv;
    sv_sum += dsv*dsv;
  }
  orunav_generic::Control c;
  c.v = dv_sum / (ref.sizeTrajectory()*1.);
  c.w = sv_sum / (ref.sizeTrajectory()*1.);
  return c;
}

inline bool validPath(const orunav_generic::PathInterface &path, double maxSteeringAngle) {
  if (path.sizePath() == 0)
    return false;
  // Simply check for nans...
  for (size_t i = 0; i < path.sizePath(); i++) {
    const orunav_generic::Pose2d& pose = path.getPose2d(i);
    if (std::isnan(pose[0]) || std::isnan(pose[1]) || std::isnan(pose[2])) {
      std::cerr << "nan in pose[" << i << "] : " << pose << std::endl;
      return false;
    }
    if (std::isnan(path.getSteeringAngle(i))) {
      std::cerr << "nan in steeringAngle[" << i << "] : " << pose << std::endl;
      return false;
    }
  }
  double min, max;
  getMinMaxSteeringAngle(path, min, max);
  if (fabs(min) > maxSteeringAngle) {
    std::cerr << "min steering angle < -" << maxSteeringAngle << std::endl;
    return false;
  }
  if (fabs(max) > maxSteeringAngle) {
    std::cerr << "max steering angle > " << maxSteeringAngle << std::endl;
    return false;
  }

  
  return true;
}

inline bool validPathIncrSteps(const orunav_generic::PathInterface &path, double maxDistIncr, double maxThIncr, double maxSteeringAngleIncr) {
  double max_dist_incr, max_th_incr, max_steering_angle_incr;
  
  getIncrementalPathDiff(path, max_dist_incr, max_th_incr, max_steering_angle_incr);
  if (max_dist_incr > maxDistIncr)
    return false;
  if (max_th_incr > maxThIncr)
    return false;
  if (max_steering_angle_incr > maxSteeringAngleIncr)
    return false;
  return true;
}


inline bool validSmoothedPath(const orunav_generic::PathInterface &path, 
                       const orunav_generic::State2d &start,
                       const orunav_generic::State2d &goal,
                       double maxSteeringAngle,
                       double maxDistOffset,
                       double maxHeadingOffset) {
  if (!validPath(path, maxSteeringAngle)) {
    std::cerr << "invalid smoothed path - see ^" << std::endl;
    return false;
  }
  
  // Check that the start - end pose is ok.
  if (!orunav_generic::validPose2dDiff(start.getPose2d(), path.getPose2d(0), maxDistOffset, maxHeadingOffset)) {
    std::cerr << "invalid smoothed path -> start pose difference to large" << std::endl;
    return false;
  }
  
  if (!orunav_generic::validPose2dDiff(goal.getPose2d(), path.getPose2d(path.sizePath()-1), maxDistOffset, maxHeadingOffset)) {
    std::cerr << "invalid smoothed path -> goal pose difference to large" << std::endl;
    return false;
  }
  
  return true;
}

// Rather specialized function - use the end part of path to make a backward forward path.
inline orunav_generic::Path getRepositioningPath(const orunav_generic::PathInterface &path,
                                          const double &dist, double &usedDist) {

  orunav_generic::Path ret;

  if (path.sizePath() == 0)
    return ret;

  // Need to get the distance...
  int idx = -1;
  
  int i = path.sizePath()-2;
  while (i >= 0) {
    double d = getPathDistanceBetweenIndexes(path, i, path.sizePath()-1);
    if (getPathDistanceBetweenIndexes(path, i, path.sizePath()-1) > dist) {
      idx = i;
      usedDist = dist;
      break;
    }
    i--;
    if (i < 0) {
      usedDist = d;
      idx = 0;
    }
  }
  
  
  // Need to traverse backwards first.
  orunav_generic::Path snippet_backward = getReversePathWithoutChangingDirection(selectPathIntervall(path, idx+1, path.sizePath()));

  orunav_generic::Path snippet_forward = selectPathIntervall(path, idx, path.sizePath());

  // Glue these things together
  addPathToPath(ret, snippet_backward);
  addPathToPath(ret, snippet_forward);

  return ret;
}



//! Return the total amount of turning in path
inline double getTotalTurning(const orunav_generic::PathInterface &path) {
  double tot = 0.;
  for (size_t i = 0; i < path.sizePath()-1; i++)
  {
    tot += fabs(path.getSteeringAngle(i) - path.getSteeringAngle(i+1));
  }
  return tot;
}


inline int getDockingPathIdx(const orunav_generic::PathInterface &path, 
                      size_t minIdx,
                      double minDockingDistance,
                      double maxDockingDistance) {
  
  std::vector<size_t> required_idx = calculateRequiredPathPointsIdx(path);
  assert(required_idx.size() > 1);
  
  size_t last_idx = required_idx.back();
  size_t idx = required_idx[required_idx.size()-2];
  
  if (idx < minIdx)
    idx = minIdx;
  
  while (idx < last_idx) {
    double dist = getDistBetween(path.getPose2d(idx), path.getPose2d(last_idx));
    
    if (dist > minDockingDistance && dist < maxDockingDistance)
      return (int)idx;
    
    idx++;
  }
  return -1;
}
 
inline void makeValidPathForTrajectoryProcessing(orunav_generic::Path &path) {
  path = orunav_generic::minIncrementalDistancePath(path, 0.00002);
  path = orunav_generic::minIntermediateDirPathPoints(path);
  //  path = orunav_generic::minIncrementalDistancePath(path, 0.00002);
}

inline void minIncrementalDistancePathCoordinatedTimes(orunav_generic::Path &path, 
                                                orunav_generic::CoordinatedTimes &cts, 
                                                double minDistance)
{
  std::vector<size_t> idx;
  path = orunav_generic::minIncrementalDistancePathIdx(path, minDistance, idx);

  if (cts.empty())
    return;

  orunav_generic::CoordinatedTimes c;
  for (size_t i = 0; i < idx.size(); i++) {
    if (idx[i] < cts.size()) // Force the path and cts size elsewhere.
      c.push_back(cts[idx[i]]);
  }
  cts = c;
}

inline void minIntermediateDirPathCoordinatedTimesPoints(orunav_generic::Path &path,
                                                   orunav_generic::CoordinatedTimes &cts)
{
  std::vector<size_t> inter_idx;
  path = orunav_generic::minIntermediateDirPathPointsIdx(path, inter_idx);
  if (inter_idx.empty())
    return;
  
  if (cts.empty())
    return;

  orunav_generic::CoordinatedTimes c;
  size_t j = 0;
  for (size_t i = 0; i < cts.size(); i++) {
    if (i == inter_idx[j]) {
      // Need to add a time point.
      c.push_back(-1.);
      j++;
    }
    c.push_back(cts[i]);
  }
  cts = c;
}

// There are some requirement on the path to perform trajectory computations (might be removed in the future though). Since the path and the constraints are directly connected we need to perform all changes to both.
inline void makeValidPathCoordinatedTimesForTrajectoryProcessing(orunav_generic::Path &path, orunav_generic::CoordinatedTimes &cts) {
  // Make sure that there is some distance separation.
  minIncrementalDistancePathCoordinatedTimes(path, cts, 0.00002);
  // Check that there is atleast two point with the same direction.
  minIntermediateDirPathCoordinatedTimesPoints(path, cts);
}

inline double coordinationPairTimeStep(const orunav_generic::CoordinatedTimes &cts, size_t idx) {
  if (cts.size() > idx) {
    return cts[idx+1]-cts[idx];
  }
  return -1.;
}


} // namespace

