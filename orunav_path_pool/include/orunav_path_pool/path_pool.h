#ifndef PATH_POOL_HH
#define PATH_POOL_HH

#include <orunav_generic/interfaces.h>
#include <orunav_path_pool/wef_utils.h>


class PathPool
{
public:
  PathPool() { }
  void addPath(const orunav_generic::Path &path) {
    paths_.push_back(path);
  }

  void loadPathDir(const std::string pathName) {
    paths_ = orunav_generic::loadPathDirTextFile(pathName);
  }

  void loadWefFile(const std::string fileName, double length, double resolution) {
    WefPathParser weffer(fileName);
    paths_ = weffer.getPaths(length, resolution);
  }

  //! WARNING, this function only looks at a single path at the time - use the ASTAR search if needed.
  bool findPath(orunav_generic::Path &p, const orunav_generic::Pose2d &start, const orunav_generic::Pose2d &goal, double maxDistOffset, double maxAngleOffset ) const
  {
    for (unsigned int i = 0; i < paths_.size(); i++)
      {
	const orunav_generic::Pose2d &p_start = paths_[i].getPose2d(0);
	if (orunav_generic::getDistBetween(start, p_start) > maxDistOffset)
	  continue;
	if (fabs(angles::normalize_angle(start(2) - p_start(2))) > maxAngleOffset)
	  continue;

	const orunav_generic::Pose2d &p_goal = paths_[i].getPose2d(paths_[i].sizePath()-1);
    
	if (orunav_generic::getDistBetween(goal, p_goal) > maxDistOffset)
	  continue;
	if (fabs(angles::normalize_angle(goal(2) - p_goal(2))) > maxAngleOffset)
	  continue;
	
	// Found the path.
	p = paths_[i];
	return true;
      }
    return false;
  }

//! WARNING, this function only looks at a single path at the time - use the ASTAR search if needed.
  orunav_generic::Paths findPaths(const orunav_generic::Pose2d &start, const orunav_generic::Pose2d &goal, double maxDistOffset, double maxAngleOffset ) const
  {
    orunav_generic::Paths ret;
    for (unsigned int i = 0; i < paths_.size(); i++)
      {
	const orunav_generic::Pose2d &p_start = paths_[i].getPose2d(0);
	if (orunav_generic::getDistBetween(start, p_start) > maxDistOffset)
	  continue;
	if (fabs(angles::normalize_angle(start(2) - p_start(2))) > maxAngleOffset)
	  continue;

	const orunav_generic::Pose2d &p_goal = paths_[i].getPose2d(paths_[i].sizePath()-1);
    
	if (orunav_generic::getDistBetween(goal, p_goal) > maxDistOffset)
	  continue;
	if (fabs(angles::normalize_angle(goal(2) - p_goal(2))) > maxAngleOffset)
	  continue;
	
	  // Found a path.
	  ret.push_back(paths_[i]);
	    }
    return ret;
  }
  
  

  const std::vector<orunav_generic::Path> &getPaths() const { return paths_; }

private:
  std::vector<orunav_generic::Path> paths_;
  double minDist_, minAngleDist_;
};


#endif
