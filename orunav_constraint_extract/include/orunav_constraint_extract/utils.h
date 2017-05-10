#pragma once

#include <orunav_constraint_extract/polygon_constraint.h>
#include <orunav_generic/path_utils.h>

namespace constraint_extract {

  //! Is the feaisble pose inside the constraint.
  inline bool validConstraintWithPose(const constraint_extract::PolygonConstraint &pc, const orunav_generic::Pose2d &pose) {
#if 0
    if (pc.getThBounds()[0] > pose(2)) {
      std::cerr << "[validConstraintCheck]: pc.getThBounds()[0] > pose(2) " << pc.getThBounds()[0] << ", " << pose(2) << std::endl;
      return false;
    }
    if (pc.getThBounds()[1] < pose(2)) {
      std::cerr << "[validConstraintCheck]: pc.getThBounds()[1] < pose(2)" << pc.getThBounds()[1] << ", " << pose(2) << std::endl;
      return false;
    }

    std::vector<double> A0, A1, b;
    pc.getInnerConstraint().getMatrixFormAsVectors(A0, A1, b);
    assert(A0.size() == A1.size());
    assert(A0.size() == b.size());
    size_t size = A0.size();

    for (size_t i = 0; i < size; i++) {
      if (A0[i]*pose(0) + A1[i]*pose(1) > b[i]) {
    	std::cerr << "[validConstraintCheck]: i : " << i << std::endl;
    	std::cerr << "A0 : " << A0[i] << std::endl;
    	std::cerr << "x : " << pose(0) << std::endl;
    	std::cerr << "A0*x : " << A0[i]*pose(0) << std::endl;
    	std::cerr << "A1 : " << A1[i] << std::endl;
    	std::cerr << "y : " << pose(1) << std::endl;
    	std::cerr << "A1*y : " << A1[i]*pose(1) << std::endl;
    	std::cerr << "A0*x+A1*y : " << A0[i]*pose(0) + A1[i]*pose(1) << std::endl;
    	std::cerr << "b : " << b[i] << std::endl;
    	return false;
      }
    }
    return true;
#else
    return pc.isFeasible(pose);
#endif
  }
  
  //! Is the feaisble pose instide the constraint.
  inline bool validConstraint(const constraint_extract::PolygonConstraint &pc) {
    return validConstraintWithPose(pc, pc.feasiblePose_);
  }

  //! Given two consecutive constraints with corresponding feasible poses from a path. Check if there is an overlap between them.
  inline bool validConstraintPathOverlap(const constraint_extract::PolygonConstraint &pc1, const orunav_generic::Pose2d &pose1,
					 const constraint_extract::PolygonConstraint &pc2, const orunav_generic::Pose2d &pose2) {
    
    // Check 1, is pose1 contained in pc2?
    if (validConstraintWithPose(pc2, pose1))
      return true;
    if (validConstraintWithPose(pc1, pose2))
      return true;

    return false;
  }

  // The returned index will indicate that the pair<idx, idx+1> has no overlap.
  std::vector<int> getInvalidConstraintPathOverlap(const constraint_extract::PolygonConstraintsVec &pcs, const orunav_generic::PathInterface &path) {
    std::vector<int> invalid_idx;
    int end_idx = path.sizePath()-1;
    for (int i = 0; i < end_idx; i++) {
      if (!validConstraintPathOverlap(pcs[i], path.getPose2d(i),
				      pcs[i+1], path.getPose2d(i+1)
				      )) {
	invalid_idx.push_back(i);
      }
    }
    return invalid_idx;
  }

  void printConstraintDistances(const constraint_extract::PolygonConstraintsVec &pcs, const orunav_generic::Pose2dContainerInterface &poses) {
    std::cout << "--Constraint distances--" << std::endl;
    assert(pcs.size() == poses.sizePose2d());
    for (size_t i = 0; i < pcs.size(); i++) {
      //      std::cout << "--------------------------------------------------------------------" << std::endl;
      std::cout << "[" << i << "] - pos : " << pcs[i].positionConstraintDistance(poses.getPose2d(i)) 
		<< " ang : " << pcs[i].angularConstraintDistance(poses.getPose2d(i)) 
		<< " feasible : " << pcs[i].isFeasible(poses.getPose2d(i))
		<< " active : " << pcs[i].isActive(poses.getPose2d(i)) << std::endl;
 
      //      pcs[i].printDebug();
      //      std::cout << "path pose : " << path.getPose2d(i) << std::endl;
    }
  }

  //! Finds the constraints that has the maximum distance to boundaries (this treats radians and meters the same way(!)).
  PolygonConstraint getMaximumDistanceConstraintToPose(const constraint_extract::PolygonConstraintsVec &pcs, const orunav_generic::Pose2d &pose) {
    assert(pcs.size() > 0);
    
    size_t max_idx = 0;
    double max = std::min(pcs[0].positionConstraintDistance(pose), 
			  pcs[0].angularConstraintDistance(pose));
    for (size_t i = 1; i < pcs.size(); i++) {
      double tmp = std::min(pcs[i].positionConstraintDistance(pose),
    			    pcs[i].angularConstraintDistance(pose));
      if (tmp > max) {
    	max = tmp;
    	max_idx = i;
      }
    }
    return pcs[max_idx];
  }
  
  //! Recompute a new constraint vector to a path by maximize the distance to boundaries.
  PolygonConstraintsVec reassignNewConstraintsToPose2dInterface(const constraint_extract::PolygonConstraintsVec &pcs,
								const orunav_generic::Pose2dContainerInterface &poses) {
    PolygonConstraintsVec constraints;
    for (size_t i = 0; i < poses.sizePose2d(); i++) {
      constraints.push_back(getMaximumDistanceConstraintToPose(pcs, poses.getPose2d(i)));
    }
    return constraints;
  }
							      
  void savePositionConstraintsTextFile(const constraint_extract::PolygonConstraintsVec &pcs, const std::string &fileName) {
    std::ofstream ofs(fileName.c_str());
    for (size_t i = 0; i < pcs.size(); i++) {
      ofs << orunav_generic::getPoint2dContainerInterfaceGnuplotString(pcs[i].getInnerConstraint(), true);
      ofs << std::endl; // Create an extra line - used also with gnuplot to separate the constraints drawn.
    }
    ofs.close();
  }

  // Fills a file with the constraints along with a corresponding path to be able to plot the th bounds in a good way.
  void saveAngularConstraintsTextFile(const constraint_extract::PolygonConstraintsVec &pcs,
				      const orunav_generic::PathInterface &path,
				      const std::string &fileName) {
    assert(pcs.size() == path.sizePath());
    std::ofstream ofs(fileName.c_str());
    for (size_t i = 0; i < pcs.size(); i++) {
      ofs << pcs[i].getThBounds()(0) << " " << pcs[i].getThBounds()(1) << " " << path.getPose2d(i)(0) << " " << path.getPose2d(i)(1) << " " << path.getPose2d(i)(2) << std::endl;
    }
  }
  

  void saveConstraintsTextFiles(const constraint_extract::PolygonConstraintsVec &pcs, 
				const orunav_generic::PathInterface &path,
				const std::string &fileNamePrefix) {
    {
      std::stringstream st;
      st << fileNamePrefix << "position.constraints.dat";
      std::string file_name = st.str();
      savePositionConstraintsTextFile(pcs, file_name);
    }
    {
      std::stringstream st;
      st << fileNamePrefix << "angular.constraints.dat";
      std::string file_name = st.str();
      saveAngularConstraintsTextFile(pcs, path, file_name);
    }
  }

orunav_geometry::Polygon computeSweepArea(const orunav_generic::PathInterface &path,
                                          const orunav_geometry::RobotModel2dInterface &model,
                                          const orunav_generic::RobotInternalState2d &state)
{
  orunav_geometry::Polygon sweep_area;
  orunav_geometry::RobotModel2dWithState robot(model);
  for (size_t i = 0; i < path.sizePath(); i++) {
    robot.update(path.getPose2d(i), state);
    sweep_area.addPolygon(robot.getPosePolygon());
  }
  return sweep_area;
}

void minIncrementalDistancePathConstraints(orunav_generic::Path &path, 
                                           constraint_extract::PolygonConstraintsVec &constraints, 
                                           double minDistance)
{
  std::vector<size_t> idx;
  path = orunav_generic::minIncrementalDistancePathIdx(path, minDistance, idx);

  constraint_extract::PolygonConstraintsVec c;
  for (size_t i = 0; i < idx.size(); i++) {
    if (idx[i] < constraints.size()) // Force the path and constraints size elsewhere.
      c.push_back(constraints[idx[i]]);
  }
  constraints = c;
}

void minIntermediateDirPathConstraintsPoints(orunav_generic::Path &path,
                                             constraint_extract::PolygonConstraintsVec &constraints)
{
  std::vector<size_t> inter_idx;
  path = orunav_generic::minIntermediateDirPathPointsIdx(path, inter_idx);
  if (inter_idx.empty())
    return;
  
  constraint_extract::PolygonConstraintsVec c;
  size_t j = 0;
  for (size_t i = 0; i < constraints.size(); i++) {
    if (i == inter_idx[j]) {
      // Need to add a constraint.
      PolygonConstraint p;
      p.thBounds_[0] = -M_PI;
      p.thBounds_[1] = M_PI;
      c.push_back(p);
      j++;
    }
    c.push_back(constraints[i]);
  }
  constraints = c;
}

// There are some requirement on the path to perform trajectory computations (might be removed in the future though). Since the path and the constraints are directly connected we need to perform all changes to both.
void makeValidPathConstraintsForTrajectoryProcessing(orunav_generic::Path &path, constraint_extract::PolygonConstraintsVec &constraints) {
  // Make sure that there is some distance separation.
  minIncrementalDistancePathConstraints(path, constraints, 0.00002);
  // Check that there is atleast two point with the same direction.
  minIntermediateDirPathConstraintsPoints(path, constraints);
}

//! Find the index to the constraints with the "largest" distance, return -1 if non is valid.
int getConstraintsIdxWithLargestDistance(const constraint_extract::PolygonConstraintsVec &constraints, const orunav_generic::Pose2d &pose, double &distance) {

  distance = -1.;
  int max_idx = -1;
  assert(constraints.size() > 0);
  for (size_t i = 0; i < constraints.size(); i++) {
    if (constraints[i].isFeasible(pose)) {
      double d = constraints[i].angularConstraintDistance(pose) + constraints[i].positionConstraintDistance(pose);
      if (d > distance) {
        max_idx = i;
        distance = d;
      }
    }
  }
  return max_idx;
}

constraint_extract::PolygonConstraintsVec
reassignConstraints(const orunav_generic::Path &path, const constraint_extract::PolygonConstraintsVec &constraints) {
 
  constraint_extract::PolygonConstraintsVec ret;
  for (size_t i = 0; i < path.sizePath(); i++) {
    orunav_generic::Pose2d pose = path.getPose2d(i);
    double distance;
    int idx = getConstraintsIdxWithLargestDistance(constraints, pose, distance);
    if (idx < 0) {
      ret.push_back(constraints[i]); // should not happen
    }
    else {
      ret.push_back(constraints[idx]);
    }
    
  }
  return ret;
}


} // namespace
