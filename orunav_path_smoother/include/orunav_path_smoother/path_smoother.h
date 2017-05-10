#pragma once

#include <orunav_generic/interfaces.h>
#include <orunav_generic/path_utils.h>
#include <orunav_geometry/b_spline2d.h>

#include <orunav_generic/random.h>

std::vector<unsigned int> generateRandomVecWithRep(int min,
						   int max,
						   unsigned int length,
						   unsigned int seed)
{
  std::vector<unsigned int> ret;
  if (min >= max)
    return ret;
  orunav_generic::UniformIntWithoutRep random(min, max, seed);
  for (unsigned int i = 0; i < length; i++)
    {
      ret.push_back(static_cast<unsigned int>(random.drawSample()));
    }
  std::sort(ret.begin(), ret.end());

  return ret;
}

class PathSmootherInterface {
  orunav_generic::Path smooth(const orunav_generic::PathInterface &path);
  //  orunav_generic::Path smooth(const orunav_generic::PathInterface &path, const orunav_generic::ConstraintInterface &constraints);
};


//! Smooths a path, note that this only smoothes a segment of the path and does not handles change in driving direction. The last and first poses in the path will always be present in the resulting path.
class PathSmoother
{
 public:
  enum Status { ALL_PATHS_SMOOTHED=4, FIRST_PATH_SMOOTHED=2, SOME_PATHS_SMOOTHED=1, NO_PATHS_SMOOTHED=0 };

  class Params
  {
  public:
    Params () 
      {
	maxNbIters = 100;
	initialKnotOffset = 1.;
	nbKnotOffsetValues = 4;
	maxNbPoses = 10;
	resolution = 0.1;
	steeringWheelDistOffset = 0.68;
	collisionResolution = 0.1;
	minSteeringAngle = -1.57;
	maxSteeringAngle = 1.57;
	tryWithoutDirectionPoints = false;
      }
    
    int maxNbIters;
    double initialKnotOffset;
    int nbKnotOffsetValues;
    int maxNbPoses;
    double resolution;
    double steeringWheelDistOffset; 
    double collisionResolution;
    double minSteeringAngle;
    double maxSteeringAngle;
    bool tryWithoutDirectionPoints;
    friend std::ostream& operator<<(std::ostream &os, const PathSmoother::Params &obj)
      {
	os << "\nmaxNbIters               : " << obj.maxNbIters;
	os << "\ninitialKnotOffset        : " << obj.initialKnotOffset;
	os << "\nnbKnotOffsetValues       : " << obj.nbKnotOffsetValues;
	os << "\nmaxNbPoses               :  " << obj.maxNbPoses;
	os << "\nresolution               : " << obj.resolution;
	os << "\nsteeringWheelDistOffset  : " << obj.steeringWheelDistOffset;
	os << "\ncollisionResolution      : " << obj.collisionResolution;
	os << "\nminSteeringAngle         : " << obj.minSteeringAngle;
	os << "\nmaxSteeringAngle         : " << obj.maxSteeringAngle;
	os << "\ntryWithoutDirectionPoints: " << obj.tryWithoutDirectionPoints;
	return os;
      }
    
  };
  

  bool smooth(const orunav_generic::PathInterface &path, const orunav_generic::CollisionCheckInterface &constraint);

  void setAllKnots(const orunav_generic::PathInterface &path);

  const BSpline2d& getBSpline2d() const { return spline_; }
  
  PathSmoother::Params params;

 private:
  std::vector<int> getNbSamplesForEachIter() const;

  orunav_generic::Path subSample(const orunav_generic::PathInterface &path, unsigned int maxNbPoints, unsigned int seed) const;

  BSpline2d spline_;
  unsigned int seed_;
};

namespace path_smoother
{
  void printStatus(int status) {
    if (status & PathSmoother::ALL_PATHS_SMOOTHED) {
      std::cout << "ALL_PATHS_SMOOTHED " << std::flush;
    }
    if (status & PathSmoother::FIRST_PATH_SMOOTHED) {
      std::cout << "FIRST_PATH_SMOOTHED " << std::flush;
    }
    if (status & PathSmoother::SOME_PATHS_SMOOTHED) {
      std::cout << "SOME_PATHS_SMOOTHED " << std::flush;
    }
    if (status & PathSmoother::NO_PATHS_SMOOTHED) {
      std::cout << "NO_PATHS_SMOOTHED " << std::flush;
    }
    std::cout << std::endl;
  }

  orunav_generic::Path smoothPathSplines(const PathSmoother::Params &params, const orunav_generic::PathInterface &path, const orunav_generic::CollisionCheckInterface &constraint, int &status, std::vector<BSpline2d> &splines)
    {
      status = 0;
      // First check, try to calculate a smoothed path path without force the change of traversal direction points.
      if (params.tryWithoutDirectionPoints)
	{
	  std::cout << "+++ smoothing path without direction points " << std::endl;

	  PathSmoother smoother;
	  smoother.params = params;
	  orunav_generic::Path smoothed_path;
	  if (smoother.smooth(path, constraint)) 	{
	    // Smoothing was successfull
	     std::cout << "+++ smoothing path without direction points - successfull " << std::endl;
	    const BSpline2d &spline = smoother.getBSpline2d();
	    spline.calcPath(smoothed_path, params.steeringWheelDistOffset, params.resolution);
	    splines.push_back(spline);
	    status = (PathSmoother::ALL_PATHS_SMOOTHED | PathSmoother::FIRST_PATH_SMOOTHED);
	    return smoothed_path;
	  }
	  std::cout << "+++ smoothing path without direction points - fail " << std::endl;
	}

      std::vector<orunav_generic::Path> sub_paths = orunav_generic::splitToDirectionalPaths(path);
      std::vector<orunav_generic::Path> smoothed_sub_paths;

      // There are still some strange behaviour of the path planner...
      // In addition to the sub_paths, we need to divide them by skipping certain required change direction points.
      

      bool all_path_smoothed = true;
      bool one_path_smoothed = false;
      for (unsigned int i = 0; i < sub_paths.size(); i++)
	{
	  std::cout << "+++ smoothing path : " << i << " (" << sub_paths.size() << ")" << std::endl;

	  PathSmoother smoother;
      
	  smoother.params = params;
	  orunav_generic::Path smoothed_path;
	  if (smoother.smooth(sub_paths[i], constraint)) 	{
	    // Smoothing was successfull
	    std::cout << "+++ smoothing path : " << i << " (" << sub_paths.size() << ") - successfull" << std::endl;
	    if (i == 0)
	      status |= PathSmoother::FIRST_PATH_SMOOTHED;
	    const BSpline2d &spline = smoother.getBSpline2d();
	    spline.calcPath(smoothed_path, params.steeringWheelDistOffset, params.resolution);
	    splines.push_back(spline);
	    one_path_smoothed = true;
	  }
	  else {
	    // Failure - use the old path. 
	    std::cout << "+++ smoothing path : " << i << " (" << sub_paths.size() << ") - fail" << std::endl;
	    all_path_smoothed = false;
	    smoothed_path = sub_paths[i];
	  }
	  smoothed_sub_paths.push_back(smoothed_path);
	}
      if (all_path_smoothed)
	status |= PathSmoother::ALL_PATHS_SMOOTHED;
      else {
	if (one_path_smoothed) {
	  status |= PathSmoother::SOME_PATHS_SMOOTHED;
	}
	else {
	  status = PathSmoother::NO_PATHS_SMOOTHED;
	}      
      }
      return concatenateDirectionalPaths(smoothed_sub_paths);
    }

  
  // Use this if the direction of motion change during traversal.
  orunav_generic::Path smoothPath(const PathSmoother::Params &params, const orunav_generic::PathInterface &path, const orunav_generic::CollisionCheckInterface &constraint, int &status) {
    std::vector<BSpline2d> splines;
    return smoothPathSplines(params, path, constraint, status, splines);
  }



} // namespace
