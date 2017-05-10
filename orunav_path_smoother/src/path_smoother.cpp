#include <orunav_path_smoother/path_smoother.h>
#include <ctime>

std::vector<int> 
PathSmoother::getNbSamplesForEachIter() const
{
  // This could be done a bit more fancy(?)
  std::vector<int> ret;
  ret.push_back(2); // Need minimum of two poses.
  
  if (params.maxNbPoses <= 2)
    return ret;
  
  int nb_per_iter = params.maxNbIters / (params.maxNbPoses - 2);
  
  int nb_poses = 3;
  int counter = 0;
  while (nb_poses <= params.maxNbPoses)
    {
      if (counter >= nb_per_iter)
	{
	  counter = 0;
	  nb_poses++;
	}
      counter++;
      ret.push_back(nb_poses);
    }
  return ret;
}

bool
PathSmoother::smooth(const orunav_generic::PathInterface &path, const orunav_generic::CollisionCheckInterface &constraint)
{
  // Steps:
  // 1) subsample the path - not really needed
  // 2) generate the spline, with varying the knot offset
  // 3) run the collsion check
  // 4) check the curvature
  // 5) calc the optimization crit.

  BSpline2d best_spline;
  BSpline2d::Evaluation best_eval;
  best_eval.setToWorst();

  double speed = 1;
  if (!orunav_generic::forwardPath(path))
    speed = -1.;

  std::cout << "+++ speed : " << speed << std::endl;
  int iter = 0;
  int constraint_fail = 0;
  int evaluation_fail = 0;
  std::vector<int> nb_poses_iter = getNbSamplesForEachIter();
  for (unsigned int j = 0; j < params.nbKnotOffsetValues; j++)
    {
      for (unsigned int i = 0; i < nb_poses_iter.size(); i++)
	{
	  seed_ += 13498798478917;
	  orunav_generic::Path sub = this->subSample(path, nb_poses_iter[i], seed_); 
	  
	  BSpline2d current_spline;
	  BSpline2d::Evaluation current_eval;
	  
	  double knot_offset = params.initialKnotOffset / pow(2.0, j);
	  //  best_spline.setPoses(sub, speed, knot_offset);
	  current_spline.setPosesAsKnots(sub, speed, knot_offset);
	  iter++;
	  if (current_spline.collision(constraint, params.collisionResolution))
	    {
	      constraint_fail++;
	      continue;
	    }
	  if (!current_spline.evaluate(current_eval, params.steeringWheelDistOffset, params.minSteeringAngle, params.maxSteeringAngle, params.resolution))
	    {
	      evaluation_fail++;
	      continue;
	    }
	  
	  //	  std::cout << "eval[" << iter++ << "] : " << current_eval << std::endl;
	  

	  if (current_eval.betterThan(best_eval)) // TODO
	    {
	      if (current_eval.valid()) 
		{
		  best_eval = current_eval;
		  best_spline = current_spline;
		}
	    }
	}
    }
  std::cout << "tot number of iters : " << iter << std::endl;
  std::cout << "constraint failures (collisions) : " << constraint_fail/(iter*1.) << std::endl;
  std::cout << "evaluation failures : " << evaluation_fail/(iter*1.) << std::endl;
  std::cout << "best eval : " << best_eval << std::endl;

  spline_ = best_spline;
  return best_eval.valid();
}

orunav_generic::Path
PathSmoother::subSample(const orunav_generic::PathInterface &path, unsigned int maxNbPoints, unsigned int seed) const
{
  assert(path.sizePath() >= 2);
  assert(maxNbPoints >= 2);
  orunav_generic::Path subsampled_path;
  // The first and last pose must be present.
    unsigned int nb_interm = 0;
    if (maxNbPoints - 2 > 1)
      nb_interm = static_cast<unsigned int>(maxNbPoints - 2);
 
  // Randomly generate a subset of idxes... keep the history of previous samples.
  unsigned int size = path.sizePath();
  
  // Always need the first one...
  subsampled_path.addPathPoint(path.getPose2d(0), path.getSteeringAngle(0));
  int min = 1; int max = size - 2;
  if (nb_interm > 0 && min < max)
    {
      if (nb_interm > max - min) {
	nb_interm = max - min;
      }
      
      std::vector<unsigned int> samples = generateRandomVecWithRep(min, max, nb_interm, seed);
      std::sort(samples.begin(), samples.end());
      for (unsigned int i = 0; i < samples.size(); i++)
	{
	  //	  std::cout << "samples[" << i << "] : " << samples[i] << std::endl;
	  subsampled_path.addPathPoint(path.getPose2d(samples[i]), path.getSteeringAngle(samples[i]));
	}
    }
  subsampled_path.addPathPoint(path.getPose2d(size-1), path.getSteeringAngle(size-1));
  return subsampled_path;
}

void
PathSmoother::setAllKnots(const orunav_generic::PathInterface &path) 
{
  double speed = 1;
  if (!orunav_generic::forwardPath(path))
    speed = -1.;

  BSpline2d spline;
  unsigned int size = path.sizePath();
  for (unsigned int i = 0; i < size; i++) {
    spline.addKnot(path.getPose2d(i), speed);
  }
  spline_ = spline;
}
