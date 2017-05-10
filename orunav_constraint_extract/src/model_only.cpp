#include <orunav_constraint_extract/model_only.h>

using namespace constraint_extract;

ConstraintExtractorModelOnly::ConstraintExtractorModelOnly(const orunav_generic::PathInterface &p, const orunav_geometry::RobotModel2dInterface &m, orunav_generic::RobotInternalState2d::LoadType loadType) : path(p), model(m), computed_(false), loadType_(loadType) {

}

const std::vector<size_t>& 
ConstraintExtractorModelOnly::getSubSampledIdx() const {
  assert(computed_);
  return sub_idx;
}

const orunav_geometry::Polygons& 
ConstraintExtractorModelOnly::getOuterConstraints() const {
  assert(computed_);
  return outerConstraints;
}

const orunav_geometry::Polygons& 
ConstraintExtractorModelOnly::getInnerConstraints() const {
  assert(computed_);
  return innerConstraints;
}

void
ConstraintExtractorModelOnly::compute() {
  // TODO, only computes outer constraints so far.

  outerConstraints.clear();
  // Subsample the path
  orunav_generic::SubsamplePath sp(path, params.minDist, params.maxDist, params.maxRotation);
  orunav_geometry::RobotModel2dWithState rm(model);
  orunav_generic::RobotInternalState2d s;
  s.loadType = loadType_;
  //const std::vector<size_t>&
  sub_idx = sp.getSubsampledIdx();

  if (params.forceParkingPolygons) {
    // Note, we need to do this using the sub_idx consistent since this is also used in the upsampling.
    assert(sub_idx.front() == 0); // Really should / must have the first part of the path here.
    assert(sub_idx.back() == path.sizePath()-1); // and the last one should also be contained.

    // Check that we don't have mulitple sub_indexes.
    if (sub_idx[1] != 1) {
      sub_idx.insert(sub_idx.begin()+1, 1); // We now have a parking slot utilizing path point 0-1.
    }
    if (sub_idx[sub_idx.size()-2] != sub_idx.back()-1) {
      sub_idx.insert(sub_idx.end()-1, sub_idx.back()-1); // The end parking using (size-2)-(size-1) path points.
    }

    // // TEST CODE BELOW - create an additional smaller polygon in conjuction with the parking one...
    double max_parking_poly_dist = 0.4;
    
    for (size_t j = sub_idx[1]; j < sub_idx[2]; j++) {
        double dist = orunav_generic::getDistBetween(path.getPose2d(j), path.getPose2d(0));
        if (dist > max_parking_poly_dist) {
            sub_idx.insert(sub_idx.begin()+2, j);
            break;
        }
    }

    for (size_t j = sub_idx[sub_idx.size()-2]; j > sub_idx[sub_idx.size()-3]; j--) {
        double dist = orunav_generic::getDistBetween(path.getPose2d(j), path.getPose2d(sub_idx.back()));
        if (dist > max_parking_poly_dist) {
            sub_idx.insert(sub_idx.end()-2, j-1);
            break;
        }
    }
  }

  //  for (size_t i = 0; i < sub_idx.size(); i++) {
  //    std::cout << "sub_idx[" << i << "]: " << sub_idx[i] << std::endl;
  //  }
  for (size_t i = 0; i < sub_idx.size() - 1; i++) {
  
      orunav_geometry::Polygon p;
      for (size_t j = sub_idx[i]; j <= sub_idx[i+1]; j++) {
          // Set the robot model at the right pose and create polygons iteratively.
          s.steeringAngle = path.getSteeringAngle(j);
          rm.update(path.getPose2d(j), s);
          
          p.addPolygon(rm.getPosePolygon());
      }
      if (params.forceOuterConvex) {
          orunav_geometry::Polygon cv(p);
          cv.convexHull();
          outerConstraints.push_back(cv);
          
      }
      else {
          outerConstraints.push_back(p);
      }
      
      
  }
  
  if (params.forceConstraintsPerPathPoint) {
    outerConstraints = upSample(outerConstraints, sub_idx);
  }
  computed_ = true;
}


orunav_geometry::Polygons 
ConstraintExtractorModelOnly::upSample(const orunav_geometry::Polygons &polygons, const std::vector<size_t>& subsample_idx) {
  assert(!subsample_idx.empty());
  orunav_geometry::Polygons new_polygons;
  assert(subsample_idx.size() >= 2);

  size_t start_idx = 0;
  size_t stop_idx = subsample_idx[subsample_idx.size()-1];
  size_t idx = 0;
  
  for (size_t i = start_idx; i < subsample_idx.size()-1; i++)
    {
      while (idx < (subsample_idx[i]+subsample_idx[i+1])/2)
	{
	  new_polygons.push_back(polygons[i]);
	  idx++;
	}
    }
  while (idx <= stop_idx)
    {
      new_polygons.push_back(polygons[polygons.size()-1]);
      idx++;
    }
  return new_polygons;
}
