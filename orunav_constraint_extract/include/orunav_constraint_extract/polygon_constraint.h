#pragma once

#include <orunav_constraint_extract/constraints.h>
#include <orunav_constraint_extract/grid_map.h>
#include <boost/serialization/split_free.hpp>

namespace constraint_extract {

  class PolygonConstraint {
  public:
    PolygonConstraint() { }
  PolygonConstraint(const Eigen::Vector2d &topLeft, const Eigen::Vector2d &bottomRight, const Eigen::Vector2d &thBounds) : thBounds_(thBounds) {
      poly_ = orunav_geometry::getPolygonFromBox(topLeft, bottomRight);
      this->computeWeight();
    }

  PolygonConstraint(orunav_generic::Point2dContainerInterface& pts, const Eigen::Vector2d &thBounds) : thBounds_(thBounds) {
      poly_ = orunav_geometry::Polygon(pts);
    }
    
    int start_point_idx, end_point_idx;

    std::vector<geometry_msgs::Point> outer_constraint_points;

    const Eigen::Vector2d& getThBounds() const { return thBounds_; }
    const orunav_geometry::Polygon& getInnerConstraint() const { return poly_; }
    
    const orunav_geometry::Polygon& getOuterConstraint() const { return outerPoly_; }
    void setOuterConstraint(const orunav_geometry::Polygon &poly) {
      outerPoly_ = poly;
    }

    inline double getThBoundWidth() const {
      return angles::normalize_angle_positive(thBounds_(1) - thBounds_(0));
    }
 
    inline double weight() const { return weight_; }
    // 
    bool findInnerConstraintMatch(const PolygonConstraint &pc) const {
      for (size_t i = 0; i < poly_.sizePoint2d(); i++) {
	if (orunav_geometry::distSqrPoint2d(this->poly_.getPoint2d(i), pc.poly_.getPoint2d(i)) > 0.01)
	  return false;
      }
      // Check the angle - we would like to find the first match containing the sweep in the other direction.
      if (this->thBounds_(0) == pc.thBounds_(0) || this->thBounds_(1) == pc.thBounds_(1))
	return false;
      return true;
    }

    void moveConstraint(const orunav_generic::Pose2d &pose) {
      orunav_geometry::movePoint2dContainer(this->poly_.points, pose);
      // Normalize the angles?
      thBounds_(0) += pose(2);
      thBounds_(1) += pose(2);
    }

    void fuseConstraint(const PolygonConstraint &pc) {
      // Operates on the th bounds...
      //pc.printDebug();
       
      if (this->thBounds_(0) > pc.thBounds_(0))
	this->thBounds_(0) = pc.thBounds_(0);
      if (this->thBounds_(1) < pc.thBounds_(1))
	this->thBounds_(1) = pc.thBounds_(1);
    }

    //! Return the smallest distance to the angular constraints.
    /*!
     * If > small positive value, the constraint is not active.
     * if < small positive value, the constraint is active.
     */
    double angularConstraintDistance(const orunav_generic::Pose2d &pose) const {
      double th_width = getThBoundWidth();
      double th_start = this->thBounds_(0);
      
      double angle_inc_pos = angles::normalize_angle_positive(pose(2) - th_start);
      return (th_width - angle_inc_pos);
    }

    //! Return the smallest distance to the position constraints.
    /*!
     * If > small positive value, the constraint is not active.
     * if < small positive value, the constraint is active.
     */
    double positionConstraintDistance(const orunav_generic::Pose2d &pose) const {
      std::vector<double> A0, A1, b;
      this->getInnerConstraint().getMatrixFormAsVectors(A0, A1, b);
      assert(A0.size() == A1.size());
      assert(A0.size() == b.size());
      size_t size = A0.size();
      
      double min_dist = std::numeric_limits<double>::max();
      for (size_t i = 0; i < size; i++) {
	min_dist = std::min(b[i] - (A0[i]*pose(0) + A1[i]*pose(1)), min_dist);
      }
      return min_dist;
    }

    bool isActive(const orunav_generic::Pose2d &pose, double thresh = 0.0001) const {
      if (positionConstraintDistance(pose) < thresh)
	return true;
      if (angularConstraintDistance(pose) < thresh)
	return true;
      return false;
    }

    bool isFeasible(const orunav_generic::Pose2d &pose) const {
      /* if (this->getThBounds()[0] > pose(2)) { */
      /* 	std::cerr << "[validConstraintCheck]: getThBounds()[0] > pose(2) " << getThBounds()[0] << ", " << pose(2) << std::endl; */
      /* 	return false; */
      /* } */
      /* if (this->getThBounds()[1] < pose(2)) { */
      /* 	std::cerr << "[validConstraintCheck]: getThBounds()[1] < pose(2)" << getThBounds()[1] << ", " << pose(2) << std::endl; */
      /* 	return false; */
      /* } */

      double th_width = getThBoundWidth();
      double th_start = this->thBounds_(0);
      
      double angle_inc_pos = angles::normalize_angle_positive(pose(2) - th_start);
      if (angle_inc_pos > th_width)
      	return false;
      
      std::vector<double> A0, A1, b;
      this->getInnerConstraint().getMatrixFormAsVectors(A0, A1, b);
      assert(A0.size() == A1.size());
      assert(A0.size() == b.size());
      size_t size = A0.size();
      
      for (size_t i = 0; i < size; i++) {
	if (A0[i]*pose(0) + A1[i]*pose(1) > b[i]) {
          //	  std::cerr << "[validConstraintCheck]: i : " << i << std::endl;
	  return false;
	}
      }
      return true;
    }

    void printDebug() const {
      std::cout << "poly_ : [" << std::endl;
      for (size_t i = 0; i < poly_.points.size(); i++) {
	std::cout << "(" << poly_.points[i](0) << "," << poly_.points[i](1) << ")" << std::flush;
      }
      std::cout << "]" << std::endl;
      std::cout << "thBounds_ : " << thBounds_ << std::endl;
      std::cout << "weight_ : " << weight_ << std::endl;
      std::cout << "feasiblePose_ : " << feasiblePose_ << std::endl;
    }
  
    orunav_geometry::Polygon poly_;
    orunav_geometry::Polygon outerPoly_;
    Eigen::Vector2d thBounds_;

    double weight_;
    orunav_generic::Pose2d feasiblePose_;
  private:
    void computeWeight() {
      // TODO - what is really the weight here -> the area sounds like a good idea but another option would be the minimum distance to the polygon corners (instead of the area). This should give more flexibility where it is needed. Like:
      //Eigen::Vector2d pt = orunav_generic::getClosestPoint2dToOrigin(poly_);
      //weight_ = std::min(pt(0), pt(1)) * poly_.getArea() * getThBoundWidth();       //      orunav_generic::getClosestPoint2dToOrigin(poly_).norm()
      weight_ = poly_.getArea() * getThBoundWidth();
    }
  
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  inline orunav_geometry::Polygon computeThSweepPolygon_(const Eigen::Vector2d &point, const Eigen::Vector2d &thBounds, orunav_geometry::RobotModel2dWithState &rm, const orunav_generic::RobotInternalState2d &s, double thIncr) {
    orunav_geometry::Polygon p;
    if (thBounds(1) - thBounds(0) < thIncr) {
      thIncr = thBounds(1) - thBounds(0);
    } 
    
    orunav_generic::Pose2d pose(point(0),
			       point(1),
			       thBounds(0));
    
    while (pose(2) < thBounds(1)) {
      rm.update(pose,s);
      p.addPolygon(rm.getPosePolygon());
      pose(2) += thIncr;
    }
    
    if (fabs(pose(2) - thBounds(1) - thIncr) > 0.0001) { // Check that we're not approx. adding the same polygon again - this will cause some weird behaviour if this difference is very small, (10^-16) 
      pose(2) = thBounds(1);
      rm.update(pose,s);
      p.addPolygon(rm.getPosePolygon());
    }
    p.convexHull();
    return p;
  }

  inline orunav_geometry::Polygon computeConstraintOuterRegion_(const PolygonConstraint &constraint, orunav_geometry::RobotModel2dWithState &rm, const orunav_generic::RobotInternalState2d &s, double th_sweep_incr) {
    orunav_geometry::Polygon p;
    const orunav_geometry::Polygon& ic = constraint.getInnerConstraint();
    const Eigen::Vector2d &thb = constraint.getThBounds();
    
    for (size_t i = 0; i < ic.sizePoint2d(); i++) {
      orunav_geometry::Polygon sweep = computeThSweepPolygon_(ic.getPoint2d(i), thb, rm, s, th_sweep_incr);
      //      if (!p.addPolygon(sweep)) {
	// The polygons are disjoint. This can happen if the inner constraints are 'large' enough.
	for (size_t j = 0; j < sweep.sizePoint2d(); j++) {
	  p.points.push_back(sweep.getPoint2d(j));
	  //	}
	//	p.convexHull();
      }
    }
    p.convexHull();
    return p;
  }


  bool polygon_constraint_sort_predicate(const PolygonConstraint &p1, const PolygonConstraint &p2)
  {
    return p1.weight() > p2.weight();
  }

  class PolygonConstraintsVec : public std::vector<PolygonConstraint, Eigen::aligned_allocator<PolygonConstraint> >, orunav_generic::Pose2dContainerInterface {
  public:
    virtual orunav_generic::Pose2d getPose2d(size_t idx) const { 
      return (*this)[idx].feasiblePose_; 
    }
    
    virtual void setPose2d(const orunav_generic::Pose2d &pose, size_t idx) { 
      (*this)[idx].feasiblePose_ = pose; 
    }
    virtual size_t sizePose2d() const { return this->size(); }
  };

  PolygonConstraintsVec selectConstraintsVecIndexes(const PolygonConstraintsVec &cons,
				   const std::vector<int> &indexes) {
    PolygonConstraintsVec ret;
    for (int i = 0; i < indexes.size(); i++) {
      ret.push_back(cons[indexes[i]]);
    }
    return ret;
  }

  class PolygonConstraintsLookup {
  public:
    class Params {
    public:
      Params() {
	x_left.push_back(-0.01);
	x_left.push_back(-0.1);
	x_left.push_back(-0.2);
	x_left.push_back(-0.3);
	x_left.push_back(-0.5);
	x_left.push_back(-0.8);
	x_left.push_back(-1.2);//  -> THIS RELATES TO THE 'OCCUPANCY' MAP AND NOT THE VECHILE COORD SYSTEM
	
	x_right.push_back(0.01);
	x_right.push_back(0.1);
	x_right.push_back(0.2);
	x_right.push_back(0.3);
	x_right.push_back(0.5);
	x_right.push_back(0.8);
	x_right.push_back(1.2);

	y_up.push_back(0.01);
	y_up.push_back(0.1);
	y_up.push_back(0.2);
	y_up.push_back(0.4);
	y_up.push_back(0.6);
	y_up.push_back(0.9);
      
	y_down.push_back(-0.01);
	y_down.push_back(-0.1);
	y_down.push_back(-0.2);
	y_down.push_back(-0.4);
	y_down.push_back(-0.6);
	y_down.push_back(-0.9);

	th_left.push_back(-M_PI/6.);
	th_left.push_back(-M_PI/12.);
	th_left.push_back(-M_PI/36.);
	th_left.push_back(-M_PI/180.);
	
	th_right.push_back(M_PI/180.);
	th_right.push_back(M_PI/36.);
	th_right.push_back(M_PI/12.);
	th_right.push_back(M_PI/6.);
      
	for (size_t i = 0; i < th_left.size(); i++) {
	  th_bounds.push_back(std::pair<double,double>(th_left[i], 0.));
	}
	for (size_t i = 0; i < th_right.size(); i++) {
	  th_bounds.push_back(std::pair<double,double>(0., th_right[i]));
	}
      
	resolution = 0.1;
	th_sweep_incr = M_PI/36.;
	use_th_bounds = false;
	skip_overlap = false; // Debug param
      }
      std::vector<double> x_left; // should contain decreasing distance 0., -0.1, -0.2, -0.3 etc...
      std::vector<double> x_right;// should contain increasing distance 0., 0.1, 0.2, 0.3 etc...
      std::vector<double> y_up;   // should contain increasing distance 0., 0.1, 0.2, 0.3 etc...
      std::vector<double> y_down; // should contain decreasing distance 0., -0.1, -0.2, -0.3 etc...
      std::vector<double> th_left;
      std::vector<double> th_right;
      std::vector<std::pair<double,double> > th_bounds; // should contain angles as: [-30., 0., -15., 0. -1., 0., 1., 0., 15., 0., 30.]
    
      double resolution;
      double th_sweep_incr;
      bool use_th_bounds;
      bool skip_overlap;

      friend std::ostream& operator<<(std::ostream &os, const PolygonConstraintsLookup::Params &obj)
      	{
      	  os << "\nx_left: ";
      	  for (size_t i = 0; i < obj.x_left.size(); i++) {
      	    os << " " << obj.x_left[i];
      	  }
      	  os << "\nx_right: ";
      	  for (size_t i = 0; i < obj.x_right.size(); i++) {
      	    os << " " << obj.x_right[i];
      	  }
      	  os << "\ny_up: ";
      	  for (size_t i = 0; i < obj.y_up.size(); i++) {
      	    os << " " << obj.y_up[i];
      	  }
      	  os << "\ny_down: ";
      	  for (size_t i = 0; i < obj.y_down.size(); i++) {
      	    os << " " << obj.y_down[i];
      	  }
      	  os << "\nresolution    : " << obj.resolution;
      	  os << "\nth_sweep_incr : " << obj.th_sweep_incr;
      	  os << "\nuse_th_bounds : " << obj.use_th_bounds;
	  os << "\nskip_overlap  : " << obj.skip_overlap;
      	  return os;
      	}
    };


  PolygonConstraintsLookup(const orunav_geometry::RobotModel2dInterface &m, const orunav_generic::RobotInternalState2d::LoadType &loadType, const PolygonConstraintsLookup::Params &params) : params_(params), model(m), loadType_(loadType), computed_(false) {}


    inline void addPose2dThSweep(orunav_generic::Pose2dVec &poses, double x, double y, double min_th, double max_th, double th_sweep_incr) const {
      double th = min_th;
      while (th < max_th) {
	poses.push_back(orunav_generic::Pose2d(x,y,th));
	th += th_sweep_incr;
      }
      poses.push_back(orunav_generic::Pose2d(x,y,max_th));
    }
    
 

    /*  inline orunav_geometry::Polygon computeThSweepPolygon(const Eigen::Vector2d &point, const Eigen::Vector2d &thBounds, orunav_geometry::RobotModel2dWithState &rm, const orunav_generic::RobotInternalState2d &s, double thIncr) const { */
    /*   orunav_geometry::Polygon p; */
    /*   if (thBounds(1) - thBounds(0) < thIncr) { */
    /* 	thIncr = thBounds(1) - thBounds(0); */
    /*   }  */

    /*   orunav_generic::Pose2d pose(point(0), */
    /* 				 point(1), */
    /* 				 thBounds(0)); */
      
    /*   while (pose(2) < thBounds(1)) { */
    /* 	rm.update(pose,s); */
    /* 	p.addPolygon(rm.getPosePolygon());	 */
    /* 	pose(2) += thIncr; */
    /*   } */
 
    /*   if (fabs(pose(2) - thBounds(1) - thIncr) > 0.0001) { // Check that we're not approx. adding the same polygon again - this will cause some weird behaviour if this difference is very small, (10^-16)  */
    /* 	pose(2) = thBounds(1); */
    /* 	rm.update(pose,s); */
    /* 	p.addPolygon(rm.getPosePolygon()); */
    /*   } */
    /*   p.convexHull(); */
    /*   return p; */
    /* } */

    orunav_geometry::Polygon computeConstraintOuterRegion(const PolygonConstraint &constraint, orunav_geometry::RobotModel2dWithState &rm) const {
      orunav_generic::RobotInternalState2d s;
      s.loadType = loadType_;
      return computeConstraintOuterRegion_(constraint, rm, s, params_.th_sweep_incr);
    }
      /* orunav_geometry::Polygon p; */
      /* const orunav_geometry::Polygon& ic = constraint.getInnerConstraint();  */
      /* const Eigen::Vector2d &thb = constraint.getThBounds(); */
      /* orunav_generic::RobotInternalState2d s; */
      /* s.loadType = loadType_; */
      
      /* for (size_t i = 0; i < ic.sizePoint2d(); i++) { */
      /* 	p.addPolygon(computeThSweepPolygon(ic.getPoint2d(i), thb, rm, s, params_.th_sweep_incr)); */
      /* } */
      /* p.convexHull(); */
      /* return p; */
    
    
    orunav_geometry::Polygon computeConstraintOuterRegion(const PolygonConstraint &constraint) const {
      orunav_geometry::RobotModel2dWithState rm(model);
      return this->computeConstraintOuterRegion(constraint, rm);
    }

    orunav_geometry::Polygon computeLargestConstraintOuterRegion() const {
      orunav_geometry::RobotModel2dWithState rm(model);
      orunav_generic::RobotInternalState2d s;
      s.loadType = loadType_;
      
      // Get the outer poses, use the min / max th_bounds and place the robot to get the outer constraints.
      // Use the parameter to get left up left rotation, etc.
      double x_left = params_.x_left.back();
      double x_right = params_.x_right.back();
      double y_up = params_.y_up.back();
      double y_down = params_.y_down.back();
      double th_left = params_.th_bounds.front().first;
      double th_right = params_.th_bounds.back().second;

      Eigen::Vector2d top_left(x_left, y_up);
      Eigen::Vector2d bottom_right(x_right, y_down);
      Eigen::Vector2d th_bounds(th_left, th_right);

      PolygonConstraint pc(top_left, bottom_right, th_bounds);
      return computeConstraintOuterRegion(pc, rm);
    }

    nav_msgs::OccupancyGrid createOccupancyGridPatch() {
      // TODO - compute the params based on the largest polygon
    
    
      nav_msgs::OccupancyGrid ret;
      ret.info.resolution = params_.resolution;
      ret.info.width = 6 / params_.resolution;
      ret.info.height = 6 / params_.resolution;
      ret.info.origin = orunav_conversions::createMsgFromPose2d(orunav_generic::Pose2d(-3.0, -3.0, 0.));
      ret.header.frame_id = std::string("/world");
    
      ret.data.resize(ret.info.width * ret.info.height);
      std::fill(ret.data.begin(), ret.data.end(), 0);
      return ret;
    }

    // The map is only needed to obtain the resolution.
    void compute() {
    

      // ---------------------------- COMPUTE THE CONSTRAINTS ------------------------
      // MASSIVE FOR LOOP GOES HERE...
      // std::vector<std::vector<size_t> > generateIndexes() const {
      /* for () { */
      /*   PolygonConstraint p(const Eigen::Vector2d &topLeft, const Eigen::Vector2d &bottomRight, const Eigen::Vector2d &thBounds) */
      // Each vector contains an entry to the parameters value x_left, x_right, y_up, y_down for all possible combinations we want to evaluate.
      /* } */
      double x_left = params_.x_left.back();
      double x_right = params_.x_right.back();
      double y_up = params_.y_up.back();
      double y_down = params_.y_down.back();
      double th_left = params_.th_bounds.front().first;
      double th_right = params_.th_bounds.back().second;

      for (size_t i = 0; i < params_.x_left.size(); i++) {
	x_left = params_.x_left[i];
	for (size_t j = 0; j < params_.x_right.size(); j++) {
          x_right = params_.x_right[j];
	  if (x_left == x_right) // If they both are zero - ignore this one.
	    continue;

	  for (size_t k = 0; k < params_.y_up.size(); k++) {
	    y_up  = params_.y_up[k];
	    for (size_t l = 0; l < params_.y_down.size(); l++) {
	      y_down = params_.y_down[l];
	      if (y_up == y_down)
		continue;
	    
	      if (params_.use_th_bounds) {
		for (size_t m = 0; m < params_.th_bounds.size(); m++) {
		  th_left = params_.th_bounds[m].first;
		  th_right = params_.th_bounds[m].second;
		
		  Eigen::Vector2d top_left(x_left, y_up);
		  Eigen::Vector2d bottom_right(x_right, y_down);
		  Eigen::Vector2d th_bounds(th_left, th_right);
		
		  PolygonConstraint pc(top_left, bottom_right, th_bounds);
		  constraints_.push_back(pc);
		}
	      }
	      else {
		for (size_t m = 0; m < params_.th_left.size(); m++) {
		  for (size_t n = 0; n < params_.th_right.size(); n++) {
		    th_left = params_.th_left[m];
		    th_right = params_.th_right[n];
		  
		    Eigen::Vector2d top_left(x_left, y_up);
		    Eigen::Vector2d bottom_right(x_right, y_down);
		    Eigen::Vector2d th_bounds(th_left, th_right);
		  
		    PolygonConstraint pc(top_left, bottom_right, th_bounds);
		    constraints_.push_back(pc);
		  } 
		}
	      }
	    }
	  }
	}
      }
    

      // Sort the constraint vector, best is first, the order is now locked we now can use a index directly to this vector.
      this->sortConstraintsOnCost();
    
      // Fill in the valid struct, since the vector is sorted and the std::map indexes the iterators based on the key - the current element will always be better (or equal) than the next.
      for (size_t i = 0; i < this->constraints_.size(); i++) {
	valid_.insert(std::pair<int, PolygonConstraint&>(i, this->constraints_[i]));
      }
    
      // ------------------------------------ BUILD THE OCCUPANCY STRUCTURE -------------------------------------
      orunav_geometry::RobotModel2dWithState rm(model);
      orunav_generic::RobotInternalState2d s;
      s.loadType = loadType_;

      // We need to first compute the smalles size of the occupancy we need to have to fit all outer constraints. Then for each PolygonConstraint we compute and store a new occupancy grid with the same size.
    
      nav_msgs::OccupancyGrid occ_map_empty = createOccupancyGridPatch();

      { // Add the biggest fotprint (the vehicle will never be outside these bound).
	largestFootPrint_ = occ_map_empty;
	orunav_geometry::Polygon p = this->computeLargestConstraintOuterRegion();
	GridPatchIdx patch_idx = getPolygonGridPatchIdx(largestFootPrint_, p);
	drawGridPatchIdxOnOccupancyMap(patch_idx, largestFootPrint_);
      }
      
      // Below is where the main computation is done which takes quite some time...
      if (computed_) return; // Already loaded...
    
      occupancyMaps_.resize(constraints_.size());
      
      // Add the corresponding occupancy map for each constraint.
      std::cout << "Progress in (%):" << std::endl;
      std::cout << "----------------" << std::endl;
      for (size_t i = 0; i < this->constraints_.size(); i++) {
	// Compute constraints and fill the occupancy map.
	nav_msgs::OccupancyGrid occ_map = occ_map_empty;
	orunav_geometry::Polygon p = this->computeConstraintOuterRegion(constraints_[i], rm);
	GridPatchIdx patch_idx = getPolygonGridPatchIdx(occ_map, p);
	drawGridPatchIdxOnOccupancyMap(patch_idx, occ_map);
	occupancyMaps_[i] = (occ_map);
        std::cout << i*100. / constraints_.size() << std::flush;
        std::putchar('\r');
      }
      std::cout << "100.000 (finally)\n" << std::endl;
    }

    /* orunav_generic::Point2dVec getOccupiedPixelsInLocalMetricCoords(const nav_msgs::OccupancyGrid &map, const orunav_generic::Pose2d &pose, const nav_msgs::OccupancyGrid &localMap) const { */
        
    
    /*   // 1) -> get the occupied pixel in the local map. */
    /*   // 2) -> move the pixels to global coords. */
    /*   // 3) -> check the real map for collisions */
    /*   // 4) -> for each collision detected return the corresponding local pixel idx. */
    /*   GridPatchIdx occ_patch_local = getOccupiedGridPatch(localMap); */
    /*   orunav_generic::Point2dVec pts = getPoint2dVecFromGridPatchIdx(localMap, occ_patch_local); */
    /*   orunav_geometry::movePoint2dContainer(pts, pose); */
    /*   GridPatchIdx occ_patch_global = getGridPatchIdxFromPoint2dContainerInterface(map, pts); */
    
    /*   // Step through and keep only the */

    /*   // Have the global occupied vaues / patch. */
    /*   GridPatchIdx collision_global = getOccupiedGridPatchIdxFromPatchIdx(map, occ_patch_global); */
    
    /*   // Rotate back to local coords */
    /*   pts = getPoint2dVecFromGridPatchIdx(map, collision_global); */
    /*   orunav_generic::Pose2d inv_pose = orunav_generic::subPose2d(pose, orunav_generic::Pose2d(0.,0.,0.)); */
    /*   orunav_geometry::movePoint2dContainer(pts, inv_pose); */
    /*   return pts; */
    /* } */

    // Return the corresponding pixel which is occupied in the map using pixel coords in the local map.
    GridPatchIdx getOccupiedPixelsInLocalPixelCoords(const nav_msgs::OccupancyGrid &map, const orunav_generic::Pose2d &pose, const nav_msgs::OccupancyGrid &localMap) const {
  
      GridPatchIdx ret;
      GridPatchIdx occ_patch_local = getOccupiedGridPatch(localMap);
      orunav_generic::Point2dVec pts = getPoint2dVecFromGridPatchIdx(localMap, occ_patch_local);
      orunav_geometry::movePoint2dContainer(pts, pose);

      assert(occ_patch_local.size() == pts.size());
      // Check trough all points, we would like to directly return the occ_patch_local index.
      for (size_t i = 0; i < pts.sizePoint2d(); i++) {
	Eigen::Vector2i pixel;
	metricToPixelOccupancyGrid(map, pts[i], pixel);
	if (isValidAndOccupied(map, pixel)) {
	  ret.push_back(occ_patch_local[i]);
	}
      }
      return ret;
    }

    nav_msgs::OccupancyGrid createLocalOccupancyGridPatchFromPose(const nav_msgs::OccupancyGrid &map, const orunav_generic::Pose2d &pose) const {
      nav_msgs::OccupancyGrid ret = this->largestFootPrint_;
      clearOccupancyValueOnGrid(ret);
      GridPatchIdx occ_patch = getOccupiedPixelsInLocalPixelCoords(map, pose, largestFootPrint_);
      drawGridPatchIdxOnOccupancyMap(occ_patch, ret);
      return ret;
    }

    bool findConstraint(const nav_msgs::OccupancyGrid &map, const orunav_generic::Pose2d &pose, PolygonConstraint &result, int &resultIdx) const {
      return findConstraint(map, pose, pose, result, resultIdx);
    }

    //--------------------------------------------------
    //! Find a the best constraint around pose where containPose is also contained in the constraints.
    /*!
     * If resultIdx returns != -1, then the returned result will reflect the best constraint without containPose. If the function returns false and resultIdx == -1 there was no constraints to be found at this pose. pose and containPose is given in the same coordinate system.
     */
    bool findConstraint(const nav_msgs::OccupancyGrid &map, const orunav_generic::Pose2d &pose, const orunav_generic::Pose2d &containPose, PolygonConstraint &result, int &resultIdx) const {

      resultIdx = -1;
      //    std::cout << "------> 1) valid_.size() : " << valid_.size(); 
      std::map<int, PolygonConstraint&> valid = valid_;    
      
      
      GridPatchIdx occ_patch = getOccupiedPixelsInLocalPixelCoords(map, pose, largestFootPrint_);
      
      // Sort the grid patch based on the distance to the center (local coords).
      // Check closest pixels first. -> has very little influence.
      std::sort(occ_patch.begin(), occ_patch.end(), grid_path_idx_sort_incr_predicate);
      
      // Check one pixel at time
      /* { */
      /* 	for (size_t i = 0; i< occ_patch.size(); i++) { */
      /* 	  //std::cout << "pixel dist[" << i << "] : " << sqrt(occ_patch[i].squaredNorm()) << std::endl; */
      /* 	  std::map<int, PolygonConstraint&>::iterator it = valid.begin(); */
      /* 	  while (it != valid.end()) { */
      /* 	    const nav_msgs::OccupancyGrid &m = occupancyMaps_[it->first]; */
      /* 	    if (isValidAndOccupied(m, occ_patch[i])) { */
      /* 	      std::map<int, PolygonConstraint&>::iterator it_erase = it; */
      /* 	      it++; // Keep the iterator defined before erasing. */
      /* 	      valid.erase(it_erase); */
      /* 	    } */
      /* 	    else { */
      /* 	      it++; */
      /* 	    } */
      /* 	  } */
      /* 	} */
      /* } */

      // Check all pixels in each iteration -> will only get the iterator to the first one and valid should only contain this element (cannot use skip overlap here).
      {
	std::map<int, PolygonConstraint&>::iterator it = valid.begin();
	while (it != valid.end()) {
	  bool found = true;
	  const nav_msgs::OccupancyGrid &m = occupancyMaps_[it->first];
	  for (size_t i = 0; i< occ_patch.size(); i++) {
	    if (isValidAndOccupied(m, occ_patch[i])) {
	      found = false;
	      it++;
	      break; // for loop
	    }
	  }
	  if (found) {
	    // Got it.
	    std::map<int, PolygonConstraint&> tmp;
	    tmp.insert(std::pair<int, PolygonConstraint&>(it->first, it->second));
	    valid = tmp;
	    break; // while loop
	  }
	}
	if (it == valid.end()) {
	  // Failed to find anything, set valid empty.
	  valid.clear();
	}
      }
  
      if (!params_.skip_overlap) {
	// Check the contain pose -> step through the valid list and find the first match.
	// In case nothing is found here... very very bad, or the size of the polygon will become very small (maybe it is better to grow along the path in the next constraints...) anyway this needs to be further elaborated.
	// Key part here... all constraints are centered around (0,0,0). Find the relative pose here.
	orunav_generic::Pose2d contain_pose_relative = orunav_generic::subPose2d(pose, containPose);
	std::map<int, PolygonConstraint&>::iterator it = valid.begin();
	while (it != valid.end()) {
	  if (!it->second.isFeasible(contain_pose_relative)) {
	    std::map<int, PolygonConstraint&>::iterator it_erase = it;
	    it++; // Keep the iterator defined before erasing.
	    valid.erase(it_erase);
	  }
	  else {
	    break; // Got it.
	  }
	}
      }
      
      if (valid.empty())
	return false;
      
      result = valid.begin()->second;
      result.feasiblePose_ = pose;
      resultIdx = valid.begin()->first;

      // The constraints are separated into left rotations and right rotations. Look for the first entry which has another rotation sweep in the queue and add that to the constraint.
      if (params_.use_th_bounds)
	{
	  std::cout << "fusing th bounds" << std::endl;
	  std::map<int, PolygonConstraint&>::iterator it = valid.begin();
	  it++;
	  while (it != valid.end()) {
	    if (result.findInnerConstraintMatch(it->second)) {
	      std::cout << "fusing the constraints..." << std::endl;
	      //result.printDebug();
	      result.fuseConstraint(it->second);
	      //result.printDebug();
	      result.moveConstraint(pose);
	      return true;
	    }
	    it++;
	  }
	}
      result.moveConstraint(pose);
      return true;
    }

    void sortConstraintsOnCost() { 
      std::sort(constraints_.begin(), constraints_.end(), polygon_constraint_sort_predicate);
    }

    const nav_msgs::OccupancyGrid& getOccupancyMap(size_t idx) const { 
      if (idx < occupancyMaps_.size())
	return occupancyMaps_[idx]; 
    
      return largestFootPrint_;
    }

    const nav_msgs::OccupancyGrid& getLargestFootPrint() const {
      return largestFootPrint_;
    }

    void setOccupancyGrids(const std::vector<nav_msgs::OccupancyGrid> &grids) {
      occupancyMaps_ = grids;
      computed_ = true;
    }
      
    const std::vector<nav_msgs::OccupancyGrid>& getOccupancyGrids() const {
      return occupancyMaps_;
    }
    
    const PolygonConstraint& getConstraint(size_t idx) const {
      assert(idx < constraints_.size());
      return constraints_[idx];
    }
    
    size_t getNbConstraints() const {
      return constraints_.size();
    }


    Params params_;

  private:
    std::vector<nav_msgs::OccupancyGrid> occupancyMaps_;
    std::vector<PolygonConstraint,Eigen::aligned_allocator<PolygonConstraint> > constraints_;
    std::map<int, PolygonConstraint&> valid_;
    const orunav_geometry::RobotModel2dInterface &model;
    orunav_generic::RobotInternalState2d::LoadType loadType_;
    
    nav_msgs::OccupancyGrid largestFootPrint_;
    bool computed_;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace
