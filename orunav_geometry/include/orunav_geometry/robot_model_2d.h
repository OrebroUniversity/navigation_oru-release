#pragma once

#include <orunav_generic/types.h>
#include <orunav_geometry/polygon.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace orunav_geometry {

  //! Represents a model of a vehicle in 2d - the main trick here is that the model should be capable of changing depending on the 'internal' state of the vehicle. For example, different steering angles (actuated vehicles), or if the vehicle has load (like a pallet), which would alther the shape.
  
  
  
  class RobotModel2dInterface {
  public:
    //! Returns a polygon (not nessecarly convex) of the bounding box of the vehicle.
    /*!
     * Everything is given in the robot coordinate frame.
     */
    virtual const Polygon& getBoundingRegion(const orunav_generic::RobotInternalState2d &s) const = 0;
  };


  // Each type could be formed as a singleton or another clever way. Another question would be to store this as parameters, but this is the simplest way for now. When entering the coordinates - do it counter clock-wise.
  
  class RobotModel2dCiTiTruck : public RobotModel2dInterface {
  public:
    RobotModel2dCiTiTruck() {
      /* wo_load.points.push_back(Eigen::Vector2d(1.6, 0.3)); */
      /* wo_load.points.push_back(Eigen::Vector2d(1.6, -0.3)); */
      /* wo_load.points.push_back(Eigen::Vector2d(-0.2, -0.3)); */
      /* wo_load.points.push_back(Eigen::Vector2d(-0.2, 0.3)); */
      wo_load.points.push_back(Eigen::Vector2d(1.9, 0.3)); //fl
      wo_load.points.push_back(Eigen::Vector2d(1.9, -0.3)); //fr
      wo_load.points.push_back(Eigen::Vector2d(-0.2, -0.3)); //rr
      wo_load.points.push_back(Eigen::Vector2d(-0.2, 0.3)); //rl

      w_load.points.push_back(Eigen::Vector2d(0.2, 0.5));
      w_load.points.push_back(Eigen::Vector2d(1.8, 0.5));
      w_load.points.push_back(Eigen::Vector2d(1.8, -0.5));
      w_load.points.push_back(Eigen::Vector2d(0.2, -0.5));
      w_load.points.push_back(Eigen::Vector2d(-0.3, -0.3));
      w_load.points.push_back(Eigen::Vector2d(-0.3, 0.3));

      w_load2.points.push_back(Eigen::Vector2d(0.2, 0.5));
      w_load2.points.push_back(Eigen::Vector2d(1.8, 0.5));
      w_load2.points.push_back(Eigen::Vector2d(1.8, -0.5));
      w_load2.points.push_back(Eigen::Vector2d(0.2, -0.5));
      w_load2.points.push_back(Eigen::Vector2d(-0.3, -0.3));
      w_load2.points.push_back(Eigen::Vector2d(-0.3, 0.3));
    }
    virtual const Polygon& getBoundingRegion(const orunav_generic::RobotInternalState2d &s) const {
      switch (s.loadType) {
      case orunav_generic::RobotInternalState2d::NO_LOAD: 
	return wo_load;
	break;
      case orunav_generic::RobotInternalState2d::EUR_PALLET:
	// TODO!!!!
	return w_load;
	break;
      case orunav_generic::RobotInternalState2d::HALF_PALLET:
	// TODO!!!!
	return w_load2;
	break;
      default:
	assert(false);
	return w_load;
	break;
      }
    }
  private:
    Polygon w_load;
    Polygon w_load2;
    Polygon wo_load;
  };

  class RobotModel2dBtTruck : public RobotModel2dInterface {
  public:
    RobotModel2dBtTruck() {
      wo_load.points.push_back(Eigen::Vector2d(2.30, 0.45)); //fl
      wo_load.points.push_back(Eigen::Vector2d(2.30, -0.45)); //fr
      wo_load.points.push_back(Eigen::Vector2d(-0.38, -0.45)); //rr
      wo_load.points.push_back(Eigen::Vector2d(-0.38, 0.45)); //rl
      
      w_load.points.push_back(Eigen::Vector2d(2.30, 0.45));
      w_load.points.push_back(Eigen::Vector2d(2.30, -0.45));
      w_load.points.push_back(Eigen::Vector2d(-0.38, -0.45));
      w_load.points.push_back(Eigen::Vector2d(-0.38, 0.45));
      
      w_load2.points.push_back(Eigen::Vector2d(2.30, 0.45));
      w_load2.points.push_back(Eigen::Vector2d(2.30, -0.45));
      w_load2.points.push_back(Eigen::Vector2d(-0.38, -0.45));
      w_load2.points.push_back(Eigen::Vector2d(-0.38, 0.45));
    }
    virtual const Polygon& getBoundingRegion(const orunav_generic::RobotInternalState2d &s) const {
      switch (s.loadType) {
      case orunav_generic::RobotInternalState2d::NO_LOAD: 
	return wo_load;
	break;
      case orunav_generic::RobotInternalState2d::EUR_PALLET:
	// TODO!!!!
	return w_load;
	break;
      case orunav_generic::RobotInternalState2d::HALF_PALLET:
	// TODO!!!!
	return w_load2;
	break;
      default:
	assert(false);
	return w_load;
	break;
      }
    }
  private:
    Polygon w_load;
    Polygon w_load2;
    Polygon wo_load;
  };
  
  class RobotModel2dCiTiTruckWithArm : public RobotModel2dInterface {
  public:
    RobotModel2dCiTiTruckWithArm() {
      /* wo_load.points.push_back(Eigen::Vector2d(1.6, 0.3)); */
      /* wo_load.points.push_back(Eigen::Vector2d(1.6, -0.3)); */
      /* wo_load.points.push_back(Eigen::Vector2d(-0.2, -0.3)); */
      /* wo_load.points.push_back(Eigen::Vector2d(-0.2, 0.3)); */

      wo_load.points.push_back(Eigen::Vector2d(1.7, 0.3));
      wo_load.points.push_back(Eigen::Vector2d(1.7, -0.3));
      wo_load.points.push_back(Eigen::Vector2d(-0.1, -0.3));
      wo_load.points.push_back(Eigen::Vector2d(-0.1, 0.3));

      /* wo_load.points.push_back(Eigen::Vector2d(1.9, 0.3)); */
      /* wo_load.points.push_back(Eigen::Vector2d(1.9, -0.3)); */

      /* wo_load.points.push_back(Eigen::Vector2d(0.88, -0.3)); */
      /* wo_load.points.push_back(Eigen::Vector2d(0.88, -0.5)); */
      /* wo_load.points.push_back(Eigen::Vector2d(0.45, -0.5)); */
      /* wo_load.points.push_back(Eigen::Vector2d(0.45, -0.3)); */
      /* wo_load.points.push_back(Eigen::Vector2d(-0.2, -0.3)); */

      /* wo_load.points.push_back(Eigen::Vector2d(-0.2, 0.3)); */
      /* wo_load.points.push_back(Eigen::Vector2d(0.2, 0.3)); */
      /* wo_load.points.push_back(Eigen::Vector2d(0.3, 0.6)); */
      /* wo_load.points.push_back(Eigen::Vector2d(0.5, 0.6)); */
      /* wo_load.points.push_back(Eigen::Vector2d(0.65, 0.3)); */


      w_load.points.push_back(Eigen::Vector2d(0.2, 0.5));
      w_load.points.push_back(Eigen::Vector2d(1.8, 0.5));
      w_load.points.push_back(Eigen::Vector2d(1.8, -0.5));
      w_load.points.push_back(Eigen::Vector2d(0.2, -0.5));
      w_load.points.push_back(Eigen::Vector2d(-0.3, -0.3));
      w_load.points.push_back(Eigen::Vector2d(-0.3, 0.3));

      w_load2.points.push_back(Eigen::Vector2d(0.2, 0.5));
      w_load2.points.push_back(Eigen::Vector2d(1.8, 0.5));
      w_load2.points.push_back(Eigen::Vector2d(1.8, -0.5));
      w_load2.points.push_back(Eigen::Vector2d(0.2, -0.5));
      w_load2.points.push_back(Eigen::Vector2d(-0.3, -0.3));
      w_load2.points.push_back(Eigen::Vector2d(-0.3, 0.3));
    }
    virtual const Polygon& getBoundingRegion(const orunav_generic::RobotInternalState2d &s) const {
      switch (s.loadType) {
      case orunav_generic::RobotInternalState2d::NO_LOAD: 
	return wo_load;
	break;
      case orunav_generic::RobotInternalState2d::EUR_PALLET:
	// TODO!!!!
	return w_load;
	break;
      case orunav_generic::RobotInternalState2d::HALF_PALLET:
	// TODO!!!!
	return w_load2;
	break;
      default:
	assert(false);
	return w_load;
	break;
      }
    }
  private:
    Polygon w_load;
    Polygon w_load2;
    Polygon wo_load;
  };

  class RobotModel2dPitViper : public RobotModel2dInterface {
  public:
    RobotModel2dPitViper() { 
      bound.points.push_back(Eigen::Vector2d(8.100, 4.125));
      bound.points.push_back(Eigen::Vector2d(-6.920, 4.125));
      bound.points.push_back(Eigen::Vector2d(-6.920, -3.430));
      bound.points.push_back(Eigen::Vector2d(8.100, -3.430));
    }
    virtual const Polygon& getBoundingRegion(const orunav_generic::RobotInternalState2d &s) const {
      // Hardcoded values only - this really is only a square
      return bound;
    }
  private:
    Polygon bound;
  };

  class RobotModel2dSnowWhite : public RobotModel2dInterface {
  public:
    RobotModel2dSnowWhite() { 
      bound.points.push_back(Eigen::Vector2d(0.86, 0.36));
      bound.points.push_back(Eigen::Vector2d(0.86, -0.36));
      bound.points.push_back(Eigen::Vector2d(-0.24, -0.36));
      bound.points.push_back(Eigen::Vector2d(-0.24, 0.36));
      /* bound.points.push_back(Eigen::Vector2d(0.6, 0.4)); */
      /* bound.points.push_back(Eigen::Vector2d(0.6, -0.4)); */
      /* bound.points.push_back(Eigen::Vector2d(-0.3, -0.4)); */
      /* bound.points.push_back(Eigen::Vector2d(-0.3, 0.4)); */
    }
    virtual const Polygon& getBoundingRegion(const orunav_generic::RobotInternalState2d &s) const {
      // Hardcoded values only - this really is only a square
      return bound;
    }
  private:
    Polygon bound;
  };
  

  class RobotModel2dOneSquareMeter : public RobotModel2dInterface {
  public:
    RobotModel2dOneSquareMeter() { 
      bound.points.push_back(Eigen::Vector2d(0.5, 0.5));
      bound.points.push_back(Eigen::Vector2d(0.5, -0.5));
      bound.points.push_back(Eigen::Vector2d(-0.5, -0.5));
      bound.points.push_back(Eigen::Vector2d(-0.5, 0.5));
    }
    virtual const Polygon& getBoundingRegion(const orunav_generic::RobotInternalState2d &s) const {
      // Hardcoded values only - this really is only a square
      return bound;
    }
  private:
    Polygon bound;
  };


  class RobotModel2dXa15 : public RobotModel2dInterface {
  public:
    RobotModel2dXa15() {
      bound.points.push_back(Eigen::Vector2d(5.3, 1.35)); //fl
      bound.points.push_back(Eigen::Vector2d(5.3, -1.35)); //fr
      bound.points.push_back(Eigen::Vector2d(-1.3, -1.35)); //rr
      bound.points.push_back(Eigen::Vector2d(-1.3, 1.35)); //rl
    }
    virtual const Polygon& getBoundingRegion(const orunav_generic::RobotInternalState2d &s) const {
      // Hardcoded values only - this really is only a square
      return bound;
    }
  private:
    Polygon bound;
  };

  class RobotModel2dHRP: public RobotModel2dInterface {
  public:
    RobotModel2dHRP() {
//      bound.points.push_back(Eigen::Vector2d(0.65-0.165, 0.25));
//      bound.points.push_back(Eigen::Vector2d(0.65-0.165, -0.25));
//      bound.points.push_back(Eigen::Vector2d(-0.165, -0.25));
//      bound.points.push_back(Eigen::Vector2d(-0.165, 0.25));
      bound.points.push_back(Eigen::Vector2d(0.65-0.165-0.1, 0.25-0.1));
      bound.points.push_back(Eigen::Vector2d(0.65-0.165-0.1, -0.25+0.1));
      bound.points.push_back(Eigen::Vector2d(-0.165+0.1, -0.25+0.1));
      bound.points.push_back(Eigen::Vector2d(-0.165+0.1, 0.25-0.1));
    }
    virtual const Polygon& getBoundingRegion(const orunav_generic::RobotInternalState2d &s) const {
      // Hardcoded values only - this really is only a square
      return bound;
    }
  private:
    Polygon bound;
  };

  // TODO, the numbering orignates from the constraint extractor - could be made a bit more fancy :-).
  class RobotModelTypeFactory {
  public:
    RobotModel2dInterface* getModel(int type_id) {
      switch (type_id) {
      case 1:
	return &model1;
      case 2:
	return &model2;
      case 3:
	return &model3;
      case 4:
	return &model4;
      case 5:
	return &model5;
      case 6:
	return &model6;
      case 7:
	return &model7;
      default:
	return &model1;
      }
    }

    boost::shared_ptr<RobotModel2dInterface> create (const std::string &type) {
      if (type == "cititruck") {
        return boost::make_shared<RobotModel2dCiTiTruck>();
      }
      else if (type == "cititruck_arm") {
        return boost::make_shared<RobotModel2dCiTiTruckWithArm>();
      }
      else if (type == "snowwhite") {
        return boost::make_shared<RobotModel2dSnowWhite>();
      }
      else if (type == "one_square_meter") {
        return boost::make_shared<RobotModel2dOneSquareMeter>();
      }
      else if (type == "hx01" || type == "xa15") {
        return boost::make_shared<RobotModel2dXa15>();
      }
      else if (type == "hrp") {
        return boost::make_shared<RobotModel2dHRP>();
      }
      else if (type == "bttruck") {
        return boost::make_shared<RobotModel2dBtTruck>();
      }
      else {
        assert(false);
      }
      return boost::make_shared<RobotModel2dCiTiTruck>();
    }


  private:
    orunav_geometry::RobotModel2dSnowWhite model1;
    orunav_geometry::RobotModel2dCiTiTruck model2;
    orunav_geometry::RobotModel2dOneSquareMeter model3;
    orunav_geometry::RobotModel2dPitViper model4;
    orunav_geometry::RobotModel2dXa15 model5;
    orunav_geometry::RobotModel2dHRP model6;
    orunav_geometry::RobotModel2dBtTruck model7;
  };

  class RobotModel2dWithState : public orunav_generic::Point2dContainerInterface, public orunav_generic::Point2dCollisionCheckInterface {
  public:
    RobotModel2dWithState(const RobotModel2dInterface &m);
    
    // Interfaces
    virtual bool collisionPoint2d(const Eigen::Vector2d &pos) const { return posePolygon.collisionPoint2d(pos); } 
    virtual void setPoint2d(const Eigen::Vector2d &pt, size_t idx) { assert(false); } // Should probably never be needed.
    virtual Eigen::Vector2d getPoint2d(size_t idx) const { return posePolygon.getPoint2d(idx); }
    virtual size_t sizePoint2d() const { return posePolygon.sizePoint2d(); }
    
    //! Must call this to update the robot model and the pose of the polygon
    void update(const orunav_generic::Pose2d &p, const orunav_generic::RobotInternalState2d &is);
    const Polygon& getPosePolygon() const { return posePolygon; }
    const orunav_generic::Pose2d& getRobotPose() const { return robotPose; }
    void getBoundingBox(Eigen::Vector2d &bottomLeft, Eigen::Vector2d &topRight) const {
      orunav_geometry::getBoundingBox2d(this->posePolygon, bottomLeft, topRight);
    }

  private:
    const RobotModel2dInterface& model;
    Polygon posePolygon;
    orunav_generic::Pose2d robotPose;
  };

  
} // namespace

