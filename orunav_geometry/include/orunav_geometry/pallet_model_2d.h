#pragma once

#include <orunav_geometry/polygon.h>

namespace orunav_geometry {

  //! Represents a model of a vehicle in 2d - the main trick here is that the model should be capable of changing depending on the 'internal' state of the vehicle. For example, different steering angles (actuated vehicles), or if the vehicle has load (like a pallet), which would alther the shape.
  
  class PalletModel2dInterface : public orunav_generic::PickUpPose2dInterface {
  public:
    //! Returns a polygon (not nessecarly convex) of the pallet
    /*!
     * Everything is given in the robot coordinate frame.
     */
    virtual const Polygon& getBoundingRegion() const = 0;
    // These are poses related to how to approach and pickup the pallet, this is highly related to the type of vehicle used. All numbers here are for the CiTiTruck. Incase another vehicle is used then this needs to be defined based on the vehicle.
    virtual const orunav_generic::Pose2dContainerInterface& getPickupPoses() const = 0;
    virtual const orunav_generic::Pose2dContainerInterface& getPickupPosesClose() const = 0;
    virtual const orunav_generic::Pose2d &getPickupPoseOffset() const = 0;
  };


  // Each type could be formed as a singleton or another clever way. Another question would be to store this as parameters, but this is the simplest way for now. When entering the coordinates - do it counter clock-wise.
  
  class PalletModel2dEUR : public PalletModel2dInterface {
  public:
    PalletModel2dEUR() {
      border.points.push_back(Eigen::Vector2d(0.6, 0.4));
      border.points.push_back(Eigen::Vector2d(0.6, -0.4));
      border.points.push_back(Eigen::Vector2d(-0.6, -0.4));
      border.points.push_back(Eigen::Vector2d(-0.6, 0.4));

      pick_poses.push_back(orunav_generic::Pose2d(2.2, 0., 0.)); // was 1.2 = one pallet length, but that risks making 'way to short path'
      pick_poses.push_back(orunav_generic::Pose2d(-2.2, 0., M_PI)); // was 1.2 = one pallet length
      pick_poses.push_back(orunav_generic::Pose2d(1.2, 0., 0.)); // Chitt - adding to imporve pallet picking?

      pick_poses_close.push_back(orunav_generic::Pose2d(0.6, 0., 0.));
      pick_poses_close.push_back(orunav_generic::Pose2d(-0.6, 0., M_PI));

      pick_pose_offset = orunav_generic::Pose2d(0., 0., 0.);
    }
    virtual const Polygon& getBoundingRegion() const {
      return border;
    }

    virtual const orunav_generic::Pose2dContainerInterface& getPickupPoses() const {
      return pick_poses;
    }
    virtual const orunav_generic::Pose2dContainerInterface& getPickupPosesClose() const {
      return pick_poses_close;
    }

    const orunav_generic::Pose2d& getPickupPoseOffset() const {
      return pick_pose_offset;
    }
    
  private:
    Polygon border;
    orunav_generic::Pose2dVec pick_poses;
    orunav_generic::Pose2dVec pick_poses_close;
    orunav_generic::Pose2d pick_pose_offset;
  };

  class PalletModel2dHalf : public PalletModel2dInterface {
  public:
    PalletModel2dHalf() { 
      bound.points.push_back(Eigen::Vector2d(0.3, 0.4));
      bound.points.push_back(Eigen::Vector2d(0.3, -0.4));
      bound.points.push_back(Eigen::Vector2d(-0.3, -0.4));
      bound.points.push_back(Eigen::Vector2d(-0.3, 0.4));

      pick_poses.push_back(orunav_generic::Pose2d(0.7, 0., 0.));
      pick_poses.push_back(orunav_generic::Pose2d(-0.7, 0., M_PI));
      
      pick_poses_close.push_back(orunav_generic::Pose2d(0.35, 0., 0.));
      pick_poses_close.push_back(orunav_generic::Pose2d(-0.35, 0., M_PI));

      pick_pose_offset = orunav_generic::Pose2d(-0.1, 0., 0.);
    }
    virtual const Polygon& getBoundingRegion() const {
      return bound;
    }

    virtual const orunav_generic::Pose2dContainerInterface& getPickupPoses() const {
      return pick_poses;
    }

    virtual const orunav_generic::Pose2dContainerInterface& getPickupPosesClose() const {
      return pick_poses_close;
    }

    const orunav_generic::Pose2d& getPickupPoseOffset() const {
      return pick_pose_offset;
    }

  private:
    Polygon bound;
    orunav_generic::Pose2dVec pick_poses;
    orunav_generic::Pose2dVec pick_poses_close;
    orunav_generic::Pose2d pick_pose_offset;
  };
  

  class PalletModel2dWithState : public orunav_generic::Point2dContainerInterface, public orunav_generic::Point2dCollisionCheckInterface, public orunav_generic::PickUpPose2dInterface {
  public:
    PalletModel2dWithState(const PalletModel2dInterface &m);
    
    // Interfaces
    virtual bool collisionPoint2d(const Eigen::Vector2d &pos) const { return posePolygon.collisionPoint2d(pos); } 
    virtual void setPoint2d(const Eigen::Vector2d &pt, size_t idx) { assert(false); } // Should probably never be needed.
    virtual Eigen::Vector2d getPoint2d(size_t idx) const { return posePolygon.getPoint2d(idx); }
    virtual size_t sizePoint2d() const { return posePolygon.sizePoint2d(); }
    
    //! Must call this to update the robot model and the pose of the polygon
    void update(const orunav_generic::Pose2d &p);
    const Polygon& getPosePolygon() const { return posePolygon; }

    const orunav_generic::Pose2dContainerInterface& getPickupPoses() const {
      return pickupPoses;
    }

    const orunav_generic::Pose2dContainerInterface& getPickupPosesClose() const {
      return pickupPosesClose;
    }

    const orunav_generic::Pose2d& getPickupPoseOffset() const {
      return pickupPoseOffset;
    }

  private:
    Polygon posePolygon;
    orunav_generic::Pose2dVec pickupPoses;
    orunav_generic::Pose2dVec pickupPosesClose;
    orunav_generic::Pose2d pickupPoseOffset;


    Polygon posePolygon_orig;
    orunav_generic::Pose2dVec pickupPoses_orig;
    orunav_generic::Pose2dVec pickupPosesClose_orig;
    orunav_generic::Pose2d pickupPoseOffset_orig;
  };
  
} // namespace
