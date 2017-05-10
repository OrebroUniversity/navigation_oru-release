#pragma once

#include <orunav_generic/pose2d.h>
#include <vector>

namespace orunav_generic
{
  class Pose2dInterface
  {
  public:
    virtual Pose2d getPose2d() const = 0;
    virtual void setPose2d(const Pose2d& pose) = 0;
  };
  
  class Pose2dContainerInterface
  {
  public:
    virtual Pose2d getPose2d(size_t idx) const = 0;
    virtual void setPose2d(const Pose2d& pose, size_t idx) = 0;
    virtual size_t sizePose2d() const = 0;
  };


    //! Returns a vector of relative poses the center of the object (pallets) from which objects can be picked up from.
  class PickUpPose2dInterface {
  public:
    //! This is where you can safely drive to the object without any risk hitting it.
    virtual const Pose2dContainerInterface& getPickupPoses() const = 0;
    //! From this point onwards you must go straight to the object centre (for the pallet the forks will already be partly under the pallet). Unless you are very certain about the object pose, you might hit it.
    virtual const Pose2dContainerInterface& getPickupPosesClose() const = 0;
  };
  
  class PositionInterface
  {
  public:
    virtual Eigen::Vector3d getPos() const = 0;
    virtual void setPos(const Eigen::Vector3d& pos) = 0;
  };
  
  class PositionContainerInterface
  {
  public:
    virtual Eigen::Vector3d getPos(size_t idx) const = 0;
    virtual void setPos(const Eigen::Vector3d& pos, size_t idx) = 0;
    virtual size_t sizePos() const = 0;
  };
  
  class Point2dContainerInterface
  {
  public:
    virtual Eigen::Vector2d getPoint2d(size_t idx) const = 0;
    virtual void setPoint2d(const Eigen::Vector2d &pt, size_t idx) = 0;
    virtual size_t sizePoint2d() const = 0;
  };

  class SteeringAngleInterface
  {
  public:
    virtual double getSteeringAngle() const = 0;
    virtual void setSteeringAngle(double angle) = 0;
  };
  
  class SteeringAngleContainerInterface
  {
  public:
    virtual double getSteeringAngle(size_t idx) const = 0;
    virtual void setSteeringAngle(double angle, size_t idx) = 0;
    virtual size_t sizeSteeringAngle() const = 0;
  };
  
  class State2dInterface : public Pose2dInterface, public SteeringAngleInterface
  {
    
  };
  
  class ControlInterface
  {
  public:
    virtual double getFwdVel() const = 0; 
    virtual void setFwdVel(double fwdVel) = 0;
    virtual double getRotVel() const = 0;
    virtual void setRotVel(double rotVel) = 0;
  };

  class LoadInterface
  {
  public:
    virtual bool getLoad() const = 0;
    virtual void setLoad(bool load) = 0;
  };

  class PathInterface : public Pose2dContainerInterface, public SteeringAngleContainerInterface
  {
  public:
    size_t sizePath() const { return sizePose2d(); }
  };

  class PathsInterface
  {
  public:
    virtual size_t sizePaths() const = 0;
    virtual PathInterface& getPath(size_t idx) = 0;
    virtual const PathInterface& getPath(size_t idx) const = 0;
  };
  
  class TrajectoryInterface : public PathInterface
  {
  public:
    virtual double getDriveVel(size_t idx) const = 0; 
    virtual void setDriveVel(double v, size_t idx) = 0;
    virtual double getSteeringVel(size_t idx) const = 0;
    virtual void setSteeringVel(double w, size_t idx) = 0;
    size_t sizeTrajectory() const { return sizePose2d(); }
  };
  
  class TrajectoryChunksInterface// : public PathsInterface
  {
  public:
    virtual TrajectoryInterface& getChunk(size_t idx) = 0;
    virtual const TrajectoryInterface& getChunk(size_t idx) const = 0;
    virtual size_t sizeChunks() const = 0;
    virtual size_t getSequenceStartNum() const = 0;
  };
  
  class CollisionCheckInterface
  {
  public:
    virtual bool collision(const orunav_generic::Pose2d &pose) const = 0;
  };
  
  class Point2dCollisionCheckInterface
  {
  public:
    virtual bool collisionPoint2d(const Eigen::Vector2d &pos) const = 0;
  };

  class DeltaTInterface
  {
  public:
    virtual double& getDeltaT(size_t idx) = 0;
    virtual const double& getDeltaT(size_t idx) const = 0;
    virtual size_t sizeDeltaTVec() const = 0;
  };

} // namespace

