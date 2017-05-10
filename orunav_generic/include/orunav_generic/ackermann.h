#pragma once

#include <Eigen/Core>
#include <orunav_generic/interfaces.h>
#include <orunav_generic/types.h>
#include <angles/angles.h>

namespace orunav_generic {

inline double getWheelAngleFromRotationPoint(const Eigen::Vector2d &currentWheelPosition,
                                             const Eigen::Vector2d &rotationPoint) {
  Eigen::Vector2d offset = rotationPoint - currentWheelPosition;
  double angle = atan2(offset[1], offset[0]) + M_PI/2.;
  if (rotationPoint[1] > 0)
    angle += M_PI;
  return angles::normalize_angle(angle);
}

inline double getWheelVelocityFactorFromRotationAndVelocityPoint(const Eigen::Vector2d &currentWheelPosition,
                                                           const Eigen::Vector2d &rotationPoint, 
                                                           const Eigen::Vector2d &velocityPoint) {
  
  Eigen::Vector2d r = velocityPoint - rotationPoint;
  Eigen::Vector2d r2 = currentWheelPosition - rotationPoint;
  return r2.norm()/r.norm();

}

// Assumes that steeringAngle != 0
inline Eigen::Vector2d getRotationPointFromSteerDriveWheel(const Eigen::Vector2d& steerDriveWheelPosition,
                                                           const double &steeringAngle) {
  Eigen::Vector2d ret;
  ret[1] = steerDriveWheelPosition[0] / tan(steeringAngle) + steerDriveWheelPosition[1];
  ret[0] = 0.; // Here the point always lies on the y axis, steerDriveWheelPosition[1] is the offset of the steerdrive wheel along the y axis.
  return ret;
}

// Assumes that steering1Angle != steering2Angle
inline Eigen::Vector2d getRotationPointFromDualSteerDriveWheel(const Eigen::Vector2d& steerDriveWheelPosition, const double &steering1Angle, const double &steering2Angle)
{
  Eigen::Vector2d ret;
  // The rotation point is given in the vehicle coordinate frame. In dual steering mode the rear wheel can also turn but the center of the rear axis is the center of the vehicle frame.
  // Currently assumes that both "virtual wheels" / wheels are located along the x axis.
  double L = steerDriveWheelPosition[0]; // offset along the x axis.
  double a = L*cos(-steering1Angle)/sin(-steering1Angle+steering2Angle);
  ret[0] = a*sin(steering2Angle);
  ret[1] = -a*cos(steering2Angle);
  return ret;
}

inline double getWheelAngleFromSteerDriveWheel(const Eigen::Vector2d& currentWheelPosition,
                                               const Eigen::Vector2d& steerDriveWheelPosition,
                                               const double &steeringAngle) {
  if (fabs(steeringAngle) < 0.00001)
    return 0.;
  
  Eigen::Vector2d rp = getRotationPointFromSteerDriveWheel(steerDriveWheelPosition, steeringAngle);
  return getWheelAngleFromRotationPoint(currentWheelPosition, rp);
}

inline double getWheelAngleFromDualSteerDriveWheel(const Eigen::Vector2d& currentWheelPosition,
                                                   const Eigen::Vector2d& steerDriveWheelPosition,
                                                   const double &steering1Angle,
                                                   const double &steering2Angle) {
  if (fabs(steering1Angle - steering2Angle) < 0.00001)
    return 0.;
  
  Eigen::Vector2d rp = getRotationPointFromDualSteerDriveWheel(steerDriveWheelPosition, steering1Angle, steering2Angle);
  return getWheelAngleFromRotationPoint(currentWheelPosition, rp);
}



// Here the velocity is given at the  steerDriveWheelPosition.
inline double getWheelVelocityFromSteerDriveWheel(const Eigen::Vector2d& currentWheelPosition,
                                                  const Eigen::Vector2d& steerDriveWheelPosition,
                                                  const double &steeringAngle,
                                                  const double &velocity) {

  if (fabs(steeringAngle) < 0.00001)
    return velocity;

  Eigen::Vector2d rp = getRotationPointFromSteerDriveWheel(steerDriveWheelPosition, steeringAngle);
  return getWheelVelocityFactorFromRotationAndVelocityPoint(currentWheelPosition, rp, steerDriveWheelPosition)*velocity;
}

// Note, here the velocity is given at the center of the rear axis!
inline double getWheelVelocityFromVehicleFrame(const Eigen::Vector2d& currentWheelPosition,
                                               const Eigen::Vector2d& steerDriveWheelPosition,
                                               const double &steering1Angle,
                                               const double &steering2Angle,
                                               const double &velocity) {

  if (fabs(steering1Angle - steering2Angle) < 0.00001)
    return velocity;

  Eigen::Vector2d rp = getRotationPointFromDualSteerDriveWheel(steerDriveWheelPosition, steering1Angle, steering2Angle);
  Eigen::Vector2d vp(0,0); // The velocity point is given at the rear axis.
  
  return getWheelVelocityFactorFromRotationAndVelocityPoint(currentWheelPosition, rp, vp)*velocity;
}

class SteerDriveWheel {
public:
  SteerDriveWheel(double x, double y) { 
    pos_[0] = x;
    pos_[1] = y;
  }
  
  double getSteeringAngle(const Eigen::Vector2d &rotationPoint) const {
    return getWheelAngleFromRotationPoint(pos_, rotationPoint);
  }

  double getVelocityFactor(const Eigen::Vector2d &rotationPoint, const Eigen::Vector2d &velocityPoint) const {
    return getWheelVelocityFactorFromRotationAndVelocityPoint(pos_, rotationPoint, velocityPoint);
  }

  const Eigen::Vector2d& getPosition2d() const { return pos_; }
  
  double steeringAngle;
  double velocityFactor;

private:
  Eigen::Vector2d pos_;
};


class AckermannModelFromSteerDrive : public Pose2dContainerInterface {
public:
  void setSteerDriveWheelPos(double x, double y) {
    sd_pos_[0] = x;
    sd_pos_[1] = y;
  }

  void addWheel(const SteerDriveWheel &wheel) {
    wheels.push_back(wheel);
  }
  
  void updateSteeringAngle(double angle) {
    if (fabs(angle) < 0.00001) {
      for (size_t i = 0; i < wheels.size(); i++) {
        wheels[i].steeringAngle = 0.;
        wheels[i].velocityFactor = 1.;
      }
      return;
    }

    Eigen::Vector2d rp = getRotationPointFromSteerDriveWheel(sd_pos_, angle);
    rp_ = rp;
    for (size_t i = 0; i < wheels.size(); i++) {
      wheels[i].steeringAngle = wheels[i].getSteeringAngle(rp);
      wheels[i].velocityFactor = wheels[i].getVelocityFactor(rp, sd_pos_);
    }

  }
  
  const Eigen::Vector2d& getSteerDriveWheelPos() const {
    return sd_pos_;
  }

  const Eigen::Vector2d& getLastRotationPoint() const {
    return rp_;
  }
  
  // Below is currently only for visualization purposes...
  // returns the wheel poses.
  virtual Pose2d getPose2d(size_t idx) const {
    Pose2d p;
    p[0] = wheels[idx].getPosition2d()[0];
    p[1] = wheels[idx].getPosition2d()[1];
    p[2] = wheels[idx].steeringAngle;
    return p;
  }
  virtual void setPose2d(const Pose2d& pose, size_t idx) {
    // Empty
  }
  virtual size_t sizePose2d() const {
    return wheels.size();

  }

  std::vector<SteerDriveWheel, Eigen::aligned_allocator<SteerDriveWheel> > wheels;
private:
  Eigen::Vector2d sd_pos_;
  Eigen::Vector2d rp_;
};

// Only for debuggin / visualization...
PositionVec generatePositionVecUsingAckermanModelFromSteerDrive(const AckermannModelFromSteerDrive& ackermann) {
  PositionVec pts;
  for (size_t i = 0; i < ackermann.wheels.size(); i++) {
    pts.push_back(Eigen::Vector3d(ackermann.wheels[i].getPosition2d()[0],
                                  ackermann.wheels[i].getPosition2d()[1],
                                  ackermann.wheels[i].velocityFactor));
  }
  pts.push_back(Eigen::Vector3d(ackermann.getLastRotationPoint()[0],
                                ackermann.getLastRotationPoint()[1],
                                0.)); // This should have the speed = zero.
  return pts;
}



//! Dual steering model, assumption is made that the vehicles frame is at the center of the rear axis.
class AckermannModelFromDualSteerDrive : public Pose2dContainerInterface {
public:
  void setSteerDriveWheelPos(double x, double y) {
    sd_pos_[0] = x;
    sd_pos_[1] = y;
  }

  void addWheel(const SteerDriveWheel &wheel) {
    wheels.push_back(wheel);
  }
  
  void updateSteeringAngle(double angle1, double angle2) {
    if (fabs(angle1 - angle2) < 0.00001 ) {
      for (size_t i = 0; i < wheels.size(); i++) {
        wheels[i].steeringAngle = 0.;
        wheels[i].velocityFactor = 1.;
      }
      return;
    }

    Eigen::Vector2d rp = getRotationPointFromDualSteerDriveWheel(sd_pos_, angle1, angle2);
    rp_ = rp;
    Eigen::Vector2d vp(0,0);
    for (size_t i = 0; i < wheels.size(); i++) {
      wheels[i].steeringAngle = wheels[i].getSteeringAngle(rp);
      wheels[i].velocityFactor = wheels[i].getVelocityFactor(rp, vp); // Vp is the centre of the rear axis.
    }

  }
  
  const Eigen::Vector2d& getSteerDriveWheelPos() const {
    return sd_pos_;
  }

  const Eigen::Vector2d& getLastRotationPoint() const {
    return rp_;
  }
  
  // Below is currently only for visualization purposes...
  // returns the wheel poses.
  virtual Pose2d getPose2d(size_t idx) const {
    Pose2d p;
    p[0] = wheels[idx].getPosition2d()[0];
    p[1] = wheels[idx].getPosition2d()[1];
    p[2] = wheels[idx].steeringAngle;
    return p;
  }
  virtual void setPose2d(const Pose2d& pose, size_t idx) {
    // Empty
  }
  virtual size_t sizePose2d() const {
    return wheels.size();

  }

  std::vector<SteerDriveWheel, Eigen::aligned_allocator<SteerDriveWheel> > wheels;
private:
  Eigen::Vector2d sd_pos_;
  Eigen::Vector2d rp_;
};

// Only for debuggin / visualization...
PositionVec generatePositionVecUsingAckermanModelFromSteerDrive(const AckermannModelFromDualSteerDrive& ackermann) {
  PositionVec pts;
  for (size_t i = 0; i < ackermann.wheels.size(); i++) {
    pts.push_back(Eigen::Vector3d(ackermann.wheels[i].getPosition2d()[0],
                                  ackermann.wheels[i].getPosition2d()[1],
                                  ackermann.wheels[i].velocityFactor));
  }
  pts.push_back(Eigen::Vector3d(ackermann.getLastRotationPoint()[0],
                                ackermann.getLastRotationPoint()[1],
                                0.)); // This should have the speed = zero.
  return pts;
}





} // namespace
