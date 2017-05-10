#include <orunav_geometry/pallet_model_2d.h>
#include <orunav_generic/path_utils.h>

using namespace orunav_geometry;

PalletModel2dWithState::PalletModel2dWithState(const PalletModel2dInterface &m) {
  
  const Polygon& poly = m.getBoundingRegion();
  posePolygon_orig = Polygon(poly); // Overwrite the poly with the current model configuration...
  pickupPoses_orig = m.getPickupPoses();
  pickupPosesClose_orig = m.getPickupPosesClose();
  pickupPoseOffset_orig = m.getPickupPoseOffset();
  
}

void
PalletModel2dWithState::update(const orunav_generic::Pose2d &p)
{
  posePolygon = posePolygon_orig;
  movePoint2dContainer(posePolygon, p); // ... and move it to the right pose.

  pickupPoses = pickupPoses_orig;
  orunav_generic::moveToOrigin(pickupPoses, p);

  pickupPosesClose = pickupPosesClose_orig;
  orunav_generic::moveToOrigin(pickupPosesClose, p);

  orunav_generic::Pose2d offset = pickupPoseOffset_orig;
  pickupPoseOffset = orunav_generic::addPose2d(p, offset);
}

