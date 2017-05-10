#pragma once

#include <orunav_geometry/polygon.h>


// First, what really is a constraint here? Essentially there is two different spatial constraints, one that covers the outer boundaries for the vehicles and the other related to the state that the controller operates on. The controller needs to operate on convex states only. The other spatial constraints are used for the scheduling / collision checks / path planning, wheras the inner ones are used for the controller and local obstacle avoidance. 
// For now on the convex polygons are keept, however, in some narrow enviornments this separation might be very useful.

namespace constraint_extract {

  //! Interface class to all constraint extractors.
  /*!
   * This is to enforce the different spatial constraints:
   * inner - used by the controller, local obstacle avoidance and
   * outer - used by path planning, scheuling etc. (could in the future be non-convex).
   */
class ConstraintExtractorInterface {
  virtual const orunav_geometry::Polygons& getOuterConstraints() const = 0;
  virtual const orunav_geometry::Polygons& getInnerConstraints() const = 0;
};




} // namespace
