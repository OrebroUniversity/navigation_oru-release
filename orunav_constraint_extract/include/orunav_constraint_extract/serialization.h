#pragma once

#include <boost/serialization/split_free.hpp>
#include <orunav_constraint_extract/polygon_constraint.h>
#include <orunav_geometry/serialization.h>

namespace boost {
  namespace serialization {
    class access;
    // --------------------------------- PolygonConstraint ----------------------------
    template<typename Archive>
      void serialize(Archive& ar, constraint_extract::PolygonConstraint& pc, const unsigned version) {
      ar & pc.poly_;
      ar & pc.thBounds_;
      ar & pc.weight_;
      ar & pc.feasiblePose_;
    }

    // --------------------------------- PolygonConstraintsVec ----------------------------
    template<typename Archive>
      void save(Archive& ar, const constraint_extract::PolygonConstraintsVec& pcs, const unsigned version) {
      size_t size = pcs.size();
      ar << size;
      for (size_t i = 0; i < size; ++i) {
      	ar << pcs[i];
      }
    }
    
    template<typename Archive>
      void load(Archive& ar, constraint_extract::PolygonConstraintsVec& pcs, const unsigned version) {
      size_t size;
      ar >> size;
      pcs.resize(size);
      for (size_t i = 0; i < size; ++i) {
      	ar >> pcs[i];
      }
    }

    /* void serialize(Archive& ar, constraint_extract::PolygonConstraintsVec& pcs, const unsigned version) { */
    /*   ar & pcs; */
    /* } */

    // --------------------------------- PolygonConstraintsLookup::Params ---------------------------
    template<typename Archive>
      void serialize(Archive& ar, constraint_extract::PolygonConstraintsLookup::Params& o, const unsigned version) {
      ar & o.x_left;
      ar & o.x_right;
      ar & o.y_up;
      ar & o.y_down;
      ar & o.th_bounds;
      ar & o.resolution;
      ar & o.th_sweep_incr;
      ar & o.use_th_bounds;
    }
    
    // --------------------------------- OccupancyGrid ---------------------------
    template<typename Archive>
      void save(Archive& ar, const nav_msgs::OccupancyGrid& o, const unsigned version) {
      
      ar << o.header.frame_id;
      ar << o.info.width;
      ar << o.info.height;
      ar << o.info.resolution;
      ar << o.info.origin.position.x;
      ar << o.info.origin.position.y;
      ar << o.info.origin.position.z;
      ar << o.info.origin.orientation.x;
      ar << o.info.origin.orientation.y;
      ar << o.info.origin.orientation.z;
      ar << o.info.origin.orientation.w;

      size_t size = o.data.size();
      ar << size;
      for (size_t i = 0; i < size; i++) {
	ar << o.data[i];
      }
    }
    
    template<typename Archive>
      void load(Archive& ar, nav_msgs::OccupancyGrid& o, const unsigned version) {
      ar >> o.header.frame_id;
      ar >> o.info.width;
      ar >> o.info.height;
      ar >> o.info.resolution;
      ar >> o.info.origin.position.x;
      ar >> o.info.origin.position.y;
      ar >> o.info.origin.position.z;
      ar >> o.info.origin.orientation.x;
      ar >> o.info.origin.orientation.y;
      ar >> o.info.origin.orientation.z;
      ar >> o.info.origin.orientation.w;
      
      size_t size;
      ar >> size;
      o.data.resize(size);
      for (size_t i = 0; i < size; i++) {
	ar >> o.data[i];
      }
    }
    
    
  }
}


BOOST_SERIALIZATION_SPLIT_FREE(nav_msgs::OccupancyGrid)
BOOST_SERIALIZATION_SPLIT_FREE(constraint_extract::PolygonConstraintsVec)
