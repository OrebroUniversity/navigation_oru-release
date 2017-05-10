#pragma once

#include <boost/serialization/split_free.hpp>
#include <orunav_generic/types.h>

// Forward declaration of class boost::serialization::access
namespace boost {
  namespace serialization {
    class access;
    
    // Serialization of Eigen::Vector2d
    template<typename Archive>
      void serialize(Archive& ar, Eigen::Vector2d& o, const unsigned int version) {
      ar & o[0] & o[1];
    }

    // Serialization of Pose2d
    template<typename Archive>
      void serialize(Archive& ar, orunav_generic::Pose2d& o, const unsigned int version) {
      ar & o[0] & o[1] & o[2];
    }
    
    template<typename Archive>
      void serialize(Archive& ar, orunav_generic::Control& o, const unsigned version) {
      ar & o.v & o.w;
    }

    template<typename Archive>
    void serialize(Archive& ar, Eigen::Affine3d &o, const unsigned version) {
      for (int i = 0; i < 16; i++) {
        ar & o.data()[i];
      }
    }

    // Pose2dVec is a mixture of stl and eigen, the default serialization didn't work here.
    /* template<typename Archive> */
    /*   void serialize(Archive& ar, orunav_generic::Pose2dVec& o, const unsigned version) { */
    /*   ar & o; */
    /* } */
    // --------------------------------- Pose2dVec ----------------------------
    template<typename Archive>
      void save(Archive& ar, const orunav_generic::Pose2dVec objs, const unsigned version) {
      size_t size = objs.sizePose2d();
      ar << size;
      for (size_t i = 0; i < size; ++i) {
	ar << objs.getPose2d(i)[0];
	ar << objs.getPose2d(i)[1];
	ar << objs.getPose2d(i)[2];
      }
    }
    
    template<typename Archive>
      void load(Archive& ar, orunav_generic::Pose2dVec& objs, const unsigned version) {
      size_t size;
      ar >> size;
      objs.resize(size);
      for (size_t i = 0; i < size; ++i) {
	ar >> objs.getPose2d(i)[0];
	ar >> objs.getPose2d(i)[1];
	ar >> objs.getPose2d(i)[2];
      }
    }
    
    // --------------------------------- Point2dVec ----------------------------
    template<typename Archive>
      void save(Archive& ar, const orunav_generic::Point2dVec objs, const unsigned version) {
      size_t size = objs.sizePoint2d();
      ar << size;
      for (size_t i = 0; i < size; ++i) {
      	ar << objs.getPoint2d(i)[0];
      	ar << objs.getPoint2d(i)[1];
      }
    }
    
    template<typename Archive>
      void load(Archive& ar, orunav_generic::Point2dVec& objs, const unsigned version) {
      size_t size;
      ar >> size;
      objs.resize(size);
      Eigen::Vector2d tmp;
      // Point2dVec didn't have a Ref get function - this also worked out fine.
      for (size_t i = 0; i < size; ++i) {
      	ar >> tmp(0);
      	ar >> tmp(1);
	objs.setPoint2d(tmp, i);
      }
    }
    

    // --------------------------------- Path ----------------------------
    template<typename Archive>
      void serialize(Archive& ar, orunav_generic::Path& p, const unsigned version) {
      ar & p.poses & p.steeringAngles;
    }
    
    // --------------------------------- CoordinatedTimes ----------------------------
    // Simply creating a public vector will cause some problems...
    template<typename Archive>
      void save(Archive& ar, const orunav_generic::CoordinatedTimes& objs, const unsigned version) {
      size_t size = objs.size();
      ar << size;
      for (size_t i = 0; i < size; ++i) {
	ar << objs[i];
      }
    }
    
    template<typename Archive>
      void load(Archive& ar, orunav_generic::CoordinatedTimes& objs, const unsigned version) {
      size_t size;
      ar >> size;
      objs.resize(size);
      for (size_t i = 0; i < size; ++i) {
	ar >> objs[i];
      }
    }

  }
}

BOOST_SERIALIZATION_SPLIT_FREE(orunav_generic::Pose2dVec)
BOOST_SERIALIZATION_SPLIT_FREE(orunav_generic::Point2dVec)
BOOST_SERIALIZATION_SPLIT_FREE(orunav_generic::CoordinatedTimes)
