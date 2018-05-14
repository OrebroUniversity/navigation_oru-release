#pragma once

#include <sensor_msgs/LaserScan.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>

namespace orunav_vehicle_execution {


  
class LaserFilterInterface {
public:
  virtual bool valid(size_t i) const;
};

 sensor_msgs::LaserScan filter_laser_scan(const LaserFilterInterface &filter, const sensor_msgs::LaserScanConstPtr &msg) {
   /* sensor_msgs::LaserScan ret; */
   /* ret = *msg; */
   /* for (size_t i = 0; i < msg->ranges.size(); i++) { */
   /*   if (!filter.isValid(i)) { */
   /*     // Invalidate this reading (setting it to max_range + 1. */
   /*     ret.ranges[i] = ret.max_range + 1; */
   /*   } */
   /* } */
 }
 
class LaserFilter {
public:
   LaserFilter() { }

   void load(const std::string &filename) {
     std::ifstream ifs(filename.c_str());
     boost::archive::text_iarchive ar(ifs);
     ar & valid_;
     ar & initialized_;
   }

   void save(const std::string &filename) const {
     std::ofstream ofs(filename.c_str());
     boost::archive::text_oarchive ar(ofs);
     ar & valid_;
     ar & initialized_;
   }

   void resize(size_t size) {
      valid_.resize(size);
      for (size_t i = 0; i < valid_.size(); i++) {
          valid_[i] = false;
      }
      initialized_ = true;
   }
  bool isInitialized() const { return initialized_; }

   void setValid(size_t i, bool valid)
   {
     if (i < 0) return;
     if (i >= valid_.size()) return;
     valid_[i] = valid;
   }

   bool valid(size_t i) const { return valid_[i]; }
private:
   std::vector<bool> valid_;
   bool initialized_;

};

class LaserFilterNavSimulated {
public:
   LaserFilterNavSimulated() { }

   bool isInitialized() const { return true; }

   bool valid(size_t i) const {
      if (i >= 205 && i <= 240) {
        return false;
      }
      if (i >= 390 && i <= 425) {
        return false;
      }
      return true;
   }
};

 
}
