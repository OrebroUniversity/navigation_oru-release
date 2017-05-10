#pragma once

#include <iostream>
#include <string>
#include <orunav_generic/utils.h>
#include <orunav_generic/interfaces.h>

class SensorLogParser {
 public:
  SensorLogParser(const std::string &fileName) {
    load(fileName);
  }
  const orunav_generic::TrajectoryInterface& getSensorStatesAndVelocities() const { return sensor_; }
  
 private:
  std::string fileName_;
  orunav_generic::Trajectory sensor_;

  bool flushToStartTag(std::ifstream &ifs) const {
    while (!ifs.eof()) {
      std::string line;
      getline(ifs, line);
      if (std::string::npos != line.find(std::string("State updated"))) {
	// Found target
	return true;
      }
    }
    return false;
  }

  orunav_generic::State2d parseStateLine(const std::string &str) const {
    orunav_generic::State2d s;
    // This is how a line look like:
    //Updated angles: x =    9.564,  y =    2.956,  theta =   -0.023,  phi =   -0.008
    unsigned pos_start = str.find("=");
    unsigned pos_end = str.find(",", pos_start);

    assert(pos_start+1 < pos_end-1);
    std::string x = str.substr(pos_start+1,pos_end-1); 
    s.pose[0] = orunav_generic::fromString<double>(x);

    //-
    pos_start = str.find("=", pos_end);
    pos_end = str.find(",", pos_start);
    assert(pos_start+1 < pos_end-1);

    std::string y = str.substr(pos_start+1,pos_end-1); 
    s.pose[1] = orunav_generic::fromString<double>(y);

    //-
    pos_start = str.find("=", pos_end);
    pos_end = str.find(",", pos_start);
    assert(pos_start+1 < pos_end-1);

    std::string theta = str.substr(pos_start+1,pos_end-1); 
    s.pose[2] = orunav_generic::fromString<double>(theta);

    //-
    pos_start = str.find("=", pos_end);

    std::string phi = str.substr(pos_start+1); // will take the whole line.
    s.steeringAngle = orunav_generic::fromString<double>(phi);

    return s;
  }

  double parseVelocityLine(const std::string &str) const {
    // This is how a line looks like:
    // Updated steering wheel speed: v = 	0
    // Note that we do not have the velocity of the change of rotation of the steering wheel - only the steering wheels forward movement
    unsigned pos_start = str.find("=");
    std::string v = str.substr(pos_start+1); 
    return orunav_generic::fromString<double>(v);
  }

  bool readStatesAndVelocites(std::ifstream &ifs) {

    std::vector<std::string> lines(3);
    
    for (int i = 0; i < 3; i++) {
      if (ifs.eof())
	return false;
      getline(ifs, lines[i]);
    }

    // Ok continue to parse the lines.
    orunav_generic::State2d sensor_state;

    sensor_state = parseStateLine(lines[1]);

    orunav_generic::Control c;
    c.v  = parseVelocityLine(lines[2]);
    c.w = 0.; // Simply don't have this.

    sensor_.add(sensor_state, c);
    return true;
  }
      

  bool load(const std::string &fileName) {

    // Here we log everything - the velocity is a good hint to look for but we need to sync this later on with the controller.log files.
    
    std::ifstream ifs;
    ifs.open(fileName.c_str());
    if (!ifs.is_open()) {
      std::cerr << __FILE__ << ":" << __LINE__ << " cannot open file : " << fileName << std::endl;
      return false;
    }

    // This is how the file looks like.

    /* Updated position: x =    9.563,  y =    2.957,  theta =   -0.022,  phi =   -0.008 */
    /* Updated angles: x =    9.563,  y =    2.957,  theta =   -0.022,  phi =   -0.008 */
    /* Updated steering wheel speed: v = 	0 */
    /* State updated | System time = 1385061460.728904 */
    while (!ifs.eof()) {
  
      if (!flushToStartTag(ifs))
	return true;  // We read the whole file...
      
      if (!readStatesAndVelocites(ifs))
	return true;
    }
    return true;
  }
  
  
};

