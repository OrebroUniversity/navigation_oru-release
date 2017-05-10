#pragma once

#include <iostream>
#include <string>
#include <orunav_generic/utils.h>
#include <orunav_generic/interfaces.h>

class ControlLogParser {
 public:
  ControlLogParser(const std::string &fileName) {
    load(fileName);
  }
  const orunav_generic::PathInterface& getSensorStates() const { return sensor_; }
  const orunav_generic::PathInterface& getExpectedStates() const { return expected_; }

  const orunav_generic::TrajectoryInterface& getReferenceStatesAndControls() const { return reference_; }
  const orunav_generic::TrajectoryInterface& getEstimatedStatesAndUsedControls() const { return estimated_; }

  const std::vector<double>& getComputedVelAtSteeringWheel() const { return computed_vel_at_steering_wheel_; }
  
 private:
  std::string fileName_;
  orunav_generic::Path sensor_, expected_;
  orunav_generic::Trajectory reference_, estimated_;
  std::vector<double> reference_vel_at_steering_wheel_;
  std::vector<double> computed_vel_at_steering_wheel_;

  bool flushToStartTag(std::ifstream &ifs) const {
    while (!ifs.eof()) {
      std::string line;
      getline(ifs, line);
      if (std::string::npos != line.find(std::string(">>> start"))) {
	// Found target
	return true;
      }
    }
    return false;
  }

  orunav_generic::State2d parseStateLine(const std::string &str) const {
    orunav_generic::State2d s;
    // This is how a line look like:
    // XXX state: x =    5.028,  y =   11.374,  theta =    1.524,  phi =   -0.040
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

  orunav_generic::Control parseControlLine(const std::string &str, double &velocityAtSteeringWheel, double &steeringWheelAngle) const {
    // This is how a line looks like:
    // XXX car control: Velocity (tangential) = -0.401, velocity (steering wheel) = -0.401, steering velocity = -0.099, steering angle = 0.014
    orunav_generic::Control c;

    unsigned pos_start = str.find("=");
    unsigned pos_end = str.find(",", pos_start);

    assert(pos_start+1 < pos_end-1);
    std::string v = str.substr(pos_start+1,pos_end-1); 
    c.v = orunav_generic::fromString<double>(v);

    //-
    pos_start = str.find("=", pos_end);
    pos_end = str.find(",", pos_start);
    assert(pos_start+1 < pos_end-1);

    std::string v_steering = str.substr(pos_start+1,pos_end-1); 
    velocityAtSteeringWheel = orunav_generic::fromString<double>(v_steering);

    //-
    pos_start = str.find("=", pos_end);
    pos_end = str.find(",", pos_start);
    assert(pos_start+1 < pos_end-1);

    std::string w = str.substr(pos_start+1,pos_end-1); 
    c.w = orunav_generic::fromString<double>(w);

    //-
    pos_start = str.find("=", pos_end);

    std::string phi = str.substr(pos_start+1); // will take the whole line.
    steeringWheelAngle = orunav_generic::fromString<double>(phi);
    
    return c;
  }

  bool readStatesAndControl(std::ifstream &ifs) {

    std::vector<std::string> lines(4);
    
    for (int i = 0; i < 4; i++) {
      if (ifs.eof())
	return false;
      getline(ifs, lines[i]);
    }

    // Check if we have a reference state...
    if (std::string::npos == lines[3].find(std::string("Reference state:"))) {
      return true; // The return statements only tells us that we're not at eof.
    }
    
    // Ok continue to parse the lines.
    orunav_generic::State2d sensor, expected, estimated, reference_state;

    sensor = parseStateLine(lines[0]);
    expected = parseStateLine(lines[1]);
    estimated = parseStateLine(lines[2]);
    reference_state = parseStateLine(lines[3]);

    sensor_.addState2dInterface(sensor);
    expected_.addState2dInterface(expected);

    // Read the control values
    for (int i = 0; i < 3; i++) {
      if (ifs.eof())
	return false;
      getline(ifs, lines[i]);
    }
    orunav_generic::Control reference_control, computed;
    double reference_velocityAtSteeringWheel, reference_steeringWheelAngle;
    double computed_velocityAtSteeringWheel, computed_steeringWheelAngle;
    reference_control = parseControlLine(lines[0], reference_velocityAtSteeringWheel, reference_steeringWheelAngle);
    // Note: lines[1] contains the qp timer value.
    computed =  parseControlLine(lines[2], computed_velocityAtSteeringWheel, computed_steeringWheelAngle);

    reference_.add(reference_state, reference_control);
    estimated_.add(estimated, computed);
    reference_vel_at_steering_wheel_.push_back(reference_velocityAtSteeringWheel);
    computed_vel_at_steering_wheel_.push_back(computed_velocityAtSteeringWheel);

    return true;
  }
      

  bool load(const std::string &fileName) {

    // We're only interested in the control values when the system were active...
    // 1) Get the >>> start tag.
    // 2) Temporary store the data...
    // 3) ... if the Reference state is not found return to 1)
    // 4) store + push_back the data...
    
    std::ifstream ifs;
    ifs.open(fileName.c_str());
    if (!ifs.is_open()) {
      std::cerr << __FILE__ << ":" << __LINE__ << " cannot open file : " << fileName << std::endl;
      return false;
    }

    // This is how the file looks like.
    /* >>> start */
    /* Sensor state: x =    5.028,  y =   11.374,  theta =    1.524,  phi =   -0.040 */
    /* Expected state: x =    5.029,  y =   11.401,  theta =    1.526,  phi =   -0.116 */
    /* Estimated state: x =    5.028,  y =   11.374,  theta =    1.524,  phi =   -0.116 */
    /* Reference state: x =    5.031,  y =   11.399,  theta =    1.569,  phi =    0.020 */
    /* Reference car control: Velocity (tangential) = -0.401, velocity (steering wheel) = -0.401, steering velocity = -0.099, steering angle = 0.014 */
    /* QP time: Timer value = 0.002438 */
    /* Computed car control: Velocity (tangential) = -0.347, velocity (steering wheel) = -0.350, steering velocity = -0.193, steering angle = -0.128 */
    /* Duration of control iteration: Timer value = 0.002529 */
    /* <<< end */
    while (!ifs.eof()) {
  
      if (!flushToStartTag(ifs))
	return true;  // We read the whole file...
      
      if (!readStatesAndControl(ifs))
	return true;
    }
    return true;
  }
  
  
};
