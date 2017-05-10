#pragma once

#include <orunav_trajectory_processor/trajectory_processor_naive.h>

#include <orunav_generic/serialization.h>


//! Trajectory processor that tries to do whatever to follow the coordination times -> including reversing.
// Will assume that ct contains points that are only in the future and not in the past
class TrajectoryProcessorNaiveCT : public TrajectoryProcessor //, public orunav_generic::DeltaTInterface
{
 public:
 TrajectoryProcessorNaiveCT() : _startIdx(0), _startIdxControl(0.,0.), _startIdxTime(-1.) { }

  void addPathInterface(const orunav_generic::PathInterface &path)
  {
    _path = orunav_generic::Path(path);
  }

  void addCoordinatedTimes(const orunav_generic::CoordinatedTimes &times) {
    _coordinatedTimes = times;
  }

  void addControlConstraintPoint(size_t stepIdx, const orunav_generic::Control &control) {
    _controlConstraintPoints.push_back(std::pair<size_t, orunav_generic::Control>(stepIdx, control));
  }  
  
  void addControlConstraintPointAsStart(size_t startIdx, const orunav_generic::Control &control) {
    _startIdxControl = control;
    _startIdx = startIdx;
  }

  void setStartIdxTime(double startIdxTime) {
    _startIdxTime = startIdxTime;
  }

  orunav_generic::Trajectory getTrajectory() {

    if (_params.debug) {
      orunav_generic::saveDoubleVecTextFile(this->_coordinatedTimes, _params.debugPrefix + std::string("tpnct_input.ct"));
      orunav_generic::savePathTextFile(this->_path, _params.debugPrefix + std::string("tpnct_input.path"));
    }
    if (_coordinatedTimes.empty()) { // No coordination times - no need with any special handling.
      return getTrajectoryNoCoordination();
    }

    if (_startIdx == 0) { // Starting from beginning? no special handling. - this is where CASE 2 will end up...
      // CANNOT DO THIS TO CLOSE TO THE END!!!
      // if (orunav_generic::coordinationPairTimeStep(_coordinatedTimes, _startIdx) > 3.) {
      //   ROS_ERROR_STREAM("---------------------=================== : " << orunav_generic::coordinationPairTimeStep(_coordinatedTimes, _startIdx));
      //   return getTrajectoryStandStill();
      // }
      return getTrajectoryBeginning();
    }

    return getTrajectoryNonReversing();

    // ------------------ More specific CT handling follows here -----------------
    std::vector<size_t> ct_idx_before = _coordinatedTimes.getTimeIdxBeforeStartIdx(_startIdx);
    // Non reversing part.
    if (ct_idx_before.empty()) {

      // Check if we have _startIdx == the first ct idx - if the time is to large, simply stand still and wait, this is typically caused by some other vehicle is breaking.
      if (_coordinatedTimes.isStartIdxInTimeIdx(_startIdx)) {
	if (_coordinatedTimes[_startIdx] - _startIdxTime > 0.5) {
	  return getTrajectoryStandStill();
	}
      }
      return getTrajectoryNonReversing();
    }
    else {
      // Need to perform reversing... compute the reversing to the first start point, send the trajectory off, the continuation is computed when new coordination times arrives.
      return getTrajectoryReversing();
    }
  }
    
  const orunav_generic::CoordinatedTimes& getCoordinatedTimes() const { return _coordinatedTimes; }
  const orunav_generic::Path& getPath() const { return _path; }
  double getStartTime() const { 
    if (_startIdxTime >= 0) 
      return _startIdxTime; 
    if (!_coordinatedTimes.empty())
      return _coordinatedTimes[0];
    return 0.;
  }

  std::vector<double> getGlobalPathTimes(double offset) const {
  
    assert(!_globalPathTimes.empty());
    std::vector<double> ret;
    for (size_t i = 0; i < _globalPathTimes.size(); i++) {
      if (_globalPathTimes[i] > 0) {
	ret.push_back(_globalPathTimes[i] + offset);
      }
      else {
	ret.push_back(-1.);
      }
    }
    return ret;
  }

  void printDebug() const {
    std::cout << "----------------------------------------------------------------------" << std::endl;
    std::cout << "path.size()             : " << _path.sizePath() << std::endl;
    std::cout << "coordinatedTimes.size() : " << _coordinatedTimes.size() << std::endl;
    std::cout << "startIdxControl.v       : " << _startIdxControl.v << std::endl;
    std::cout << "startIdxControl.w       : " << _startIdxControl.w << std::endl;
    std::cout << "startIdxTime            : " << _startIdxTime << std::endl;
    std::cout << "startIdx                : " << _startIdx << std::endl;
    std::cout << "globalPathTimes.size()  : " << _globalPathTimes.size() << std::endl;
    std::cout << "controlConstraintPoints.size() : " << _controlConstraintPoints.size() << std::endl;

    for (size_t i = 0; i < _controlConstraintPoints.size(); i++) {
      std::cout << "constraint[" << i << "] : idx : " << _controlConstraintPoints[i].first;
      std::cout << " control.v " << _controlConstraintPoints[i].second.v;
      std::cout << " control.w " << _controlConstraintPoints[i].second.w << std::endl;
    }
  }
  
 protected:

  orunav_generic::Trajectory getTrajectoryNoCoordination() {
    std::cerr << "getTrajectoryNoCoordination()" << std::endl;
    TrajectoryProcessorNaive gen;
    gen.setParams(this->_params);
    gen.addPathInterface(_path);
    gen.addControlConstraintPoints(this->_controlConstraintPoints);
    gen.addControlConstraintPointAsStart(_startIdx, _startIdxControl);
    
    orunav_generic::Trajectory traj = gen.getTrajectory();
    this->setGlobalPathTimes(gen);
    std::cerr << "getTrajectoryNoCoordination() - end" << std::endl;
    return traj;
  }
  
  orunav_generic::Trajectory getTrajectoryBeginning() {
    std::cerr << "getTrajectoryBeginning()" << std::endl;
    TrajectoryProcessorNaive gen;
    gen.setParams(this->_params);
    gen.addPathInterface(_path);
    gen.addControlConstraintPoints(this->_controlConstraintPoints);
    gen.addControlConstraintPointAsStart(_startIdx, _startIdxControl);
    gen.addCoordinatedTimes(_coordinatedTimes);
    orunav_generic::Trajectory traj = gen.getTrajectory();
    this->setGlobalPathTimes(gen); // Very important to add after the getTrajectoy call!
    std::cerr << "getTrajectoryBeginning() - end" << std::endl;
    return traj;
  }
  

  orunav_generic::Trajectory getTrajectoryStandStill() {
    std::cerr << "getTrajectoryStandStill()" << std::endl;
    orunav_generic::Trajectory traj;
    orunav_generic::State2d s(_path, _startIdx);
    orunav_generic::Control c(0.,0.);
   
    // Each step is 0.06 secs. Add 100 of these -> 6 seconds. (TODO - add as a params if needed).
    // Also clear the global path time -- this is getting quite ugly this should be re-designed ASAP.
    _globalPathTimes.resize(1000);
    for (size_t i = 0; i < 1000; i++) { 
      traj.add(s, c);
      _globalPathTimes[i] = i * this->_params.timeStep;
    }
    std::cerr << "getTrajectoryStandStill() - end" << std::endl;
    return traj;
  }
  
  orunav_generic::Trajectory getTrajectoryNonReversing() {
    std::cerr << "getTrajectoryNonReversing()" << std::endl;
    orunav_generic::Trajectory traj;
    {
      TrajectoryProcessorNaive gen;
      gen.setParams(this->_params);
      gen.addPathInterface(_path);
      gen.addControlConstraintPoints(this->_controlConstraintPoints);
      gen.addControlConstraintPointAsStart(_startIdx, _startIdxControl);
      gen.computeDts();
      
      /* int first_ct_idx = _coordinatedTimes.getTimeIdxAfterStartIdx(_startIdx); */
      /* //assert(first_ct_idx >= (int)_startIdx); // Could happen.... */
      /* std::cerr << "getTrajectoryNonReversing() - _startIdx : " << _startIdx << ", first_ct_idx : " << first_ct_idx << std::endl; */
      /* if (first_ct_idx >= _startIdx) { */
      /* 	for (size_t i = static_cast<size_t>(first_ct_idx-1); i >= _startIdx; i--) { */
      /* 	  _coordinatedTimes[i] = _coordinatedTimes[i+1] - gen.getDeltaT(i-_startIdx); */
      /* 	} */
      /* } */
      if (_startIdxTime > 0)
	_coordinatedTimes[_startIdx] = _startIdxTime;
      
      orunav_generic::saveDoubleVecTextFile(this->_coordinatedTimes, "tpnct_non_reversing_input.ct");
      orunav_generic::savePathTextFile(this->_path, "tpnct_non_reversing_input.path");
	  
    }
    // --------------------------------
    {
      TrajectoryProcessorNaive gen;
      gen.setParams(this->_params);
      gen.addPathInterface(_path);
      gen.addControlConstraintPoints(this->_controlConstraintPoints);
      gen.addControlConstraintPointAsStart(_startIdx, _startIdxControl);
      gen.addCoordinatedTimes(_coordinatedTimes);
      
      traj = gen.getTrajectory();
      this->setGlobalPathTimes(gen);
    }
    std::cerr << "getTrajectoryNonReversing() - end" << std::endl;
    return traj;
  }
 
  orunav_generic::Trajectory getTrajectoryReversing() {

    // Simply stand still for now...
    return getTrajectoryStandStill();

    std::cerr << "getTrajectoryReversing()" << std::endl;
    
    // 1) need to start from _startIdx and move backwards to the first ct point in the vector...
    std::vector<size_t> ct_idx_before = _coordinatedTimes.getTimeIdxBeforeStartIdx(_startIdx);

    // 2) since we only can move forward (from startIdx to the end of the path) we need to reverse the path and recompute the startIdx as well as alter the path length to contain the state points of startIdx ... ct_idx_before[0].
    std::cerr << "getTrajectoryReversing() -1" << std::endl;
    
    orunav_generic::Path rev_full_path = orunav_generic::getReversePathWithoutChangingDirection(_path);
    std::cerr << "getTrajectoryReversing() -2" << std::endl;
    assert(_path.sizePath() - _startIdx > 0);
    size_t rev_start_idx = _path.sizePath() - _startIdx - 1;
    std::cerr << "getTrajectoryReversing() -3" << std::endl;
    assert(_path.sizePath() - ct_idx_before[0] > 0);
    size_t rev_ct = _path.sizePath() - ct_idx_before[0] - 1; 
    // Remove the end of rev_path (to only contain rev_ct but nothing more).
    std::cerr << "getTrajectoryReversing() -4" << std::endl;
    orunav_generic::Path rev_path = orunav_generic::selectPathIntervall(rev_full_path, 0, rev_ct+1);
    
    // Compute a trajectory without any time constraint.
    TrajectoryProcessorNaive gen;
    std::cerr << "getTrajectoryReversing() -5" << std::endl;
    gen.setParams(this->_params);
    gen.addPathInterface(rev_path);
    //gen.addControlConstraintPoints(this->_controlConstraintPoints);
    std::cerr << "getTrajectoryReversing() -6" << std::endl;
    
    std::cerr << "rev_start_idx : " << rev_start_idx << " rev_ct : " << rev_ct << " rev_path.size() : " << rev_path.sizePath() << std::endl;
    //gen.addControlConstraintPointAsStart(rev_start_idx, _startIdxControl);
    
    std::cerr << "getTrajectoryReversing() - 6.5" << std::endl;
    orunav_generic::Trajectory traj = gen.getTrajectory();
    std::cerr << "getTrajectoryReversing() -7" << std::endl;
    //    this->setGlobalPathTimes(gen);

    std::cerr << "getTrajectoryReversing() - end" << std::endl;
    return traj;
  }

  orunav_generic::Path _path; // Original path
  orunav_generic::CoordinatedTimes _coordinatedTimes; // Coordinated times (if available -> must have same size as _path)

  size_t _startIdx;
  orunav_generic::Control _startIdxControl;

  std::vector<std::pair<size_t, orunav_generic::Control> > _controlConstraintPoints;

  double _startIdxTime;

  void setGlobalPathTimes(const TrajectoryProcessorNaive &gen) {
    this->_globalPathTimes = gen.getGlobalPathTimes(0.); // Offset = 0.
  }


  std::vector<double> _globalPathTimes;

private:
  friend class boost::serialization::access;
  
  template<typename Archive>
    void serialize(Archive& ar, const unsigned version) {
    ar & _path & _coordinatedTimes & _startIdxControl & _startIdx & _controlConstraintPoints & _startIdxTime & _globalPathTimes;// & _path;
  }
  
};

