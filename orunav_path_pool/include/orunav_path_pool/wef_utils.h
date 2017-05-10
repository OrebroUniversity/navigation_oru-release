#ifndef WEF_UTILS_H
#define WEF_UTILS_H

#include <iostream>
#include <fstream>
//#include "stdafx.h"
#include <tinyxml.h>
#include <orunav_generic/interfaces.h>
#include <orunav_generic/path_utils.h>
#include <orunav_geometry/b_spline2d.h>
#include <angles/angles.h>

class ControlPoint
{
public:
  double x,y;
  double speed;
  void convertMilimeters2Meters()
  {
    this->x *= 0.001;
    this->y *= 0.001;
    this->speed *= 0.001;
  }
  friend std::ostream& operator<<(std::ostream &os, const ControlPoint &obj)
  {
    os << "pos[" << obj.x << ", " << obj.y << "] speed:" << obj.speed;
    return os;
  }
};

class ControlPoints : public std::vector<ControlPoint>
{
public:

  void getVectors(std::vector<double> &x, std::vector<double> &y, std::vector<double> &speed, bool reversing) const
  {
    // WARNING!!! BE AWARE OF THE ORDER OF THE CONTROL POINTS.
    x.resize(this->size()); y.resize(this->size()); speed.resize(this->size());
    if (this->size() == 0)
      return;
    
    for (unsigned int i = 0; i < this->size()-1; i++)
      {
	x[i+1] = (*this)[i].x;
	y[i+1] = (*this)[i].y;
	speed[i+1] = (*this)[i].speed;
      }
    
    unsigned last = this->size() - 1;
    x[0] = (*this)[last].x;
    y[0] = (*this)[last].y;
    speed[0] = (*this)[last].speed;
  }

  bool isReversing(const orunav_generic::Path &path, const orunav_generic::Pose2dContainerInterface &startGoal) const
  {
    assert(startGoal.sizePose2d() == 2);
    for (unsigned int i = 0; i < startGoal.sizePose2d(); i++) { // Don't want to explicitly code if the start comes first or not.
      if (orunav_generic::getDistBetween(path.getPose2d(0), startGoal.getPose2d(i)) < 0.01) { // TODO, threshold?
	// Check the heading.
	double angle_diff = path.getPose2d(2)(2) - startGoal.getPose2d(i)(2);
	angle_diff = angles::normalize_angle(angle_diff);
	  if (fabs(angle_diff) > M_PI/2.)
	    return true;
	  else
	    return false;
      }
    }
    assert(false); // This indicates some error between the spline and the pose.
    return false;
  } 
  
  bool canComputeSpline() const {
    if (this->size() >= 4)
      return true;
    return false;
  }

  // NOTE/WARNING!!! The difference here is that orunav Paths have the states encoded directly, where as the wef file contains the motionof the path (e.g. driving backwards and forwards have different splines) and not the state. Therefore we need to find out the direction of the vehicle to make the state correct. To do this the start and goal poses are also utilized. Simply we create 2 paths and the paths that are consistent with the start and goal pose are used.
  orunav_generic::Path getPath(double L, double resolution, const orunav_generic::Pose2dContainerInterface &startAndGoal) const
  {
    {
      orunav_generic::Path ret;
      std::vector<double> x, y, speed;
      if (this->canComputeSpline()) {
	this->getVectors(x,y,speed, false);
      }
      else {
	std::cerr << "WARNING --- cannot compute spline, number of control points : " << this->size() << std::endl;
	return ret;
      }
      BSpline2d s;
      s.setKnots(x,y,speed);
      s.calcPath(ret, L, resolution);
      if (!this->isReversing(ret, startAndGoal))
	{
	  //	  std::cout << "FORWARD!" << std::endl;
	  return ret;
	}
      else
	{
	  //	  std::cout << "BACKWARD!" << std::endl;
	  // Going backwards, fix the heading.
	  return orunav_generic::getReversePath(ret);
	}
    }
  }

  std::vector<Eigen::Vector3d> getControlPointsVec() const
  {
    std::vector<Eigen::Vector3d> ret;

    for (unsigned int i = 0; i < this->size(); i++)
      {
	ret.push_back(Eigen::Vector3d((*this)[i].x,
				      (*this)[i].y,
				      0.));
      }
    return ret;
  }
};

class WefPathParser
{
public:
  WefPathParser(const std::string &fileName) : _fileName(fileName) {
    process();
  }

  void process() {
    _pathsControlPoints.clear();
    _startAndGoalPoses.clear();
    TiXmlDocument doc(_fileName.c_str() );
    if (doc.LoadFile())
      {
	  TiXmlHandle docHandle( &doc );
	  this->parsePathNodes( &doc );
	}
      else
	{
	  // load failed
	  std::cout << "WARNING! Failed to load : " << _fileName << std::endl;
	}
    

  }

  std::vector<orunav_generic::Path> getPaths(double L, double resolution) {
  
    std::vector<orunav_generic::Path> ret;
    process();

    // For each set of control points, create a spline and subsample that given the resolution value.
    for (unsigned int i = 0; i < _pathsControlPoints.size(); i++)
      {
	if (_pathsControlPoints[i].canComputeSpline()) {
	  ret.push_back(_pathsControlPoints[i].getPath(L, resolution, _startAndGoalPoses[i]));
	}
	else {
	  // Here we assume that we have a line. 
	  //	  std::cout << "Line detected - start : \n" << _startAndGoalPoses[i][0] << " goal : \n" << _startAndGoalPoses[i][1] << std::endl;
	  ret.push_back(orunav_generic::createStraightPath(_startAndGoalPoses[i][0], _startAndGoalPoses[i][1], resolution));
	}
	    
      }
    return ret;
  }

  orunav_generic::Pose2dVec getGoalPoses() { 
    orunav_generic::Pose2dVec ret;
    for (unsigned int i = 0; i < _startAndGoalPoses.size(); i++) {
      // The last one of the pair should be the goal :-)
      assert(_startAndGoalPoses[i].sizePose2d() == 2);
      ret.push_back(_startAndGoalPoses[i][1]);
    }
    return ret;
  }

  
  std::vector<ControlPoints> getPathsControlPoints() {     return _pathsControlPoints;     }
  const std::vector<ControlPoints>& getAllControlPoints() const { return _pathsControlPoints; }
  const std::vector<orunav_generic::Pose2dVec>& getStartAndGoalPoses() const { return _startAndGoalPoses; }
  

private:
  std::string _fileName;
  std::vector<ControlPoints> _pathsControlPoints;
  std::vector<orunav_generic::Pose2dVec> _startAndGoalPoses;

  bool getPartPoint(TiXmlElement* pElement, orunav_generic::Pose2d &p)
  {
    if ( !pElement ) return false;
    
    TiXmlAttribute* pAttrib=pElement->FirstAttribute();
    bool valid = false;
    while (pAttrib)
      {
	//WARNING!!! : mm and degrees cdeg (degrees * 100)
	if (pAttrib->Name() == std::string("CoordX"))
	  {
	    valid = true;
	    pAttrib->QueryDoubleValue(&p(0));
	    p(0) *= 0.001;
	  }
	if (pAttrib->Name() == std::string("CoordY"))
	  {
	    pAttrib->QueryDoubleValue(&p(1));
	    p(1) *= 0.001;
	  }
	if (pAttrib->Name() == std::string("Angle"))
	  {
	    double degrees;
	    pAttrib->QueryDoubleValue(&degrees);
	    degrees *= 0.01; // <- !!!
	    p(2) = angles::from_degrees(degrees);
	  }
	pAttrib=pAttrib->Next();
      }
    return valid;
  }

  
  bool getControlPoint(TiXmlElement* pElement, ControlPoint &p)
  {
    if ( !pElement ) return false;
    
    TiXmlAttribute* pAttrib=pElement->FirstAttribute();
    bool valid = false;
    while (pAttrib)
      {
	if (pAttrib->Name() == std::string("CoordX"))
	  {
	    valid = true;
	    pAttrib->QueryDoubleValue(&p.x);
	  }
	if (pAttrib->Name() == std::string("CoordY"))
	  {
	    pAttrib->QueryDoubleValue(&p.y);
	  }
	if (pAttrib->Name() == std::string("Speed"))
	  {
	    pAttrib->QueryDoubleValue(&p.speed);
	  }
	pAttrib=pAttrib->Next();
      }
    p.convertMilimeters2Meters();
    return valid;
  }

void parsePathNodes( TiXmlNode* pParent)
{
  if ( !pParent ) return;
  
  TiXmlNode* pChild;
  int t = pParent->Type();
  
  switch ( t )
    {
    case TiXmlNode::TINYXML_DOCUMENT:
      break;
      
    case TiXmlNode::TINYXML_ELEMENT:
      // Only cares about the "Segments", "Segment" and "ControlPoint" feilds here.
      if (pParent->ValueStr() == std::string("Segments")) 
	{
	  //	  std::cout << "Found [Segments] -> great!" << std::endl;
	}
      if (pParent->ValueStr() == std::string("Segment"))
	{
	  ControlPoints cp;
	  _pathsControlPoints.push_back(cp);
	  orunav_generic::Pose2dVec pv;
	  _startAndGoalPoses.push_back(pv);
	  //  std::cout << "Found [Segment] # " << _pathsControlPoints.size() << std::endl;
	}
      if (pParent->ValueStr() == std::string("ControlPoint")) 
	{
	  ControlPoint p; 
	  if (getControlPoint(pParent->ToElement(), p))
	    {
	      _pathsControlPoints.back().push_back(p);
	      //  std::cout << "Found [ControlPoint] # " << _pathsControlPoints.back().size() << std::endl;
	    }
	  else
	    {
	      std::cout << "Found [ControlPoint] # FAILED PARSING(!)" << std::endl;
	    }
	}
      if (pParent->ValueStr() == std::string("PartPoint"))
	{
	  orunav_generic::Pose2d p;
	  if (getPartPoint(pParent->ToElement(), p))
	    {
	      _startAndGoalPoses.back().push_back(p);
	      //  std::cout << "Found [PartPoint] # " << _startAndGoalPoses.back().size() << std::endl;
	    }
	  else
	    {
	      std::cout << "Found [PartPoint] # FAILED PARSING(!)" << std::endl;
	    }
	}
      if (pParent->ValueStr() == std::string("PartLine"))
	{
	  // This indicates that we have a straight line between the two PartPoints.
	  // Currently we don't need to do anything here, but: whenever we have an entry within _pathsControlPoints[] that are empty we will see if the PartPoints (in _startAndGoalPoses) lies on a line. In that case we will try to connet these two using a straight line.
	}
      break;
      
      //<Segment Id="5448" StartPoint="3935" EndPoint="3933" Weight="2858" Length="1681"><PlcAttr><PlcFlag Bit="15"/></PlcAttr><Parts><PartPoint CoordX="129165" CoordY="167113" Angle="18000"/><PartLine Speed="588" RefPntSpeed="1"/><PartPoint CoordX="127484" CoordY="167113" Angle="18000"/></Parts></Segment>

      //<Segment Id="5405" StartPoint="3879" EndPoint="3894" Weight="11625" Length="4376"><PlcAttr><PlcFlag Bit="4"/></PlcAttr><Parts><PartPoint CoordX="120254" CoordY="155711" Angle="27000"/><PartSpline><ControlPoint CoordX="120254" CoordY="156466" Speed="588"/><ControlPoint CoordX="120254" CoordY="155711" Speed="588"/><ControlPoint CoordX="120254" CoordY="154956" Speed="588"/><ControlPoint CoordX="121003" CoordY="154478" Speed="588"/><ControlPoint CoordX="122062" CoordY="155065" Speed="588"/><ControlPoint CoordX="122495" CoordY="156134" Speed="588"/><ControlPoint CoordX="122495" CoordY="156817" Speed="588"/><ControlPoint CoordX="122495" CoordY="157500" Speed="588"/></PartSpline><PartPoint CoordX="122495" CoordY="156817" Angle="9000"/></Parts></Segment>

    case TiXmlNode::TINYXML_COMMENT:
      break;
      
    case TiXmlNode::TINYXML_UNKNOWN:
      break;
      
    case TiXmlNode::TINYXML_TEXT:
      break;
      
    case TiXmlNode::TINYXML_DECLARATION:
      break;
    default:
      break;
    }
  for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) 
    {
      parsePathNodes( pChild );
    }
}


};

#endif

