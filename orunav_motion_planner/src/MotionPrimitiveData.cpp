/**
 * @file MotionPrimitiveData.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Apr 1, 2013
 *      Author: marcello
 */

#include "orunav_motion_planner/MotionPrimitiveData.h"
#include <math.h>


MotionPrimitiveData::MotionPrimitiveData() {
	// Initialize all to 0
	ID_ = 0;
	startOrientID_ = 0;
	finalOrientID_ = 0;
	startSteeringID_ = 0;
	finalSteeringID_ = 0;
	xOffset_ = 0;
	yOffset_ = 0;
	traj_.clear();
	distance_ = 0;
	costmult_ = 0;
	motiondir_ = 0;
	cellsSwept_.clear();
	cellsOccFinal_.clear();
}

MotionPrimitiveData::~MotionPrimitiveData() {
	for (std::vector<vehicleSimplePoint*>::iterator it = traj_.begin(); it != traj_.end(); it++) {
		delete *it;
	}
	for (std::vector<cellPosition*>::iterator it = cellsSwept_.begin(); it != cellsSwept_.end(); it++) {
		delete *it;
	}

	for (std::vector<cellPosition*>::iterator it = cellsOccFinal_.begin(); it != cellsOccFinal_.end(); it++) {
		delete *it;
	}
	traj_.clear();
	cellsSwept_.clear();
	cellsOccFinal_.clear();
}

void MotionPrimitiveData::setID(unsigned short int id) {
	ID_ = id;
}

void MotionPrimitiveData::setTrajectory(std::vector<vehicleSimplePoint*> trajectory,
		uint8_t orientationAngles, uint8_t steeringAnglePartitions) {

	traj_ = trajectory;

	// xOffset
	double offset = traj_.back()->x - traj_.front()->x;
	if (offset < 0) {
		xOffset_ = (int) ((fabs(offset) + (WP::WORLD_SPACE_GRANULARITY/3)) / WP::WORLD_SPACE_GRANULARITY);
		xOffset_ = - xOffset_;
	} else {
		xOffset_ = (int) ((fabs(offset) + (WP::WORLD_SPACE_GRANULARITY/3)) / WP::WORLD_SPACE_GRANULARITY);
	}
	// yOffset
	offset = traj_.back()->y - traj_.front()->y;
	if (offset < 0) {
		yOffset_ = (int) ((fabs(offset) + (WP::WORLD_SPACE_GRANULARITY/3)) / WP::WORLD_SPACE_GRANULARITY);
		yOffset_ = - yOffset_;
	} else {
		yOffset_ = (int) ((fabs(offset) + (WP::WORLD_SPACE_GRANULARITY/3)) / WP::WORLD_SPACE_GRANULARITY);
	}

	double orientationGranularity = (2*M_PI) / orientationAngles;

	// we assume that the angles are normalized
	double orientation = traj_.front()->orient;
	if (orientation < (0 - WP::CALCULATION_APPROXIMATION_ERROR)) {
		orientation = orientation + 2*M_PI;
	}

	startOrientID_ = (orientation + (orientationGranularity / 3)) / orientationGranularity;

	orientation = traj_.back()->orient;
	if (orientation < (0 - WP::CALCULATION_APPROXIMATION_ERROR)) {
		orientation = orientation + 2*M_PI;
	}
	finalOrientID_ = (orientation + (orientationGranularity / 3)) /orientationGranularity;

	double steeringGranularity = (2*M_PI) / steeringAnglePartitions;

	orientation = traj_.front()->steering;
	if (orientation < (0 - WP::CALCULATION_APPROXIMATION_ERROR)) {
		orientation = orientation + 2*M_PI;
	}
	startSteeringID_ = (orientation + (steeringGranularity / 3)) / steeringGranularity;

	orientation = traj_.back()->steering;
	if (orientation < (0 - WP::CALCULATION_APPROXIMATION_ERROR)) {
		orientation = orientation + 2*M_PI;
	}
	finalSteeringID_ = (orientation + (steeringGranularity / 3)) / steeringGranularity;
}

void MotionPrimitiveData::setDistance(double distance) {
	distance_ = distance;
}

void MotionPrimitiveData::setCostMultiplier(double costmult) {
	costmult_ = costmult;
}

void MotionPrimitiveData::setMotionDirection(short int motiondir) {
	motiondir_ = motiondir;
}

void MotionPrimitiveData::setSweptCells(std::vector<cellPosition*> cellsSwept){
	cellsSwept_ = cellsSwept;
}

void MotionPrimitiveData::setOccCells(std::vector<cellPosition*> cellsOcc){
	cellsOccFinal_ = cellsOcc;
}

short unsigned int MotionPrimitiveData::getID(){
	return ID_;
}

uint8_t MotionPrimitiveData::getStartingOrientationID(){
	return startOrientID_;
}

uint8_t MotionPrimitiveData::getFinalOrientationID() {
	return finalOrientID_;
}

uint8_t MotionPrimitiveData::getStartingSteeringID() {
	return startSteeringID_;
}

uint8_t MotionPrimitiveData::getFinalSteeringID() {
	return finalSteeringID_;
}

short int MotionPrimitiveData::getXOffset() {
	return xOffset_;
}

short int MotionPrimitiveData::getYOffset() {
	return yOffset_;
}

std::vector<vehicleSimplePoint*> MotionPrimitiveData::getTrajectory() {
	return traj_;
}

double MotionPrimitiveData::getDistanceCovered() {
	return distance_;
}

double MotionPrimitiveData::getCostMultiplier() {
	return costmult_;
}

short int MotionPrimitiveData::getMotionDirection() {
	return motiondir_;
}

std::vector<cellPosition*> MotionPrimitiveData::getSweptCells() {
	return cellsSwept_;
}

std::vector<cellPosition*> MotionPrimitiveData::getOccCells() {
	return cellsOccFinal_;
}



