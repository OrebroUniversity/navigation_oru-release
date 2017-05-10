/**
 * @file Configuration.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Mar 14, 2011
 *      Author: marcello
 */

#include "orunav_motion_planner/Configuration.h"

Configuration::Configuration(unsigned short int xCell, unsigned short int yCell,
		uint8_t orientID, uint8_t steerID, VehicleMission* vm) {
	xcell_ = xCell;
	ycell_ = yCell;
	orient_ = orientID;
	steering_ = steerID;
	primitiveOfThisConfiguration_ = 0;
	mission_ = vm;
}

Configuration::~Configuration() {
}

Configuration::Configuration(const Configuration& origin) :	xcell_ (origin.xcell_), ycell_ (origin.ycell_),
		orient_ (origin.orient_), steering_(origin.steering_){
	mission_ = origin.mission_;
	primitiveOfThisConfiguration_ = origin.primitiveOfThisConfiguration_;
}

void Configuration::setConfigurationPrimitive(MotionPrimitiveData* prim) {
	if (primitiveOfThisConfiguration_ != 0) {
		delete primitiveOfThisConfiguration_;
	}
	primitiveOfThisConfiguration_ = prim;
}

unsigned short int Configuration::getXCell() {
	return xcell_;
}

unsigned short int Configuration::getYCell() {
	return ycell_;
}

uint8_t Configuration::getOrientationID() {
	return orient_;
}

uint8_t Configuration::getSteeringID() {
	return steering_;
}

double Configuration::getXCoordinate() {
	return xcell_ * WP::WORLD_SPACE_GRANULARITY;
}

double Configuration::getYCoordinate() {
	return ycell_ * WP::WORLD_SPACE_GRANULARITY;
}

double Configuration::getOrientation() {
	double orient = this->getOrientationID() * ((2* M_PI) / this->getMission()->getVehicleModel()->getOrientationAngles());
	if (orient > M_PI) {
		orient = atan2(sin(orient), cos(orient));
	}
	return orient;
}

double Configuration::getSteering() {
	double steering = this->getSteeringID() * ((2* M_PI) / this->getMission()->getVehicleModel()->getSteeringAnglePartitions());
	if (steering > M_PI) {
		steering = atan2(sin(steering), cos(steering));
	}
	return steering;
}

VehicleMission* Configuration::getMission() {
	return mission_;
}

MotionPrimitiveData* Configuration::getPrimitiveOfThisConfiguration() {
	return primitiveOfThisConfiguration_;
}

std::vector<vehicleSimplePoint> Configuration::getTrajectory() {
	std::vector<vehicleSimplePoint> trajectory;
	// this configuration has been generated from a non-dummy motion primitive
	/** @todo CHECK */
	if (primitiveOfThisConfiguration_ != 0 && this->getPrimitiveOfThisConfiguration()->getDistanceCovered() > 0) {
		// calculate it from the motion primitive
		double startX = (xcell_ - this->getPrimitiveOfThisConfiguration()->getXOffset()) * WP::WORLD_SPACE_GRANULARITY;
		double startY = (ycell_ - this->getPrimitiveOfThisConfiguration()->getYOffset()) * WP::WORLD_SPACE_GRANULARITY;
		std::vector<vehicleSimplePoint*> traj = primitiveOfThisConfiguration_->getTrajectory();
		for (std::vector<vehicleSimplePoint*>::iterator it = traj.begin(); it != traj.end(); it++) {
			vehicleSimplePoint sp;
			sp.x = startX + (*it)->x;
			sp.y = startY + (*it)->y;
			sp.orient = (*it)->orient;
			sp.steering = (*it)->steering;
			trajectory.push_back(sp);
		}
	} else {
		trajectory.clear();
	}
	return trajectory;
}

double Configuration::getTrajectoryLength() {
	if (primitiveOfThisConfiguration_ != 0) {
		return primitiveOfThisConfiguration_->getDistanceCovered();
	} else {
		return 0;
	}
}

std::vector<cellPosition> Configuration::getCellsOccupied() {
	std::vector<cellPosition> oc;
	int startxcell = xcell_ - this->getPrimitiveOfThisConfiguration()->getXOffset();
	int startycell = ycell_ - this->getPrimitiveOfThisConfiguration()->getYOffset();
	std::vector<cellPosition*> occCells = primitiveOfThisConfiguration_->getOccCells();
	for (std::vector<cellPosition*>::iterator it = occCells.begin(); it != occCells.end(); it++) {
		cellPosition cp;
		cp.x_cell = (*it)->x_cell + startxcell;
		cp.y_cell = (*it)->y_cell + startycell;
		oc.push_back(cp);
	}
	return oc;
}

std::vector<cellPosition> Configuration::getCellsSwept() {
	std::vector<cellPosition> oc;
	int startxcell = xcell_ - this->getPrimitiveOfThisConfiguration()->getXOffset();
	int startycell = ycell_ - this->getPrimitiveOfThisConfiguration()->getYOffset();
	std::vector<cellPosition*> sweptCells = primitiveOfThisConfiguration_->getSweptCells();
	for (std::vector<cellPosition*>::iterator it = sweptCells.begin(); it != sweptCells.end(); it++) {
		cellPosition cp;
		cp.x_cell = (*it)->x_cell + startxcell;
		cp.y_cell = (*it)->y_cell + startycell;
		oc.push_back(cp);
	}
	return oc;
}

double Configuration::getCostToThisConfiguration() {
	if (primitiveOfThisConfiguration_ != 0) {
		return primitiveOfThisConfiguration_->getDistanceCovered() * primitiveOfThisConfiguration_->getCostMultiplier();
	} else {
		return 0;
	}
}

int Configuration::getMotionDirectionToThisConfiguration() {
	if (primitiveOfThisConfiguration_ != 0) {
		return primitiveOfThisConfiguration_->getMotionDirection();
	} else {
		return 0;
	}
}

double Configuration::estimateCostFromThisConfiguration(Configuration* conf1) {
	if (WP::USE_HEURISTIC_ESTIMATION) {
		double 	ht = this->getMission()->getVehicleModel()->getHeuristicValueFromTable(
				this->xcell_, this->ycell_, this->getOrientationID(), this->getSteeringID(),
				conf1->getXCell(), conf1->getYCell(), conf1->getOrientationID(), conf1->getSteeringID());
		if (ht != -1) {
			return ht;
		}
		// if the value is not present in the heuristic table, return the euclidean distance
		double xdist = (this->getXCell() - conf1->getXCell()) * WP::WORLD_SPACE_GRANULARITY;
		double ydist = (this->getYCell() - conf1->getYCell()) * WP::WORLD_SPACE_GRANULARITY;
		return sqrt(pow(xdist, 2) + pow(ydist, 2));
	} else {
		return 0;
	}
}

bool Configuration::equalConfigurations(Configuration* conf1) {
	if (xcell_ == conf1->getXCell() && ycell_ == conf1->getYCell() &&
			this->getOrientationID() == conf1->getOrientationID() &&
			this->getSteeringID() == conf1->getSteeringID()) {
		return true;
	} else {
		return false;
	}
}

uint32_t Configuration::getHash() {
	uint32_t result = (xcell_ << 22) | ((ycell_ << 12) | ((this->getOrientationID() << 6) | this->getSteeringID()));
	return result;
}

void Configuration::print() {
	char info[80];
	double granularity = (2* M_PI) / this->getMission()->getVehicleModel()->getOrientationAngles();
	double orient = this->getOrientationID() * granularity;
	if (orient > M_PI) {
		orient = atan2(sin(orient), cos(orient));
	}
	granularity = (2* M_PI) / this->getMission()->getVehicleModel()->getSteeringAnglePartitions();
	double steering = this->getSteeringID() * granularity;
	if (steering > M_PI) {
		steering = atan2(sin(steering), cos(steering));
	}
	sprintf(info, "[Vehicle %d] x: %3.2f \ty: %3.2f \to: %1.4f \ts: %1.4f", this->getMission()->getVehicleID(),
			xcell_ * WP::WORLD_SPACE_GRANULARITY, ycell_ * WP::WORLD_SPACE_GRANULARITY, orient, steering);
	writeLogLine(std::string(info), "Configuration", WP::LOG_FILE);
}

std::string Configuration::printToString() {
	char info[80];
	double granularity = (2* M_PI) / this->getMission()->getVehicleModel()->getOrientationAngles();
	double orient = this->getOrientationID() * granularity;
	if (orient > M_PI) {
		orient = atan2(sin(orient), cos(orient));
	}
	granularity = (2* M_PI) / this->getMission()->getVehicleModel()->getSteeringAnglePartitions();
	double steering = this->getSteeringID() * granularity;
	if (steering > M_PI) {
		steering = atan2(sin(steering), cos(steering));
	}
	sprintf(info, "[Vehicle %d] x: %3.2f \ty: %3.2f \to: %1.4f \ts: %1.4f", this->getMission()->getVehicleID(),
			xcell_ * WP::WORLD_SPACE_GRANULARITY, ycell_ * WP::WORLD_SPACE_GRANULARITY, orient, steering);
	return std::string(info);
}

