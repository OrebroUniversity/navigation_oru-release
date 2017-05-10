/**
 * @file VehicleMission.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Mar 28, 2013
 *      Author: marcello
 */

#include "orunav_motion_planner/VehicleMission.h"

#include "orunav_motion_planner/UnicycleModel.h"
#include "orunav_motion_planner/CarModel.h"
#include "orunav_motion_planner/LHDModel.h"
#include "orunav_motion_planner/CarConfiguration.h"
#include "orunav_motion_planner/UnicycleConfiguration.h"
#include "orunav_motion_planner/LHDConfiguration.h"



// init the static counter
unsigned short int VehicleMission::vehicleNum_ = 0;
unsigned short int VehicleMission::activeMissions_ = 0;

VehicleMission::VehicleMission(VehicleModel* m, double start_x, double start_y, double start_o, double start_steering,
		double goal_x, double goal_y, double goal_o, double goal_steering, std::string name /*= ""*/) {
	vm_ = m;
	vehicleID_ = vehicleNum_;
	// increment the static counter and the number of active missions
	vehicleNum_++;
	activeMissions_++;

	unsigned short int startXcell, startYcell, goalXcell, goalYcell;
	uint8_t startOrientationID, startSteeringID, goalSteeringID, goalOrientationID;

	// snap the start pose into the grid
	if (fmod(start_x, WP::WORLD_SPACE_GRANULARITY) >= (WP::WORLD_SPACE_GRANULARITY / 2)) {
		startXcell = ceil(start_x * (1/WP::WORLD_SPACE_GRANULARITY));
	} else {
		startXcell = floor(start_x * (1/WP::WORLD_SPACE_GRANULARITY));
	}

	if (fmod(start_y, WP::WORLD_SPACE_GRANULARITY) >= (WP::WORLD_SPACE_GRANULARITY / 2)) {
		startYcell = ceil(start_y * (1/WP::WORLD_SPACE_GRANULARITY));
	} else {
		startYcell = floor(start_y * (1/WP::WORLD_SPACE_GRANULARITY));
	}

	startOrientationID = vm_->getClosestAllowedOrientationID(start_o);
	startSteeringID = vm_->getClosestAllowedSteeringID(start_steering);

	// snap the goal pose into the grid
	if (fmod(goal_x, WP::WORLD_SPACE_GRANULARITY) >= (WP::WORLD_SPACE_GRANULARITY / 2)) {
		goalXcell = ceil(goal_x * (1/WP::WORLD_SPACE_GRANULARITY));
	} else {
		goalXcell = floor(goal_x * (1/WP::WORLD_SPACE_GRANULARITY));
	}

	if (fmod(goal_y, WP::WORLD_SPACE_GRANULARITY) >= (WP::WORLD_SPACE_GRANULARITY / 2)) {
		goalYcell = ceil(goal_y * (1/WP::WORLD_SPACE_GRANULARITY));
	} else {
		goalYcell = floor(goal_y * (1/WP::WORLD_SPACE_GRANULARITY));
	}

	goalOrientationID = vm_->getClosestAllowedOrientationID(goal_o);
	goalSteeringID = vm_->getClosestAllowedSteeringID(goal_steering);

	// create a dummy motion primitive for the starting point
	MotionPrimitiveData* startPrimitive = new MotionPrimitiveData();
	double startOrient = startOrientationID * ((M_PI*2) / vm_->getOrientationAngles());
	double startSteer = startSteeringID * ((M_PI*2) / vm_->getSteeringAnglePartitions());
	std::vector<vehicleSimplePoint*> traj;
	vehicleSimplePoint* p = new vehicleSimplePoint;
	p->x = startXcell*WP::WORLD_SPACE_GRANULARITY;
	p->y = startYcell*WP::WORLD_SPACE_GRANULARITY;
	p->orient = startOrient;
	p->steering = startSteer;
	traj.push_back(p);
	startPrimitive->setTrajectory(traj, vm_->getOrientationAngles(),vm_->getSteeringAnglePartitions());
	// Now get the occupied and swept cells
	vehicleSimplePoint* dummypoint = new vehicleSimplePoint;
	dummypoint->x = 0;
	dummypoint->y = 0;
	dummypoint->orient = startOrient;
	dummypoint->steering = startSteer;
	std::vector<cellPosition*> sweptCells = vm_->getCellsOccupiedInPosition(dummypoint);
	std::vector<cellPosition*> occCells = vm_->getCellsOccupiedInPosition(dummypoint);
	startPrimitive->setOccCells(occCells);
	startPrimitive->setSweptCells(sweptCells);
	delete dummypoint;

	if (dynamic_cast<CarModel*> (m)) {
		start_ = new CarConfiguration(startXcell, startYcell, startOrientationID, startSteeringID, this);
		goal_ = new CarConfiguration(goalXcell, goalYcell, goalOrientationID, goalSteeringID, this);
	} else if (dynamic_cast<LHDModel*> (m)) {
		start_ = new LHDConfiguration(startXcell, startYcell, startOrientationID, startSteeringID, this);
		goal_ = new LHDConfiguration(goalXcell, goalYcell, goalOrientationID, goalSteeringID, this);
	} else if (dynamic_cast<UnicycleModel*> (m)) {
		start_ = new UnicycleConfiguration(startXcell, startYcell, startOrientationID, startSteeringID, this);
		goal_ = new UnicycleConfiguration(goalXcell, goalYcell, goalOrientationID, goalSteeringID, this);
	} else {
		writeLogLine(std::string("ERROR: vehicle model not recognized!"), "VehicleMission", WP::LOG_FILE);
		exit(0);
	}

	start_->setConfigurationPrimitive(startPrimitive);

	vehicleName_ = name;
	gridDistanceFromGoal_.clear();
}

VehicleMission::~VehicleMission() {
	MotionPrimitiveData* mpd = start_->getPrimitiveOfThisConfiguration();
	delete mpd;
	delete start_;
	delete goal_;
	for (std::vector<std::vector<double> >::iterator it = gridDistanceFromGoal_.begin(); it != gridDistanceFromGoal_.end(); it++ ){
		(*it).clear();
	}
	gridDistanceFromGoal_.clear();
	// reduce the number of active missions and reset the ID counter if this was the last one
	activeMissions_--;
	if (activeMissions_ == 0) {
		vehicleNum_ = 0;
	}
}

void VehicleMission::setGridDistanceFromGoal(std::vector<std::vector<double> > distances) {
	for (std::vector<std::vector<double> >::iterator it = gridDistanceFromGoal_.begin(); it != gridDistanceFromGoal_.end(); it++ ){
		(*it).clear();
	}
	gridDistanceFromGoal_.clear();
	gridDistanceFromGoal_.resize(distances.size());
	for (unsigned int i = 0; i < gridDistanceFromGoal_.size(); i++) {
		gridDistanceFromGoal_[i].resize(distances[i].size());
		for (unsigned int j = 0; j < gridDistanceFromGoal_[i].size(); j++) {
			gridDistanceFromGoal_[i][j] = distances[i][j];
		}
	}
}

Configuration* VehicleMission::getStartConfiguration() {
	return start_;
}

Configuration* VehicleMission::getGoalConfiguration() {
	return goal_;
}

unsigned short int VehicleMission::getVehicleID() {
	return vehicleID_;
}

std::string VehicleMission::getVehicleName() {
	return vehicleName_;
}

VehicleModel*  VehicleMission::getVehicleModel() {
	return vm_;
}

double VehicleMission::getGridDistanceFromGoal(int x, int y) {
	if (gridDistanceFromGoal_.size() != 0) {
		return gridDistanceFromGoal_[y][x];
	} else {
		return 0;
	}
}
