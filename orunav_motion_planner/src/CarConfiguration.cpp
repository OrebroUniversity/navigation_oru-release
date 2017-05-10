/**
 * @file CarConfiguration.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Dec 8, 2011
 *      Author: marcello
 */

#include "orunav_motion_planner/CarConfiguration.h"

CarConfiguration::CarConfiguration(unsigned short int xCell, unsigned short int yCell,
		uint8_t orientID, uint8_t steerID, VehicleMission* vm) : Configuration(xCell, yCell, orientID, steerID, vm) {
	// check that the model is the correct one
	if (!dynamic_cast<CarModel*> (vm->getVehicleModel())) {
		exit(0);
	}
}

CarConfiguration::~CarConfiguration() {
}

CarConfiguration::CarConfiguration(const CarConfiguration& origin) : Configuration(origin){};

std::vector<Configuration*> CarConfiguration::generateNewConfigurations() {

	std::vector<Configuration*> result;
	std::vector<MotionPrimitiveData*> primitives;
	primitives = dynamic_cast<CarModel*>
	(this->getMission()->getVehicleModel())->getApplicablePrimitives(this->getOrientationID(), this->getSteeringID());

	for (std::vector<MotionPrimitiveData*>::iterator primit = primitives.begin(); primit != primitives.end(); primit++) {
		// create a new Configuration for each primitive
		int xc = (*primit)->getXOffset() + this->getXCell();
		int yc = (*primit)->getYOffset() + this->getYCell();
		// configurations must be in non-negative cells
		if (xc >= 0 && yc >= 0) {
			CarConfiguration* newConf = new CarConfiguration(xc, yc, (*primit)->getFinalOrientationID(),
					(*primit)->getFinalSteeringID(), this->getMission());
			newConf->setConfigurationPrimitive((*primit));
			result.push_back(newConf);
		}
	}
	return result;
}

std::vector<Configuration*> CarConfiguration::generateNewConfigurations(World* w) {

	std::vector<Configuration*> result;
	std::vector<MotionPrimitiveData*> primitives;
	primitives = this->getMission()->getVehicleModel()->selectApplicablePrimitives(w, this->getXCell(),
			this->getYCell(), this->getOrientationID(), this->getSteeringID());

	for (std::vector<MotionPrimitiveData*>::iterator primit = primitives.begin(); primit != primitives.end(); primit++) {
		// create a new Configuration for each primitive
		int xc = (*primit)->getXOffset() + this->getXCell();
		int yc = (*primit)->getYOffset() + this->getYCell();
		// configurations must be in non-negative cells
		if (xc >= 0 && yc >= 0) {
			CarConfiguration* newConf = new CarConfiguration(xc, yc, (*primit)->getFinalOrientationID(),
					(*primit)->getFinalSteeringID(), this->getMission());
			newConf->setConfigurationPrimitive((*primit));
			result.push_back(newConf);
		}
	}
	return result;
}



Configuration* CarConfiguration::clone() {
	return new CarConfiguration(*this);
}

