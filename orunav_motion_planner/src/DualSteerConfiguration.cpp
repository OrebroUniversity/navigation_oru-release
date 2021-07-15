/**
 * @file DualSteerConfiguration.cpp
 * @author Michele Cecchi
 *
 *  Created on: Jul 11, 2021
 *      Author: michele
 */

#include "orunav_motion_planner/DualSteerConfiguration.h"

DualSteerConfiguration::DualSteerConfiguration(unsigned short int xCell, unsigned short int yCell,
		uint8_t orientID, uint8_t steerID, VehicleMission* vm) : Configuration(xCell, yCell, orientID, steerID, vm) {
	// check that the model is the correct one
	if (!dynamic_cast<DualSteerModel*> (vm->getVehicleModel())) {
		exit(0);
	}
}

DualSteerConfiguration::~DualSteerConfiguration() {
}

DualSteerConfiguration::DualSteerConfiguration(const DualSteerConfiguration& origin) : Configuration(origin){};

std::vector<Configuration*> DualSteerConfiguration::generateNewConfigurations() {

	std::vector<Configuration*> result;
	std::vector<MotionPrimitiveData*> primitives;
	primitives = dynamic_cast<DualSteerModel*>
	(this->getMission()->getVehicleModel())->getApplicablePrimitives(this->getOrientationID(), this->getSteeringID());

	for (std::vector<MotionPrimitiveData*>::iterator primit = primitives.begin(); primit != primitives.end(); primit++) {
		// create a new Configuration for each primitive
		int xc = (*primit)->getXOffset() + this->getXCell();
		int yc = (*primit)->getYOffset() + this->getYCell();
		// configurations must be in non-negative cells
		if (xc >= 0 && yc >= 0) {
			DualSteerConfiguration* newConf = new DualSteerConfiguration(xc, yc, (*primit)->getFinalOrientationID(),
					(*primit)->getFinalSteeringID(), this->getMission());
			newConf->setConfigurationPrimitive((*primit));
			result.push_back(newConf);
		}
	}
	return result;
}

std::vector<Configuration*> DualSteerConfiguration::generateNewConfigurations(World* w) {

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
			DualSteerConfiguration* newConf = new DualSteerConfiguration(xc, yc, (*primit)->getFinalOrientationID(),
					(*primit)->getFinalSteeringID(), this->getMission());
			newConf->setConfigurationPrimitive((*primit));
			result.push_back(newConf);
		}
	}
	return result;
}



Configuration* DualSteerConfiguration::clone() {
	return new DualSteerConfiguration(*this);
}

