/**
 * @file CarConfiguration.h
 * @author Marcello Cirillo
 *
 *  Created on: Dec 8, 2011
 *      Author: marcello
 */

#ifndef CARCONFIGURATION_H_
#define CARCONFIGURATION_H_

#include "Configuration.h"
#include "CarModel.h"

/**
 * @class CarConfiguration
 * Configuration for a car-like vehicle
 */
class CarConfiguration: public Configuration {


public:
	/**
	 * Create a new CarConfiguration given the x,y cell position and the vehicle mission, which contains also the model.
	 * @param xCell The cell position of the Configuration on the x axis
	 * @param yCell The cell position of the Configuration on the y axis
	 * @param orientID The orientation angle ID of this configuration
	 * @param steerID The steering angle ID of this configuration
	 * @param vm The VehicleMission for this vehicle
	 */
	CarConfiguration(unsigned short int xCell, unsigned short int yCell, uint8_t orientID, uint8_t steerID, VehicleMission* vm);

	~CarConfiguration();

	/**
	 * Copy constructor
	 * @param origin The original configuration
	 */
	CarConfiguration(const CarConfiguration& origin);

	/**
	 * Generate new Configuration starting from the current one.
	 * @deprecated Inefficient function. Use generateNewConfigurations(World* w)
	 * @returns Vector of pointers to new Configuration
	 */
	std::vector<Configuration*> generateNewConfigurations();

	/**
	 * Generates new Configuration starting from the current one.
	 * The Configuration returned by this function are guaranteed to be
	 * collision free. The generation itself is more efficient, as this
	 * function uses the MotionPrimitiveSelector data structure.
	 * @param w A pointer to the world, for collision detection
	 * @return Vector of pointers to new Configuration
	 */
	std::vector<Configuration*> generateNewConfigurations(World* w);

	/**
	 * Clone the Configuration and returns a pointer to the newly created object
	 * @returns A pointer to the newly created Configuration
	 */
	Configuration* clone();

};

#endif /* CARCONFIGURATION_H_ */
