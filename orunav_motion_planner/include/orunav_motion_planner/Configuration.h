/**
 * @file Configuration.h
 * @author Marcello Cirillo
 *
 *  Created on: Mar 14, 2011
 *      Author: marcello
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <vector>
#include <cstdlib>
#include <iostream>
#include <iomanip>

#include "Utils.h"
#include "VehicleMission.h"
#include "WorldParameters.h"

#include "MotionPrimitiveData.h"

/**
 * @class Configuration
 * Virtual class that characterizes a VehicleModel in the environment:
 * position, velocity and trajectory to get to the current Configuration.
 */
class Configuration {

private:
	/** X cell position on grid with current world / model granularity */
	unsigned short int xcell_;
	/** Y cell position on grid with current world / model granularity */
	unsigned short int ycell_;
	/** The orientation angle ID of this configuration */
	uint8_t orient_;
	/** The steering angle ID of this configuration */
	uint8_t steering_;
	/** Pointer to the VehicleMission for this agent, which also contains the VehicleModel */
	VehicleMission* mission_;
	/** Pointer to the motion primitive data that generated this configuration */
	MotionPrimitiveData* primitiveOfThisConfiguration_;

public:
	/**
	 * Create a new configuration given the x,y cell position and the vehicle mission, which contains also the model.
	 * @param xCell The cell position of the Configuration on the x axis
	 * @param yCell The cell position of the Configuration on the y axis
	 * @param orientID The orientation angle ID of this configuration
	 * @param steerID The steering angle ID of this configuration
	 * @param vm The VehicleMission for this vehicle
	 */
	Configuration(unsigned short int xCell, unsigned short int yCell, uint8_t orientID, uint8_t steerID, VehicleMission* vm);

	virtual ~Configuration();

	/**
	 * Copy constructor. This can be invoked only by derived classes, as this is a virtual class
	 * @param origin The Configuration to be copied
	 */
	Configuration(const Configuration& origin);

	/**
	 * Set the pointer to the motion primitive that generated this configuration
	 * @param prim A pointer to the motion primitive data that generated this configuration
	 */
	void setConfigurationPrimitive(MotionPrimitiveData* prim);

	/**
	 * Returns the cell position of the Configuration on the x axis
	 * @returns The cell position of the Configuration on the x axis
	 */
	unsigned short int getXCell();

	/**
	 * Returns the cell position of the Configuration on the y axis
	 * @returns The cell position of the Configuration on the y axis
	 */
	unsigned short int getYCell();

	/**
	 * Returns the orientation angle ID of this configuration
	 * @return The orientation angle ID of this configuration
	 */
	uint8_t getOrientationID();

	/**
	 * Returns the steering angle ID of this configuration
	 * @return The steering angle ID of this configuration
	 */
	uint8_t getSteeringID();

	/**
	 * Returns the coordinate of this Configuration on the X axis, in meters
	 * @return The coordinate of this Configuration on the X axis, in meters
	 */
	double getXCoordinate();

	/**
	 * Returns the coordinate of this Configuration on the Y axis, in meters
	 * @return The coordinate of this Configuration on the Y axis, in meters
	 */
	double getYCoordinate();

	/**
	 * Returns the orientation of this Configuration, in radians
	 * @return The orientation of this Configuration, in radians
	 */
	double getOrientation();

	/**
	 * Returns the steering of this Configuration, in radians
	 * @return The steering of this Configuration, in radians
	 */
	double getSteering();

	/**
	 * Returns the a pointer to the VehicleMission of the Configuration
	 * @returns Pointer to the VehicleMission
	 */
	VehicleMission* getMission();

	/**
	 * Returns a pointer to the motion primitive data that generated this configuration
	 * @returns Pointer to the MotionPrimitiveData
	 */
	MotionPrimitiveData* getPrimitiveOfThisConfiguration();

	/**
	 * Get the trajectory generated to reach the current Configuration
	 * @returns A vector of vehicleSimplePoint
	 */
	std::vector<vehicleSimplePoint> getTrajectory();

	/**
	 * Get the distance traveled to reach the current Configuration
	 * (that is, the length of the trajectory)
	 * @returns The space traveled to reach this configuration
	 */
	double getTrajectoryLength();

	/**
	 * Returns the cells of the World that are occupied at the end of the trajectory
	 * associated with this Configuration.
	 * @returns Vector of cellPosition
	 */
	std::vector<cellPosition> getCellsOccupied();

	/**
	 * Returns the cells of the World that are occupied at some point by the vehicle during the
	 * trajectory associated with this Configuration.
	 * @returns Vector of cellPosition
	 */
	std::vector<cellPosition> getCellsSwept();

	/**
	 * Virtual method to calculate the G value in A* for this configuration
	 * The method only considers info available in the Configuration itself
	 * (e.g., in path planning it does not consider terrain properties)
	 * @returns The cost in meters
	 */
	virtual double getCostToThisConfiguration();

	/**
	 * Returns the motion direction of this configuration, that is, if the vehicle
	 * has been moving forward (:= 1) or backward (:= -1) to reach this Configuration.
	 * @return The motion direction ( forward := 1 | backward != -1)
	 */
	int getMotionDirectionToThisConfiguration();

	/**
	 * Virtual method to estimate the cost (in space) from the current
	 * configuration to a target configuration
	 * This method is used to calculate the H value in A* and, if available, it uses the value
	 * from the heuristic table of the model
	 * @param conf1 The Configuration from which to estimate the distance
	 * @returns The estimated cost from this to the target configuration
	 */
	virtual double estimateCostFromThisConfiguration(Configuration* conf1);

	/**
	 * Verify if two Configuration are equivalent
	 * @param conf1 The Configuration to check for equivalency
	 */
	virtual bool equalConfigurations(Configuration* conf1);

	/**
	 * Return the hash value of this configuration
	 * @return The hash value, expressed in uint_32
	 */
	virtual uint32_t getHash();

	/**
	 * Generate new Configuration starting from the current one.
	 * Pure virtual function
	 * @deprecated Inefficient function. Use generateNewConfigurations(World* w)
	 * @returns Vector of pointers to new Configuration
	 */
	virtual std::vector<Configuration*> generateNewConfigurations() = 0;

	/**
	 * Generates new Configuration starting from the current one.
	 * The Configuration returned by this function are guaranteed to be
	 * collision free. The generation itself is more efficient, as this
	 * function uses the MotionPrimitiveSelector data structure.
	 * @param w A pointer to the world, for collision detection
	 * @return Vector of pointers to new Configuration
	 */
	virtual std::vector<Configuration*> generateNewConfigurations(World* w) = 0;


	/**
	 * Print the relevant information of the Configuration to log
	 */
	virtual void print();

	/**
	 * Return the relevant information of the Configuration in a string
	 * @returns The string with the info
	 */
	virtual std::string printToString();

	/**
	 * Clone the Configuration and returns a pointer to the newly created object
	 * Purely virtual method. Necessary to allow derived classes to call the appropriate
	 * copy constructor
	 * @returns A pointer to the newly created Configuration
	 */
	virtual Configuration* clone() = 0;
};


/**
 * @struct ConfigurationVectorHash
 * Provides the operator to generate a hash value for a vector of configurations
 */
struct ConfigurationVectorHash {
	std::uint64_t operator()(std::vector<Configuration*> k) const {
		// calculate the spacing between different hashes
		unsigned short int spacing = 64 / k.size();
		unsigned short int offset = 0;
		uint64_t result = 0;
		for (std::vector<Configuration*>::iterator it = k.begin(); it != k.end(); it++){
			result = (((*it)->getHash() << (offset * spacing)) | result);
			offset = offset + 1;
		}
		return result;
	}
};

/**
 * @struct ConfigurationVectorEqual
 * Provides the operator to compare two vectors of Configuration pointers and returns if the two vectors are identical
 */
struct ConfigurationVectorEqual {
	bool operator()(std::vector<Configuration*> lhs, std::vector<Configuration*> rhs) const {
		bool equalConfigurations = true;
		// check if the sizes of the vectors are identical
		if (lhs.size() != rhs.size()) {
			if (WP::LOG_LEVEL >= 1) {
				writeLogLine(std::string("Configuration vector sizes inconsistent"), "Configuration", WP::LOG_FILE);
				exit(0);
			}
		}

		// extra check to ascertain that the two Configurations refer to the same vehicle
		for (unsigned int index = 0; index < rhs.size(); index++) {
			if (rhs[index]->getMission()->getVehicleID() != lhs[index]->getMission()->getVehicleID()){
				if (WP::LOG_LEVEL >= 1) {
					writeLogLine(std::string("Vehicle IDs inconsistent"), "Configuration", WP::LOG_FILE);
					exit(0);
				}
			}
			// check Configuration by Configuration
			if (!(lhs[index]->equalConfigurations(rhs[index]))) {
				equalConfigurations = false;
				break;
			}
		}
		return equalConfigurations;
	}
};



#endif /* CONFIGURATION_H_ */
