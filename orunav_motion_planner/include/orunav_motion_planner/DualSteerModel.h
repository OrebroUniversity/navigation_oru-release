/**
 * @file DualSteerModel.h
 * @author Michele Cecchi
 *
 *  Created on: Jul 11, 2021
 *      Author: michele
 */

#ifndef DUALSTEERMODEL_H_
#define DUALSTEERMODEL_H_

#include "VehicleModel.h"
#include "WorldParameters.h"

/**
 * @class DualSteerModel
 * Models a car basic properties
 * Note: The current position of a car is represented by the middle point
 * of the rear axel
 */
class DualSteerModel: public VehicleModel {

	/** The length of the car, from the back axle to the front, in meters */
	double carFrontLength_;
	/** The length of the car, from the back axle to the back, in meters */
	double carBackLength_;
	/** The car max steering angle, in radians */
	double carMaxSteeringAngle_;
	/** number of primitives set */
	int sets;
	int currentSet;

protected:

	/** The name of the file containing the motion primitives */
	
/**
	 * Implementation of the virtual function. Generates the file that contains the
	 * additional data of the motion primitives (length, occupancy)
	 */
	
	void generatePrimitiveAdditionalData();

	/**
	 * Implementation of the virtual function. Load motion primitives
	 */
	void loadPrimitiveLookupTable();

public:

	/**
	 * Constructor of a DualSteerModel
	 * The modelPrimitive source file must have normalized angles (-pi / pi).
	 * The physical width and length (front and back from the back axle)
	 * of the Car are contained in the model primitives file
	 * @param modelPrimitivesFilename The name of the file of the motion primitives, without path!
	 */
	DualSteerModel(std::string modelPrimitivesFilename[], int set);
	
	DualSteerModel(std::array<std::string,5>  modelPrimitivesFilename, int set);
	virtual ~DualSteerModel();

	/**
	 * Returns a vectors of pointers to the cells occupied by a car like vehicle in
	 * a specific point.
	 * @param p A pointer to the point where the vehicle is positioned
	 * @return The vector of pointers of cells occupied
	 */
	virtual std::vector<cellPosition*> getCellsOccupiedInPosition(vehicleSimplePoint* p);


	/**
	 * Get the length of the car, from the back axle to the front, in meters
	 * @returns the length of the car, from the back axle to the front, in meters
	 */
	double getCarFrontLength();

	/**
	 * Get the length of the car, from the back axle to the back, in meters
	 * @returns the length of the car, from the back axle to the back, in meters
	 */
	double getCarBackLength();

	/**
	 * Get the car max steering angle, in radians
	 * @returns the car max steering angle, in radians
	 */
	double getCarMaxSteeringAngle();

	/**
	/**
	 * Retrieve the motion primitives from the primitive lookup table
	 * @param orientationID The orientation ID of the starting configuration
	 * @param steeringID The steering ID of the starting configuration, 0 by default
	 * @returns a vector of motion primitives motionPrimitiveData
	 */
	std::vector<MotionPrimitiveData*> getApplicablePrimitives(uint8_t orientationID, uint8_t steeringID);

	/**
	/**
	 *defines the heuristics with which to choose the sets of primitives to use
	 */
	std::vector<int>  selectSet();

};

#endif /* DualSteerModel_H_ */
