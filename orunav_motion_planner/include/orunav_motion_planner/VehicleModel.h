/**
 * @file VehicleModel.h
 * @author Marcello Cirillo
 *
 *  Created on: Mar 29, 2011
 *      Author: marcello
 */

#ifndef VEHICLEMODEL_H_
#define VEHICLEMODEL_H_


#include <map>
#include <fstream>
#include <boost/regex.hpp>
#include <sstream>
#include <iostream>
#include <exception>
#include <math.h>

#include "MotionPrimitiveData.h"
#include "Utils.h"
#include "MotionPrimitiveSelector.h"

#define 	CELL_DELIMITER	9999

/**
 * @class VehicleModel
 * Base class that describes the model of a vehicle. It contains information
 * about the width and length (in meters) of the vehicle, the x,y granularity at
 * which it operates, the number of orientation angles and steering angles which it supports and
 * the number steering angle partitions (calculated as (2*M_PI)/steeringAngleGranularity).
 * It also contains the lookup table for the motion primitives and the heuristic table.
 */
class VehicleModel {

	friend class ModelUtilities;

protected:

	/** The width of the vehicle (meters) */
	double width_;
	/** The length of the vehicle (meters) */
	double length_;

	/** The range at which the vehicle can interfere with other vehicles:
	 * sum of the longest primitive + the max between the diagonal of the vehicle. In meters */
	double interferenceRange_;

	/** The world granularity (grid size) for which this VehicleModel has been built */
	double vehicleGranularity_;

	/** The number of angles in which the vehicle can be oriented. They are assumed equally spaced */
	uint8_t orientationAngles_;
	/** The number of steering angle partitions for this vehicle, calculated as (2*M_PI)/steeringAngleGranularity.
	 * This value does not reflect the actual number of steering angles supported by the vehicle, but gives information
	 * about their granularity */
	uint8_t steeringAnglePartitions_;
	/** The number of steering angles supported by the vehicle. The steering angles are supposed to be symmetric with
	 * respect to the 0 angle (== central axis of the vehicle). A vehicle with 16 steeringAnglePartitions_ could also have
	 * only 3 or 5 as steeringAngleCardinality_ */
	uint8_t steeringAngleCardinality_;

	/** The name of the file containing the motion primitives */
	std::string motionPrimitivesFilename_;
	/** The name of the file containing the additional data for each primitive */
	std::string motionPrimitiveAdditionalDataFilename_;

	/** Motion primitives lookup table type: the two uint8_t are the orientation ID and the steering ID */
	typedef std::map<std::pair<uint8_t, uint8_t>, std::vector<MotionPrimitiveData*> > motionPrimitivesLookup;
	/** Motion primitives lookup entry pair type: the two uint8_t are the orientation ID and the steering ID */
	typedef std::pair<std::pair<uint8_t, uint8_t>, std::vector<MotionPrimitiveData*> > motionPrimitivesLookupEntry;

	/** Motion primitive selector lookup table type: see motionPrimitivesLookup */
	typedef std::map<std::pair<uint8_t, uint8_t>, MotionPrimitiveSelector* > motionPrimitiveSelectorLookup;
	/** Motion primitives selector lookup entry pair type: see motionPrimitivesLookupEntry */
	typedef std::pair<std::pair<uint8_t, uint8_t>, MotionPrimitiveSelector* > motionPrimitiveSelectorLookupEntry;

	/** Motion primitives lookup table */
	motionPrimitivesLookup modelMotionPrimitivesLT_;

	/** Motion primitive selector lookup table */
	motionPrimitiveSelectorLookup modelMotionPrimitivesSelectorLT_;

	/** The Heuristic table should be saved by the destructor only if it has been modified */
	bool newEntriesInHT_;

	/**
	 * Purely virtual function. Load motion primitives.
	 */
	virtual void loadPrimitiveLookupTable() = 0;

	/**
	 * Once the modelMotionPrimitivesLT_ has been loaded, we can calculate the modelMotionPrimitivesSelector_
	 */
	void prepareMotionPrimitiveSelectorTable();

	/**
	 * Purely virtual function. Generates the file that contains the
	 * additional data of the motion primitives (length, occupancy)
	 */
	virtual void generatePrimitiveAdditionalData() = 0;

	/**
	 * Correct the distances among primitives to fully support the 8-axis symmetry
	 * avoiding discrepancies among identical primitives
	 */
	void adjustPrimitiveDistancesWith8AxisSymmetry();

	/**
	 * Get the primitive lookup table of the vehicle
	 * @return The table containing all the motion primitives of the vehicle
	 */
	motionPrimitivesLookup getPrimitivesLookupTable();

	/**
	 * Set the primitive lookup table for the vehicle
	 * @param prim The new table containing the motion primitives to be used by the vehicle
	 */
	void setPrimitivesLookupTable(motionPrimitivesLookup prim);

	/** Heuristic function lookup table type */
	typedef std::map<uint64_t, double> heuristicLookupTable;
	/** Heuristic lookup entry pair type: uint64_t is a combination of x,y cell offsets and steering and orientation IDs
	 * key = (dx << 48) | ((dy << 32) | ((startOrientID << 24) | ((goalOrientID << 16) | ((startSteeringID << 8) | goalSteeringID)))) */
	typedef std::pair<uint64_t, double> heuristicLookupEntry;

	/** Heuristic function lookup table */
	heuristicLookupTable modelHeuristicLT_;

	/** Heuristic lookup table filename */
	std::string heuristicLTFilename_;

	/**
	 * Load heuristic function from file
	 * @param filename String indicating the file name. The path is attached in the function
	 */
	void loadHeuristicTable(std::string filename);

	/**
	 * Get the key for the heuristic table corresponding to given start and goal poses
	 * @param startX Start cell on the X axis
	 * @param startY Start cell on the Y axis
	 * @param startOrientID Start orientation ID
	 * @param startSteeringID Start steering ID
	 * @param goalX Goal cell on the X axis
	 * @param goalY Goal cell on the Y axis
	 * @param goalOrientID Goal orientation ID
	 * @param goalSteeringID Goal steering ID
	 */
	uint64_t calculateHeuristicTableKey(
			unsigned short int startX, unsigned short int startY, uint8_t startOrientID, uint8_t startSteeringID,
			unsigned short int goalX, unsigned short int goalY, uint8_t goalOrientID, uint8_t goalSteeringID);

public:

	/**
	 * Constructor of a vehicle model. All the information about the Vehicle are included
	 * into the motion primitives file.
	 * @param modelPrimitivesFilename The name of the file of the motion primitives,
	 * *without* path and extension (mprim)!
	 */
	VehicleModel(std::string modelPrimitivesFilename);

	virtual ~VehicleModel();

	/**
	 * Purely virtual function which returns a vectors of pointers to the cells occupied by a vehicle in
	 * a specific point.
	 * @param p A pointer to the point where the vehicle is positioned
	 * @return The vector of pointers of cells occupied
	 */
	virtual std::vector<cellPosition*> getCellsOccupiedInPosition(vehicleSimplePoint* p) = 0;

	/**
	 * Get the width of the vehicle (in meters)
	 * @returns The width of the vehicle
	 */
	double getWidth();

	/**
	 * Get the length of the vehicle (in meters)
	 * @returns The length of the vehicle
	 */
	double getLength();

	/**
	 * Get the range at which the vehicle can interfere with other vehicles:
	 * sum of the longest primitive + the max between the diagonal of the vehicle. In meters
	 * @return The interference range
	 */
	double getInterferenceRange();

	/**
	 * Get the world granularity for which this vehicle has been built
	 * @return The VehicleModel granularity (grid size)
	 */
	double getModelGranularity();

	/**
	 * Get the number of angles in which the vehicle can be oriented. They are assumed equally spaced
	 * @returns The number of angles in which the vehicle can be oriented
	 */
	uint8_t getOrientationAngles();

	/**
	 * Get the number of steering angle partitions for this vehicle, calculated as (2*M_PI)/steeringAngleGranularity.
	 * This value does not reflect the actual number of steering angles supported by the vehicle, but gives information
	 * about their granularity
	 * @returns The number of steering angle partitions for this vehicle
	 */
	uint8_t getSteeringAnglePartitions();

	/**
	 * Get the number of steering angles supported by the vehicle. The steering angles are supposed to be symmetric with
	 * respect to the 0 angle (== central axis of the vehicle). A vehicle with 16 steering angle partitions could also have
	 * only 3 or 5 as steering angle cardinality
	 * @returns The number of steering angles supported by the vehicle
	 */
	uint8_t getSteeringAngleCardinality();

	/**
	 * Given an angle, it returns the ID of the closest allowed orientation angle for this vehicle
	 * @param angle The angle to evaluate
	 * @return The ID of the closest allowed orientation angle for this vehicle
	 */
	uint8_t getClosestAllowedOrientationID(double angle);

	/**
	 * Given an angle, it returns the ID of the closest allowed steering angle for this vehicle
	 * @param angle The angle to evaluate
	 * @return The ID of the closest allowed steering angle for this vehicle
	 */
	uint8_t getClosestAllowedSteeringID(double angle);


	/**
	 * Get the filename of the model primitives, with path and extension
	 * @returns The filename
	 */
	std::string getModelPrimitivesFilename();

	/**
	 * Retrieve the motion primitives from the primitive lookup table
	 * @param orientationID The orientation ID of the starting configuration
	 * @param steeringID The steering ID of the starting configuration, 0 by default
	 * @returns a vector of motion primitives motionPrimitiveData
	 */
	virtual std::vector<MotionPrimitiveData*> getApplicablePrimitives(uint8_t orientationID, uint8_t steeringID = 0);


	/**
	 * @TODO check if it works
	 * @param startXcell
	 * @param startYcell
	 * @param orientationID
	 * @param steeringID
	 * @return
	 */
	std::vector<MotionPrimitiveData*> selectApplicablePrimitives(World* w, short int startXcell, short int startYcell,
			uint8_t orientationID, uint8_t steeringID = 0);


	/**
	 * Add entry to the heuristic lookup table
	 * @param startX Start cell on the X axis
	 * @param startY Start cell on the Y axis
	 * @param startOrientID Start orientation ID
	 * @param startSteeringID Start steering ID
	 * @param goalX Goal cell on the X axis
	 * @param goalY Goal cell on the Y axis
	 * @param goalOrientID Goal orientation ID
	 * @param goalSteeringID Goal steering ID
	 * @param value The value for this heuristic entry
	 */
	void setEntryInHeuristicTable(unsigned short int startX, unsigned short int startY, uint8_t startOrientID, uint8_t startSteeringID,
			unsigned short int goalX, unsigned short int goalY, uint8_t goalOrientID, uint8_t goalSteeringID, double value);

	/**
	 * Get value from heuristic table. If no value exists in the table
	 * is not found, the table returns -1
	 * @param startX Start cell on the X axis
	 * @param startY Start cell on the Y axis
	 * @param startOrientID Start orientation ID
	 * @param startSteeringID Start steering ID
	 * @param goalX Goal cell on the X axis
	 * @param goalY Goal cell on the Y axis
	 * @param goalOrientID Goal orientation ID
	 * @param goalSteeringID Goal steering ID
	 * @returns The value from the heuristic table. -1 if not found
	 */
	double getHeuristicValueFromTable(unsigned short int startX, unsigned short int startY, uint8_t startOrientID, uint8_t startSteeringID,
			unsigned short int goalX, unsigned short int goalY, uint8_t goalOrientID, uint8_t goalSteeringID);

	/**
	 * Check if a specific heuristic value is in the table.
	 * @param startX Start cell on the X axis
	 * @param startY Start cell on the Y axis
	 * @param startOrientID Start orientation ID
	 * @param startSteeringID Start steering ID
	 * @param goalX Goal cell on the X axis
	 * @param goalY Goal cell on the Y axis
	 * @param goalOrientID Goal orientation ID
	 * @param goalSteeringID Goal steering ID
	 * @returns True is the value is in the table
	 */
	bool isHeuristicValueInTable(unsigned short int startX, unsigned short int startY, uint8_t startOrientID, uint8_t startSteeringID,
			unsigned short int goalX, unsigned short int goalY, uint8_t goalOrientID, uint8_t goalSteeringID);

	/**
	 * Save the heuristic table to file. Uses the heuristicLTFilename, stored when the
	 * lookup table has been loaded.
	 */
	void saveHeuristicTable();

};

#endif /* VEHICLEMODEL_H_ */
