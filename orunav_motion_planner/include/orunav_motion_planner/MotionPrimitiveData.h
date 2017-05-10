/**
 * @file MotionPrimitiveData.h
 * @author Marcello Cirillo
 *
 *  Created on: Apr 1, 2013
 *      Author: marcello
 */

#ifndef MOTIONPRIMITIVEDATA_H_
#define MOTIONPRIMITIVEDATA_H_

#include <cstdint>
#include "Utils.h"
#include "WorldParameters.h"



/**
 * @class MotionPrimitiveData
 * Encodes a single motion primitive. The motion primitive is mainly characterized by a path,
 * in which each pose is encoded by a vehicleSimplePoint (x,y,orientation,steering).
 * The vehicleSimplePoint encodes the information in meters and radians with the initial
 * (x,y) position of the path being typically (0,0).
 * The class stores also additional information, such as the IDs of the steering and orientation
 * angles both at the beginning and at the end of the path, the primitive ID, the offsets (in cells)
 * of the motion, the overall distance covered, the cost multiplier associated with this primitive,
 * the direction of the motion and the cells swept by the vehicle during the motion.
 */
class MotionPrimitiveData {

protected:

	/** The ID of the primitive */
	unsigned short int ID_;
	/** The final orientation angle ID of this primitive */
	uint8_t finalOrientID_;
	/** The final steering angle ID of this primitive */
	uint8_t finalSteeringID_;

	/** The angle ID of the starting orientation */
	uint8_t startOrientID_;
	/** The angle ID of the starting steering */
	uint8_t startSteeringID_;

	/** The cell offset of the primitive from the initial pose to the goal pose on the x axis */
	short int xOffset_;
	/** The cell offset of the primitive from the initial pose to the goal pose on the y axis */
	short int yOffset_;
	/** The points touched by the path */
	std::vector<vehicleSimplePoint*> traj_;

	/** The distance covered */
	double distance_;
	/** The cost multiplier for trajectories (e.g., when moving backwards) */
	double costmult_;
	/** The direction of the motion: 1 := forward | -1 := backwards */
	short int motiondir_;
	/** The discrete positions of the cells swept by the vehicle during the motion */
	std::vector<cellPosition*> cellsSwept_;
	/** The discrete positions of the cells occupied by the vehicle at the end of the motion */
	std::vector<cellPosition*> cellsOccFinal_;


public:

	/**
	 * Constructor. Initialize all the values to null or 0.
	 */
	MotionPrimitiveData();

	/**
	 * Destructor
	 */
	~MotionPrimitiveData();

	/**
	 * Equality operator
	 * @param rhs The motionPrimitiveData to compare against
	 * @return true if the MotionPrimitiveData are identical
	 */
	bool operator==(const MotionPrimitiveData & rhs) const  {
		return (ID_ == rhs.ID_) && (xOffset_ == rhs.xOffset_) && (yOffset_ == rhs.yOffset_) &&
				(finalOrientID_ == rhs.finalOrientID_) && (finalSteeringID_ == rhs.finalSteeringID_) &&
				(startOrientID_ == rhs.startOrientID_) && (startSteeringID_ == rhs.startSteeringID_);
	}

	/**
	 * Set the ID of the primitive
	 * @param id The ID of the primitive
	 */
	void setID(unsigned short int id);

	/**
	 * Set the trajectory of this motion primitive.
	 * This method will also assign startOrientID_, endOrientID_, startSteeringID_, endSteeringID_ to the primitive
	 * @param trajectory A Vector to pointers of vehicleSimplePoints
	 * @param orientationAngles The number of orientation angles used by the model
	 * @param steeringAnglePartitions The number of steering angle partitions for this vehicle,
	 * calculated as (2*M_PI)/steeringAngleGranularity. This parameter does not reflect the actual number of
	 * steering angles used by the vehicle.
	 */
	void setTrajectory(std::vector<vehicleSimplePoint*> trajectory, uint8_t orientationAngles, uint8_t steeringAnglePartitions);

	/**
	 * Set the distance covered by the motion primitive
	 * @param distance The distance covered
	 */
	void setDistance(double distance);

	/**
	 * Set the cost multiplier for this primitive
	 * @param costmult The cost multiplier
	 */
	void setCostMultiplier(double costmult);

	/**
	 * Set the direction of the motion of this primitive: 1 := forward | -1 := backwards
	 * @param motiondir The motion direction
	 */
	void setMotionDirection(short int motiondir);

	/**
	 * Set the vector of cells swept by this primitive
	 * @param cellsSwept The discrete positions of the cells swept by the vehicle during the motion
	 */
	void setSweptCells(std::vector<cellPosition*> cellsSwept);

	/**
	 * Set the vector of cells occupied by the vehicle at the end of this primitive
	 * @param cellsOcc The discrete positions of the cells occupied by the vehicle at the end of the motion
	 */
	void setOccCells(std::vector<cellPosition*> cellsOcc);

	/**
	 * Get the ID of the primitive
	 * @return The ID of the primitive
	 */
	unsigned short int getID();

	/**
	 * Get the orientation angle ID of this motion primitive
	 * @return the orientation angle ID of this motion primitive
	 */
	uint8_t getFinalOrientationID();

	/**
	 * Get the final steering angle ID of this motion primitive
	 * @return the steering angle ID of this motion primitive
	 */
	uint8_t getFinalSteeringID();

	/**
	 * Get the angle ID of the starting orientation of this motion primitive
	 * @return the angle ID of the starting orientation of this motion primitive
	 */
	uint8_t getStartingOrientationID();

	/**
	 * Get the angle ID of the starting steering (if applicable), 0 by default
	 * @return the angle ID of the starting of this motion primitive
	 */
	uint8_t getStartingSteeringID();

	/**
	 * Get the cell offset of the primitive from the initial to the goal pose on the x axis
	 * @return the cell offset of the primitive from the initial to the goal pose on the x axis
	 */
	short int getXOffset();

	/**
	 * Get the cell offset of the primitive from the initial to the goal pose on the y axis
	 * @return the cell offset of the primitive from the initial to the goal pose on the y axis
	 */
	short int getYOffset();

	/**
	 * Get the points touched by the path
	 * @return A Vector of pointers to vehicleSimplePoints of the points touched by the path
	 */
	std::vector<vehicleSimplePoint*> getTrajectory();


	/**
	 * Get the distance covered by the primitive
	 * @return The distance covered by the primitive (in meters)
	 */
	double getDistanceCovered();

	/**
	 * Get the cost multiplier of this primitive
	 * @return The cost multiplier associated with this primitive
	 */
	double getCostMultiplier();

	/**
	 * Get the motion direction of this primitive: 1 := forward | -1 := backwards
	 * @return The motion direction of this primitive
	 */
	short int getMotionDirection();

	/**
	 * Get the discrete positions of the cells swept by the vehicle during the motion
	 * @return The discrete positions of the cells swept by the vehicle during the motion
	 */
	std::vector<cellPosition*> getSweptCells();

	/**
	 * Get the discrete positions of the cells occupied by the vehicle at the end of the motion
	 * @return The discrete positions of the cells occupied by the vehicle at the end of the motion
	 */
	std::vector<cellPosition*> getOccCells();



};
#endif /* MOTIONPRIMITIVEDATA_H_ */
