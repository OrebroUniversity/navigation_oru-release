/**
 * @file CollisionDetector.h
 * @author Marcello Cirillo
 *
 *  Created on: Apr 6, 2011
 *      Author: marcello
 */

#ifndef COLLISIONDETECTOR_H_
#define COLLISIONDETECTOR_H_

#include "CollisionDetectorInterface.h"
#include "WorldOccupancyMap.h"
#include "Configuration.h"

/**
 * @class CollisionDetector
 * This class determines if a given Configuration and its trajectory
 * are inside the World and if they do not collide with any object.
 * It implements the virtual class CollisionDetectorInterface
 */
class CollisionDetector : public CollisionDetectorInterface {

	/** A pointer to the WorldOccupancyMap containing the obstacles */
	 WorldOccupancyMap* map_;

public:

	/**
	 * Instantiate a new CollisionDetector
	 * @param occupancyMap A pointer to the WorldOccupancyMap containing the obstacles
	 */
	CollisionDetector(WorldOccupancyMap* occupancyMap);

	virtual ~CollisionDetector();

	/**
	 * Check if the Configuration (and the trajectory that lead to it)
	 * is collision free (and inside the world's boundaries)
	 * @param conf pointer to the Configuration to check
	 * @returns true if the Configuration is collision free
	 */
	bool isCollisionFree(Configuration* conf);

	/**
	 * Check if a single cell is blocked in the world
	 * @param cellXcoord, cellYcoord The coordinates of the cell to check
	 * @return true if the cell is blocked
	 */
	bool isBlocked(short int cellXcoord, short int cellYcoord);

};

#endif /* COLLISIONDETECTOR_H_ */
