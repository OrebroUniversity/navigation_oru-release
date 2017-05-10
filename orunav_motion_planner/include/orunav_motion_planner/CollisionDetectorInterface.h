/**
 * @file CollisionDetectorInterface.h
 * @author Marcello Cirillo
 *
 *  Created on: Apr 8, 2011
 *      Author: marcello
 */

#ifndef COLLISIONDETECTORINTERFACE_H_
#define COLLISIONDETECTORINTERFACE_H_

class Configuration;
struct cellPosition;

/**
 * @class CollisionDetectorInterface
 * This class provides a simple interface (virtual method)
 * that should be implemented by all CollisionDetectors
 */
class CollisionDetectorInterface {
public:

	virtual ~CollisionDetectorInterface(){}

	/**
	 * Check if the Configuration (and the trajectory that lead to it)
	 * is collision free (and inside the world's boundaries)
	 * @param conf pointer to the Configuration to check
	 * @returns true if there are no collisions
	 */
	virtual bool isCollisionFree(Configuration* conf) = 0;

	/**
	 * Check if a single cell is blocked in the world
	 * @param cellXcoord, cellYcoord The coordinates of the cell to check
	 * @return true if the cell is blocked
	 */
	virtual bool isBlocked(short int cellXcoord, short int cellYcoord) = 0;
};

#endif /* COLLISIONDETECTORINTERFACE_H_ */
