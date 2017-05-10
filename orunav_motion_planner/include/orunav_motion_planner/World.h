/**
 * @file World.h
 * @author Marcello Cirillo
 *
 *  Created on: Feb 18, 2011
 *      Author: marcello
 */

#ifndef WORLD_H_
#define WORLD_H_

#include <queue>
#include "WorldOccupancyMap.h"
#include "CollisionDetector.h"
#include "DiscWorldVisualizer.h"
#include "VehicleMission.h"

/**
 * @class World
 * The class World describes the environment for the motion planning
 * It allows a graphical visualization of the environment.
 * The world contains the WorldMap*, that is, the occupancy map of the world
 */
class World {

	/** Pointer to the WorldMap that contains occupancy information */
	WorldOccupancyMap* map_;

	/** Pointer to the collision detector */
	CollisionDetectorInterface* cd_;

	/** A Vector containing a mission for each Vehicle */
	std::vector<VehicleMission*> missions_;

	/** The size of the environment on the x axis */
	double xSize_;
	/** The size of the environment on the y axis */
	double ySize_;

	/** The number of cells on the x axis */
	unsigned short int xCells_;
	/** The number of cells on the y axis */
	unsigned short int yCells_;

	/** Visualization flag (true to allow graphics) */
	bool visualization_;
	/** Pointer to visualizer */
	DiscWorldVisualizer* visualizer_;

	/**
	 * Calculate the grid-distance from goal for each vehicle taking obstacles into account.
	 * This values are then stored in the VehicleMission.
	 */
	void calculateGridDistancesFromGoals();

	/**
	 * Simple private structure to represent a cell on a grid and its associated cost
	 */
	struct simpleCell {
		unsigned short int x;
		unsigned short int y;
		double cost;
	};

public:

	/**
	 * Instantiate a new world from a WorldMap
	 * @param m Pointer to the WorldMap
	 * @param model The model of the Vehicle for which the world is created
	 */
	World(WorldOccupancyMap* m, std::vector<VehicleMission*> missions);

	/**
	 * Copy constructor
	 */
	World(const World&);

	~World();

	/**
	 * Get the world size on the x coordinate
	 * @returns the size of the World along the x axis
	 */
	double getXWorldSize();

	/**
	 * Get the world size on the y coordinate
	 * @returns the size of the World along the y axis
	 */
	double getYWorldSize();

	/**
	 * Get the pointer to the CollisionDetector
	 * @returns The pointer to the CollisionDetector
	 */
	CollisionDetectorInterface* getCollisionDetector();

	/**
	 * Generate a DiscWorldVisualiser object.
	 * To be invoked after constructor, goal and start setting.
	 * @param scale Defines the scale factor to be used in the world representation (pixels per meter)
	 */
	void enableVisualization(int scale);

	/**
	 * Check if the visualization is enabled
	 * @returns True if visualization is enabled
	 */
	bool isVisualizationEnabled();

	/**
	 * Reset the visualization, drawing start, goal and occupancy
	 */
	void resetVisualization();

	/**
	 * If visualization is enabled, this method is used to visualize a vector of
	 * Configuration pointers.
	 * @param confs The vector of Configuration pointers to be visualized
	 */
	void visualizeConfigurations(std::vector<Configuration*> confs);

	/**
	 * Check if the World has obstacles (e.g., if it is a simple
	 * world or if it has been created from a map)
	 * @returns If the World has obstacles
	 */
	bool containsObstacles();

	/**
	 * Add the cells in the vector as obstacles
	 * @param obstacles The vector of cells to be added as obstacles
	 */
	void addObstacles(std::vector<cellPosition> obstacles);

	/**
	 * Restores the cells passed as arguments as they were in the occupancy map
	 * @param freecells The vector of cells to be restored
	 */
	void removeObstacles(std::vector<cellPosition> freecells);

};

#endif /* WORLD_H_ */
