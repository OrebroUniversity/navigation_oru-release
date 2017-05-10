/**
 * @file WorldOccupancyMap.h
 * @author Marcello Cirillo
 *
 *  Created on: Apr 6, 2011
 *      Author: marcello
 */

#ifndef WORLDOCCUPANCYMAP_H_
#define WORLDOCCUPANCYMAP_H_

#include <fstream>
#include <string>
#include <boost/regex.hpp>
#include <stdlib.h>
#include <algorithm>
#include <iostream>
#include <math.h>

#include "orunav_motion_planner/Utils.h"
#include "orunav_motion_planner/WorldParameters.h"

/**
 * @class WorldOccupancyMap
 * This class loads the occupancy map of the world from a file
 * and stores both data and metadata (e.g.: map resolution) in
 * a WorldOccupancyMap object
 */
class WorldOccupancyMap {

	/** The number of cells on the X axis of the map */
	int xcells_;
	/** The number of cells on the Y axis of the map */
	int ycells_;

	/** The number of cells on the X axis of the original map */
	int originalxcells_;
	/** The number of cells on the Y axis of the original map */
	int originalycells_;

	/** Granularity of the occupancy map: meters per cell */
	double granularity_;
	/** Granularity of the original occupancy map: meters per cell */
	double originalGranularity_;

	/** Flag that indicates if the map contains obstacles or not */
	bool containsObstacles_;

	/** The occupancy map, whose granularity may change */
	std::vector<std::vector<double> > occupancyMap_;
	/** Same size of the occupancyMap_, keeps track of the obstacles for restoring the
	 * original values to the cells when needed */
	std::vector<std::vector<double> > originalObstacles_;
	/** The original occupancy map, as loaded from file */
	std::vector<std::vector<double> > originalOccupancyMap_;

	/**
	 * Private method to split a single line of the file into a vector of doubles
	 * @param s The line of the file
	 * @param f The value separator
	 * @return A vector of double representing a line in the map
	 */
	std::vector<double> splitMapLine(const std::string& s, const std::string& f);

	/**
	 * Decrease the Map granularity
	 * @param times Final granularity = granularity / times
	 */
	void decreaseGranularity(int times);

	/**
	 * Increase the Map granularity
	 * @param times Final granularity = granularity * times
	 */
	void increaseGranularity(int times);

public:

	/**
	 * Constructor. Load the map from a file
	 * @param filename The name of the file to load
	 */
	WorldOccupancyMap(std::string filename);

	/**
	 * Constructor. Creates an empty map given the number of x and y cells and the granularity
	 * @param xcells The number of cells on the x axis
	 * @param ycells The number of cells on the y axis
	 * @param mapGranularity The granularity of the map (meters per cell)
	 */
	WorldOccupancyMap(unsigned short int xcells, unsigned short int ycells, double mapGranularity);

	WorldOccupancyMap() { }

	virtual ~WorldOccupancyMap();

	/**
	 * Return the x size of the map, in meters
	 * @returns The x size (meters)
	 */
	double getXSize();

	/**
	 * Return the y size of the map, in meters
	 * @returns The y size (meters)
	 */
	double getYSize();

	/**
	 * Returns the number of cells on the x axis of the map
	 * @return The number of cells on the x axis of the map
	 */
	unsigned short int getXCells();

	/**
	 * Returns the number of cells on the y axis of the map
	 * @return The number of cells on the y axis of the map
	 */
	unsigned short int getYCells();

	/**
	 * Return the granularity of the occupancy map, in meters per cell
	 * @returns The granularity of the occupancy map
	 */
	double getGranularity();

	/**
	 * Change the granularity of the Map.
	 * NOTE: if newGran < currentGranularity, the final granularity will be
	 * <= newGran (newGran will be an upper bound)
	 * if newGran > currentGranularity, the final resolution will be
	 * >= newGran (newGran will be a lower bound)
	 * @param newGran The new granularity, in meters per cell
	 */
	void scaleGranularity(double newGran);

	/**
	 * Return the occupancy map
	 * @returns The occupancy map of the World
	 */
	std::vector<std::vector<double> > getMap();

	/**
	 * Restore the original occupancy map, as loaded from file
	 */
	void restoreOriginalMap();

	/**
	 * Set the occupancy map as a portion of the current one
	 * @param xfrom,yfrom Coordinates (in meters) of the point of the current
	 * occupancy map that will become the left lower corner of the submap
	 * @param xto, yto Coordinates (in meters) of the point of the current
	 * occupancy map that will become the top right corner of the submap
	 */
	void selectSubMap(double xfrom, double yfrom, double xto, double yto);

	/**
	 * Get the value associated to a specific CELL in the current occupancyMap
	 * occupancyMap[y_cell][x_cell].
	 * @param x_cell, y_cell Coordinates of the cell to look for
	 * @returns The associated value
	 */
	double getOccupancyValueInCell(unsigned short int x_cell, unsigned short int y_cell);

	/**
	 * Add the cells in the vector as obstacles in the occupancy map
	 * @param obstacles The vector of cells to be added as obstacles to the map
	 */
	void addObstacles(std::vector<cellPosition> obstacles);

	/**
	 * Restores the cells passed as arguments as they were in the occupancy map
	 * @param freecells The vector of cells to be restored
	 */
	void removeObstacles(std::vector<cellPosition> freecells);

	/**
	 * Check if the WorldOccupancyMap contains obstacles (e.g., it has been created by a map
	 * @return True if the map contains obstacles
	 */
	bool containsObstacles();

 	/**
	 * Initialize the map. Note this will assign both the map and the original map.
	 */
	void initialize(int xcells, int ycells, double granularity, const std::vector<std::vector<double> > &occupancyMap);

};

#endif /* WORLDOCCUPANCYMAP_H_ */
