/**
 * @file CollisionDetector.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Apr 6, 2011
 *      Author: marcello
 */

#include "orunav_motion_planner/CollisionDetector.h"

CollisionDetector::CollisionDetector(WorldOccupancyMap* occupancyMap) {
	map_ = occupancyMap;
}

CollisionDetector::~CollisionDetector() {
}

bool CollisionDetector::isCollisionFree(Configuration* conf) {
	std::vector<cellPosition> sweptCells = conf->getCellsSwept();
	for (std::vector<cellPosition>::iterator it = sweptCells.begin(); it != sweptCells.end(); it++) {
		if ((*it).y_cell >= map_->getYCells() || (*it).x_cell >= map_->getXCells() || (*it).y_cell < 0 || (*it).x_cell < 0) {
			return false;
		}
		if (map_->getOccupancyValueInCell((*it).x_cell, (*it).y_cell) >= WP::OCCUPANCY_THRESHOLD) {
			return false;
		}
	}
	return true;
}

bool CollisionDetector::isBlocked(short int cellXcoord, short int cellYcoord) {
	if (cellYcoord >= map_->getYCells() || cellXcoord >= map_->getXCells() || cellYcoord < 0 || cellXcoord < 0)
		return true;
	if (map_->getOccupancyValueInCell(cellXcoord, cellYcoord) >= WP::OCCUPANCY_THRESHOLD) {
		return true;
	}
	return false;
}
