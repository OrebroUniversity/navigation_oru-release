/**
 * @file World.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Feb 18, 2011
 *      Author: marcello
 */

#include "orunav_motion_planner/World.h"


World::World(WorldOccupancyMap* m, std::vector<VehicleMission*> missions) {
	missions_ = missions;
	this->map_ = m;

	// check if any of the models in the map has a different granularity
	for(std::vector<VehicleMission*>::iterator it = missions.begin(); it != missions.end(); it++) {
		if (fabs(map_->getGranularity() - (*it)->getVehicleModel()->getModelGranularity()) > WP::CALCULATION_APPROXIMATION_ERROR) {
			writeLogLine(std::string("Map/VehicleModel granularity discrepancy! Fatal Error."), "World", WP::LOG_FILE);
			exit(0);
		}
	}
	xSize_ = m->getXSize();
	ySize_ = m->getYSize();
	xCells_ = (int) ((xSize_ / WP::WORLD_SPACE_GRANULARITY) + WP::CALCULATION_APPROXIMATION_ERROR);
	yCells_ = (int) ((ySize_ / WP::WORLD_SPACE_GRANULARITY) + WP::CALCULATION_APPROXIMATION_ERROR);
	// initialize the collision detector
	cd_ = new CollisionDetector(map_);
	// by default the world is not visualized
	visualization_ = false;
	visualizer_ = 0;
	// now calculate the grid distances for each VehicleMission
	if (map_->containsObstacles()) {
		calculateGridDistancesFromGoals();
	}
}

World::World(const World& other) : xSize_(other.xSize_), ySize_(other.ySize_), xCells_(other.xCells_),
		yCells_(other.yCells_),visualization_(other.visualization_) {
	if (other.visualization_) {
		this->visualizer_ = other.visualizer_;
	}
	if (other.map_) {
		this->map_ = other.map_;
	}
	if (other.cd_) {
		this->cd_ = other.cd_;
	}
}

World::~World() {
	// The Configurations in the discWorld Map are cleaned up when the PathNode are deleted
	if (visualization_) {
		delete visualizer_;
	}
	// delete the collision detector
	delete cd_;
}

void World::calculateGridDistancesFromGoals() {

	for (std::vector<VehicleMission*>::iterator mit = missions_.begin(); mit != missions_.end(); mit++ ) {
		// initialize a new grid
		std::vector<std::vector<double> > gridDistances;
		gridDistances.resize(yCells_);
		for (unsigned int i = 0; i < gridDistances.size(); i++) {
			gridDistances[i].resize(xCells_);
			// reset the values
			for (unsigned int j = 0; j < gridDistances[i].size(); j++) {
				gridDistances[i][j] = INFINITY;
			}
		}

		// breadth first expansion queue
		std::queue<simpleCell*> expQueue;
		// goal configuration cost 0
		simpleCell* c = new simpleCell();
		c->x = (*mit)->getGoalConfiguration()->getXCell();
		c->y = (*mit)->getGoalConfiguration()->getYCell();
		c->cost = 0;
		gridDistances[c->y][c->x] = 0;
		expQueue.push(c);
		// cost of the diagonal, calculated only once
		double diagCost = sqrt(pow(WP::WORLD_SPACE_GRANULARITY, 2) * 2);

		while (expQueue.size() > 0) {
			simpleCell* parent = expQueue.front();
			// is this parent out dated?
			if (parent->cost <= gridDistances[parent->y][parent->x] + WP::CALCULATION_APPROXIMATION_ERROR) {
				for (int dy = -1; dy <= 1; dy++) {
					for (int dx = -1; dx <= 1; dx++) {
						short int new_x = parent->x + dx;
						short int new_y = parent->y + dy;
						// check if in the boundaries
						if (new_x >= 0 && new_y >= 0 && new_x < (int) gridDistances[0].size() && new_y < (int) gridDistances.size()) {
							// check clear of obstacles
							if (!this->containsObstacles() || !(this->map_->getOccupancyValueInCell(new_x, new_y) >= WP::OCCUPANCY_THRESHOLD)) {
								double newCost = parent->cost;
								if (dx == 0 || dy == 0) {
									newCost += WP::WORLD_SPACE_GRANULARITY; // not a diagonal
								} else {
									newCost += diagCost; // diagonal
								}
								if (newCost < gridDistances[new_y][new_x]) {
									gridDistances[new_y][new_x] = newCost;
									simpleCell* newCell = new simpleCell;
									newCell->x = new_x;
									newCell->y = new_y;
									newCell->cost = newCost;
									expQueue.push(newCell);
								}
							}
						}
					}
				}
			}
			expQueue.pop();
			delete parent;
		} // while expQueue
		// assign the gridDistances to the VehicleMission
		(*mit)->setGridDistanceFromGoal(gridDistances);
	}

}

double World::getXWorldSize() {
	return xSize_;
}

double World::getYWorldSize() {
	return ySize_;
}


CollisionDetectorInterface* World::getCollisionDetector() {
	return cd_;
}

void World::enableVisualization(int scale) {
	visualization_ = true;
	// accounting for the approximation to the world limit
	visualizer_ = new DiscWorldVisualizer(xCells_, yCells_,	scale * WP::WORLD_SPACE_GRANULARITY);
	if (this->map_) {
		visualizer_->drawOccupancy(this->map_->getMap());
	}
	for(std::vector<VehicleMission*>::iterator mit = missions_.begin(); mit != missions_.end(); mit++ ) {
		visualizer_->drawStart((*mit)->getStartConfiguration(), (*mit)->getVehicleID());
		visualizer_->drawGoal((*mit)->getGoalConfiguration(), (*mit)->getVehicleID());
	}
}

bool World::isVisualizationEnabled() {
	return visualization_;
}

void World::resetVisualization() {
	if (visualization_) {
		visualizer_->resetVisualizer();
		if (this->map_) {
			visualizer_->drawOccupancy(map_->getMap());
		}
		for(std::vector<VehicleMission*>::iterator mit = missions_.begin(); mit != missions_.end(); mit++ ) {
			visualizer_->drawStart((*mit)->getStartConfiguration(), (*mit)->getVehicleID());
			visualizer_->drawGoal((*mit)->getStartConfiguration(), (*mit)->getVehicleID());
		}
	}
}

void World::visualizeConfigurations(std::vector<Configuration*> confs) {
	if (visualization_) {
		visualizer_->drawConfigurations(confs);
	}
}

bool World::containsObstacles() {
	return map_->containsObstacles();
}

void World::addObstacles(std::vector<cellPosition> obstacles) {
	map_->addObstacles(obstacles);
}

void World::removeObstacles(std::vector<cellPosition> freecells) {
	map_->removeObstacles(freecells);
}
