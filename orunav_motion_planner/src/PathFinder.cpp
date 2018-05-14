/**
 * @file PathFinder.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Apr 11, 2011
 *      Author: marcello
 */

#include "orunav_motion_planner/PathFinder.h"

PathFinder::PathFinder(double x, double y) {
	unsigned short int xCells = (unsigned short int) (x / WP::WORLD_SPACE_GRANULARITY + WP::CALCULATION_APPROXIMATION_ERROR);
	unsigned short int yCells = (unsigned short int) (y / WP::WORLD_SPACE_GRANULARITY + WP::CALCULATION_APPROXIMATION_ERROR);
	// create a new empty map
	worldMap_ = new WorldOccupancyMap(xCells, yCells, WP::WORLD_SPACE_GRANULARITY);
	xSize_ = x;
	ySize_ = y;
	dataGatheringForVehicleHT_ = false;
	timeBound_ = 0;
	myWorld_ = 0;
	missions_.clear();
}

PathFinder::PathFinder(std::string filename) {
	worldMap_ = new WorldOccupancyMap(std::string().append(WP::MAPS_DIR).append(filename).append(".map"));
	xSize_ = worldMap_->getXSize();
	ySize_ = worldMap_->getYSize();
	dataGatheringForVehicleHT_ = false;
	timeBound_ = 0;
	myWorld_ = 0;
	missions_.clear();
}

PathFinder::PathFinder(const WorldOccupancyMap& map) {
	worldMap_ = new WorldOccupancyMap();
	*worldMap_ = map;
	xSize_ = worldMap_->getXSize();
	ySize_ = worldMap_->getYSize();
	dataGatheringForVehicleHT_ = false;
	timeBound_ = 0;
	myWorld_ = 0;
	missions_.clear();
}

PathFinder::~PathFinder() {
	if (worldMap_) {
		delete worldMap_;
	}
	if (myWorld_) {
		delete myWorld_;
	}
}

void PathFinder::selectSubMap(double xfrom, double yfrom, double xto, double yto) {
	if (worldMap_) {
		worldMap_->selectSubMap(xfrom, yfrom, xto, yto);
		xSize_ = worldMap_->getXSize();
		ySize_ = worldMap_->getYSize();
	} else {
		if (WP::LOG_LEVEL >= 1) {
			writeLogLine(std::string("WARNING: WorldMap not found!"), "PathFinder", WP::LOG_FILE);
		}
	}
}

void PathFinder::addMission(VehicleMission* m) {
	missions_.push_back(m);
}

void PathFinder::enableDataGatheringForVehicleHT() {
	dataGatheringForVehicleHT_ = true;
}

void PathFinder::setTimeBound(int seconds) {
	timeBound_ = seconds;
}

std::vector<std::vector<Configuration*>> PathFinder::solve(bool visualization) {

	std::vector<std::vector<Configuration*> > result;
	result.clear();

	// do we have missions?
	if (missions_.size() == 0) {
		writeLogLine(std::string("No missions to accomplish"), "PathFinder", WP::LOG_FILE);
		return result;
	}

	// check if all VehicleModels have the same world granularity
	for (std::vector<VehicleMission*>::iterator it =  missions_.begin(); it != missions_.end(); it ++) {
		if (fabs((*it)->getVehicleModel()->getModelGranularity() - WP::WORLD_SPACE_GRANULARITY) > WP::CALCULATION_APPROXIMATION_ERROR) {
      writeLogLine(std::string("FATAL ERROR: mismatching model resolutions (check also that the primitive file was loaded)"), "PathFinder", WP::LOG_FILE);
			return result;
		}
	}

	// first, check if PathFinder has been properly invoked
	if (dataGatheringForVehicleHT_ && timeBound_ > 0) {
		if (WP::LOG_LEVEL >= 1) {
			writeLogLine(std::string("FATAL ERROR: data gathering not possible with time restrictions"),
					"PathFinder", WP::LOG_FILE);
		}
		return result;
	}

	if (WP::LOG_LEVEL >= 1) {
		std::string line = std::string("Solving -- ");
		line.append(boost::posix_time::to_iso_string(boost::posix_time::second_clock::local_time()));
		writeLogLine(line, "PathFinder", WP::LOG_FILE);
	}

	// create the world
	myWorld_ = new World(worldMap_, missions_);
	if (WP::LOG_LEVEL >= 1) {
		if(worldMap_->containsObstacles()) {
			writeLogLine(std::string("Creating World from Map"), "PathFinder", WP::LOG_FILE);
		} else {
			writeLogLine(std::string("Creating simple World"), "PathFinder", WP::LOG_FILE);
		}
	}

	// enable world visualization
	if (visualization) {
		int scale = xSize_ > ySize_ ? (int) (1200 / xSize_) : (int) (1000 / ySize_);
		myWorld_->enableVisualization(scale);
	}

	// create the initial node
	std::vector<Configuration*> initialConfs;
	initialConfs.clear();
	for (std::vector<VehicleMission*>::iterator it = missions_.begin(); it != missions_.end(); it++) {
		// clone the start configurations
		Configuration* conf = (*it)->getStartConfiguration()->clone();
		initialConfs.push_back(conf);
	}

	// create the first node and pass it to the PathPlanner (clone it, because it
	// shouldn't be destroyed when the planner is done)
	PathNode* startNode = new PathNode(initialConfs, myWorld_, 0);

	// reset the clock
	boost::posix_time::ptime startTime(boost::posix_time::microsec_clock::local_time());

	// solve the problem: check if to use A* or ARA*
	std::vector<Node*> solution;
	if (timeBound_ > 0) {
		if (WP::LOG_LEVEL >= 1) {
			char line[50];
			sprintf(line, "Using ARA* planner [%d seconds]", timeBound_);
			writeLogLine(std::string(line), "PathFinder", WP::LOG_FILE);
		}
		ARAStarPathPlanner* planner = new ARAStarPathPlanner(startNode, myWorld_, startTime, timeBound_);
		solution = planner->solve();
		delete planner;
	} else {
		if (WP::LOG_LEVEL >= 1) {
			writeLogLine("Using A* algorithm", "PathFinder", WP::LOG_FILE);
		}
		AStarPathPlanner* planner = new AStarPathPlanner(startNode, myWorld_);
		if (dataGatheringForVehicleHT_) {
			planner->enableHTDataCollection();
		}
		solution = planner->solve();
		delete planner;
	}

	// stop the clock
	boost::posix_time::ptime endTime(boost::posix_time::microsec_clock::local_time());
	boost::posix_time::time_duration duration(endTime - startTime);

	if (WP::LOG_LEVEL >= 1) {
		char line[50];
		sprintf(line, "Execution time: %lu milliseconds", 	(unsigned long int) duration.total_milliseconds());
		writeLogLine(std::string(line), "PathFinder", WP::LOG_FILE);
	}

	if (WP::LOG_LEVEL >= 2) {
		PathNode* pn;
		for (std::vector<Node*>::iterator it = solution.begin(); it != solution.end(); it++) {
			pn = dynamic_cast<PathNode*>(*it);
			pn->print();
		}
	}

	// prepare the result to return, extracting the Configurations. Cleanup
	PathNode* pn;
	unsigned short int numberOfVehicles = missions_.size();
	result.resize(numberOfVehicles);
	for (std::vector<Node*>::iterator it = solution.begin(); it != solution.end(); it++) {
		pn = dynamic_cast<PathNode*>(*it);
		for (unsigned short int i = 0; i < numberOfVehicles; i ++) {
			Configuration* c = (pn->getConfigurations())[i];
			result[i].push_back(c->clone());
		}
		delete pn;
	}

	// remove duplicate Configurations from the results
	std::vector<Configuration*> tmp;
	for (unsigned short int i = 0; i < numberOfVehicles; i ++) {
		tmp.clear();
		while (result[i].size() > 0) {
			if (result[i].size() > 1 && result[i][0]->equalConfigurations(result[i][1])) {
				Configuration* config = result[i][0];
				delete config;
				result[i].erase(result[i].begin());
			} else {
				tmp.push_back(result[i][0]->clone());
				Configuration* config = result[i][0];
				delete config;
				result[i].erase(result[i].begin());
			}
		}
		result[i].clear();
		result[i] = tmp;
	}

	return result;
}

std::vector<waypoint*> PathFinder::extractWaypoints(std::vector<Configuration*> path) {
	std::vector<waypoint*> waypoints;
	double distanceFromStart = 0;
	for (std::vector<Configuration*>::iterator it = path.begin(); it != path.end(); it++) {
		waypoint* w = new waypoint;
		w->x = (*it)->getXCoordinate();
		w->y = (*it)->getYCoordinate();
		w->steeringAngle = (*it)->getSteering();
		distanceFromStart += (*it)->getTrajectoryLength();
		w->distanceFromStartOfPath = distanceFromStart;
		waypoints.push_back(w);
	}
	return waypoints;
}

void PathFinder::printPaths(std::vector<std::vector<Configuration*> > paths) {

	if (paths.front().empty()) {
		return;
	}

	// reset world visualization
	if (myWorld_->isVisualizationEnabled()) {
		myWorld_->resetVisualization();
	}
	if (WP::LOG_LEVEL >= 1 && paths.size() == 0) {
		writeLogLine(std::string("No solution found"), "PathFinder", WP::LOG_FILE);
	} else if (WP::LOG_LEVEL >= 3 && paths.size() != 0) {
		writeLogLine(std::string("Solution:"), "PathFinder", WP::LOG_FILE);
	}
	for (std::vector<std::vector<Configuration*> >::iterator it = paths.begin(); it != paths.end(); it++) {
		if (myWorld_->isVisualizationEnabled()) {
			myWorld_->visualizeConfigurations(*it);
		}
		if (WP::LOG_LEVEL >= 4) {
			std::vector<Configuration*> confs = (*it);
			for (std::vector<Configuration*>::iterator confit = confs.begin(); confit != confs.end(); confit++) {
				// print also the intermediate points
				std::vector<vehicleSimplePoint> traj = (*confit)->getTrajectory();
				for (std::vector<vehicleSimplePoint>::iterator spit = traj.begin(); spit != traj.end(); spit++) {
					char line[100];
					sprintf(line, "\t%.4f\t%.4f\t%.4f\t%.4f", (*spit).x, (*spit).y, (*spit).orient, (*spit).steering);
					writeLogLine(std::string(line), "PathFinder", WP::LOG_FILE);
				}
			}
		} else if (WP::LOG_LEVEL >= 3) {
			char line[100];
			sprintf(line, "VehicleID : %d", (*it).front()->getMission()->getVehicleID());
			writeLogLine(std::string(line), "PathFinder", WP::LOG_FILE);
			std::vector<Configuration*> confs = (*it);
			for (std::vector<Configuration*>::iterator confit = confs.begin(); confit != confs.end(); confit++) {
				char line[100];
				sprintf(line, "Configuration: \tx: %.3f\ty: %.3f\to: %.3f\ts: %.3f",
						(*confit)->getXCoordinate(), (*confit)->getYCoordinate(), (*confit)->getOrientation(), (*confit)->getSteering());
				writeLogLine(std::string(line), "PathFinder", WP::LOG_FILE);
			}
		}
	}
	if (myWorld_->isVisualizationEnabled()) {
#ifdef _WIN32
		Sleep(5);
#else
		sleep(5);
#endif
	}
}
