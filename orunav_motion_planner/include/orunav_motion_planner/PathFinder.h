/**
 * \mainpage Multi-Robot Timeless Path Finder Library
 *
 * @image html author.png
 * @author Marcello Cirillo
 *
 * @image html orientation_angles.png
 *
 * test
 *
 * @todo FINISH THE DOCUMENTATION
 *
 * \section intro_sec Introduction
 * The TimelessPathFinder library provides different functionalities and models to calculate paths for
 * both holonomic and non-holonomic vehicles. Currently, three different types of vehicles are supported:
 * Unicycles, Car-like vehicles and Waist actuated vehicles.
 *
 *
 * \section example_sec Examples
 *
 * @code
 * WP::setLogLevel(2);
 * @endcode
 *
 * @see WP (WorldParameters)
 *
 * @code
 * WP::setLogLevel(3);
 * WP::setLogFile("/home/marcello/test.txt");
 * WP::setSaveFinalVisualizationToFile(true);
 *
 * //load the model
 * CarModel* model = new CarModel("SnowWhite_16_3_02");
 *
 * VehicleMission* mission1 = new VehicleMission(model, 2, 1.8, 0   , 0, 6, 1.8, 0   , 0, std::string("myfirstvehicle"));
 * VehicleMission* mission2 = new VehicleMission(model, 6, 1.5, M_PI, 0, 2, 1.5, M_PI, 0, "mysecondvehicle");
 *
 * PathFinder* pf = new PathFinder(10, 5);
 * pf->addMission(mission1);
 * pf->addMission(mission2);
 * pf->setTimeBound(1600);
 *
 * std::vector<std::vector<Configuration*> > sol = pf->solve(true);
 * pf->printPaths(sol);
 *
 * // cleanup
 * delete pf;
 * delete mission1;
 * delete mission2;
 * delete model;

 * for (std::vector<std::vector<Configuration*> >::iterator it = sol.begin(); it != sol.end(); it++) {
 *		std::vector<Configuration*> confs = (*it);
 *		for (std::vector<Configuration*>::iterator confit = confs.begin(); confit != confs.end(); confit++) {
 *			delete *confit;
 *		}
 *		confs.clear();
 * }
 * sol.clear();
 * @endcode
 *
 * \section model_generation Model Generation
 *
 * \section model_refinement Model Refinement
 *
 * @code
 * CarModel* model = new CarModel("SnowWhite_16_3_02");
 * ModelUtilities::generateModelHeuristicTable(model, 7, 0);
 * delete model;
 * @endcode
 *
 * @code
 * CarModel* model = new CarModel("SnowWhite_16_3_02");
 * ModelUtilities::reduceModelMotionPrimitiveSet(model, 1.4);
 * delete model;
 * @endcode
 *
 *
 */

/**
 * @file PathFinder.h
 * @author Marcello Cirillo
 *
 *  Created on: Apr 11, 2011
 *      Author: marcello
 */

#ifndef PATHFINDER_H_
#define PATHFINDER_H_

#include <math.h>
#include "boost/date_time/posix_time/posix_time.hpp"

#include "World.h"
#include "WorldOccupancyMap.h"
#include "ARAStarPathPlanner.h"
#include "AStarPathPlanner.h"
#include "PathNode.h"

/**
 * @class PathFinder
 * PathFinder provides a high level interface to multiple robot PathPlanners, managing the main data
 * structures required and providing methods to iteratively solve a single planning problem.
 * PathFinder can use two different algorithms: A* and ARA*.
 * By default PathFinder is invoked to calculate an optimal solution, using a standard A* algorithm,
 * and to avoid data collection for the HeuristicTable. However, high level function allow PathFinder
 * to harvest data for the HeuristicTable (only using an optimal A* search and for a single vehicle
 * at a time) OR to provide a solution in a bounded time (ARA*). The two options are mutually exclusive.
 */
class PathFinder {

private:

	/** The pointer to the World */
	World* myWorld_;

	/** A Vector containing pointers to the VehicleMissions */
	std::vector<VehicleMission*> missions_;

	/** If true (false, by default) enables the PathPlanner to extract data for the
	 * current vehicle Heuristic Table during the search. Only possible using an
	 * optimal A* search and a single vehicle*/
	bool dataGatheringForVehicleHT_;

	/** If different than 0, it specifies the time bound (in seconds) allowed to the
	 * PathPlanner to provide a feasible, albeit not optimal solution. If the timeBound
	 * variable is set, then ARA* is employed */
	int timeBound_;

	/** Pointer to the initial world occupancy map -- if provided */
	WorldOccupancyMap* worldMap_;

	/** Size of the World, x axis */
	double xSize_;
	/** Size of the World, y axis */
	double ySize_;

public:

	/**
	 * Create a new PathFinder object
	 * @param x,y Dimensions of the World
	 */
	PathFinder(double x, double y);

	/**
	 * Create a new PathFinder object from map
	 * @param filename The file name of the map
	 */
	PathFinder(std::string filename);

	/**
	 * Create a new PathFinder directly using a map object
	 * @param map The map (will be copied).
	 */
	PathFinder(const WorldOccupancyMap& map);

	virtual ~PathFinder();

	/**
	 * Set the occupancy map (if it exists) as a portion of the current one
	 * @param xfrom,yfrom Coordinates (in meters) of the point of the current
	 * occupancy map that will become the left lower corner of the submap
	 * @param xto, yto Coordinates (in meters) of the point of the current
	 * occupancy map that will become the top right corner of the submap
	 */
	void selectSubMap(double xfrom, double yfrom, double xto, double yto);

	/**
	 * Add a mission for a single vehicle to PathFinder
	 * @param m A pointer to the VehicleMission to be solved
	 */
	void addMission(VehicleMission* m);

	/**
	 * Solve the problem
	 * @param visualization Enable/disable visualization
	 * @returns A vector of vectors of pointers to Configurations solving the problem (1 vector per vehicle)
	 */
	std::vector<std::vector<Configuration*> > solve(bool visualization);

	/**
	 * Extract the waypoints from a solution vector (of Configuration)
	 * @param path A vector of Configuration from which the solution is extracted
	 * @returns A vector of waypoints
	 */
	std::vector<waypoint*> extractWaypoints(std::vector<Configuration*> path);

	/**
	 * Enables the data gathering to populate the heuristic table of the
	 * vehicle. Only possible with an optimal A* search and a single vehicle.
	 */
	void enableDataGatheringForVehicleHT();

	/**
	 * Set the time bound for the PathPlanner to return a feasible, albeit not optimal
	 * solution. When timeBound is greater than 0, an ARA* algorithm is employed.
	 * @param seconds The time bound (in seconds) for the PathPlanner to provide a solution
	 */
	void setTimeBound(int seconds);

	/**
	 * Prints the final paths on the log file and on the visualization (it active)
	 * @param paths The vector of Configuration vectors to print
	 */
	void printPaths(std::vector<std::vector<Configuration*> > paths);

};

#endif /* PATHFINDER_H_ */
