/**
 * @file VehicleMission.h
 * @author Marcello Cirillo
 *
 *  Created on: Mar 28, 2013
 *      Author: marcello
 */

#ifndef VEHICLEMISSION_H_
#define VEHICLEMISSION_H_

#include <string>
#include <vector>
#include <math.h>

class Configuration;
#include "VehicleModel.h"
class CarModel;
class LHDModel;
class UnicycleModel;



/**
 * @class VehicleMission This class contains the information about the mission
 * of a single vehicle: vehicle model, name, ID, initial and goal pose
 */
class VehicleMission {


private:

	/** The vehicleID_ counter */
	static unsigned short int vehicleNum_;
	/** The number of active missions */
	static unsigned short int activeMissions_;


	/** The VehicleModel used for the planning */
	VehicleModel* vm_;
	/** The initial configuration of the VehicleModel */
	Configuration* start_;
	/** The desired goal state */
	Configuration* goal_;
	/** The Vehicle's ID */
	unsigned short int vehicleID_;
	/** The vehicle's name */
	std::string vehicleName_;
	/** A map which stores the anti-depression heuristic for the vehicle in Worlds with obstacles: initialized as
	 * an empty matrix and updated if necessary by the World, it contains the grid distance from goal in the form [y][x] */
	std::vector<std::vector<double> > gridDistanceFromGoal_;

public:
	/**
	 * Set a mission for a vehicle: VehicleModel, name (optional) start and goal pose and vehicle name
	 * @param m
	 * @param start_x Starting pose, x coordinate
	 * @param start_y Starting pose, y coordinate
	 * @param start_o Starting pose, orientation
	 * @param start_steering Starting pose, steering angle
	 * @param goal_x Goal pose, x coordinate
	 * @param goal_y Goal pose, y coordinate
	 * @param goal_o Goal pose, orientation
	 * @param goal_steering Goal pose, steering angle
	 * @param name Name of the vehicle
	 */
	VehicleMission(VehicleModel* m, double start_x, double start_y, double start_o, double start_steering,
			double goal_x, double goal_y, double goal_o, double goal_steering, std::string name = "");

	~VehicleMission();

	/**
	 * Assign to the VehicleMission the gridmap of cell-wise distance from goal in a World with
	 * obstacles.
	 * @param distances The grid of int with the distances from the goal position of this vehicle
	 */
	void setGridDistanceFromGoal(std::vector<std::vector<double> > distances);

	/**
	 * Get a pointer to the start configuration for this Vehicle's mission
	 * @return Start configuration of this Vehicle
	 */
	Configuration* getStartConfiguration();

	/**
	 * Get a pointer to the goal configuration for this Vehicle's mission
	 * @return Goal configuration of this Vehicle
	 */
	Configuration* getGoalConfiguration();

	/**
	 * Get this vehicle's ID
	 * @return Vehicle's ID
	 */
	unsigned short int getVehicleID();

	/**
	 * Get this vehicle's name
	 * @return Vehicle's name
	 */
	std::string getVehicleName();

	/**
	 * Get a pointer to this Vehicle's model
	 * @return A pointer to this vehicle's VehicleModel
	 */
	VehicleModel*  getVehicleModel();

	/**
	 * Get the grid distance from any cell to the goal cell for this mission
	 * @param x The x cell coordinate
	 * @param y The y cell coordinate
	 * @return The distance from goal on grid
	 */
	double getGridDistanceFromGoal(int x, int y);
};


#endif /* VEHICLEMISSION_H_ */
