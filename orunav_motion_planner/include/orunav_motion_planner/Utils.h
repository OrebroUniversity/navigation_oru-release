/**
 * @file Utils.h
 * @author Marcello Cirillo
 *
 * General utility functions and data structures.
 *
 *  Created on: Mar 14, 2011
 *      Author: marcello
 */

#ifndef UTILS_H_
#define UTILS_H_

#ifdef _WIN32 // windows only
#define _USE_MATH_DEFINES
#endif

#include <vector>
#include "math.h"
#include <typeinfo>
#include <string>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <algorithm>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>


#ifndef _WIN32 // linux systems only
#include <sys/resource.h>
#endif

#ifdef _WIN32
#include <limits>
#include <Windows.h>
#define		INFINITY	std::numeric_limits<float>::infinity()
inline double round( double x ) {return floor( x + 0.5 );} // windows math.h does not include the round function
#endif

// definition for fast use
/** Defines the -pi constant */
#define MINUS_M_PI -3.1416


/**
 * @struct simplePoint
 * Encodes a simple data structure representing a point
 * in a two-dimensional space.
 */
struct simplePoint {
public:
	/** Default constructor */
	simplePoint() {
		x = 0;
		y = 0;
		orient = 0;
	}
	/** Initialize the simplePoint */
	void initSimplePoint(double px, double py, double po) {
		x = px;
		y = py;
		orient = po;
	}
	/** The x coordinate, in meters */
	double x;
	/** The y coordinate, in meters */
	double y;
	/** The orientation, in radians 0 is the angle of the x axis */
	double orient;
};


/**
 * @struct vehicleSimplePoint
 * Expands the simplePoint to encode the steering angle of the vehicle
 */
struct vehicleSimplePoint: public simplePoint {
public:
	/** Default initialization */
	vehicleSimplePoint() : simplePoint() {
		steering = 0;
	}
	/** Initialize the vehicleSimplePoint */
	void initVehicleSimplePoint(double px, double py, double po, double ps) {
		x = px;
		y = py;
		orient = po;
		steering = ps;
	}
	/** The steering angle of a vehicle traversing the point */
	double steering;
};


/**
 * @struct waypoint
 * Encodes a waypoint of a path, for a steering vehicle
 */
struct waypoint {
	/** The x coordinate, in meters */
	double x;
	/** The y coordinate, in meters */
	double y;
	/** The steering angle, in radians */
	double steeringAngle;
	/** Distance from the beginning of the path, in meters */
	double distanceFromStartOfPath;
	/** Print waypoint */
	void print() {
		std::cout << "[Waypoint] \tx: " << x << "\ty: " << y << "\tsteering: " << steeringAngle
				<< "\tdistance: " << distanceFromStartOfPath << std::endl;
	}
};

/**
 * @struct cellPosition
 * Cell position in a matrix
 * Encodes the x_cell,y_cell discrete coordinates of a cell
 */
struct cellPosition {
	short int x_cell; /**< x coordinate */
	short int y_cell; /**< y coordinate */

	cellPosition(short int x, short int y) {
		x_cell = x;
		y_cell = y;
	}

	cellPosition() {
		x_cell = 0;
		y_cell = 0;
	}

	bool operator==(const cellPosition& rhs) const  {
		return (x_cell == rhs.x_cell) && (y_cell == rhs.y_cell);
	}

	struct cellPositionHash{
		int operator()(cellPosition c) const {
			return (c.x_cell << sizeof(short int)) & c.y_cell;
		}
	};
};


/**
 * Returns the cells occupied on a grid of a given granularity by the rectangle passed as parameter.
 * The points must be passed in CLOCKWISE order
 * @param p1 The first point of the rectangle -- orientation is ignored
 * @param p2 The second point of the rectangle -- orientation is ignored
 * @param p3 The third point of the rectangle -- orientation is ignored
 * @param p4 The fourth point of the rectangle -- orientation is ignored
 * @param granularity The granularity of the grid
 * @return The cells occupied by the rectangle
 */
std::vector<cellPosition*> getOccupiedCells(simplePoint p1, simplePoint p2, simplePoint p3, simplePoint p4, double granularity);


/**
 * Calculate the length of a path composed by vehicleSimplePoint
 * @param path The path, composed by a sequence of points
 * @returns The length of the path
 */
double calculateLength(std::vector<vehicleSimplePoint*> path);

/**
 * Add angle measurements in degrees.
 * @param d1 Degrees
 * @param d2 Degrees
 * @param digitprec Number of decimal figures in the output
 * @returns The sum of the two angles
 */
double addDegrees(double d1, double d2, int digitprec);

/**
 * Add angle measurements in radiants.
 * Normalize to pi, -pi and rounds it up to 5 decimals
 * @param a1 Radiants
 * @param a2 Radiants
 * @param digitprec Number of decimal figures in the output
 * @returns The normalized sum of the two angles, with 5 digits decimal precision
 */
double addAnglesRadians(double a1, double a2, int digitprec);

/**
 * Round up an angle (in radiants) using a rouding base
 * @param a The angle to round
 * @param base The rounding base
 * @param digitprec Number of decimal figures in the output
 * @returns The rounded up angle
 */
double roundAngle(double a, double base, int digitprec);

/**
 * Calculate in absolute value the difference between two angles, in radians.
 * The result is always <= pi
 * @param a,b angles in radians
 * @param digitprec Number of decimal figures in the output
 * @returns The difference
 */
double calculateAngleDifference(double a, double b, int digitprec);

/**
 * Round a number to a specific decimal position. Calling the function
 * roundNumber(-23.32334, 4) will result in -23.3233, while
 * roundNumber(-23.32335, 4) will result in -23.3234
 * @param n The number to round
 * @param dec The decimal figure at which to round
 * @returns The rounded number
 */
double roundNumberToDecimal(double n, double dec);

/**
 * Floor a positive number to a given (positive) base number with a certain tolerance
 * @param n The number to floor
 * @param base The rounding base
 * @param tolerance
 * @returns The floored number
 */
double floorPositiveNumber(double n, double base, double tolerance);

/**
 * Write a log line into the log file. If filename is equal to "stdout"
 * then the line is printed on the console.
 * @param line The line to log
 * @param filename Log file
 */
void writeLogLine(std::string line, std::string filename);

/**
 * Write a log line into the log file. If filename is equal to "stdout"
 * then the line is printed on the console.
 * @param line The line to log
 * @param className The name of the class that is calling the logger
 * @param filename Log file
 */
void writeLogLine(std::string line, const char* className, std::string filename);


#ifndef _WIN32 // linux function only
/**
 * Increase the stack size available to the main thread of the program
 */
void increaseStackSize(long int newSize);
#endif

#endif /* UTILS_H_ */
