/**
 * @file DiscWorldVisualizer.h
 * @author Marcello Cirillo
 *
 *  Created on: Mar 2, 2011
 *      Author: marcello
 */

#ifndef WORLDVISUALIZER_H_
#define WORLDVISUALIZER_H_

#include "opencv/cv.h"
#include "opencv/highgui.h"
#ifdef _WIN32 // windows
#include <boost/thread/thread.hpp>
#else  // linux
#include <boost/thread.hpp>
#endif
#include <boost/date_time.hpp>

#include "WorldParameters.h"
#include "LHDConfiguration.h"
#include "CarConfiguration.h"
#include "UnicycleConfiguration.h"

#include "CarModel.h"
#include "LHDModel.h"
#include "UnicycleModel.h"

#define AVAIL_COLORS	5

/**
 * @class DiscWorldVisualizer
 * This class exports methods to visualize a discretized version of the world.
 * The display routines are run in a separate thread.
 */
class DiscWorldVisualizer {

private:

	/** define the colors for 5 different vehicles */
	static unsigned short int R[5];
	static unsigned short int G[5];
	static unsigned short int B[5];

	/** The image to be displayed */
	IplImage* worldDisplayImg_;

	/** Mutex to avoid deadlocks when two threads try to work on the image */
	boost::mutex update_mutex_;

	/** The number of cells on the x axis on the discretized representation of the World */
	int xTotCels_;
	/** the number of cells on the y axis on the discretized representation of the World */
	int yTotCels_;
	/** Size of the images */
	int imgXsize_;
	int imgYsize_;
	/** Scale factor for the world representation, in pixels per cell */
	int scaleFactor_;

	/** boolean to signal the display thread to stop */
	bool threadStop_;

	/** visualization thread */
	boost::thread visualizedThread_;

	/**
	 * Stop the thread.
	 */
	void stopVisualization();

	/**
	 * Meta-method that selects the appropriate Configuration drawing
	 * function
	 * @param conf The configuration to draw
	 * @param R,G,B Color
	 */
	void drawConfiguration(Configuration* conf, int R, int G, int B);

	/**
	 * Draw a Configuration: position (x,y) and orientation
	 * @param conf The configuration to draw
	 * @param R,G,B Color
	 */
	void drawBaseConfiguration(Configuration* conf, int R, int G, int B);

	/**
	 * Draw a CarConfiguration: position (x,y) and orientation
	 * @todo CHECK wheels orientation static
	 * @param conf The configuration to draw
	 * @param R,G,B Color
	 */
	void drawCarConfiguration(CarConfiguration* conf, int R, int G, int B);

	/**
	 * Draw a waist actuated vehicle configuration
	 * @todo CHECK waist actuation always equal to 0
	 * @param conf The LHDConfiguration to draw
	 * @param R,G,B Color
	 */
	void drawLHDConfiguration(LHDConfiguration* conf, int R, int G, int B);

	/**
	 * Draw a path composed by vehicleSimplePoint
	 * @param path vehicleSimplePoint vector
	 */
	void drawPath(std::vector<vehicleSimplePoint> path, int R, int G, int B);

	/**
	 * Draw a dot in the position specified
	 * @param x,y The coordinates of the centre
	 * @param radius The radius of the filled circle
	 * @param R,G,B Color
	 */
	void drawDot(double x, double y, double radius, int R, int G, int B);

	/**
	 * Draw a line between two points
	 * @param xfrom,yfrom The coordinates of the line start
	 * @param xto, yto The coordinates of the line end
	 * @param R,G,B The color of the line
	 */
	void drawLine(double xfrom, double yfrom, double xto, double yto, int R, int G, int B);

	/**
	 * Draw a colour filled rectangle given x,y of two opposite corners
	 * @param xfrom,yfrom The coordinates of the first corner
	 * @param xto, yto The coordinates of the opposite corner
	 * @param R,G,B The color of the rectangle
	 */
	void drawFilledRectangle(double xfrom, double yfrom, double xto, double yto, int R, int G, int B);

	/**
	 * Draw a rectangle given x,y of two opposite corners
	 * @param xfrom,yfrom The coordinates of the first corner
	 * @param xto, yto The coordinates of the opposite corner
	 * @param R,G,B The color of the rectangle
	 */
	void drawRectangle(double xfrom, double yfrom, double xto, double yto, int R, int G, int B);

public:
	/**
	 * The constructor
	 * @param xCels Number of cells on the x axis
	 * @param yCels Number of cells on the y axis
	 * @param scale The scale factor for the visualization
	 */
	DiscWorldVisualizer(int xCels, int yCels, int scale);
	virtual ~DiscWorldVisualizer();

	/**
	 * Reset the visualization to the simple grid
	 */
	void resetVisualizer();

	/**
	 * Draw the occupancy map in different shades according to the occupancy value
	 * @param map Pointer to the occupancy map
	 */
	void drawOccupancy(std::vector<std::vector<double> > map);

	/**
	 * Draw a start Configuration
	 * @param conf The Configuration of the starting point
	 * @param vehicleID The ID of the vehicle to draw
	 */
	void drawStart(Configuration* conf, int vehicleID = 0);

	/**
	 * Draw a goal Configuration
	 * @param conf The Configuration representing the goal
	 * @param vehicleID The ID of the vehicle to draw
	 */
	void drawGoal(Configuration* conf, int vehicleID = 0);

	/**
	 * Draw a vector of Configuration pointers color coded by vehicle ID
	 * @param confs The vector of Configuration pointers to draw
	 */
	void drawConfigurations(std::vector<Configuration*> confs);

	/**
	 * Display the image of the world. This function is designed to be used in
	 * separate thread using the DisplayDiscWorld_thread_wrapper
	 */
	void display();

};

/**
 * Wrapper function that allows the display function of the visualizer
 * to be called as a thread
 */
static inline void DisplayDiscWorld_thread_wrapper(DiscWorldVisualizer* v) {
	v->display();
}

#endif /* WORLDVISUALIZER_H_ */
