/**
 * @file WorldParameters.h
 * @author Marcello Cirillo
 *
 *  Created on: Mar 18, 2011
 *      Author: marcello
 */

#ifndef WORLDPARAMETERS_H_
#define WORLDPARAMETERS_H_

#ifdef _WIN32 // windows only
#define _USE_MATH_DEFINES
#endif

#include "math.h"
#include <string>

/**
 * @class WP
 * Static class that defines the world parameters used
 */
class WP {
public:

	/**
	 * Define the available node expansion method the planner can use in successor generation
	 */
	typedef enum NodeExpansionMethod {
		NAIVE 	= 0,  //!< NAIVE Base method
		EP 		= 1,  //!< EP Expansion Pruning
		FSG 	= 2,  //!< FSG Fast Successor Generation
		EPFSG 	= 3   //!< EPFSG Expansion Pruning + Fast Successor Generation
	} NodeExpansionMethod;


	/** Select the expansion method for successor generation */
	static NodeExpansionMethod NODE_EXPANSION_METHOD;



	/** Log level for extra console output (0,1,2,3) */
	static int LOG_LEVEL;
	/** The log file name and path -- defulat value: stdout, that means the log output is printed directly on console */
	static std::string LOG_FILE;
	/** If the planner has a graphical visualization and the visualizer allows it,
	 * this flag enables to save the final image of the visualization to a file. */
	static bool SAVE_FINAL_VISUALIZATION_TO_FILE;

	/** The directory containing the motion primitives for the models */
	static std::string PRIMITIVES_DIR;
	/** The directory containing the heuristic lookup tables */
	static std::string TABLES_DIR;
	/** The directory containing the maps */
	static std::string MAPS_DIR;

	/** The max size of the expansion queue */
	static unsigned long int EXPANSION_QUEUE_MAX;
	/** The cost cut-off threshold (no nodes with a cost higher than this value are expanded */
	static double COST_CUTOFF;


	/** Flag to enable/disable the garbage collection (removal of useless nodes, time expensive but memory efficient */
	static bool ALLOW_GARBAGE_COLLECTION;
	/** Flag to enable/disable the use of goal distance heuristic estimation in the planners:
	 * A* with a null heuristic estimation is equivalent to a Dijkstra's algorithm.
	 * Note: a null heuristic is a consistent heuristic */
	static bool USE_HEURISTIC_ESTIMATION;
	/** Heuristic value multiplier: useful for Anytime A* algorithms. Equal to 1 (A* requirement) by default */
	static double HEURISTIC_VALUE_MULTIPLIER;

	/** Define the space granularity to employ in the discretized World, in meters */
	static double WORLD_SPACE_GRANULARITY;
	/** Define allowed the tolerated calculation approximation error */
	static double CALCULATION_APPROXIMATION_ERROR;
	/** Define the occupancy threshold that is considered not navigable */
	static double OCCUPANCY_THRESHOLD;

	/** Define how many decimal figures should be used */
	static int DECIMAL_APPROXIMATION;


	/**
	 * Change the node expansion method
	 * @param method The expansion method to use
	 */
	static void setExpansionMethod(WP::NodeExpansionMethod method);

	/**
	 * Change the directory where the motion primitives are stored
	 * @param dir The directory
	 */
	static void setPrimitivesDir(std::string dir);

	/**
	 * Change the directory where the heuristic lookup tables are stored
	 * @param dir The directory
	 */
	static void setTablesDir(std::string dir);

	/**
	 * Change the directory where the maps are stored
	 * @param dir The directory
	 */
	static void setMapsDir(std::string dir);

	/**
	 * Change the max size of the expansion queue
	 * @param size The new max size of the expansion queue
	 */
	static void setExpansionQueueMaxSize(unsigned long int size);

	/**
	 * Change the cost cutoff value
	 * @param cost The new cost cutoff value
	 */
	static void setCostCutoff(double cost);

	/**
	 * Enable/disable the garbage collection to eliminate useless nodes
	 * @param enable The new value of the flag
	 */
	static void enableGarbageCollection(bool enable);

	/**
	 * Enable/disable the use of goal distance heuristic estimation in the planners
	 * @param enable The new value of the flag
	 */
	static void enableUseOfHeuristicEstimation(bool enable);

	/**
	 * Change the heuristic value multiplier
	 * @param multiplier The new heuristic value multiplier
	 */
	static void setHeuristicValueMultiplier(double multiplier);

	/**
	 * Change the Log level
	 * @param level New log level
	 */
	static void setLogLevel(int level);

	/**
	 * Change the WORLD_SPACE_GRANULARITY
	 * @param granularity The new WORLD_SPACE_GRANULARITY
	 */
	static void setWorldSpaceGranularity(double granularity);

	/**
	 * Change the CALCULATION_APPROXIMATION_ERROR parameter
	 * @param approx The new CALCULATION_APPROXIMATION_ERROR
	 */
	static void setApproximationError(double approx);

	/**
	 * Change the OCCUPANCY_THRESHOLD parameter
	 * @param threshold The new OCCUPANCY_THRESHOLD
	 */
	static void setOccupancyThreshold(double threshold);

	/**
	 * Change the file used for logging
	 * @param filename The new LOG filename
	 */
	static void setLogFile(std::string filename);

	/**
	 * Enable image saving. *IF* supported by the visualizer, the final visualized
	 * image is saved when the destructor is invoked.
	 * @param enable Enable/disable image saving
	 */
	static void setSaveFinalVisualizationToFile(bool enable);
};

#endif /* WORLDPARAMETERS_H_ */
