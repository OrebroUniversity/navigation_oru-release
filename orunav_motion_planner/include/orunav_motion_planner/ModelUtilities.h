/**
 * @file ModelUtilities.h
 * @author Marcello Cirillo
 *
 *  Created on: Oct 8, 2012
 *      Author: marcello
 */

#ifndef MODELUTILITIES_H_
#define MODELUTILITIES_H_

#include <math.h>
#include <iomanip>
#ifdef _WIN32 // windows
	#include <boost/thread/thread.hpp>
#else  // linux
	#include <boost/thread.hpp>
#endif

#include "VehicleModel.h"
#include "WorldParameters.h"
#include "VehicleMission.h"
#include "PathFinder.h"

/**
 * @class ModelUtilities
 * A collection of utility functions for VehicleModel
 */
class ModelUtilities {

private:

	static bool lengthSortingFunction(MotionPrimitiveData* p1, MotionPrimitiveData* p2) {
		return p1->getDistanceCovered() > p2->getDistanceCovered();
	};

	/**
	 * Save the motion primitive set lookup table of the VehicleModel on file
	 * @param m The VehicleModel whose motion primitive lookup table should be saved
	 */
	static void saveReducedPrimitiveLookupTable(VehicleModel* m);

	/**
	 * Fill the motion primitive set lookup table using the 8-axis symmetry -- that is,
	 * delete all the motion primitives which have been deleted in the first sector
	 * @param m The VehicleModel whose table should be changed
	 */
	static void fillPrimitiveLookupTableWith8AxisSymmetry(VehicleModel* m);


public:

	ModelUtilities();
	virtual ~ModelUtilities();

	/**
	 * Generate the heuristic table for a VehicleModel, calculating all the entries up to a defined
	 * cut-off cost (Dijkstra Algorithm) and then filling up the table for poses up to a specific distance.
	 * @param m Pointer to the VehicleModel
	 * @param costCutoff The cost at which to stop
	 * @param fillDistance The distance required to fill up the heuristic table (-1 to avoid filling)
	 */
	static void generateModelHeuristicTable(VehicleModel* m, double costCutoff, double fillDistance);

	/**
	 * Reduce the motion primitive set of a model by trying to decompose each of them in turn in a subset of the
	 * others. The decomposition is performed by removing a motion primitive from the set of the allowed ones and
	 * by performing an optimal A* search to find the best solution leading from the starting point of the
	 * selected motion primitives to its goal pose.
	 * An additional cost is tolerated in the decomposition. Such cost is specified in the additionalCostTolerance.
	 * At the end, the new table is saved on file.
	 * @param m The VehicleModel with the motion primitive set to be reduced
	 * @param additionalCostToleranceMultiplier The additional cost tolerated in the decomposition
	 */
	static void reduceModelMotionPrimitiveSet(VehicleModel* m, double additionalCostToleranceMultiplier);

};

#endif /* MODELUTILITIES_H_ */
