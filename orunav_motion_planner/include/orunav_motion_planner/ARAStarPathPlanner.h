/**
 * @file ARAStarPathPlanner.h
 * @brief Contains the ARAStarPathPlanner class
 * @author Marcello Cirillo
 *
 *  Created on: Jan 24, 2012
 *      Author: marcello
 */

#ifndef ARASTARPATHPLANNER_H_
#define ARASTARPATHPLANNER_H_

#include <unordered_map>
#include "AStar.h"
#include "PathNode.h"

/**
 * @class ARAStarPathPlanner
 * ARAStarPathPlanner based on ARA* (Anytime Repairing A*) search.
 * The search is performed on PathNode instances
 */
class ARAStarPathPlanner: public AStar {

	/** The clock time at which the ARAStarPathPlanner is invoked */
	boost::posix_time::ptime startingTime_;

	/** The number of seconds given to the planner to find a solution */
	int secondsToCalculatePath_;


	/** Pointer to a representation of the World, where the
	 * environment information and the goal Position are stored */
	World* w_;

	/**
	 * A hash table which contains a single entry for each Configuration vector as key and a pointer
	 * to the minimum cost PathNode which contains that vector.
	 */
	std::unordered_map<std::vector<Configuration*>, PathNode*, ConfigurationVectorHash, ConfigurationVectorEqual> uniqueNodes_;

	/**
	 * Returns the path found cloning the Configurations in each PathNode
	 * @param end_node A pointer to the final node of the path
	 * @returns The cloned path
	 */
	std::vector<Node*> clonePath(PathNode* end_node);

	/**
	 * Generates an improved path, using a specific heuristic multiplier
	 * @returns The improved solution
	 */
	std::vector<Node*> improvePath();

	/**
	 * The current F value of the goal node, initialized as INFINITY
	 */
	double currentGoalFValue_;

public:

	/**
	 * Path planner constructor.
	 * @param startNode A pointer to the PathNode from which the search
	 * is initiated
	 * @param world A pointer to the World representation
	 * @param startTime The time (in seconds) at which the ARAStarPathPlanner is invoked
	 * @param timeAllowed The number of seconds given to the planner to find a solution
	 */
	ARAStarPathPlanner(PathNode* startNode, World* world, boost::posix_time::ptime startTime, int timeAllowed);
	~ARAStarPathPlanner();

	/**
	 * Solve the planning problem.
	 */
	std::vector<Node*> solve();

};

#endif /* ARASTARPATHPLANNER_H_ */
