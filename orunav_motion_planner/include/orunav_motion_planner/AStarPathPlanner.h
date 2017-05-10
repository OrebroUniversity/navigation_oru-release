/**
 * @file AStarPathPlanner.h
 * @brief Contains the AStarPathPlanner class
 * @author Marcello Cirillo
 *
 *  Created on: Feb 18, 2011
 *      Author: marcello
 */

#ifndef ASTARPATHPLANNER_H_
#define ASTARPATHPLANNER_H_

#include <unordered_map>

#include "AStar.h"
#include "PathNode.h"


/**
 * @class AStarPathPlanner
 * AStarPathPlanner based on A* search.
 * The search is performed on PathNode instances
 * As this class is based on the AStar class, the solution found is optimal (lowest cost path)
 * under the condition of using an admissible heuristic.
 * Consistent heuristics are not required (no closed set).
 *
 * This planner can also be used to populate the heuristic table. In such case, a
 * consistent (null) heuristic can be used, reducing the search to a
 * Dijkstra's algorithm : configurations in expanded nodes would therefore be
 * at the minimal distance from the starting configuration @see WP::USE_HEURISTIC_ESTIMATION
 *
 */
class AStarPathPlanner: public AStar {

	/**
	 * Pointer to a representation of the World, where the
	 * environment information and the goal Position are stored
	 */
	World* w_;

	/**
	 * A hash table which contains a single entry for each Configuration vector as key and a pointer
	 * to the minimum cost PathNode which contains that vector.
	 */
	std::unordered_map<std::vector<Configuration*>, PathNode*, ConfigurationVectorHash, ConfigurationVectorEqual> uniqueNodes_;

	/**
	 * Flag to enable / disable the collection of information for generating
	 * the heuristic table of the vehicle the planner is working for.
	 */
	bool heuristicTableDataCollectionEnabled_;

	/**
	 * Returns the path found cloning the Configurations in each PathNode
	 * @param end_node A pointer to the final node of the path
	 * @returns The cloned path
	 */
	std::vector<Node*> clonePath(PathNode* end_node);

	/**
	 * Extract the data from the Node to save in the heuristic table of the
	 * vehicle model we are planning for
	 * @param node The node to be processed
	 */
	void extractHTDataFromNode(PathNode* node);

public:

	/**
	 * Path planner constructor.
	 * @param startNode A pointer to the PathNode from which the search
	 * is initiated
	 * @param world A pointer to the World representation
	 */
	AStarPathPlanner(PathNode* startNode, World* world);
	~AStarPathPlanner();

	/**
	 * Enable data collection for the Heuristic Table of the vehicle the
	 * planner is working for
	 */
	void enableHTDataCollection();

	/**
	 * Solve the planning problem.
	 */
	std::vector<Node*> solve();

};

#endif /* ASTARPATHPLANNER_H_ */
