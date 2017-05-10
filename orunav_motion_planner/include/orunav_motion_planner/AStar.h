/**
 * @file AStar.h
 * @author Marcello Cirillo
 *
 *  Created on: Jan 24, 2012
 *      Author: marcello
 */

#ifndef ASTAR_H_
#define ASTAR_H_

#include <vector>
#include "math.h"

#include "Node.h"

/**
 * @class AStar
 * AStar defines the basic data structures to implement A* or
 * ARA* (Anytime Repairing A*) based search algorithms.
 *
 * The A* algorithm is optimal (that is, it finds the solution with the lowest
 * cost) under the condition of using an admissible heuristic.
 * Consistent (monotonic) heuristics are not required, as
 * this implementation does not use a closed set -- a Node which has already been
 * expanded can be expanded again in case a less costly path is found which leads to it.
 */
class AStar {

private:
	/** Vector of pointers to Node class instances yet to be expanded
	 * expansion queue is accessed and ordered as a binary heap */
	std::vector<Node*> openList_;

	/** Vector of pointers to Node class instances already expanded */
	std::vector<Node*> expandedList_;

	/** Vector of pointers to inconsistent Nodes, only used in ARA* algorithms */
	std::vector<Node*> inconsistentList_;


protected:

	/** A dummy node which contains the termination conditions */
	Node* goalNode_;

	/**
	 * returns the openList size
	 * @returns The number of Nodes in the list
	 */
	int openListSize();

	/**
	 * Returns the vector of pointers to Node in the OpenList
	 * @return The vector of pointers to Node in the OpenList
	 */
	std::vector<Node*> getOpenList();

	/**
	 * returns the expandedList size
	 * @returns The number of Nodes in the list
	 */
	int expandedListSize();

	/**
	 * Insert a new Node into the openList
	 * @param child The pointer to the Node to be inserted
	 */
	void insertNewNodeForExpansion(Node* child);


	/**
	 * Insert a node into the inconsistentList -- ARA* ONLY
	 * @param n The pointer to the Node to be inserted
	 */
	void insertNodeIntoInconsistentList(Node* n);

	/**
	 * Return the min F value of all the nodes in the expansion queue
	 * (e.g., get the F value of the next candidate for extraction)
	 * RETURNS -1 if the openList is empty
	 * To be used with ARA*
	 * @return the F value of the extraction candidate
	 */
	double getFValueOfExtractionCandidate();

	/**
	 * Extract the best Node candidate from the expansionQueue and
	 * insert it into the closedList. Returns 0 in case no nodes are left
	 * @returns Pointer to the Node
	 */
	Node* extractNode();

	/**
	 * ARA* ONLY
	 * Prepare the node lists for a new iteration of the ARA* algorithm:
	 * - delete the expandedList nodes (not necessary for further expansions)
	 * - create a new openList
	 * - move all nodes from old openList and inconsistentList to the new one
	 *   (discounting the H and F values of the nodes), unless marked as notToBeExpanded
	 * - reset all the lists
	 * @param oldHeuristicValueMultiplier The heuristic value multiplier used in the previous iteration
	 * @param newHeuristicValueMultiplier The heuristic value multiplier to be used in the next iteration
	 */
	void prepareListsForNextIteration(double oldHeuristicValueMultiplier, double newHeuristicValueMultiplier);

	/**
	 * Permanently deletes from the openList all the nodes that are marked as not expandable
	 */
	void purgeUnexpandableNodes();

public:
	/**
	 * Creates a new AStar object
	 * @param startNode The starting node for the search
	 */
	AStar(Node* startNode);

	virtual ~AStar();

	/**
	 * Pure virtual function with the search algorithm
	 */
	virtual std::vector<Node*> solve() = 0;
};

#endif /* ASTAR_H_ */
