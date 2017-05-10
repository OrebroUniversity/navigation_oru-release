/**
 * @file PathNode.h
 * @brief contains the class PathNode
 * @author Marcello Cirillo
 *
 *  Created on: Feb 18, 2011
 *      Author: marcello
 */

#ifndef PATHNODE_H_
#define PATHNODE_H_

#include "Node.h"
#include "Configuration.h"
#include "World.h"


/**
 * @class PathNode
 * Class that represents the basic exploration unit of a motion planning algorithm.
 */
class PathNode: public Node {

	/** Vector of pointers to vehicle Configurations representing the state of the system.
	 * The Configurations are ordered in ascending VehicleID */
	std::vector<Configuration*> currentConfigurations_;

	/** Pointer to a World representation */
	World* myWorld_;

	/**
	 * Ordering function to ensure that in every new PathNode the Configuration vector
	 * is ordered in the same way. The ordering is based on the vehicleID to which
	 * the Configuration belongs.
	 * @param c1 Pointer to the first Configuration
	 * @param c2 Pointer to the second Configuration
	 * @return true if the ID of the vehicle of the first Configuration is < the one of the second
	 */
	static bool configurationOrderingFunction (Configuration* c1, Configuration* c2) {
		return c1->getMission()->getVehicleID() < c2->getMission()->getVehicleID();
	}


public:

	/**
	 * Constructor of a new PathNode
	 * @param configurations Vector of pointers to the configurations of each vehicle in the world
	 * @param w Pointer to the World representation
	 * @param parent Pointer to the parent PathNode
	 */
	PathNode(std::vector<Configuration*> configurations, World* w, PathNode* parent);

	/**
	 * Constructor of a new PathNode in which only some of the configurations are changed wrt its
	 * parent node. In this case, only the new Configurations have to be used for the calculation
	 * of the COST.
	 * @param newConfs The new Configurations to add to the PathNode
	 * @param unchangedConfs The unchanged Configurations to add to the Path Node
	 * @param w Pointer to the World representation
	 * @param parent Pointer to the parent PathNode
	 */
	PathNode(std::vector<Configuration*> newConfs, std::vector<Configuration*> unchangedConfs, World* w, PathNode* parent);

	~PathNode();

	/**
	 * Generate the children reachable from the current PathNode
	 * @returns a vector of pointers to new Nodes
	 * @todo Still to include the obstacle handling
	 */
	std::vector<Node*> generateChildren();

	/**
	 * Get a pointer to the Configurations associated to this PathNode
	 * @returns Vector to Configuration pointers
	 */
	std::vector<Configuration*> getConfigurations();

	/**
	 * Check if this node and the one passed as parameter have equivalent Configurations
	 * @param pn Pointer to the Node to compare
	 */
	bool equalContent(Node* pn);

	/**
	 * Clone the Node and return a pointer to a new Node object
	 * The clone function does not preserve the pointer to the parent
	 * Overload of base class method
	 * @returns A pointer to a new Node
	 */
	Node* clone();

	/**
	 * Print information about the node and the Configuration
	 * it points to (on log)
	 */
	void print();
};

#endif /* PATHNODE_H_ */
