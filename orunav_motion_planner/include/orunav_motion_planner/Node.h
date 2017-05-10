/**
 * @file Node.h
 * @brief definition of a virtual Node
 * @author Marcello Cirillo
 *
 *  Created on: Feb 18, 2011
 *      Author: marcello
 */

#ifndef NODE_H_
#define NODE_H_

#include <vector>
#include <string>
#include <inttypes.h>

/**
 * @class Node
 * Virtual class that defines the basic exploration unit of a search algorithm.
 */
class Node {

protected:

	/** Node counter */
	static long unsigned int nodeCounter_;
//	static long unsigned int successorExpansions_;
	/** Node ID */
	long unsigned int nodeID_;


	/** Actual cost to reach the Node from starting point (G) */
	double G_;

	/** Heuristic estimation of the cost to reach the goal from the Node (H) */
	double H_;

	/** Total estimated cost from start to goal through Node (F: F = G + H) */
	double F_;

	/** Pointer to the parent Node */
	Node* parentNode_;

	/** This flag is true if the node's content should not be expanded (e.g.: in A* we found a path to
	 * its content whose cost is more advantageous. */
	bool notToBeExpanded_;

	/** This flag is true if the node has been already been expanded */
	bool alreadyExpanded_;

public:

	Node() {
		nodeID_ = nodeCounter_;
		nodeCounter_++;
		H_ = 0;
		G_ = 0;
		F_ = 0;
		parentNode_ = 0;
		notToBeExpanded_ = false;
		alreadyExpanded_ = false;
	}

	unsigned long int getNodeID() {
		return nodeID_;
	}

	virtual ~Node() {
	}

	/**
	 * Returns true if the node's content has already been expanded
	 * (that is, if the alreadyEpanded flag is set to true)
	 * @returns True if the content of this node has already been expanded
	 */
	bool isNotToBeExpanded() {
		return notToBeExpanded_;
	}

	/**
	 * Set the node as not to be expanded
	 */
	void setNotToBeExpanded() {
		notToBeExpanded_ = true;
	}

	/**
	 * Returns true if this Node instance has been already expanded
	 * @returns true if the node has been already expanded
	 */
	bool hasBeenExpanded() {
		return alreadyExpanded_;
	}

	/**
	 * Set the node as already expanded
	 */
	void setAsAlreadyExpanded() {
		alreadyExpanded_ = true;
	}

	/**
	 * Set the node as not already expanded
	 */
	void setAsNotExpanded() {
		alreadyExpanded_ = false;
	}

	/**
	 * Pure virtual function. Generate the children reachable from the Node
	 * @returns std::vector<Node*> A vector of pointers to Nodes
	 */
	virtual std::vector<Node*> generateChildren() = 0;

	/**
	 * Pure virtual function to establish the equivalence between the contents of two nodes
	 * @param pn A pointer to the Node to compare
	 */
	virtual bool equalContent(Node* pn) = 0;

	/**
	 * Pure virtual function. Print info about the node on log
	 */
	virtual void print() = 0;

	/**
	 * Get the actual cost to reach the Node from starting point
	 * @returns The cost
	 */
	double getG() {
		return G_;
	}

	/**
	 * Get the total estimated cost from start to goal through Node
	 * @returns Total estimated cost
	 */
	double getF() {
		return F_;
	}

	/**
	 * Get the heuristic estimation of the cost to reach the goal
	 * @returns The H value of the Node
	 */
	double getH() {
		return H_;
	}

	/**
	 * Get the pointer to the parent Node
	 * @returns A pointer to the parent Node
	 */
	Node* getParent() {
		return parentNode_;
	}

	/**
	 * Clone the Node and return a pointer to a new Node object
	 * The clone function does not preserve the pointer to the parent
	 * Purely virtual function
	 * @returns A pointer to a new Node
	 */
	virtual Node* clone() = 0;

	/**
	 * Set the pointer to the parent Node
	 * @param parent The pointer to the parent Node
	 */
	void setParent(Node* parent) {
		this->parentNode_ = parent;
	}

	/**
	 * Update the H value (and consequently the F value) of a node, using a new
	 * discount coefficient
	 * @param coeff The coefficient to which H should be multiplied
	 */
	void updateHFValues(double coeff) {
		H_ = H_ * coeff;
		F_ = G_ + H_;
	}
};

#endif /* NODE_H_ */
