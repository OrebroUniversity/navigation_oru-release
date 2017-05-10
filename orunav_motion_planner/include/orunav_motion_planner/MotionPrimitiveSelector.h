/**
 * @file MotionPrimitiveSelector.h
 * @author Marcello Cirillo
 *
 *  Created on: May 24, 2013
 *      Author: marcello
 */

#ifndef MOTIONPRIMITIVESELECTOR_H_
#define MOTIONPRIMITIVESELECTOR_H_

// for the SuccessorGeneratorNode sorting
#include <algorithm>
#include <unordered_map>

#include "MotionPrimitiveData.h"
#include "Utils.h"

class World;


/**
 * @struct This node is the basic element of the procedure used to select
 * the applicable motion primitives. It contains a vector of cellPosition,
 * a vector of pointers to MotionPrimitiveData and a vector of pointers to
 * other MPSelectorNode.
 * The cells in the node are swept only by the primitives in this node.
 * The primitives in the node sweep the cells in the node, but may sweep
 * other cells too, in other nodes
 */
struct MPSelectorNode {
	/** List of the cells swept only by the primitives in this node */
	std::vector<cellPosition> cells_;
	/** Primitives which sweeps the cells in this node (but may sweep other cells as well, in other nodes */
	std::vector<MotionPrimitiveData*> primitives_;
	/** Pointers to subsuming nodes */
	std::vector<MPSelectorNode*> subsumingNodes_;
	/** Flag: marked if all the primitives in this node are not executable */
	bool marked_;

	/** ID, for debugging purposes */
	unsigned short int ID_;

	/**
	 * Constructor with only one cellPosition and no MotionPrimitiveData
	 * @param loc The only cell initialized, (0,0) by default
	 */
	MPSelectorNode(cellPosition loc = cellPosition(0,0))	{
		ID_ = 0;
		cells_.clear();
		primitives_.clear();
		subsumingNodes_.clear();
		marked_ = false;
		cells_.push_back(loc);
	}

	/**
	 * Constructor which initializes the MPSelectorNode with only one MotionPrimitiveData
	 * and no cellPosition
	 * @param prim Pointer to the MotionPrimitiveData to initialize this node
	 */
	MPSelectorNode(MotionPrimitiveData* prim) {
		ID_ = 0;
		cells_.clear();
		primitives_.clear();
		subsumingNodes_.clear();
		marked_ = false;
		primitives_.push_back(prim);
	}

	/**
	 * @struct to compare nodes for sorting purposes
	 */
	struct compareNodes{
		bool operator()(const MPSelectorNode & n1, const MPSelectorNode & n2) const {
			// The node with the larger number of primitives go first
			if(n1.primitives_.size() > n2.primitives_.size())	return true;
			if(n1.primitives_.size() < n2.primitives_.size())	return false;

			// They have an equal number of primitives
			// If they have one primitive each, sort wrt the primitive id, in increasing order
			if(n1.primitives_.size() == 1)	return n1.primitives_[0]->getID() < n2.primitives_[0]->getID();
			// If they have more than one primitive, sort wrt the number of cells, in increasing order
			if(n1.primitives_.size() > 1)	return n1.cells_.size() < n2.cells_.size();
			return false;
		}
	};

	void print(std::ostream & out, bool displayCells = true, bool displayPrimitives = true, bool displaySubsumingNodes = true){
		if(displayCells) {
			out << "cells: { ";
			for (unsigned int i = 0; i < cells_.size(); i++) {
				out << "(" << cells_[i].x_cell << "," << cells_[i].y_cell << ") ";
			}
			out << "}";
			out << "\t";
		}
		if(displayPrimitives) {
			out << "primitives: { ";
			for (unsigned int i = 0; i < primitives_.size(); i++) {
				out << primitives_[i]->getID() << " ";
			}
			out << "}";
			out << "\t";
		}
		if(displaySubsumingNodes) {
			out << "supersets: { ";
			for (unsigned int i = 0; i < subsumingNodes_.size(); i++) {
				out << subsumingNodes_[i]->ID_ << " ";
			}
			out << "}";
		}
		out << std::endl;
	}
};

/**
 * @class MotionPrimitiveSelector
 * All the functions necessary to generate and manage a vector of MPSelectorNode for the selection
 * of applicable MotionPrimitives.
 * WARNING: a MotionPrimitiveSelector object can only be created using a vector of MotionPrimitiveData
 * which can be applied from the same POSE. The selector efficiently returns those primitives which are
 * applicable given the size of the world and the obstacles in the world
 */
class MotionPrimitiveSelector {

private:

	std::vector<MPSelectorNode> selectorNodes_;

	/**
	 * Verifies if the first vector of pointers to MotionPrimitiveData passed as argument
	 * contains the second. The function assumes that the MotionPrimitiveData pointers
	 * are ordered with the same criterion with respect to the MotionPrimitiveData ID.
	 * @param prim1 The container vector
	 * @param prim2 The contained vector
	 * @return True if prim1 contains prim2
	 */
	static bool contains(std::vector<MotionPrimitiveData*> & prim1, std::vector<MotionPrimitiveData*> & prim2);

	/**
	 * Given a vector of pointers to MotionPrimitiveData, it generates an initial list of nodes and saves it in selectorNodes_
	 * Each node either:
	 * -- contains only 1 cell and all the motion primitives which sweep it
	 * -- contains only a single motion primitive and no cell
	 * @param primitives A vector of pointers to MotionPrimitiveData
	 */
	void generateInitialNodes(std::vector<MotionPrimitiveData*> & primitives);

	/**
	 * The vector of nodes in selectorNodes_ must contain only nodes that either contain
	 * a single cell and a list of MotionPrimitiveData OR a single MotionPrimitiveData and no cell.
	 * Each motion pritimitive list is sorted, with respect to the order in which the function
	 * generateInitialNodes processed the primitives
	 */
	void mergeNodes();

	/**
	 * Populate the vector of subsuming nodes of each node in selectorNodes_ in a minimal way
	 * (e.g., do not add redundant pointers)
	 */
	void identifyMinimumSupersets();


public:
	MotionPrimitiveSelector(std::vector<MotionPrimitiveData*> primitives);

	~MotionPrimitiveSelector();

	/**
	 * Returns a vector of pointers to MotionPrimitiveData. All the primitives returned are
	 * directly applicable as they are guaranteed to be collision free
	 * @param w A pointer to the World
	 * @param startXcell The starting cell for the new primitives on the x axis
	 * @param startYcell The starting cell for the new primitives on the y axis
	 * @return The vector of pointers to applicable MotionPrimitiveData
	 */
	std::vector<MotionPrimitiveData*> getValidPrimitives(World* w, short int startXcell, short int startYcell);
};

#endif /* MOTIONPRIMITIVESELECTOR_H_ */
