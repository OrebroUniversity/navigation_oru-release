/**
 * @file MotionPrimitiveSelector.cpp
 * @author Marcello Cirillo
 *
 *  Created on: May 24, 2013
 *      Author: marcello
 */

#include "orunav_motion_planner/MotionPrimitiveSelector.h"
#include "orunav_motion_planner/World.h"

MotionPrimitiveSelector::MotionPrimitiveSelector(std::vector<MotionPrimitiveData*> primitives) {
	selectorNodes_.clear();
	this->generateInitialNodes(primitives);

//	for(unsigned int i = 0; i < selectorNodes_.size(); i++ ) {
//		selectorNodes_[i].print(std::cout, true, true, true);
//	}


	this->mergeNodes();


//	for(unsigned int i = 0; i < selectorNodes_.size(); i++ ) {
//		selectorNodes_[i].print(std::cout, true, true, true);
//	}

	// sort the nodes
	std::sort(selectorNodes_.begin(), selectorNodes_.end(), MPSelectorNode::compareNodes());


	for(unsigned int i = 0; i < selectorNodes_.size(); i++ ) {
		// DO NOT DELETE THIS
		selectorNodes_[i].ID_ = i;
//		selectorNodes_[i].print(std::cout, true, true, true);
	}

	this->identifyMinimumSupersets();

//	std::cout << "Number of nodes: " << selectorNodes_.size() << std::endl;
//	long int edges = 0;
//	for (int i = 0; i < selectorNodes_.size(); i ++) {
//		edges += selectorNodes_[i].subsumingNodes_.size();
//	}
//	std::cout << "Number of edges: " << edges << std::endl;

//	unsigned int sum1 = 0;
//	unsigned int sum2 = 0;
//	unsigned int sum3 = 0;
//	for(unsigned int i = 0; i < selectorNodes_.size(); i++ ) {
//		sum1 += selectorNodes_[i].cells_.size() * selectorNodes_[i].primitives_.size();
//		sum2 += selectorNodes_[i].cells_.size();
//		sum3 += selectorNodes_[i].subsumingNodes_.size();
//	}
//
//	std::cout << "Old way: " << sum1 << std::endl;
//	std::cout << "New way: " << sum2 + sum3 << std::endl;
//	std::cout << "Cells: " << sum2 << std::endl;
//	std::cout << "Edges: " << sum3 << std::endl;
//	std::cout << "Nodes: " << selectorNodes_.size() << std::endl;
//	std::cout << "Primitives: " << primitives.size() << std::endl;


//	for(unsigned int i = 0; i < selectorNodes_.size(); i++ ) {
//		selectorNodes_[i].print(std::cout, true, true, true);
//	}
}

MotionPrimitiveSelector::~MotionPrimitiveSelector() {
	selectorNodes_.clear();
}

bool MotionPrimitiveSelector::contains(std::vector<MotionPrimitiveData*> & prim1, std::vector<MotionPrimitiveData*> & prim2){
	unsigned int i = 0;
	unsigned int j = 0;
	while(i < prim1.size())	{
		//if (prim1[i]->getID() > prim2[j]->getID())	return false;
		if (prim1[i]->getID() == prim2[j]->getID())		j++;
		if (j == prim2.size())							return true;
		i++;
	}
	return false;
}

void MotionPrimitiveSelector::generateInitialNodes(std::vector<MotionPrimitiveData*> & primitives){

	// Generate nodes that contain a single cell and all the motion primitives that sweep it
	// Generate nodes that contain a single motion primitive and no cells
	selectorNodes_.clear();

	// keep track of all the unique cellPosition which appear in any of the primitives
	std::unordered_map<cellPosition, int, cellPosition::cellPositionHash> cells;

//	std::cout << "Primitives: " << primitives.size() << std::endl;
//	long int totalCellsSwept = 0;
//	long int uniqueCellsSwept = 0;

	// Iterate over the motion primitives
	for (unsigned int i = 0; i < primitives.size(); i++){
		// For each motion primitive, generate a 0-cell 1-primitive node
		selectorNodes_.push_back(MPSelectorNode(primitives[i]));

		std::vector<cellPosition*> cellsSwept = primitives[i]->getSweptCells();

//		/** @test */
//		totalCellsSwept += cellsSwept.size();


		// Iterate over the cells swept by the motion primitive
		for (unsigned int j = 0; j < cellsSwept.size(); j++){

			// If it is the first time we see this cell, create a new node
			// for it and use the hash table to point to its location in the nodes vector
			if(cells.find(*cellsSwept[j]) == cells.end()){
				cells[*cellsSwept[j]] = selectorNodes_.size();
				selectorNodes_.push_back(MPSelectorNode(*cellsSwept[j]));
				// uniqueCellsSwept ++;
			}

			// Add the primitive to the set of primitives that sweep the cell
			selectorNodes_[cells[*cellsSwept[j]]].primitives_.push_back(primitives[i]);
		}

	}

//	/** @test */
//	std::cout << "total cells swept : " << totalCellsSwept << std::endl;
//	std::cout << "unique cells swept : " << uniqueCellsSwept << std::endl;
}

void MotionPrimitiveSelector::mergeNodes(){

	// additional struct to hash a set of primitives -- the equality operator is left to the single MotionPrimitiveData
	struct primitiveSetHash{
		int operator()(const std::vector<MotionPrimitiveData*> & v) const{
			int key = v.size();
			for (unsigned int i = 0; i < 3 && i < v.size(); i++)	{
				key = (key << 4) & v[i]->getID();
			}
			return key;
		}
	};

	// check if a primitive set is unique
	std::unordered_map<std::vector<MotionPrimitiveData*>, int, primitiveSetHash> primitiveSets;

	// iterate over the nodes
	for (unsigned int i = 0; i < selectorNodes_.size(); i++) {
		// is it the first time we see this set of primitives?
		if (primitiveSets.find(selectorNodes_[i].primitives_) == primitiveSets.end())
			// add it to the hash
			primitiveSets[selectorNodes_[i].primitives_] = i;
		else{
			// find the preceding node that contains this set of primitives
			int j = primitiveSets[selectorNodes_[i].primitives_];
			// if it is not a dummy node with no cells, merge the two nodes
			// by adding the single cell from the current node to the list of the previous node
			if(selectorNodes_[i].cells_.size() > 0) 	{
				selectorNodes_[j].cells_.push_back(selectorNodes_[i].cells_[0]);
			}
			// replace the i th node with the last node on the vector
			selectorNodes_[i] = selectorNodes_.back();
			// pop the last element
			selectorNodes_.pop_back();
			// we want to process the new node at location i
			i--;
			// Note that if nodes[i] is the last node, it replaces itself and is then popped.
			// At the next iteration, 'i' will be out of bounds, and the algorithm terminates
		}
	}
}

void MotionPrimitiveSelector::identifyMinimumSupersets(){
	// go over all the nodes, except for the first one
	for (unsigned int i = 1; i < selectorNodes_.size(); i++) {
		// clear the marks of all the previous nodes
		for (int j = i-1; j >= 0; j--)
			selectorNodes_[j].marked_ = false;
		// go over all the previous nodes, in reverse order
		for (int j = i-1; j >= 0; j--) {
			// if a node has been marked, propagate its mark to its subsuming nodes
			if (selectorNodes_[j].marked_ == true) {
				for (unsigned int k = 0; k < selectorNodes_[j].subsumingNodes_.size(); k++) {
					selectorNodes_[j].subsumingNodes_[k]->marked_ = true;
				}
			}
			// if the node is not marked, and it is a superset
			else if (contains(selectorNodes_[j].primitives_, selectorNodes_[i].primitives_)){
				// add an edge from i to j
				selectorNodes_[i].subsumingNodes_.push_back(&selectorNodes_[j]);
				// mark all the subsuming nodes of j (which, by transitivity, would be supersets of i)
				for (unsigned int k = 0; k < selectorNodes_[j].subsumingNodes_.size(); k++) {
					selectorNodes_[j].subsumingNodes_[k]->marked_ = true;
				}
			}
		}
	}
}


std::vector<MotionPrimitiveData*> MotionPrimitiveSelector::getValidPrimitives(World* w,
		short int startXcell, short int startYcell) {
	std::vector<MotionPrimitiveData*> primitives;
	primitives.clear();

	// assume there are no collisions
	for (unsigned int i = 0; i < selectorNodes_.size(); i++) {
		selectorNodes_[i].marked_ = false;

		// Check if any of the subsuming nodes have been marked
		for (unsigned int j = 0; j < selectorNodes_[i].subsumingNodes_.size(); j++) {
			// if a subsuming node is marked
			if (selectorNodes_[i].subsumingNodes_[j]->marked_) {
				// the primitives listed in the current node are invalid: mark the current node as well
				selectorNodes_[i].marked_ = true;
				// at least one subsuming node is marked, no reason to check the others
				break;
			}
		}
		// the node is still unmarked: check for collisions
		if (selectorNodes_[i].marked_ == false) {
			std::vector<cellPosition> cells = selectorNodes_[i].cells_;
			for (unsigned int j = 0; j < cells.size(); j++) {
				// check to see if nodes[i].cells[j] IS BLOCKED
				if (w->getCollisionDetector()->isBlocked(startXcell + cells[j].x_cell, startYcell + cells[j].y_cell)) {
					// the primitives listed in the current node node are invalid, so mark the current node node
					selectorNodes_[i].marked_ = true;
					break;	// At least one cell is blocked, no reason to check the others
				}
			}
		}
		// If the node is not marked and has only one primitive
		if (selectorNodes_[i].marked_ == false && selectorNodes_[i].primitives_.size() == 1) {
			// Add it to the list of applicable primitives
			primitives.push_back(selectorNodes_[i].primitives_[0]);
		}
	}
	return primitives;
}
