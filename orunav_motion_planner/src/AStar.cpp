/**
 * @file AStar.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Jan 24, 2012
 *      Author: marcello
 */

#include "orunav_motion_planner/AStar.h"
#include <iostream>
#include <cstdlib>
#include <iomanip>


AStar::AStar(Node* startNode) {
	this->insertNewNodeForExpansion(startNode);
	goalNode_ = 0;
}

int AStar::openListSize() {
	return openList_.size();
}

std::vector<Node*> AStar::getOpenList() {
	return openList_;
}

int AStar::expandedListSize() {
	return expandedList_.size();
}

AStar::~AStar() {
	// clean up the vectors
	while (!openList_.empty()) {
		delete openList_.back();
		openList_.pop_back();
	}
	while (!expandedList_.empty()) {
		delete expandedList_.back();
		expandedList_.pop_back();
	}
	while (!inconsistentList_.empty()) {
		delete inconsistentList_.back();
		inconsistentList_.pop_back();
	}
	if (goalNode_) {
		delete goalNode_;
	}
}

/** Binary heap implementation for insertion and extraction of Nodes from the expansion Queue */
void AStar::insertNewNodeForExpansion(Node* child) {

	// insert the new item at the end
	openList_.push_back(child);
	int m = openList_.size();
	double child_F = 0;
	double parent_F = 0;
	while (m != 1) { // While item hasn't bubbled to the top (m=1)
		// Check if child is < parent. If so, swap them. In case they are =, use G as tie breaker
		// Remember that the items are shifted one position (expansionQueue[0] == position 1)
		child_F = openList_[m - 1]->getF();
		parent_F = openList_[(m / 2) - 1]->getF();

		// checks between doubles can be prone to approximation errors
		if (child_F < parent_F) {
			Node* temp = openList_[(m / 2) - 1];
			openList_[(m / 2) - 1] = openList_[m - 1];
			openList_[m - 1] = temp;
			m = (m / 2);
		} else {
			break;
		}
	}
}

void AStar::insertNodeIntoInconsistentList(Node* n) {
	inconsistentList_.push_back(n);
}

double AStar::getFValueOfExtractionCandidate() {
	// if there are nodes not to be expanded, remove them!
	while (openList_.size() != 0 && openList_[0]->isNotToBeExpanded()) {
		Node* np = openList_[0];
		openList_[0] = openList_.back();
		openList_.pop_back();
		if (openList_.size() > 1) {
			unsigned int v = 1;
			unsigned int u;
			while (true) {
				u = v;
				if (2 * u + 1 <= openList_.size()) { // if both children exist
					// Select the lowest of the two children
					if (openList_[u - 1]->getF() >= openList_[2 * u - 1]->getF()) {
						v = 2 * u;
					}
					if (openList_[v - 1]->getF() >= openList_[2 * u]->getF()) {
						v = 2 * u + 1;
					}
				} else if (2 * u <= openList_.size()) { // only 1 child
					// check if the parent's F is greater than the child's
					if (openList_[u - 1]->getF() >= openList_[2 * u - 1]->getF()) {
						v = 2 * u;
					}
				}
				if (u != v) {
					// if parent's F > one or both of its children, swap them
					Node* temp = openList_[u - 1];
					openList_[u - 1] = openList_[v - 1];
					openList_[v - 1] = temp;
				} else {
					break;
				}
			}
		}
		// this node is to be thrown away
		delete np;
	}

	if (openList_.size() == 0) {
		// the expansionQueue is empty
		return -1;
	}
	return openList_[0]->getF();
}

Node* AStar::extractNode() {
	bool nodeExtracted = false;
	Node* np = 0;

	while (!nodeExtracted) {
		// is the expansionQueue empty?
		if (openList_.size() == 0) {
			return 0;
		}

		// extract the first element and replace it with the last
		np = openList_[0];
		openList_[0] = openList_.back();
		openList_.pop_back();

		if (openList_.size() > 1) {
			unsigned int v = 1;
			unsigned int u;
			while (true) {
				u = v;
				if (2 * u + 1 <= openList_.size()) { // if both children exist
					// Select the lowest of the two children
					if (openList_[u - 1]->getF() >= openList_[2 * u - 1]->getF()) {
						v = 2 * u;
					}
					if (openList_[v - 1]->getF() >= openList_[2 * u]->getF()) {
						v = 2 * u + 1;
					}
				} else if (2 * u <= openList_.size()) { // only 1 child
					// check if the parent's F is greater than the child's
					if (openList_[u - 1]->getF() >= openList_[2 * u - 1]->getF()) {
						v = 2 * u;
					}
				}
				if (u != v) {
					// if parent's F > one or both of its children, swap them
					Node* temp = openList_[u - 1];
					openList_[u - 1] = openList_[v - 1];
					openList_[v - 1] = temp;
				} else {
					break;
				}
			}
		}
		// we have to check if we have already expanded a node with the same content and minor cost
		if (!np->isNotToBeExpanded()) {
			nodeExtracted = true;
			expandedList_.push_back(np);
			np->setAsAlreadyExpanded();
		} else {
			delete np;
		}
	}
	return np;
}

// ARA* ONLY
void AStar::prepareListsForNextIteration(double oldHeuristicValueMultiplier, double newHeuristicValueMultiplier) {
	double heuristicCoefficient = newHeuristicValueMultiplier / oldHeuristicValueMultiplier;

	// move all elements from openList to the inconsistentList
	for (std::vector<Node*>::iterator it = openList_.begin(); it != openList_.end(); it++) {
			insertNodeIntoInconsistentList((*it));
	}
	// clear the openList_ and re-populate it, removing the nodes whose cost has already proven them
	// worthless for further expansion
	openList_.clear();
	for (std::vector<Node*>::iterator it = inconsistentList_.begin(); it != inconsistentList_.end(); it++) {
		if (!(*it)->isNotToBeExpanded()) {
			(*it)->updateHFValues(heuristicCoefficient);
			insertNewNodeForExpansion((*it));
		} else {
			delete *it;
		}
	}

	inconsistentList_.clear();
	// Remove the closed flag from the nodes expanded at the previous iteration.
	for (std::vector<Node*>::iterator it = expandedList_.begin(); it != expandedList_.end(); it++) {
		(*it)->setAsNotExpanded();
	}
	//expandedList_.clear();
}

void AStar::purgeUnexpandableNodes() {
	unsigned int purgednodes = 0;
	std::vector<Node*> expandableNodes;
	while (!openList_.empty()) {
		Node* n = openList_.back();
		if (n->isNotToBeExpanded()) {
			purgednodes++;
			delete n;
		} else {
			expandableNodes.push_back(n);
		}
		openList_.pop_back();
	}
	for (std::vector<Node*>::iterator it = expandableNodes.begin(); it != expandableNodes.end(); it++) {
		insertNewNodeForExpansion((*it));
	}
}
