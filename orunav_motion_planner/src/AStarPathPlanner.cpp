/**
 * @file AStarPathPlanner.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Feb 18, 2011
 *      Author: marcello
 */

#include "orunav_motion_planner/AStarPathPlanner.h"
#include <iostream>

AStarPathPlanner::AStarPathPlanner(PathNode* startNode, World* world) : AStar(startNode) {
	w_ = world;
	heuristicTableDataCollectionEnabled_ = false;

	// insert the startNode into the uniqueNodes list
	std::vector<Configuration*> newConfs = startNode->getConfigurations();
	std::vector<Configuration*> clonedConfs;
	clonedConfs.clear();
	for (std::vector<Configuration*>::iterator it = newConfs.begin(); it != newConfs.end(); it ++) {
		clonedConfs.push_back((*it)->clone());
	}
	std::pair<std::vector<Configuration*>, PathNode*> entry (clonedConfs, startNode);
	uniqueNodes_.insert(entry);

	std::vector<Configuration*> goalConfs;
	std::vector<Configuration*> startConfs = startNode->getConfigurations();
	for (std::vector<Configuration*>::iterator it = startConfs.begin(); it != startConfs.end(); it ++ ) {
		goalConfs.push_back((*it)->getMission()->getGoalConfiguration()->clone());
	}
	goalNode_ = new PathNode(goalConfs, w_, 0);
}

AStarPathPlanner::~AStarPathPlanner() {
	for (std::unordered_map<std::vector<Configuration*>, PathNode*, ConfigurationVectorHash, ConfigurationVectorEqual>::iterator it = uniqueNodes_.begin(); it != uniqueNodes_.end(); it ++) {
		std::vector<Configuration*> confs = (*it).first;
		for (std::vector<Configuration*>::iterator it =  confs.begin(); it != confs.end(); it++) {
			delete *it;
		}
		// remove the references to the nodes
		(*it).second = 0;
	}
	uniqueNodes_.clear();
}

void AStarPathPlanner::enableHTDataCollection() {
	heuristicTableDataCollectionEnabled_ = true;
}

void AStarPathPlanner::extractHTDataFromNode(PathNode* node) {

	// first check: only one vehicle is allowed for heuristic extraction
	if (node->getConfigurations().size() > 1) {
		if (WP::LOG_LEVEL >= 1) {
			writeLogLine("Multiple vehicles: heuristic extraction not allowed", "AStarPathPlanner", WP::LOG_FILE);
		}
		exit(0);
	}

	// if this value is already in the heuristic table, then we already have the whole path
	if (!node->getConfigurations().front()->getMission()->getVehicleModel()->isHeuristicValueInTable(
			node->getConfigurations().front()->getMission()->getStartConfiguration()->getXCell(),
			node->getConfigurations().front()->getMission()->getStartConfiguration()->getYCell(),
			node->getConfigurations().front()->getMission()->getStartConfiguration()->getOrientationID(),
			node->getConfigurations().front()->getMission()->getStartConfiguration()->getSteeringID(),
			node->getConfigurations().front()->getXCell(),
			node->getConfigurations().front()->getYCell(),
			node->getConfigurations().front()->getOrientationID(),
			node->getConfigurations().front()->getSteeringID())) {
		double cost = 0;
		// cost from this configuration to itself
		node->getConfigurations().front()->getMission()->getVehicleModel()->setEntryInHeuristicTable(
				node->getConfigurations().front()->getXCell(),
				node->getConfigurations().front()->getYCell(),
				node->getConfigurations().front()->getOrientationID(),
				node->getConfigurations().front()->getSteeringID(),
				node->getConfigurations().front()->getXCell(),
				node->getConfigurations().front()->getYCell(),
				node->getConfigurations().front()->getOrientationID(),
				node->getConfigurations().front()->getSteeringID(),
				cost);
		PathNode* tmp = node;
		while (tmp->getParent()) {
			cost += tmp->getConfigurations().front()->getCostToThisConfiguration();
			tmp = dynamic_cast<PathNode*> (tmp->getParent());
			node->getConfigurations().front()->getMission()->getVehicleModel()->setEntryInHeuristicTable(
					tmp->getConfigurations().front()->getXCell(),
					tmp->getConfigurations().front()->getYCell(),
					tmp->getConfigurations().front()->getOrientationID(),
					tmp->getConfigurations().front()->getSteeringID(),
					node->getConfigurations().front()->getXCell(),
					node->getConfigurations().front()->getYCell(),
					node->getConfigurations().front()->getOrientationID(),
					node->getConfigurations().front()->getSteeringID(),
					cost);
		}
	}
}

std::vector<Node*> AStarPathPlanner::solve() {

	bool solutionFound = false;
	std::vector<Node*> sol;

	// DEBUG
	unsigned long int progress = 10000;

	/** @todo ->>> remove!!!! */
	//boost::posix_time::ptime startTime(boost::posix_time::microsec_clock::local_time());
	//boost::posix_time::time_duration timeElapsed;

	while (solutionFound == false) {

		// garbage collection
		if (WP::ALLOW_GARBAGE_COLLECTION && this->expandedListSize() % 10000 == 0) {
			unsigned long int before = this->openListSize();
			this->purgeUnexpandableNodes();
			if (WP::LOG_LEVEL >= 2) {
				char line[150];
				sprintf(line, "Garbage collection of unexpandable nodes [%lu of %lu nodes cleared]", before - openListSize(), before);
				writeLogLine(line, "AStarPathPlanner", WP::LOG_FILE);
			}
		}

		if (this->openListSize() >= (int) WP::EXPANSION_QUEUE_MAX) {
			if (WP::LOG_LEVEL >= 1) {
				writeLogLine("Expansion queue space exhausted", "AStarPathPlanner", WP::LOG_FILE);
				sol.clear();
				return sol;
			}
		}

		/** @todo ->>> remove!!!! */
//		timeElapsed = boost::posix_time::microsec_clock::local_time() - startTime;
//		if (timeElapsed.total_seconds() > 180) {
//			if (WP::LOG_LEVEL >= 1) {
//				writeLogLine("No more time - No solution found", "AStarPathPlanner", WP::LOG_FILE);
//				sol.clear();
//				return sol;
//			}
//		}

		// get a new candidate for expansion
		PathNode* candidate = dynamic_cast<PathNode*> (this->extractNode());

		// no mode nodes to expand: return an empty path
		if (!candidate) {
			if (WP::LOG_LEVEL >= 1) {
				writeLogLine("No more nodes to expand", "AStarPathPlanner", WP::LOG_FILE);
			}
			sol.clear();
			return sol;
		}

		// check if the cost of this node is the limit set for expansion
		if(candidate->getG() >= WP::COST_CUTOFF) {
			if (WP::LOG_LEVEL >= 1) {
				writeLogLine("Cost cutoff reached. No more expansions", "AStarPathPlanner", WP::LOG_FILE);
			}
			sol.clear();
			return sol;
		}

		// extract HT data from expanded nodes only when using a consistent heuristic (or the same admissible heuristic)
		if (heuristicTableDataCollectionEnabled_ && !WP::USE_HEURISTIC_ESTIMATION) {
			extractHTDataFromNode(candidate);
		}

		if (WP::LOG_LEVEL >= 1) {
			w_->visualizeConfigurations(candidate->getConfigurations());
		}

		if (WP::LOG_LEVEL >= 5) {
			candidate->print();
		}

		// check if we have reached the goal
		if (candidate->equalContent(goalNode_)) {
			if (heuristicTableDataCollectionEnabled_) {
				extractHTDataFromNode(candidate);
			}
			solutionFound = true;
			sol = this->clonePath(candidate);
			if (WP::LOG_LEVEL >= 2) {
				char line[150];
				sprintf(line, "Open list: %d \tExpanded list: %d", openListSize(), expandedListSize());
				writeLogLine(line, "AStarPathPlanner", WP::LOG_FILE);
			}
			// exit the loop
			break;
		}

		// if solution not yet found, generate children
		std::vector<Node*> children = (candidate->generateChildren());

		for (std::vector<Node*>::iterator it = children.begin(); it != children.end(); it++) {
			PathNode* child = dynamic_cast<PathNode*> (*it);

			// check if we already have a PathNode with the same content
			std::vector<Configuration*> childConfs = child->getConfigurations();
			std::unordered_map<std::vector<Configuration*>, PathNode* , ConfigurationVectorHash, ConfigurationVectorEqual>::iterator equalit = uniqueNodes_.find(childConfs);

			// the node is completely new: clone its configurations and save it into the uniqueList
			if (equalit == uniqueNodes_.end()) {
				std::vector<Configuration*> newConfs = child->getConfigurations();
				std::vector<Configuration*> clonedConfs;
				for (std::vector<Configuration*>::iterator it = newConfs.begin(); it != newConfs.end(); it ++) {
					clonedConfs.push_back((*it)->clone());
				}
				std::pair<std::vector<Configuration*>, PathNode*> entry (clonedConfs, child);
				uniqueNodes_.insert(entry);
				this->insertNewNodeForExpansion(child);
				if (WP::LOG_LEVEL >= 3) {
					w_->visualizeConfigurations(child->getConfigurations());
				}

			} else {
				// the node we have already is better
				if ((*equalit).second->getG() < (child->getG() + 10e-8)) {
					delete child;
				} else {
					// the new node is better: the existing one should not be expanded and the entry must be substituted
					(*equalit).second->setNotToBeExpanded();
					(*equalit).second = child;
					this->insertNewNodeForExpansion(child);
					if (WP::LOG_LEVEL >= 3) {
						w_->visualizeConfigurations(child->getConfigurations());
					}
				}
			}
		}
		if (WP::LOG_LEVEL >= 2 && progress == 10000) {
			char line[150];
			sprintf(line, "Open list: %d \tExpanded list: %d", openListSize(), expandedListSize());
			writeLogLine(line, "AStarPathPlanner", WP::LOG_FILE);
			progress = 0;
		}
		progress += 1;
	} // end while
	// solution has been found
	return sol;
}

std::vector<Node*> AStarPathPlanner::clonePath(PathNode* end_node) {
	std::vector<Node*> result;
	while (end_node) {
		result.push_back(end_node->clone());
		end_node = dynamic_cast<PathNode*> (end_node->getParent());
	}
	// restore links to parents
	Node* parent;
	Node* son;
	std::vector<Node*>::iterator it = result.begin();
	son = (*it);
	it++;
	while (it != result.end()) {
		parent = (*it);
		son->setParent(parent);
		son = parent;
		it++;
	}
	std::vector<Node*> reversed_result;
	std::vector<Node*>::reverse_iterator rit;
	for (rit = result.rbegin(); rit != result.rend(); rit++) {
		reversed_result.push_back(*rit);
	}
	return reversed_result;
}

