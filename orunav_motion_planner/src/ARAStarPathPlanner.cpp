/**
 * @file ARAStarPathPlanner.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Jan 24, 2012
 *      Author: marcello
 */

#include "orunav_motion_planner/ARAStarPathPlanner.h"
#include <iostream>

ARAStarPathPlanner::ARAStarPathPlanner(PathNode* startNode, World* world, boost::posix_time::ptime startTime, int timeAllowed) :
AStar(startNode) {
	w_ = world;
	currentGoalFValue_ = INFINITY;
	startingTime_ = startTime;
	secondsToCalculatePath_ = timeAllowed;

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

ARAStarPathPlanner::~ARAStarPathPlanner() {
	for (std::unordered_map<std::vector<Configuration*>, PathNode*, ConfigurationVectorHash, ConfigurationVectorEqual>::iterator it = uniqueNodes_.begin();
			it != uniqueNodes_.end(); it ++) {
		std::vector<Configuration*> confs = (*it).first;
		for (std::vector<Configuration*>::iterator it =  confs.begin(); it != confs.end(); it++) {
			delete *it;
		}
		// remove the references to the nodes
		(*it).second = 0;
	}
	uniqueNodes_.clear();
}

std::vector<Node*> ARAStarPathPlanner::solve() {

	std::vector<Node*> sol;
	WP::setHeuristicValueMultiplier(3);
	sol = improvePath();

	if (sol.size() > 0) {
		if (w_->isVisualizationEnabled()) {
			w_->resetVisualization();
			for (std::vector<Node*>::iterator it = sol.begin(); it != sol.end(); it++) {
				w_->visualizeConfigurations(dynamic_cast<PathNode*>(*it)->getConfigurations());
			}
		}
	}

	boost::posix_time::time_duration timeElapsed(boost::posix_time::microsec_clock::local_time() - startingTime_);

	// while we can improve the solution and we still have time, continue
	while (WP::HEURISTIC_VALUE_MULTIPLIER > 1 && (timeElapsed.total_seconds() < secondsToCalculatePath_)) {

		if (WP::LOG_LEVEL >= 2) {
			char line[250];
			sprintf(line, "Solution optimality bound: %2.2f", WP::HEURISTIC_VALUE_MULTIPLIER);
			writeLogLine(line, "ARAStarPathPlanner", WP::LOG_FILE);
		}

		double oldHeuristicValueMultiplier = WP::HEURISTIC_VALUE_MULTIPLIER;
		WP::setHeuristicValueMultiplier(oldHeuristicValueMultiplier - 0.2);
		this->prepareListsForNextIteration(oldHeuristicValueMultiplier, WP::HEURISTIC_VALUE_MULTIPLIER);

		std::vector<Node*> newSol = improvePath();

		if (newSol.size() > 0) {
			for (std::vector<Node*>::iterator it = sol.begin(); it != sol.end(); it++ ){
				delete *it;
			}
			sol.clear();
			sol = newSol;
			if (w_->isVisualizationEnabled()) {
				w_->resetVisualization();
				for (std::vector<Node*>::iterator it = sol.begin(); it != sol.end(); it++) {
					w_->visualizeConfigurations(dynamic_cast<PathNode*>(*it)->getConfigurations());
				}
			}

			if (WP::LOG_LEVEL >= 2 && WP::HEURISTIC_VALUE_MULTIPLIER <= 1.01) {
				writeLogLine("Optimal solution found", "ARAStarPathPlanner", WP::LOG_FILE);
			}
		}
		timeElapsed = boost::posix_time::microsec_clock::local_time() - startingTime_;
	}

	return sol;
}

std::vector<Node*> ARAStarPathPlanner::improvePath() {
	std::vector<Node*> sol;
	sol.clear();
	unsigned int progress = 10000;

	boost::posix_time::time_duration timeElapsed(boost::posix_time::microsec_clock::local_time() - startingTime_);

	PathNode* candidate;
	// we do not extract the candidate until we know that it will be used -- use this instead of the openListSize,
	// because in the openList there might be only a node marked as not to be extracted
	while (this->getFValueOfExtractionCandidate() >= 0) {

		timeElapsed = boost::posix_time::microsec_clock::local_time() - startingTime_;

		// check if we cannot improve over the previous solution or if we have exhausted the time limit
		if (this->getFValueOfExtractionCandidate() >= currentGoalFValue_ ||
				(timeElapsed.total_seconds() >= secondsToCalculatePath_)) {
			break;
		}
		// now we can safely extract the candidate
		candidate = dynamic_cast<PathNode*> (this->extractNode());

		// garbage collection
		if (WP::ALLOW_GARBAGE_COLLECTION && this->expandedListSize() % 10000 == 0) {
			unsigned long int before = this->openListSize();
			this->purgeUnexpandableNodes();
			if (WP::LOG_LEVEL >= 2) {
				char line[150];
				sprintf(line, "Garbage collection of unexpandable nodes [%lu of %lu nodes cleared]", before - openListSize(), before);
				writeLogLine(line, "ARAStarPathPlanner", WP::LOG_FILE);
			}
		}

		if (this->openListSize() >= (int) WP::EXPANSION_QUEUE_MAX) {
			if (WP::LOG_LEVEL >= 1) {
				writeLogLine("Open list space exhausted", "ARAStarPathPlanner", WP::LOG_FILE);
				return sol;
			}
		}

		if (WP::LOG_LEVEL >= 1) {
			w_->visualizeConfigurations(candidate->getConfigurations());
		}

		if (WP::LOG_LEVEL >= 5) {
			candidate->print();
		}
		// check if we reached the goal
		if (candidate->equalContent(goalNode_)) {
			currentGoalFValue_ = candidate->getF();
			sol = this->clonePath(candidate);
			if (WP::LOG_LEVEL >= 2) {
				char line[150];
				sprintf(line, "Open list: %d \tExpanded list: %d", openListSize(), expandedListSize());
				writeLogLine(line, "ARAStarPathPlanner", WP::LOG_FILE);
			}
			// exit the loop
			break;
		}

		// if solution not yet found, generate children
		std::vector<Node*> children = candidate->generateChildren();
		std::vector<Node*>::iterator it;
		for (it = children.begin(); it != children.end(); it++) {
			PathNode* child = dynamic_cast<PathNode*> (*it);

			// check if we already have a PathNode with the same content
			std::unordered_map<std::vector<Configuration*>, PathNode* ,ConfigurationVectorHash, ConfigurationVectorEqual>::iterator equalit = uniqueNodes_.find(child->getConfigurations());

			// the node is completely new: clone its configurations and save it into the uniqueList
			if (equalit == uniqueNodes_.end()) {
				std::vector<Configuration*> confs = child->getConfigurations();
				std::vector<Configuration*> clonedConfs;
				for (std::vector<Configuration*>::iterator confit = confs.begin(); confit != confs.end(); confit ++) {
					clonedConfs.push_back((*confit)->clone());
				}
				std::pair<std::vector<Configuration*>, PathNode*> entry (clonedConfs, child);
				uniqueNodes_.insert(entry);
				this->insertNewNodeForExpansion(child);
				if (WP::LOG_LEVEL >= 3) {
					w_->visualizeConfigurations(child->getConfigurations());
				}
			} else {
				// the node we already have is better
				if ((*equalit).second->getG() < (child->getG() + 10e-8)) {
					delete child;
				} else {
					// the node we have expanded is better:
					if ((*equalit).second->hasBeenExpanded()) {
						(*equalit).second = child;
						this->insertNodeIntoInconsistentList(child);
					} else  {
						(*equalit).second->setNotToBeExpanded();
						(*equalit).second = child;
						this->insertNewNodeForExpansion(child);
						if (WP::LOG_LEVEL >= 3) {
							w_->visualizeConfigurations(child->getConfigurations());
						}
					}
				}
			}
		}
		if (WP::LOG_LEVEL >= 2 && progress == 10000) {
			char line[150];
			sprintf(line, "Open list: %d \tExpanded list: %d", openListSize(), expandedListSize());
			writeLogLine(line, "ARAStarPathPlanner", WP::LOG_FILE);
			progress = 0;
		}
		progress += 1;
	} // end while

	// computation terminated
	if (WP::LOG_LEVEL >= 1) {
		if (this->openListSize() == 0) {
			writeLogLine("No more nodes to expand", "ARAStarPathPlanner", WP::LOG_FILE);
		} else if (timeElapsed.total_seconds() >= secondsToCalculatePath_) {
			writeLogLine("No more time to improve current solution", "ARAStarPathPlanner", WP::LOG_FILE);
			if (WP::LOG_LEVEL >= 2) {
				char line[150];
				sprintf(line, "Open list: %d \tExpanded list: %d", openListSize(), expandedListSize());
				writeLogLine(line, "ARAStarPathPlanner", WP::LOG_FILE);
			}
		}
	}
	return sol;
}

std::vector<Node*> ARAStarPathPlanner::clonePath(PathNode* end_node) {
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

