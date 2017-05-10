/**
 * @file PathNode.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Feb 18, 2011
 *      Author: marcello
 */

#include "orunav_motion_planner/PathNode.h"

/** initialize static counter */
long unsigned int Node::nodeCounter_ = 0;


PathNode::PathNode(std::vector<Configuration*> configurations, World* w, PathNode* parent) {
	G_ = 0;
	myWorld_ = w;
	parentNode_ = parent;

	// grid distance from goal, only in the presence of obstacles. Not admissible
	double h1 = 0;
	// distance estimation from goal
	double h2 = 0;
	// the cost is the sum of the costs of the individual configurations
	if (parentNode_ != 0) {
		G_ += parentNode_->getG();
	}
	for (std::vector<Configuration*>::iterator cit = configurations.begin(); cit != configurations.end(); cit++) {
		currentConfigurations_.push_back((*cit));
		G_ += (*cit)->getCostToThisConfiguration();
		if(myWorld_->containsObstacles()) {
			h1 += (*cit)->getMission()->getGridDistanceFromGoal((*cit)->getXCell(), (*cit)->getYCell());
		}
		h2 += (*cit)->estimateCostFromThisConfiguration((*cit)->getMission()->getGoalConfiguration());
	}

	// sort the Configuration vector
	if (currentConfigurations_.size() > 1) {
		std::sort(currentConfigurations_.begin(), currentConfigurations_.end(), configurationOrderingFunction);
	}
	// double heuristic -- take the max
	H_ = h1 > h2 ? h1 : h2;
	// for anytime algorithms, the heuristic weight can be changed
	H_ = H_ * WP::HEURISTIC_VALUE_MULTIPLIER;
	F_ = G_ + H_;
}

PathNode::PathNode(std::vector<Configuration*> newConfs, std::vector<Configuration*> unchangedConfs, World* w, PathNode* parent){

	G_ = 0;
	myWorld_ = w;
	parentNode_ = parent;

	// grid distance from goal, only in the presence of obstacles. Not admissible
	double h1 = 0;
	// distance estimation from goal
	double h2 = 0;
	// the cost is the sum of the costs of the individual configurations
	if (parentNode_ != 0) {
		G_ += parentNode_->getG();
	}
	/** @todo this can be sped up considerably by caching the heuristic estimations */
	// new configurations: calculate the cost
	for (std::vector<Configuration*>::iterator cit = newConfs.begin(); cit != newConfs.end(); cit++) {
		currentConfigurations_.push_back((*cit));
		G_ += (*cit)->getCostToThisConfiguration();
		if(myWorld_->containsObstacles()) {
			h1 += (*cit)->getMission()->getGridDistanceFromGoal((*cit)->getXCell(), (*cit)->getYCell());
		}
		h2 += (*cit)->estimateCostFromThisConfiguration((*cit)->getMission()->getGoalConfiguration());
	}
	for (std::vector<Configuration*>::iterator cit = unchangedConfs.begin(); cit != unchangedConfs.end(); cit++) {
		// here we do not update the cost, as it is already considered in the parent
		currentConfigurations_.push_back((*cit));
		if(myWorld_->containsObstacles()) {
			h1 += (*cit)->getMission()->getGridDistanceFromGoal((*cit)->getXCell(), (*cit)->getYCell());
		}
		h2 += (*cit)->estimateCostFromThisConfiguration((*cit)->getMission()->getGoalConfiguration());
	}

	// sort the Configuration vector
	if (currentConfigurations_.size() > 1) {
		std::sort(currentConfigurations_.begin(), currentConfigurations_.end(), configurationOrderingFunction);
	}

	// double heuristic -- take the max
	H_ = h1 > h2 ? h1 : h2;
	// for anytime algorithms, the heuristic weight can be changed
	H_ = H_ * WP::HEURISTIC_VALUE_MULTIPLIER;
	F_ = G_ + H_;
}

PathNode::~PathNode() {
	if (currentConfigurations_.size() != 0) {
		for (std::vector<Configuration*>::iterator cit = currentConfigurations_.begin(); cit != currentConfigurations_.end(); cit++) {
			delete (*cit);
		}
	}
}

std::vector<Node*> PathNode::generateChildren() {
	std::vector<Node*> children;


	if(WP::NODE_EXPANSION_METHOD == WP::NodeExpansionMethod::NAIVE) {

		// successor generation: general case
		// for n robots and b available actions per robot, we expand n*b nodes
		// -- select configuration to expand
		// -- add the positions of the other vehicles to the occupancy map
		// -- expand
		// -- remove the positions of the other vehicles from the map

		for (std::vector<Configuration*>::iterator confit = currentConfigurations_.begin(); confit != currentConfigurations_.end(); confit++) {

			std::vector<Configuration*> newConfigurations = (*confit)->generateNewConfigurations();
			std::vector<Configuration*> oldConfigurations;
			oldConfigurations.clear();

			// save all the configurations from the other vehicles
			for (std::vector<Configuration*>::iterator it = currentConfigurations_.begin(); it != currentConfigurations_.end(); it++) {
				if (it != confit) {
					oldConfigurations.push_back(*it);
					// add the position of the other vehicles as obstacles
					std::vector<cellPosition> occ = (*it)->getCellsOccupied();
					myWorld_->addObstacles(occ);
				}
			}

			// create the new nodes
			for (std::vector<Configuration*>::iterator newconfit = newConfigurations.begin(); newconfit != newConfigurations.end(); newconfit++) {
				if (myWorld_->getCollisionDetector()->isCollisionFree(*newconfit)) {

					std::vector<Configuration*> updatedConfigurations;
					updatedConfigurations.clear();
					updatedConfigurations.push_back(*newconfit);

					// clone the old configurations now that we know that a new node must be created
					std::vector<Configuration*> clonedOldConfigurations;
					clonedOldConfigurations.clear();
					for (std::vector<Configuration*>::iterator oldconfit = oldConfigurations.begin(); oldconfit != oldConfigurations.end(); oldconfit++) {
						clonedOldConfigurations.push_back((*oldconfit)->clone());
					}

					// only updated configurations have changed wrt the parent PathNode
					PathNode *newChild = new PathNode(updatedConfigurations, clonedOldConfigurations, myWorld_, this);
					children.push_back(newChild);
				} else {
					// delete unused Configuration
					delete (*newconfit);
				}
			}

			// remove the obstacles for the next iteration
			for (std::vector<Configuration*>::iterator it = oldConfigurations.begin(); it != oldConfigurations.end(); it++) {
				std::vector<cellPosition> free = (*it)->getCellsOccupied();
				myWorld_->removeObstacles(free);
			}

		}
		return children;
	} // end 	if(WP::NODE_EXPANSION_METHOD == 0)



	if(WP::NODE_EXPANSION_METHOD == WP::NodeExpansionMethod::EP) {

		// successor generation with bubbling up: avoid useless expansions -- Expansion Pruning
		// -- vehicles are ordered by their position in the configuration vector
		// -- check which vehicle has moved at the previous step
		// -- move all the vehicles lower or equal in order to the one that moved last
		// -- move all the vehicles higher in order that might have a collision

		// select the vehicles which should move. First detect the one that has moved last (if any)
		unsigned short int previousMover = 0;
		std::vector<Configuration*> prevConfs;
		prevConfs.clear();
		unsigned int totVehicles = currentConfigurations_.size();
		// if there is no parent or if there is only one vehicle, the first vehicle is the one to move
		if (this->getParent() && totVehicles > 1) {
			prevConfs = (dynamic_cast<PathNode*>(this->getParent()))->getConfigurations();
			for (unsigned short int index = 0; index < totVehicles; index ++) {
				// the configurations are different: this is the one that moved at the last iteration
				if (!currentConfigurations_[index]->equalConfigurations(prevConfs[index])){
					previousMover = index;
					break;
				}
			}
		}

		// move the vehicles lower or equal in order
		for (unsigned int i = previousMover; i < totVehicles; i++ ) {

			std::vector<Configuration*> newConfigurations = currentConfigurations_[i]->generateNewConfigurations();
			std::vector<Configuration*> oldConfigurations;
			oldConfigurations.clear();

			// save all the configurations from the other vehicles
			for (unsigned int j = 0; j < totVehicles; j++ ) {
				if (i != j) {
					oldConfigurations.push_back(currentConfigurations_[j]);
					// add the position of the other vehicles as obstacles
					std::vector<cellPosition> occ = currentConfigurations_[j]->getCellsOccupied();
					myWorld_->addObstacles(occ);
				}
			}

			// create the new nodes
			for (std::vector<Configuration*>::iterator newconfit = newConfigurations.begin(); newconfit != newConfigurations.end(); newconfit++) {
				if (myWorld_->getCollisionDetector()->isCollisionFree(*newconfit)) {

					std::vector<Configuration*> updatedConfigurations;
					updatedConfigurations.clear();
					updatedConfigurations.push_back(*newconfit);

					// clone the old configurations now that we know that a new node must be created
					std::vector<Configuration*> clonedOldConfigurations;
					clonedOldConfigurations.clear();
					for (std::vector<Configuration*>::iterator oldconfit = oldConfigurations.begin(); oldconfit != oldConfigurations.end(); oldconfit++) {
						clonedOldConfigurations.push_back((*oldconfit)->clone());
					}

					// only updated configurations have changed wrt the parent PathNode
					PathNode *newChild = new PathNode(updatedConfigurations, clonedOldConfigurations, myWorld_, this);
					children.push_back(newChild);

				} else {
					// delete unused Configuration
					delete (*newconfit);
				}
			}

			// remove the obstacles for the next iteration
			for (std::vector<Configuration*>::iterator it = oldConfigurations.begin(); it != oldConfigurations.end(); it++) {
				std::vector<cellPosition> free = (*it)->getCellsOccupied();
				myWorld_->removeObstacles(free);
			}
		}


		if (previousMover != 0) {

			// we consider 2 positions of the last mover: the current one and its previous one
			double prevMovXpos = currentConfigurations_[previousMover]->getXCoordinate();
			double prevMovYpos = currentConfigurations_[previousMover]->getYCoordinate();

			double prevMovXposLast = 0;
			double prevMovYposLast = 0;
			if (prevConfs.size() > 0) {
				prevMovXposLast = prevConfs[previousMover]->getXCoordinate();
				prevMovYposLast = prevConfs[previousMover]->getYCoordinate();
			}

			// now move the vehicles higher in order which may have a collision
			for (unsigned int i = 0; i < previousMover; i++ ) {

				// is there a possibility of collision?
				double distance1 = sqrt(pow(currentConfigurations_[i]->getXCoordinate() -  prevMovXpos, 2) +
						pow(currentConfigurations_[i]->getYCoordinate() -  prevMovYpos, 2));
				double distance2 = sqrt(pow(currentConfigurations_[i]->getXCoordinate() -  prevMovXposLast, 2) +
						pow(currentConfigurations_[i]->getYCoordinate() -  prevMovYposLast, 2));
				double interferenceDistance = currentConfigurations_[i]->getMission()->getVehicleModel()->getInterferenceRange() +
						currentConfigurations_[previousMover]->getMission()->getVehicleModel()->getInterferenceRange();
				if (distance1 <= interferenceDistance && distance2 <= interferenceDistance) {

					std::vector<Configuration*> newConfigurations = currentConfigurations_[i]->generateNewConfigurations();
					std::vector<Configuration*> oldConfigurations;
					oldConfigurations.clear();

					// save all the configurations from the other vehicles
					for (unsigned int j = 0; j < totVehicles; j++ ) {
						if (i != j) {
							oldConfigurations.push_back(currentConfigurations_[j]);
							// add the position of the other vehicles as obstacles
							std::vector<cellPosition> occ = currentConfigurations_[j]->getCellsOccupied();
							myWorld_->addObstacles(occ);
						}
					}

					// create the new nodes
					for (std::vector<Configuration*>::iterator newconfit = newConfigurations.begin(); newconfit != newConfigurations.end(); newconfit++) {
						if (myWorld_->getCollisionDetector()->isCollisionFree(*newconfit)) {

							std::vector<Configuration*> updatedConfigurations;
							updatedConfigurations.clear();
							updatedConfigurations.push_back(*newconfit);

							// clone the old configurations now that we know that a new node must be created
							std::vector<Configuration*> clonedOldConfigurations;
							clonedOldConfigurations.clear();
							for (std::vector<Configuration*>::iterator oldconfit = oldConfigurations.begin(); oldconfit != oldConfigurations.end(); oldconfit++) {
								clonedOldConfigurations.push_back((*oldconfit)->clone());
							}

							// only updated configurations have changed wrt the parent PathNode
							PathNode *newChild = new PathNode(updatedConfigurations, clonedOldConfigurations, myWorld_, this);
							children.push_back(newChild);

						} else {
							// delete unused Configuration
							delete (*newconfit);
						}
					}

					// remove the obstacles for the next iteration
					for (std::vector<Configuration*>::iterator it = oldConfigurations.begin(); it != oldConfigurations.end(); it++) {
						std::vector<cellPosition> free = (*it)->getCellsOccupied();
						myWorld_->removeObstacles(free);
					}
				}
			}
		}
		return children;

	}// end 	if(WP::NODE_EXPANSION_METHOD == 1)



	if(WP::NODE_EXPANSION_METHOD == WP::NodeExpansionMethod::FSG) {

		// successor generation: general case AND primitive hierarchies -- Fast Successor Generation
		// for n robots and b available actions per robot, we expand n*b nodes
		// -- select configuration to expand
		// -- add the positions of the other vehicles to the occupancy map
		// -- expand
		// -- remove the positions of the other vehicles from the map

		for (std::vector<Configuration*>::iterator confit = currentConfigurations_.begin(); confit != currentConfigurations_.end(); confit++) {

			std::vector<Configuration*> oldConfigurations;
			oldConfigurations.clear();

			// save all the configurations from the other vehicles
			for (std::vector<Configuration*>::iterator it = currentConfigurations_.begin(); it != currentConfigurations_.end(); it++) {
				if (it != confit) {
					oldConfigurations.push_back(*it);
					// add the position of the other vehicles as obstacles
					std::vector<cellPosition> occ = (*it)->getCellsOccupied();
					myWorld_->addObstacles(occ);
				}
			}

			std::vector<Configuration*> newConfigurations = (*confit)->generateNewConfigurations(myWorld_);

			// create the new nodes
			for (std::vector<Configuration*>::iterator newconfit = newConfigurations.begin(); newconfit != newConfigurations.end(); newconfit++) {
				std::vector<Configuration*> updatedConfigurations;
				updatedConfigurations.clear();
				updatedConfigurations.push_back(*newconfit);

				// clone the old configurations now that we know that a new node must be created
				std::vector<Configuration*> clonedOldConfigurations;
				clonedOldConfigurations.clear();
				for (std::vector<Configuration*>::iterator oldconfit = oldConfigurations.begin(); oldconfit != oldConfigurations.end(); oldconfit++) {
					clonedOldConfigurations.push_back((*oldconfit)->clone());
				}

				// only updated configurations have changed wrt the parent PathNode
				PathNode *newChild = new PathNode(updatedConfigurations, clonedOldConfigurations, myWorld_, this);
				children.push_back(newChild);
			}

			// remove the obstacles for the next iteration
			for (std::vector<Configuration*>::iterator it = oldConfigurations.begin(); it != oldConfigurations.end(); it++) {
				std::vector<cellPosition> free = (*it)->getCellsOccupied();
				myWorld_->removeObstacles(free);
			}

		}
		return children;
	} // end 	if(WP::NODE_EXPANSION_METHOD == 2)



	if(WP::NODE_EXPANSION_METHOD == WP::NodeExpansionMethod::EPFSG) {

		// successor generation with bubbling up (avoid useless expansions) AND primitive hierarchies -- Expansion Pruning + Fast Successor Generation
		// -- vehicles are order by their position in the configuration vector
		// -- check which vehicle has moved at the previous step
		// -- move all the vehicles lower or equal in order to the one that moved last
		// -- move all the vehicles higher in order that might have a collision
		// select the vehicles which should move. First detect the one that has moved last (if any)

		unsigned short int previousMover = 0;
		std::vector<Configuration*> prevConfs;
		prevConfs.clear();
		unsigned int totVehicles = currentConfigurations_.size();
		// if there is no parent or if there is only one vehicle, the first vehicle is the one to move
		if (this->getParent() && totVehicles > 1) {
			prevConfs = (dynamic_cast<PathNode*>(this->getParent()))->getConfigurations();
			for (unsigned short int index = 0; index < totVehicles; index ++) {
				// the configurations are different: this is the one that moved at the last iteration
				if (!currentConfigurations_[index]->equalConfigurations(prevConfs[index])){
					previousMover = index;
					break;
				}
			}
		}

		// move the vehicles lower or equal in order
		for (unsigned int i = previousMover; i < totVehicles; i++ ) {

			std::vector<Configuration*> oldConfigurations;
			oldConfigurations.clear();

			// save all the configurations from the other vehicles
			for (unsigned int j = 0; j < totVehicles; j++ ) {
				if (i != j) {
					oldConfigurations.push_back(currentConfigurations_[j]);
					// add the position of the other vehicles as obstacles
					std::vector<cellPosition> occ = currentConfigurations_[j]->getCellsOccupied();
					myWorld_->addObstacles(occ);
				}
			}

			std::vector<Configuration*> newConfigurations = currentConfigurations_[i]->generateNewConfigurations(myWorld_);

			// create the new nodes
			for (std::vector<Configuration*>::iterator newconfit = newConfigurations.begin(); newconfit != newConfigurations.end(); newconfit++) {
				std::vector<Configuration*> updatedConfigurations;
				updatedConfigurations.clear();
				updatedConfigurations.push_back(*newconfit);

				// clone the old configurations now that we know that a new node must be created
				std::vector<Configuration*> clonedOldConfigurations;
				clonedOldConfigurations.clear();
				for (std::vector<Configuration*>::iterator oldconfit = oldConfigurations.begin(); oldconfit != oldConfigurations.end(); oldconfit++) {
					clonedOldConfigurations.push_back((*oldconfit)->clone());
				}

				// only updated configurations have changed wrt the parent PathNode
				PathNode *newChild = new PathNode(updatedConfigurations, clonedOldConfigurations, myWorld_, this);
				children.push_back(newChild);
			}

			// remove the obstacles for the next iteration
			for (std::vector<Configuration*>::iterator it = oldConfigurations.begin(); it != oldConfigurations.end(); it++) {
				std::vector<cellPosition> free = (*it)->getCellsOccupied();
				myWorld_->removeObstacles(free);
			}
		}

		if (previousMover != 0) {

			// we consider 2 positions of the last mover: the current one and its previous one
			double prevMovXpos = currentConfigurations_[previousMover]->getXCoordinate();
			double prevMovYpos = currentConfigurations_[previousMover]->getYCoordinate();

			double prevMovXposLast = 0;
			double prevMovYposLast = 0;
			if (prevConfs.size() > 0) {
				prevMovXposLast = prevConfs[previousMover]->getXCoordinate();
				prevMovYposLast = prevConfs[previousMover]->getYCoordinate();
			}

			// now move the vehicles higher in order which may have a collision
			for (unsigned int i = 0; i < previousMover; i++ ) {

				// is there a possibility of collision?
				double distance1 = sqrt(pow(currentConfigurations_[i]->getXCoordinate() -  prevMovXpos, 2) +
						pow(currentConfigurations_[i]->getYCoordinate() -  prevMovYpos, 2));
				double distance2 = sqrt(pow(currentConfigurations_[i]->getXCoordinate() -  prevMovXposLast, 2) +
						pow(currentConfigurations_[i]->getYCoordinate() -  prevMovYposLast, 2));
				double interferenceDistance = currentConfigurations_[i]->getMission()->getVehicleModel()->getInterferenceRange() +
						currentConfigurations_[previousMover]->getMission()->getVehicleModel()->getInterferenceRange();
				if (distance1 <= interferenceDistance && distance2 <= interferenceDistance) {

					std::vector<Configuration*> oldConfigurations;
					oldConfigurations.clear();

					// save all the configurations from the other vehicles
					for (unsigned int j = 0; j < totVehicles; j++ ) {
						if (i != j) {
							oldConfigurations.push_back(currentConfigurations_[j]);
							// add the position of the other vehicles as obstacles
							std::vector<cellPosition> occ = currentConfigurations_[j]->getCellsOccupied();
							myWorld_->addObstacles(occ);
						}
					}

					std::vector<Configuration*> newConfigurations = currentConfigurations_[i]->generateNewConfigurations(myWorld_);

					// create the new nodes
					for (std::vector<Configuration*>::iterator newconfit = newConfigurations.begin(); newconfit != newConfigurations.end(); newconfit++) {
						std::vector<Configuration*> updatedConfigurations;
						updatedConfigurations.clear();
						updatedConfigurations.push_back(*newconfit);

						// clone the old configurations now that we know that a new node must be created
						std::vector<Configuration*> clonedOldConfigurations;
						clonedOldConfigurations.clear();
						for (std::vector<Configuration*>::iterator oldconfit = oldConfigurations.begin(); oldconfit != oldConfigurations.end(); oldconfit++) {
							clonedOldConfigurations.push_back((*oldconfit)->clone());
						}

						// only updated configurations have changed wrt the parent PathNode
						PathNode *newChild = new PathNode(updatedConfigurations, clonedOldConfigurations, myWorld_, this);
						children.push_back(newChild);
					}

					// remove the obstacles for the next iteration
					for (std::vector<Configuration*>::iterator it = oldConfigurations.begin(); it != oldConfigurations.end(); it++) {
						std::vector<cellPosition> free = (*it)->getCellsOccupied();
						myWorld_->removeObstacles(free);
					}
				}
			}
		}
		return children;
	}

	return children;

}

std::vector<Configuration*> PathNode::getConfigurations() {
	return currentConfigurations_;
}

bool PathNode::equalContent(Node* pn) {
	ConfigurationVectorEqual t;
	return t.operator ()((dynamic_cast<PathNode*> (pn))->getConfigurations(), currentConfigurations_);
}

void PathNode::print() {
	std::string str = std::string("PathNode ");
	char info[90];
	sprintf(info, "\t[c: %3.2f \th: %3.2f \tf: %3.2f] :", G_, H_, F_);
	str.append(std::string(info));
	for (std::vector<Configuration*>::iterator it = currentConfigurations_.begin(); it != currentConfigurations_.end(); it++ ) {
		str.append(std::string("\n\t"));
		str.append((*it)->printToString());
	}
	writeLogLine(str, "PathNode", WP::LOG_FILE);
}

Node* PathNode::clone() {
	std::vector<Configuration*> newConfs;
	newConfs.clear();
	for (std::vector<Configuration*>::iterator it = currentConfigurations_.begin(); it != currentConfigurations_.end(); it++) {
		newConfs.push_back((*it)->clone());
	}
	PathNode* n = new PathNode(newConfs, this->myWorld_, 0);
	n->F_ = this->getF();
	n->G_ = this->getG();
	n->H_ = this->getH();
	n->notToBeExpanded_ = this->notToBeExpanded_;
	n->alreadyExpanded_ = this->alreadyExpanded_;
	return n;
}
