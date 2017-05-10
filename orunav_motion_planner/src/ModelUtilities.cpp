/**
 *  @file ModelUtilities.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Oct 8, 2012
 *      Author: marcello
 */

#include "orunav_motion_planner/ModelUtilities.h"

ModelUtilities::ModelUtilities() {
}

ModelUtilities::~ModelUtilities() {
}

void generateModelHeuristicTableEntry(VehicleModel* m, uint8_t startOrientID, uint8_t startSteerID,
		uint8_t goalOrientID, uint8_t goalSteerID, short int dx, short int dy) {
	std::ostringstream line;
	line << "[startOrientID = " << (unsigned int) startOrientID << "; startSteerID = " << (unsigned int) startSteerID <<
			"; goalOrientID = " << (unsigned int) goalOrientID << "; goalSteerID = " << (unsigned int) goalSteerID <<
			"; dx = " << dx << "; dy = " << dy << "]";
	// check if the goal is already in the heuristic table
	if (m->isHeuristicValueInTable(0, 0, startOrientID, startSteerID, dx, dy, goalOrientID, goalSteerID)) {
		if (WP::LOG_LEVEL >= 1) {
			line << "\tAlready in heuristic table";
			writeLogLine(line.str(), "ModelUtilities", WP::LOG_FILE);
		}
	} else {
		if (WP::LOG_LEVEL >= 1) {
			line << "\tCalculating...";
			writeLogLine(line.str(), "ModelUtilities", WP::LOG_FILE);
		}
		double delta = dx > 300 ? dx / 2 : 150;

		PathFinder * pf = new PathFinder((dx + delta) * 2 * WP::WORLD_SPACE_GRANULARITY, (delta + dy) * 2 * WP::WORLD_SPACE_GRANULARITY);
		pf->enableDataGatheringForVehicleHT();

		VehicleMission* mission = new VehicleMission(m,
				(delta + dx) * m->getModelGranularity(), (delta + dy) * m->getModelGranularity(),
				startOrientID * ((2* M_PI) / m->getOrientationAngles()), startSteerID * ((2* M_PI) / m->getSteeringAnglePartitions()),
				(delta + 2 * dx) * m->getModelGranularity(), (delta + 2 * dy) * m->getModelGranularity(),
				goalOrientID * ((2* M_PI) / m->getOrientationAngles()), goalSteerID * ((2* M_PI) / m->getSteeringAnglePartitions()));

		pf->addMission(mission);

		// ALL points of the solution are heuristic table entries
		std::vector<std::vector<Configuration*> > solution = pf->solve(false);

		// cleanup
		for (std::vector<std::vector<Configuration*> >::iterator it = solution.begin(); it != solution.end(); it++) {
			std::vector<Configuration*> confs = (*it);
			for (std::vector<Configuration*>::iterator confit = confs.begin(); confit != confs.end(); confit++) {
				delete (*confit);
			}
		}
		// cleanup
		delete mission;
		delete pf;
	}
}

void ModelUtilities::generateModelHeuristicTable(VehicleModel* m, double costCutoff, double fillDistance) {

	// enable garbage collection: maybe slower, but memory efficient
	WP::enableGarbageCollection(true);

	// Dijkstra's algorithm with cutoff
	WP::enableUseOfHeuristicEstimation(false);
	WP::setCostCutoff(costCutoff);

	// set the distance of the goal so as it will never be reached by the exploration before the cost cutoff
	int cutoffDX = ((costCutoff / m->getModelGranularity()) * 1.2);

	for (uint8_t orientID = 0; orientID <= (m->getOrientationAngles() / 8); orientID++) {
		// a bit convoluted, but should account for all the steering angles allowed
		for (uint8_t steerID = ((m->getSteeringAnglePartitions() - ((m->getSteeringAngleCardinality() - 1) / 2)) % m->getSteeringAnglePartitions());
				steerID != ((m->getSteeringAngleCardinality() - 1) / 2) + 1; steerID = steerID + 1) {
			steerID = steerID % m->getSteeringAnglePartitions();

			if (WP::LOG_LEVEL >= 1) {
				std::ostringstream log_line;
				log_line << "Calculating Dijkstra for initial pose ["
						<< (unsigned) orientID << "," << (unsigned) steerID << "] with cut-off cost = " << costCutoff;
				writeLogLine(log_line.str(), "ModelUtilities", WP::LOG_FILE);
			}
			generateModelHeuristicTableEntry(m, orientID, steerID, orientID, steerID, cutoffDX, cutoffDX);
		}
	}

	m->saveHeuristicTable();

	// avoid filling negative distances
	if (fillDistance >= 0) {
		// fill up the table
		WP::enableUseOfHeuristicEstimation(true);
		WP::setExpansionQueueMaxSize((unsigned) 4000000);
		WP::setCostCutoff(INFINITY);

		int cellDistance = ceil(fillDistance / m->getModelGranularity());

		for (uint8_t startOID = 0; startOID <= (m->getOrientationAngles() / 8); startOID++) {
			// a bit convoluted, but should account for all the steering angles allowed
			for (uint8_t startSID = ((m->getSteeringAnglePartitions() - ((m->getSteeringAngleCardinality() - 1) / 2)) % m->getSteeringAnglePartitions());
					startSID != ((m->getSteeringAngleCardinality() - 1) / 2) + 1; startSID = startSID + 1) {
				startSID = startSID % m->getSteeringAnglePartitions();

				for (uint8_t goalOID = 0; goalOID < m->getOrientationAngles(); goalOID++) {
					// a bit convoluted, but should account for all the steering angles allowed
					for (uint8_t goalSID = ((m->getSteeringAnglePartitions() - ((m->getSteeringAngleCardinality() - 1) / 2)) % m->getSteeringAnglePartitions());
							goalSID != ((m->getSteeringAngleCardinality() - 1) / 2) + 1; goalSID = goalSID + 1) {
						goalSID = goalSID % m->getSteeringAnglePartitions();

						for (int dx = -cellDistance; dx <= cellDistance; dx++) {
							for (int dy = -cellDistance; dy <= cellDistance; dy++) {
								generateModelHeuristicTableEntry(m, startOID, startSID, goalOID, goalSID, dx, dy);
							}
						}
					}
				}
				// save the HT of the model at the end of each iteration
				m->saveHeuristicTable();
			}
		}
	}
	return;
}


void ModelUtilities::reduceModelMotionPrimitiveSet(VehicleModel* m, double additionalCostToleranceMultiplier){

	WP::setExpansionMethod(WP::NodeExpansionMethod::NAIVE);

	// save the original motion primitive lookup table for proper clean-up and create a working copy
	VehicleModel::motionPrimitivesLookup originalTable = m->getPrimitivesLookupTable();
	VehicleModel::motionPrimitivesLookup workingTable;

	// create a duplicate of the original motionPrimitivesLookup table
	for (VehicleModel::motionPrimitivesLookup::iterator it = originalTable.begin(); it != originalTable.end(); it++) {
		// sort the primitive vectors according to length
		sort((*it).second.begin(), (*it).second.end(), lengthSortingFunction);
		workingTable.insert(VehicleModel::motionPrimitivesLookupEntry((*it).first, (*it).second));
	}

	// we work under the assumption of the 8-axis symmetry
	for (uint8_t orientID = 0; orientID <= (m->getOrientationAngles() / 8); orientID++) {
		// a bit convoluted, but should account for all the steering angles allowed
		for (uint8_t steerID = ((m->getSteeringAnglePartitions() - ((m->getSteeringAngleCardinality() - 1) / 2)) % m->getSteeringAnglePartitions());
				steerID != ((m->getSteeringAngleCardinality() - 1) / 2) + 1; steerID = steerID + 1) {

			steerID = steerID % m->getSteeringAnglePartitions();

			// get the primitives for this particular combination of orientID and steerID
			VehicleModel::motionPrimitivesLookup::iterator it;
			it = workingTable.find(std::pair<uint8_t, uint8_t>(orientID, steerID));
			std::vector<MotionPrimitiveData*> primitives = (*it).second;

			if (WP::LOG_LEVEL >= 1) {
				std::ostringstream log_line;
				log_line << "Initial primitives " << "(oID: " << (unsigned) orientID <<
						", sID: " << (unsigned) steerID << ") = " << primitives.size();
				writeLogLine(log_line.str(), "ModelUtilities", WP::LOG_FILE);
			}

			unsigned int iterations = primitives.size();
			for (unsigned int i = 0 ; i < iterations ; i ++)  {

				// get the primitive that we try to decompose -- longest first
				MotionPrimitiveData* extracted = primitives.front();
				primitives.erase(primitives.begin());
				(*it).second.clear();
				(*it).second = primitives;

				m->setPrimitivesLookupTable(workingTable);

				// try to decompose the primitive
				double delta = ceil(extracted->getDistanceCovered() * 10);
				double primitiveCost = extracted->getDistanceCovered() * extracted->getCostMultiplier();

				std::ostringstream primitive_info;
				primitive_info << "(oID: " << (unsigned) orientID << "; sID: " << (unsigned) steerID <<
						"; ID: " << extracted->getID() << ") to xcell: " <<  extracted->getXOffset() <<
						"; ycell: " << extracted->getYOffset() << "; oID: " << (unsigned) extracted->getFinalOrientationID() <<
						"; sID: " << (unsigned) extracted->getFinalSteeringID() << " [" << primitiveCost << "]";

				if (WP::LOG_LEVEL >= 1) {
					std::ostringstream log_line;
					log_line << "Decomposing primitive " << primitive_info.str();
					writeLogLine(log_line.str(), "ModelUtilities", WP::LOG_FILE);
				}

				// set a limited expansion queue and the expansion cost
				WP::setCostCutoff(primitiveCost * additionalCostToleranceMultiplier * 1.05);
				WP::setExpansionQueueMaxSize(500000);
				PathFinder * pf = new PathFinder(delta*2, delta*2);

				VehicleMission* mission = new VehicleMission(m,
						delta, delta,
						orientID * ((2* M_PI) / m->getOrientationAngles()),
						steerID * ((2* M_PI) / m->getSteeringAnglePartitions()),
						delta + (extracted->getXOffset() * m->getModelGranularity()),
						delta + (extracted->getYOffset() * m->getModelGranularity()),
						extracted->getFinalOrientationID() * ((2* M_PI) / m->getOrientationAngles()),
						extracted->getFinalSteeringID() * ((2* M_PI) / m->getSteeringAnglePartitions()));

				// add the mission to this PathFinder
				pf->addMission(mission);

				// ALL points of the solution are heuristic table entries
				std::vector<std::vector<Configuration*> > solution = pf->solve(false);
				std::vector<Configuration*> solfront = solution.front();

				double alternativePathCost = 0;
				if (solfront.size() > 0) {
					for (std::vector<Configuration*>::iterator solit = solfront.begin(); solit != solfront.end(); solit++) {
						alternativePathCost += (*solit)->getCostToThisConfiguration();
					}
				}

				if (solfront.size() == 0 || alternativePathCost > (primitiveCost * additionalCostToleranceMultiplier)) {
					// the primitive is not decomposable: insert it again in the primitives
					primitives.push_back(extracted);
					(*it).second.clear();
					(*it).second = primitives;
					m->setPrimitivesLookupTable(workingTable);
					if (WP::LOG_LEVEL >= 1) {
						std::ostringstream log_line;
						log_line << "The Primitive 	" << primitive_info.str() <<
								" is NOT decomposable. Alternative cost: " << alternativePathCost;
						writeLogLine(log_line.str(), "ModelUtilities", WP::LOG_FILE);
					}
				} else {
					if (WP::LOG_LEVEL >= 1) {
						std::ostringstream log_line;
						log_line << "The Primitive 	" << primitive_info.str() <<
								" is decomposable. Alternative cost: " << alternativePathCost;
						writeLogLine(log_line.str(), "ModelUtilities", WP::LOG_FILE);
					}
				}

				// cleanup
				for (std::vector<std::vector<Configuration*> >::iterator solit = solution.begin(); solit != solution.end(); solit++) {
					std::vector<Configuration*> configs = (*solit);
					for (std::vector<Configuration*>::iterator confit = configs.begin(); confit != configs.end(); confit++) {
						delete (*confit);
					}
				}
				delete pf;
				delete mission;
			}
			if (WP::LOG_LEVEL >= 1) {
				std::ostringstream log_line;
				log_line << "Final primitives " << "(oID: " << (unsigned) orientID <<
						", sID: " << (unsigned) steerID << ") = " << primitives.size();
				writeLogLine(log_line.str(), "ModelUtilities", WP::LOG_FILE);
			}
		}
	}

	// save the new Primitive Lookup Table, applying the 8-axis symmetry
	fillPrimitiveLookupTableWith8AxisSymmetry(m);
	saveReducedPrimitiveLookupTable(m);

	// restore the original table for proper clean-up
	m->setPrimitivesLookupTable(originalTable);
}


void ModelUtilities::fillPrimitiveLookupTableWith8AxisSymmetry(VehicleModel* m) {

	VehicleModel::motionPrimitivesLookup primitivesTable = m->getPrimitivesLookupTable();

	// reduce the rest of the primitive lookup table according to the 8-axis symmetry
	// -------------------------------------------------------------------------
	// Second sector  (pi/4 - pi/2]
	// -------------------------------------------------------------------------
	for (uint8_t origOrientID = 0; origOrientID < m->getOrientationAngles() / 8; origOrientID++) {
		// a bit convoluted, but should account for all the steering angles allowed
		for (uint8_t origSteerID = ((m->getSteeringAnglePartitions() - ((m->getSteeringAngleCardinality() - 1) / 2)) % m->getSteeringAnglePartitions());
				origSteerID != ((m->getSteeringAngleCardinality() - 1) / 2) + 1; origSteerID = origSteerID + 1) {

			origSteerID = origSteerID % m->getSteeringAnglePartitions();

			uint8_t symmSteerID = (m->getSteeringAnglePartitions() - origSteerID) % m->getSteeringAnglePartitions();
			uint8_t symmOrientID = (m->getOrientationAngles()/4) - origOrientID;

			VehicleModel::motionPrimitivesLookup::iterator originalIterator;
			originalIterator = primitivesTable.find(std::pair<uint8_t, uint8_t>(origOrientID, origSteerID));
			std::vector<MotionPrimitiveData*> originalPrimitives = (*originalIterator).second;

			VehicleModel::motionPrimitivesLookup::iterator symmIterator;
			symmIterator = primitivesTable.find(std::pair<uint8_t, uint8_t>(symmOrientID, symmSteerID));
			std::vector<MotionPrimitiveData*> oldSymmPrimitives = (*symmIterator).second;

			std::vector<MotionPrimitiveData*> newSymmPrimitives;
			newSymmPrimitives.clear();

			// locate the new rotated primitives by ID to be stored
			for(std::vector<MotionPrimitiveData*>::iterator primit = originalPrimitives.begin(); primit != originalPrimitives.end(); primit++) {
				bool primitiveFound = false;

				std::ostringstream orig_primitive_info;
				orig_primitive_info << "[oID: " << (unsigned) origOrientID << ", sID: " << (unsigned) origSteerID <<
						", ID: " << (*primit)->getID() << "]";

				for(std::vector<MotionPrimitiveData*>::iterator symmprimit = oldSymmPrimitives.begin();
						symmprimit != oldSymmPrimitives.end(); symmprimit++) {
					if ((*primit)->getID() == (*symmprimit)->getID()) {
						// we should have found it. Check and save
						if ((fabs((*primit)->getDistanceCovered() - (*symmprimit)->getDistanceCovered()) > (*primit)->getDistanceCovered() * 0.05) ||
								(*primit)->getCostMultiplier() != (*symmprimit)->getCostMultiplier()) {
							if (WP::LOG_LEVEL >= 1) {
								std::ostringstream log_line;
								log_line << "ERROR! Primitive mismatch: " << orig_primitive_info.str() <<
										"\t[oID: " << (unsigned) symmOrientID << ", sID: " << (unsigned) symmSteerID <<
										", ID: " << (*symmprimit)->getID() << "]";
								writeLogLine(log_line.str(), "ModelUtilities", WP::LOG_FILE);
							}
						} else {
							primitiveFound = true;
							if (WP::LOG_LEVEL >= 2) {
								std::ostringstream log_line;
								log_line << "Primitive found: " << orig_primitive_info.str() <<
										"\t[oID: " << (unsigned) symmOrientID << ", sID: " << (unsigned) symmSteerID <<
										", ID: " << (*symmprimit)->getID() << "]";
								writeLogLine(log_line.str(), "ModelUtilities", WP::LOG_FILE);
							}
							newSymmPrimitives.push_back((*symmprimit));
						}
					}
				}
				if (!primitiveFound) {
					std::ostringstream log_line;
					log_line << "Primitive not found (second sector)! " << orig_primitive_info.str();
					writeLogLine(log_line.str(), "ModelUtilities", WP::LOG_FILE);
					exit(0);
				}
			}
			(*symmIterator).second.clear();
			(*symmIterator).second = newSymmPrimitives;
		}
	}

	// -------------------------------------------------------------------------
	// y axis symmetry  (pi/2 - pi]
	// -------------------------------------------------------------------------
	for (uint8_t origOrientID = 0; origOrientID < m->getOrientationAngles() / 4; origOrientID++) {
		// a bit convoluted, but should account for all the steering angles allowed
		for (uint8_t origSteerID = ((m->getSteeringAnglePartitions() - ((m->getSteeringAngleCardinality() - 1) / 2)) % m->getSteeringAnglePartitions());
				origSteerID != ((m->getSteeringAngleCardinality() - 1) / 2) + 1; origSteerID = origSteerID + 1) {

			origSteerID = origSteerID % m->getSteeringAnglePartitions();

			uint8_t symmSteerID = (m->getSteeringAnglePartitions() - origSteerID) % m->getSteeringAnglePartitions();
			uint8_t symmOrientID = (m->getOrientationAngles() / 2) - origOrientID;

			VehicleModel::motionPrimitivesLookup::iterator originalIterator;
			originalIterator = primitivesTable.find(std::pair<uint8_t, uint8_t>(origOrientID, origSteerID));
			std::vector<MotionPrimitiveData*> originalPrimitives = (*originalIterator).second;

			VehicleModel::motionPrimitivesLookup::iterator symmIterator;
			symmIterator = primitivesTable.find(std::pair<uint8_t, uint8_t>(symmOrientID, symmSteerID));
			std::vector<MotionPrimitiveData*> oldSymmPrimitives = (*symmIterator).second;

			std::vector<MotionPrimitiveData*> newSymmPrimitives;
			newSymmPrimitives.clear();

			// locate the new rotated primitives by ID to be stored
			for(std::vector<MotionPrimitiveData*>::iterator primit = originalPrimitives.begin(); primit != originalPrimitives.end(); primit++) {
				bool primitiveFound = false;

				std::ostringstream orig_primitive_info;
				orig_primitive_info << "[oID: " << (unsigned) origOrientID << ", sID: " << (unsigned) origSteerID <<
						", ID: " << (*primit)->getID() << "]";

				for(std::vector<MotionPrimitiveData*>::iterator symmprimit = oldSymmPrimitives.begin();
						symmprimit != oldSymmPrimitives.end(); symmprimit++) {
					if ((*primit)->getID() == (*symmprimit)->getID()) {
						// we should have found it. Check and save
						if ((fabs((*primit)->getDistanceCovered() - (*symmprimit)->getDistanceCovered()) > (*primit)->getDistanceCovered() * 0.05) ||
								(*primit)->getCostMultiplier() != (*symmprimit)->getCostMultiplier()) {
							if (WP::LOG_LEVEL >= 1) {
								std::ostringstream log_line;
								log_line << "ERROR! Primitive mismatch: "  << orig_primitive_info.str() <<
										"\t[oID: " << (unsigned) symmOrientID << ", sID: " << (unsigned) symmSteerID <<
										", ID: " << (*symmprimit)->getID() << "]";
								writeLogLine(log_line.str(), "ModelUtilities", WP::LOG_FILE);
							}
						} else {
							primitiveFound = true;
							if (WP::LOG_LEVEL >= 2) {
								std::ostringstream log_line;
								log_line << "Primitive found: " << orig_primitive_info.str() <<
										"\t[oID: " << (unsigned) symmOrientID << ", sID: " << (unsigned) symmSteerID <<
										", ID: " << (*symmprimit)->getID() << "]";
								writeLogLine(log_line.str(), "ModelUtilities", WP::LOG_FILE);
							}
							newSymmPrimitives.push_back((*symmprimit));
						}
					}
				}
				if (!primitiveFound) {
					std::ostringstream log_line;
					log_line << "Primitive not found (y axis symmetry)! " << orig_primitive_info.str();
					writeLogLine(log_line.str(), "ModelUtilities", WP::LOG_FILE);
					exit(0);
				}
			}
			(*symmIterator).second.clear();
			(*symmIterator).second = newSymmPrimitives;
		}
	}


	// -------------------------------------------------------------------------
	// x axis symmetry (pi - 2*pi) -- we start from 1 because we already have 0
	// -------------------------------------------------------------------------
	for (uint8_t origOrientID = 1; origOrientID < m->getOrientationAngles() / 2; origOrientID++) {
		// a bit convoluted, but should account for all the steering angles allowed
		for (uint8_t origSteerID = ((m->getSteeringAnglePartitions() - ((m->getSteeringAngleCardinality() - 1) / 2)) % m->getSteeringAnglePartitions());
				origSteerID != ((m->getSteeringAngleCardinality() - 1) / 2) + 1; origSteerID = origSteerID + 1) {

			origSteerID = origSteerID % m->getSteeringAnglePartitions();

			uint8_t symmSteerID = (m->getSteeringAnglePartitions() - origSteerID) % m->getSteeringAnglePartitions();
			uint8_t symmOrientID = m->getOrientationAngles() - origOrientID;

			VehicleModel::motionPrimitivesLookup::iterator originalIterator;
			originalIterator = primitivesTable.find(std::pair<uint8_t, uint8_t>(origOrientID, origSteerID));
			std::vector<MotionPrimitiveData*> originalPrimitives = (*originalIterator).second;

			VehicleModel::motionPrimitivesLookup::iterator symmIterator;
			symmIterator = primitivesTable.find(std::pair<uint8_t, uint8_t>(symmOrientID, symmSteerID));
			std::vector<MotionPrimitiveData*> oldSymmPrimitives = (*symmIterator).second;

			std::vector<MotionPrimitiveData*> newSymmPrimitives;
			newSymmPrimitives.clear();

			// locate the new rotated primitives by ID to be stored
			for(std::vector<MotionPrimitiveData*>::iterator primit = originalPrimitives.begin(); primit != originalPrimitives.end(); primit++) {
				bool primitiveFound = false;

				std::ostringstream orig_primitive_info;
				orig_primitive_info << "[oID: " << (unsigned) origOrientID << ", sID: " << (unsigned) origSteerID <<
						", ID: " << (*primit)->getID() << "]";

				for(std::vector<MotionPrimitiveData*>::iterator symmprimit = oldSymmPrimitives.begin();
						symmprimit != oldSymmPrimitives.end(); symmprimit++) {
					if ((*primit)->getID() == (*symmprimit)->getID()) {
						// we should have found it. Check and save
						if ((fabs((*primit)->getDistanceCovered() - (*symmprimit)->getDistanceCovered()) > (*primit)->getDistanceCovered() * 0.05) ||
								(*primit)->getCostMultiplier() != (*symmprimit)->getCostMultiplier()) {
							if (WP::LOG_LEVEL >= 1) {
								std::ostringstream log_line;
								log_line << "ERROR! Primitive mismatch: " << orig_primitive_info.str() <<
										"\t[oID: " << (unsigned) symmOrientID << ", sID: " << (unsigned) symmSteerID <<
										", ID: " << (*symmprimit)->getID() << "]";
								writeLogLine(log_line.str(), "ModelUtilities", WP::LOG_FILE);
							}
						} else {
							primitiveFound = true;
							if (WP::LOG_LEVEL >= 2) {
								std::ostringstream log_line;
								log_line << "Primitive found: " << orig_primitive_info.str() <<
										"\t[oID: " << (unsigned) symmOrientID << ", sID: " << (unsigned) symmSteerID <<
										", ID: " << (*symmprimit)->getID() << "]";
								writeLogLine(log_line.str(), "ModelUtilities", WP::LOG_FILE);
							}
							newSymmPrimitives.push_back((*symmprimit));
						}
					}
				}
				if (!primitiveFound) {
					std::ostringstream log_line;
					log_line << "Primitive not found (x axis symmetry)! " << orig_primitive_info.str();
					writeLogLine(log_line.str(), "ModelUtilities", WP::LOG_FILE);
					exit(0);
				}
			}
			(*symmIterator).second.clear();
			(*symmIterator).second = newSymmPrimitives;
		}
	}
	// set the new table
	m->setPrimitivesLookupTable(primitivesTable);
}


void ModelUtilities::saveReducedPrimitiveLookupTable(VehicleModel* m){

	// get the primitives
	VehicleModel::motionPrimitivesLookup primitivesTable = m->getPrimitivesLookupTable();

	std::string newMotionPrimitivesFilename;
	newMotionPrimitivesFilename.clear();
	newMotionPrimitivesFilename.append(m->getModelPrimitivesFilename());
	newMotionPrimitivesFilename.erase(newMotionPrimitivesFilename.end()-5, newMotionPrimitivesFilename.end());
	newMotionPrimitivesFilename.append("reduced.mprim");

	std::string line;
	std::ifstream fin((m->getModelPrimitivesFilename()).c_str(), std::ifstream::in);
	std::ofstream fout(newMotionPrimitivesFilename.c_str(), std::ifstream::trunc);

	if (WP::LOG_LEVEL >= 1) {
		std::ostringstream logLine;
		logLine << "Original motion primitives filename:  " << m->getModelPrimitivesFilename();
		writeLogLine(logLine.str(), "ModelUtilities", WP::LOG_FILE);
		logLine.str("");
		logLine.clear();
		logLine << "New motion primitives filename:  " << newMotionPrimitivesFilename;
		writeLogLine(logLine.str(), "ModelUtilities", WP::LOG_FILE);
	}

	// a line which contains the ID of the primitive
	static const boost::regex id("^primID: (.+?)$");
	boost::smatch what;

	// copy the first lines of the original file
	if (fin.is_open() && fout.is_open()) {
		while (fin.good()) {
			getline(fin, line);

			boost::regex_match(line, what, id, boost::match_extra);
			if (what[0].matched) {
				// we reached the first primitive: stop copying
				break;
			} else {
				fout << line << std::endl;
			}
		}
		// close the original file
		fin.close();

		for(VehicleModel::motionPrimitivesLookup::iterator it = primitivesTable.begin(); it != primitivesTable.end(); it++ ) {
			unsigned short int primind = 0;
			std::vector<MotionPrimitiveData*> data = (*it).second;
			for(std::vector<MotionPrimitiveData*>::iterator primit = data.begin(); primit != data.end(); primit++) {
				fout << std::setprecision(1) << std::fixed;
				fout << "primID: " << primind << std::endl;
				fout << "startangle_c: " << (unsigned int) (*primit)->getStartingOrientationID() << std::endl;
				fout << "startsteer_c: " << (unsigned int) (*primit)->getStartingSteeringID() << std::endl;
				fout << "additionalactioncostmult: " << (*primit)->getCostMultiplier() << std::endl;
				fout << "motiondirection: " << (*primit)->getMotionDirection() << std::endl;
				fout << "intermediateposes: " << (*primit)->getTrajectory().size() << std::endl;
				fout << std::setprecision(4) << std::fixed;
				std::vector<vehicleSimplePoint*> traj = (*primit)->getTrajectory();
				for (std::vector<vehicleSimplePoint*>::iterator trajit = traj.begin(); trajit != traj.end() ; trajit++) {
					fout << (*trajit)->x << " " << (*trajit)->y << " " <<
							(*trajit)->orient << " " <<  (*trajit)->steering << std::endl;
				}
				primind ++;
			}
		}
	}
	// close the target file
	fout.close();
}
