/**
 * @file VehicleModel.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Mar 29, 2011
 *      Author: marcello
 */

#include "orunav_motion_planner/VehicleModel.h"

VehicleModel::VehicleModel(std::string modelPrimitivesFilename) {
	// temporary values
	width_ = -1;
	length_ = -1;
	vehicleGranularity_ = -1;
	interferenceRange_ = -1;

	// by default, we consider a single steering angle and a single partition
	steeringAngleCardinality_ = 1;
	steeringAnglePartitions_ = 1;
	orientationAngles_ = 1;
	// the files with primitives, additional data and heuristic values
	motionPrimitivesFilename_ = std::string();
	motionPrimitiveAdditionalDataFilename_ = std::string();
	heuristicLTFilename_ = std::string();
	(motionPrimitivesFilename_.append(WP::PRIMITIVES_DIR)).append(modelPrimitivesFilename).append(".mprim");
	(motionPrimitiveAdditionalDataFilename_.append(WP::PRIMITIVES_DIR)).append(modelPrimitivesFilename).append(".adat");
	(heuristicLTFilename_.append(WP::TABLES_DIR)).append(modelPrimitivesFilename).append(".hst");
	// load heuristic table
	loadHeuristicTable(heuristicLTFilename_);
	newEntriesInHT_ = false;
}

VehicleModel::~VehicleModel() {
	// save the table we used
	if (newEntriesInHT_) {
		this->saveHeuristicTable();
	}
	for (motionPrimitivesLookup::iterator it = modelMotionPrimitivesLT_.begin(); it != modelMotionPrimitivesLT_.end(); it++) {
		std::vector<MotionPrimitiveData*> data = (*it).second;
		for (std::vector<MotionPrimitiveData*>::iterator primIt = data.begin(); primIt != data.end(); primIt++) {
			delete *primIt;
		}
	}
	for (motionPrimitiveSelectorLookup::iterator it = modelMotionPrimitivesSelectorLT_.begin();
			it != modelMotionPrimitivesSelectorLT_.end(); it++) {
		MotionPrimitiveSelector* s = (*it).second;
		delete s;
	}
	modelHeuristicLT_.clear();
	modelMotionPrimitivesLT_.clear();
	modelMotionPrimitivesSelectorLT_.clear();
}

VehicleModel::motionPrimitivesLookup VehicleModel::getPrimitivesLookupTable() {
	return modelMotionPrimitivesLT_;
}

void VehicleModel::setPrimitivesLookupTable(VehicleModel::motionPrimitivesLookup prim) {
	// set a new primitiveLookupTable and re-calculate the motion primitive selector
	modelMotionPrimitivesLT_.clear();
	modelMotionPrimitivesLT_ = prim;

	// recalculate the selector -- if the expansion method so requires
	if (WP::NODE_EXPANSION_METHOD == WP::NodeExpansionMethod::FSG || WP::NODE_EXPANSION_METHOD == WP::NodeExpansionMethod::EPFSG) {
		for (motionPrimitiveSelectorLookup::iterator it = modelMotionPrimitivesSelectorLT_.begin();
				it != modelMotionPrimitivesSelectorLT_.end(); it++) {
			MotionPrimitiveSelector* s = (*it).second;
			delete s;
		}
		this->prepareMotionPrimitiveSelectorTable();
	}
}

void VehicleModel::prepareMotionPrimitiveSelectorTable() {
	modelMotionPrimitivesSelectorLT_.clear();
	// for each entry in the modelMotionPrimitivesLT_, create a corresponding entry into the
	// modelMotionPrimitiveSelectorLT_
	for (motionPrimitivesLookup::iterator it = modelMotionPrimitivesLT_.begin(); it != modelMotionPrimitivesLT_.end(); it ++ ) {
		MotionPrimitiveSelector* selector = new MotionPrimitiveSelector((*it).second);
		modelMotionPrimitivesSelectorLT_.insert(
				motionPrimitiveSelectorLookupEntry(std::pair<uint8_t, uint8_t>((*it).first.first,(*it).first.second), selector));
	}
}

void VehicleModel::adjustPrimitiveDistancesWith8AxisSymmetry() {
	// now apply the symmetry to UNIFORM the costs (distance):
	std::vector<MotionPrimitiveData*> origPrimitives;
	std::vector<MotionPrimitiveData*> symmPrimitives;
	VehicleModel::motionPrimitivesLookup::iterator originalit;
	VehicleModel::motionPrimitivesLookup::iterator symmetricit;
	uint8_t symmOrientID, symmSteerID;

	// ----------------------------------------
	// second sector: (pi/4 - pi/2]
	// ----------------------------------------

	for (uint8_t origOrientID = 0; origOrientID < orientationAngles_ / 8; origOrientID++) {
		// a bit convoluted, but should account for all the steering angles allowed
		for (uint8_t origSteerID = ((steeringAnglePartitions_ - ((steeringAngleCardinality_ - 1) / 2)) % steeringAnglePartitions_);
				origSteerID != ((steeringAngleCardinality_ - 1) / 2) + 1; origSteerID = origSteerID + 1) {

			origSteerID = origSteerID % steeringAnglePartitions_;

			originalit = modelMotionPrimitivesLT_.find(std::pair<uint8_t, uint8_t>(origOrientID, origSteerID));

			// does this combination of orientation and steering have primitives? If so, conform the costs
			if (originalit != modelMotionPrimitivesLT_.end()) {
				symmOrientID = (orientationAngles_/4) - origOrientID;
				symmSteerID = (steeringAnglePartitions_ - origSteerID) % steeringAnglePartitions_;
				symmetricit = modelMotionPrimitivesLT_.find(std::pair<uint8_t, uint8_t>(symmOrientID,symmSteerID));

				if (symmetricit == modelMotionPrimitivesLT_.end()) {
					std::ostringstream log_line;
					log_line << "Symmetric primitives not found (second sector)! ["
							<< (unsigned) symmOrientID << ", " << (unsigned)  symmSteerID << "]";
					writeLogLine(log_line.str(), "VehicleModel", WP::LOG_FILE);
					exit(0);
				}
				origPrimitives = (*originalit).second;
				symmPrimitives = (*symmetricit).second;
				// locate the new symmetric primitive by ID
				for(std::vector<MotionPrimitiveData*>::iterator primit = origPrimitives.begin(); primit != origPrimitives.end(); primit++) {
					bool primitiveFound = false;
					for(std::vector<MotionPrimitiveData*>::iterator symmprimit = symmPrimitives.begin();
							symmprimit != symmPrimitives.end(); symmprimit++) {
						if ((*primit)->getID() == (*symmprimit)->getID()) {
							// we should have found it. Check and copy the distance
							if ((*primit)->getCostMultiplier() != (*symmprimit)->getCostMultiplier() ||
									fabs((*primit)->getDistanceCovered() - (*symmprimit)->getDistanceCovered()) > (*primit)->getDistanceCovered() * 0.05) {
								if (WP::LOG_LEVEL >= 1) {
									std::ostringstream log_line;
									log_line << "ERROR! Primitive mismatch. " <<
											"o. orientID: " << (unsigned) origOrientID << " o. steerID: " << (unsigned) origSteerID <<
											"s. orientID: " << (unsigned) symmOrientID << " s. steerID: " << (unsigned) symmSteerID <<
											"\to. ID: " << (*primit)->getID() << " r. ID: " << (*symmprimit)->getID();
									writeLogLine(log_line.str(), "VehicleModel", WP::LOG_FILE);
								}
							} else {
								primitiveFound = true;
								(*symmprimit)->setDistance((*primit)->getDistanceCovered());
							}
						}
					}
					if (!primitiveFound) {
						std::ostringstream log_line;
						log_line << "Primitive not found (second sector)! [" << (unsigned) origOrientID
								<< "," << (unsigned) origSteerID << "]" << " ID: " << (*primit)->getID() << ")";
						writeLogLine(log_line.str(), "VehicleModel", WP::LOG_FILE);
						exit(0);
					}
				}
			}
		}
	}

	// ----------------------------------------
	//  (pi/2 - pi]
	// ----------------------------------------

	for (uint8_t origOrientID = 0; origOrientID < orientationAngles_ / 4; origOrientID++) {
		// a bit convoluted, but should account for all the steering angles allowed
		for (uint8_t origSteerID = ((steeringAnglePartitions_ - ((steeringAngleCardinality_ - 1) / 2)) % steeringAnglePartitions_);
				origSteerID != ((steeringAngleCardinality_ - 1) / 2) + 1; origSteerID = origSteerID + 1) {

			origSteerID = origSteerID % steeringAnglePartitions_;

			originalit = modelMotionPrimitivesLT_.find(std::pair<uint8_t, uint8_t>(origOrientID, origSteerID));

			// does this combination of orientation and steering have primitives? If so, conform the costs
			if (originalit != modelMotionPrimitivesLT_.end()) {
				symmOrientID = (orientationAngles_/2) - origOrientID;
				symmSteerID = (steeringAnglePartitions_ - origSteerID) % steeringAnglePartitions_;
				symmetricit = modelMotionPrimitivesLT_.find(std::pair<uint8_t, uint8_t>(symmOrientID,symmSteerID));

				if (symmetricit == modelMotionPrimitivesLT_.end()) {
					std::ostringstream log_line;
					log_line << "Symmetric primitives not found (third sector)! ["
							<< (unsigned) symmOrientID << ", " << (unsigned) symmSteerID << "]";
					writeLogLine(log_line.str(), "VehicleModel", WP::LOG_FILE);
					exit(0);
				}
				origPrimitives = (*originalit).second;
				symmPrimitives = (*symmetricit).second;
				// locate the new symmetric primitive by ID
				for(std::vector<MotionPrimitiveData*>::iterator primit = origPrimitives.begin(); primit != origPrimitives.end(); primit++) {
					bool primitiveFound = false;
					for(std::vector<MotionPrimitiveData*>::iterator symmprimit = symmPrimitives.begin();
							symmprimit != symmPrimitives.end(); symmprimit++) {
						if ((*primit)->getID() == (*symmprimit)->getID()) {
							// we should have found it. Check and copy the distance
							if ((*primit)->getCostMultiplier() != (*symmprimit)->getCostMultiplier() ||
									fabs((*primit)->getDistanceCovered() - (*symmprimit)->getDistanceCovered()) > (*primit)->getDistanceCovered() * 0.05) {
								if (WP::LOG_LEVEL >= 1) {
									std::ostringstream log_line;
									log_line << "ERROR! Primitive mismatch. " <<
											"o. orientID: " << (unsigned) origOrientID << " o. steerID: " << (unsigned) origSteerID <<
											"s. orientID: " << (unsigned) symmOrientID << " s. steerID: " << (unsigned) symmSteerID <<
											"\to. ID: " << (*primit)->getID() << " r. ID: " << (*symmprimit)->getID();
									writeLogLine(log_line.str(), "VehicleModel", WP::LOG_FILE);
								}
							} else {
								primitiveFound = true;
								(*symmprimit)->setDistance((*primit)->getDistanceCovered());
							}
						}
					}
					if (!primitiveFound) {
						std::ostringstream log_line;
						log_line << "Primitive not found (third sector)! [" << (unsigned) origOrientID
								<< "," << (unsigned) origSteerID << "]" << " ID: " << (*primit)->getID() << ")";
						writeLogLine(log_line.str(), "VehicleModel", WP::LOG_FILE);
						exit(0);
					}
				}
			}
		}
	}
	// ----------------------------------------
	// (pi - 2*pi]
	// ----------------------------------------
	for (uint8_t origOrientID = 1; origOrientID < orientationAngles_ / 2; origOrientID++) {
		// a bit convoluted, but should account for all the steering angles allowed
		for (uint8_t origSteerID = ((steeringAnglePartitions_ - ((steeringAngleCardinality_ - 1) / 2)) % steeringAnglePartitions_);
				origSteerID != ((steeringAngleCardinality_ - 1) / 2) + 1; origSteerID = origSteerID + 1) {

			origSteerID = origSteerID % steeringAnglePartitions_;

			originalit = modelMotionPrimitivesLT_.find(std::pair<uint8_t, uint8_t>(origOrientID, origSteerID));

			// does this combination of orientation and steering have primitives? If so, conform the costs
			if (originalit != modelMotionPrimitivesLT_.end()) {
				symmOrientID = orientationAngles_ - origOrientID;
				symmSteerID = (steeringAnglePartitions_ - origSteerID) % steeringAnglePartitions_;
				symmetricit = modelMotionPrimitivesLT_.find(std::pair<uint8_t, uint8_t>(symmOrientID,symmSteerID));

				if (symmetricit == modelMotionPrimitivesLT_.end()) {
					std::ostringstream log_line;
					log_line << "Symmetric primitives not found (fourth sector)! ["
							<< (unsigned) symmOrientID << ", " << (unsigned) symmSteerID << "]";
					writeLogLine(log_line.str(), "VehicleModel", WP::LOG_FILE);
					exit(0);
				}
				origPrimitives = (*originalit).second;
				symmPrimitives = (*symmetricit).second;
				// locate the new symmetric primitive by ID
				for(std::vector<MotionPrimitiveData*>::iterator primit = origPrimitives.begin(); primit != origPrimitives.end(); primit++) {
					bool primitiveFound = false;
					for(std::vector<MotionPrimitiveData*>::iterator symmprimit = symmPrimitives.begin();
							symmprimit != symmPrimitives.end(); symmprimit++) {
						if ((*primit)->getID() == (*symmprimit)->getID()) {
							// we should have found it. Check and copy the distance
							if ((*primit)->getCostMultiplier() != (*symmprimit)->getCostMultiplier() ||
									fabs((*primit)->getDistanceCovered() - (*symmprimit)->getDistanceCovered()) > (*primit)->getDistanceCovered() * 0.05) {
								if (WP::LOG_LEVEL >= 1) {
									std::ostringstream log_line;
									log_line << "ERROR! Primitive mismatch. " <<
											"o. orientID: " << (unsigned) origOrientID << " o. steerID: " << (unsigned) origSteerID <<
											"s. orientID: " << (unsigned) symmOrientID << " s. steerID: " << (unsigned) symmSteerID <<
											"\to. ID: " << (*primit)->getID() << " r. ID: " << (*symmprimit)->getID();
									writeLogLine(log_line.str(), "VehicleModel", WP::LOG_FILE);
								}
							} else {
								primitiveFound = true;
								(*symmprimit)->setDistance((*primit)->getDistanceCovered());
							}
						}
					}
					if (!primitiveFound) {
						std::ostringstream log_line;
						log_line << "Primitive not found (fourth sector)! [" << (unsigned) origOrientID << ","
								<< (unsigned) origSteerID << "]" << " ID: " << (*primit)->getID() << ")";
						writeLogLine(log_line.str(), "VehicleModel", WP::LOG_FILE);
						exit(0);
					}
				}
			}
		}
	}
}


void VehicleModel::loadHeuristicTable(std::string filename) {
	heuristicLTFilename_ = filename;
	if (WP::LOG_LEVEL >= 1) {
		std::ostringstream logLine;
		logLine << "Loading file:  " << heuristicLTFilename_;
		writeLogLine(logLine.str(), "VehicleModel", WP::LOG_FILE);
	}
	std::string line;
	std::ifstream f(heuristicLTFilename_.c_str(), std::ifstream::in);
	static const boost::regex entry("^(.+?)\t(.+?)$");
	boost::smatch what;
	if (f.is_open()) {
		while (f.good()) {
			getline(f, line);
			boost::regex_match(line, what, entry, boost::match_extra);
			if (what[0].matched) {
				uint64_t key;
				std::istringstream(what[1].str()) >> std::hex >> key;
				modelHeuristicLT_[key] = atof(what[2].str().c_str());
			}
		}
	}
	f.close();
}

void VehicleModel::saveHeuristicTable() {
	std::ofstream f;
	f.open(heuristicLTFilename_.c_str(), std::ios::trunc);
	for (heuristicLookupTable::iterator it = modelHeuristicLT_.begin(); it != modelHeuristicLT_.end(); it++) {
		f << std::hex << it->first << "\t" << it->second << std::endl;
	}
	f.close();
}

double VehicleModel::getWidth() {
	return width_;
}

double VehicleModel::getLength() {
	return length_;
}

double VehicleModel::getInterferenceRange() {
	return interferenceRange_;
}

double VehicleModel::getModelGranularity() {
	return vehicleGranularity_;
}

uint8_t VehicleModel::getOrientationAngles() {
	return orientationAngles_;
}

uint8_t VehicleModel::getSteeringAngleCardinality() {
	return steeringAngleCardinality_;
}

uint8_t VehicleModel::getSteeringAnglePartitions() {
	return steeringAnglePartitions_;
}

uint8_t VehicleModel::getClosestAllowedOrientationID(double angle) {
	// normalize
	angle = atan2(sin(angle), cos(angle));
	if (angle < 0 ) {
		angle = 2*M_PI + angle;
	}

	double orientationGranularity = (2*M_PI) / orientationAngles_;
	uint8_t id;
	// check which way to round
	if (fmod(angle, orientationGranularity) > (orientationGranularity / 2)) {
		id = ceil(angle/orientationGranularity);
	} else {
		id = floor(angle/orientationGranularity);
	}

	// prevents weird behavior if the angle is negative and close to 0
	id = id % orientationAngles_;

	return id;
}


uint8_t VehicleModel::getClosestAllowedSteeringID(double angle) {

	if (steeringAngleCardinality_ == 1) {
		return 0;
	}

	// if not trivial, normalize
	angle = atan2(sin(angle), cos(angle));
	if (angle < 0 ) {
		angle = 2*M_PI + angle;
	}
	double steeringGranularity = (2*M_PI) / steeringAnglePartitions_;
	uint8_t id;
	// check which way to round
	if (fmod(angle, steeringGranularity) > (steeringGranularity / 2)) {
		id = ceil(angle/steeringGranularity);
	} else {
		id = floor(angle/steeringGranularity);
	}

	// prevents weird behavior if the angle is negative and close to 0
	/** @todo check */
	id = id % steeringAnglePartitions_;

	// and now we check if the id falls into the allowed steering angles
	if (id <= ((steeringAngleCardinality_-1)/2) || id >= (steeringAnglePartitions_ - ((steeringAngleCardinality_-1)/2))) {
		return id;
	} else if (id <= steeringAnglePartitions_/2) {
		return (steeringAngleCardinality_-1)/2;
	}
	return (steeringAnglePartitions_ - ((steeringAngleCardinality_-1)/2));
}


std::string VehicleModel::getModelPrimitivesFilename() {
	return motionPrimitivesFilename_;
}


std::vector<MotionPrimitiveData*> VehicleModel::getApplicablePrimitives(uint8_t orientationID, uint8_t steeringID) {
	motionPrimitivesLookup::iterator it;
	it = modelMotionPrimitivesLT_.find(std::pair<uint8_t, uint8_t>(orientationID, steeringID));
	if (it == modelMotionPrimitivesLT_.end()) {
		std::vector<MotionPrimitiveData*> dummy;
		std::ostringstream logLine;
		logLine << "KEY NOT FOUND [" << (int) orientationID  << "," << (int) steeringID << "]";
		writeLogLine(logLine.str(), "VehicleModel", WP::LOG_FILE);
		return dummy;
	} else {
		return (*it).second;
	}
}



std::vector<MotionPrimitiveData*> VehicleModel::selectApplicablePrimitives(
		World* w, short int startXcell, short int startYcell, uint8_t orientationID, uint8_t steeringID) {
	motionPrimitiveSelectorLookup::iterator it;
	it = modelMotionPrimitivesSelectorLT_.find(std::pair<uint8_t, uint8_t>(orientationID, steeringID));
	if (it == modelMotionPrimitivesSelectorLT_.end()) {
		std::vector<MotionPrimitiveData*> dummy;
		std::ostringstream logLine;
		logLine << "KEY NOT FOUND [" << (int) orientationID  << "," << (int) steeringID << "]";
		writeLogLine(logLine.str(), "VehicleModel", WP::LOG_FILE);
		return dummy;
	} else {
		MotionPrimitiveSelector* s = (*it).second;
		std::vector<MotionPrimitiveData*> result = s->getValidPrimitives(w, startXcell, startYcell);
		return result;
	}
}




uint64_t VehicleModel::calculateHeuristicTableKey(
		unsigned short int startX, unsigned short int startY, uint8_t startOrientID, uint8_t startSteeringID,
		unsigned short int goalX, unsigned short int goalY, uint8_t goalOrientID, uint8_t goalSteeringID){

	short int dx = goalX - startX;
	short int dy = goalY - startY;
	// first symmetry -- pi/2pi or 0/-pi
	if (startOrientID > (orientationAngles_/2)) {
		dy = -dy;
		startOrientID = orientationAngles_ - startOrientID;
		goalOrientID = goalOrientID == 0? goalOrientID : (orientationAngles_ - goalOrientID);
		startSteeringID = (steeringAnglePartitions_ - startSteeringID) % steeringAnglePartitions_;
		goalSteeringID = (steeringAnglePartitions_ - goalSteeringID) % steeringAnglePartitions_;
	}
	// second symmetry -- pi/2 / pi
	if (startOrientID > (orientationAngles_ / 4)) {
		dx = -dx;
		startOrientID = (orientationAngles_/2) - startOrientID;
		goalOrientID = (orientationAngles_ + (orientationAngles_ / 2) - goalOrientID) % orientationAngles_;
		startSteeringID = (steeringAnglePartitions_ - startSteeringID) % steeringAnglePartitions_;
		goalSteeringID = (steeringAnglePartitions_ - goalSteeringID) % steeringAnglePartitions_;
	}
	// third symmetry -- pi/4 / pi/2
	if (startOrientID > (orientationAngles_ / 8)) {
		int temp = dx;
		dx = dy;
		dy = temp;
		startOrientID = (orientationAngles_/4) - startOrientID;
		goalOrientID = (orientationAngles_ + (orientationAngles_ / 4) - goalOrientID) % orientationAngles_;
		startSteeringID = (steeringAnglePartitions_ - startSteeringID) % steeringAnglePartitions_;
		goalSteeringID = (steeringAnglePartitions_ - goalSteeringID) % steeringAnglePartitions_;
	}

	uint64_t key = 0;
	key = key | (uint16_t) dx;
	key = key << 16;
	key = key | (uint16_t) dy;
	key = key << 8;
	key = key | (uint8_t) startOrientID;
	key = key << 8;
	key = key | (uint8_t) goalOrientID;
	key = key << 8;
	key = key | (uint8_t) startSteeringID;
	key = key << 8;
	key = key | (uint8_t) goalSteeringID;
	return key;
}


void VehicleModel::setEntryInHeuristicTable(
		unsigned short int startX, unsigned short int startY, uint8_t startOrientID, uint8_t startSteeringID,
		unsigned short int goalX, unsigned short int goalY, uint8_t goalOrientID, uint8_t goalSteeringID, double value) {

	uint64_t key = calculateHeuristicTableKey(startX, startY, startOrientID, startSteeringID,
			goalX, goalY, goalOrientID, goalSteeringID);

	// first check if the value is not already in the table
	heuristicLookupTable::iterator it;
	it = modelHeuristicLT_.find(key);

	if (it == modelHeuristicLT_.end()) {
		modelHeuristicLT_.insert(heuristicLookupEntry(key, value));
		newEntriesInHT_ = true;
		if (WP::LOG_LEVEL >= 2) {
			std::ostringstream line;
			line << "[" << std::hex << key << "] = " << value;
			writeLogLine(line.str(), "VehicleModel", WP::LOG_FILE);
			line.clear();
		}
	}
}


double VehicleModel::getHeuristicValueFromTable(
		unsigned short int startX, unsigned short int startY, uint8_t startOrientID, uint8_t startSteeringID,
		unsigned short int goalX, unsigned short int goalY, uint8_t goalOrientID, uint8_t goalSteeringID) {

	uint64_t key = calculateHeuristicTableKey(startX, startY, startOrientID, startSteeringID,
			goalX, goalY, goalOrientID, goalSteeringID);

	// check if the element is present in the lookup table
	heuristicLookupTable::iterator it;
	it = modelHeuristicLT_.find(key);
	if (it == modelHeuristicLT_.end()) {
		return -1;
	}
	return it->second;
}


bool VehicleModel::isHeuristicValueInTable(
		unsigned short int startX, unsigned short int startY, uint8_t startOrientID, uint8_t startSteeringID,
		unsigned short int goalX, unsigned short int goalY, uint8_t goalOrientID, uint8_t goalSteeringID) {

	uint64_t key = calculateHeuristicTableKey(startX, startY, startOrientID, startSteeringID,
			goalX, goalY, goalOrientID, goalSteeringID);

	// check if the element is present in the lookup table
	heuristicLookupTable::iterator it;
	it = modelHeuristicLT_.find(key);
	if (it == modelHeuristicLT_.end()) {
		return false;
	}
	return true;
}
