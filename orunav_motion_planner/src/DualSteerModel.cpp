/**
 * DualSteerModel.cpp
 *
 *  Created on: Jul 11, 2021
 *      Author: michele
 */

#include "orunav_motion_planner/DualSteerModel.h"
#include <iostream>


DualSteerModel::DualSteerModel(std::array<std::string,5>  modelPrimitivesFilenameS, int set) : VehicleModel(modelPrimitivesFilenameS, set) {
	// initialization, then overwritten by the data read from the primitives file
	carFrontLength_ = 0;
	carBackLength_ = 0;
	carMaxSteeringAngle_ = 0;
	sets = set;

	for(int i = 0; i < sets; i++){ //for each set
		motionPrimitivesFilenameS_[i] = std::string();
		motionPrimitiveAdditionalDataFilenameS_[i] = std::string();
		heuristicLTFilenameS_[i] = std::string();
		(motionPrimitivesFilenameS_[i].append(WP::PRIMITIVES_DIR)).append(modelPrimitivesFilenameS[i]).append(".mprim");
		(motionPrimitiveAdditionalDataFilenameS_[i].append(WP::PRIMITIVES_DIR)).append(modelPrimitivesFilenameS[i]).append(".adat");
		(heuristicLTFilenameS_[i].append(WP::TABLES_DIR)).append(modelPrimitivesFilenameS[i]).append(".hst");
		// load heuristic table
		//loadHeuristicTable(heuristicLTFilenameS_[i]);
        currentSet = i;
        this->loadPrimitiveLookupTable();
        totalPrimitivesSets += totalPrimitives;
		totalPrimitives *= 0;
		newEntriesInHT_ = false;
	}
}

DualSteerModel::~DualSteerModel() {
}

std::vector<cellPosition*> DualSteerModel::getCellsOccupiedInPosition(vehicleSimplePoint* p) {

		double base_orient = addAnglesRadians(p->orient, M_PI_2, WP::DECIMAL_APPROXIMATION);

		double x_bottom_centre = p->x - this->getCarBackLength() * cos(p->orient);
		double y_bottom_centre = p->y - this->getCarBackLength() * sin(p->orient);

		// get the vertices of the rectangle
		simplePoint b_l, b_r, t_l, t_r;

		b_l.x = x_bottom_centre - ((this->getWidth() / 2) * cos(base_orient));
		b_l.y = y_bottom_centre - ((this->getWidth() / 2) * sin(base_orient));

		b_r.x = x_bottom_centre + ((this->getWidth() / 2) * cos(base_orient));
		b_r.y = y_bottom_centre + ((this->getWidth() / 2) * sin(base_orient));

		t_l.x = b_l.x + this->getLength() * cos(p->orient);
		t_l.y = b_l.y + this->getLength() * sin(p->orient);

		t_r.x = b_r.x + this->getLength() * cos(p->orient);
		t_r.y = b_r.y + this->getLength() * sin(p->orient);

		std::vector<cellPosition*> cellsOccByPos = getOccupiedCells(b_l, t_l, t_r, b_r, this->getModelGranularity());

		return cellsOccByPos;
}

void DualSteerModel::loadPrimitiveLookupTable() {
	int set = currentSet;
	
	// load the file with the primitives
	if (WP::LOG_LEVEL >= 1) {
		std::ostringstream logLine;
		logLine << "Loading motion primitives: " << motionPrimitivesFilenameS_[set];
		writeLogLine(logLine.str(), "\x1B[34mDualSteerModel \034[0m", WP::LOG_FILE);
	}
	std::string line;
	std::ifstream f(motionPrimitivesFilenameS_[set].c_str(), std::ifstream::in);

	// the cost multiplier of the primitive (e.g., when moving backwards).
	// set to 1 by default
	double costMultiplier = 1;
	// the motion direction of the primitive -- 1 (forward) by default
	int motionDirection = 1;

	// regular expressions to look for in the file
	static const boost::regex poses("^intermediateposes: (.+?)$");
	// the additional cost multiplier
	static const boost::regex costmult("^additionalactioncostmult: (.+?)$");
	// the motion direction
	static const boost::regex motiondir("^motiondirection: (.+?)$");
	// get the resolution for which this model has been built
	static const boost::regex resolution("^resolution_m: (.+?)$");

	// get the number of orientation angles and set the WorldParameter
	static const boost::regex angles("^numberofangles: (.+?)$");
	// get the number of steering angle partitions for this vehicle, calculated as (2*M_PI)/steeringAngleGranularity
	static const boost::regex steeringPartitions("^steeringanglepartitions: (.+?)$");
	// get the number of steering angles supported by the vehicle
	static const boost::regex steeringCardinality("^steeringanglecardinality: (.+?)$");
	// get the primitiveID
	static const boost::regex id("^primID: (.+?)$");
	// get the car front length
	static const boost::regex frontLength("^car_length_front: (.+?)$");
	// get the car front length
	static const boost::regex backLength("^car_length_back: (.+?)$");
	// get the car width
	static const boost::regex width("^car_width: (.+?)$");
	// get the car max steering angle (in radians)
	static const boost::regex maxSteering("^max_steering_radians: (.+?)$");

	boost::smatch what;

	int primID = -1;

	if (f.is_open()) {
		writeLogLine("readingData", "\x1B[36mDualSteerModel \036[0m", WP::LOG_FILE);
		while (f.good()) {
			getline(f, line);

			// dimensions of the car
			boost::regex_match(line, what, frontLength, boost::match_extra);
			if (what[0].matched) {
				this->carFrontLength_ = (double) atof(what[1].str().c_str());
			}

			boost::regex_match(line, what, backLength, boost::match_extra);
			if (what[0].matched) {
				this->carBackLength_ = (double) atof(what[1].str().c_str());
			}

			boost::regex_match(line, what, width, boost::match_extra);
			if (what[0].matched) {
				this->width_ = (double) atof(what[1].str().c_str());
			}

			// max steering angle of the car
			boost::regex_match(line, what, maxSteering, boost::match_extra);
			if (what[0].matched) {
				this->carMaxSteeringAngle_ = (double) atof(what[1].str().c_str());
			}

			// update the orientationAngles_ of this vehicle
			boost::regex_match(line, what, angles, boost::match_extra);
			if (what[0].matched) {
				this->orientationAngles_ = atoi(what[1].str().c_str());
			}

			// update the steeringAnglePartitions_ of this vehicle
			boost::regex_match(line, what, steeringPartitions, boost::match_extra);
			if (what[0].matched) {
				this->steeringAnglePartitions_ = atoi(what[1].str().c_str());
			}

			// update the steeringAngleCardinality_ of this vehicle
			boost::regex_match(line, what, steeringCardinality, boost::match_extra);
			if (what[0].matched) {
				this->steeringAngleCardinality_ = atoi(what[1].str().c_str());
			}

			// update the WP::WORLD_SPACE_GRANULARITY
			boost::regex_match(line, what, resolution, boost::match_extra);
			if (what[0].matched) {
				double worldRes = atof(what[1].str().c_str());
                if (set == 0 || worldRes < WP::WORLD_SPACE_GRANULARITY) {
                    vehicleGranularity_ = worldRes;
                    WP::setWorldSpaceGranularity(vehicleGranularity_);
                }
                else vehicleGranularity_ = WP::WORLD_SPACE_GRANULARITY;
			}

			boost::regex_match(line, what, id, boost::match_extra);
			if (what[0].matched) {
				primID = atoi(what[1].str().c_str());
			}

			// the additional cost multiplier -- if exist is right before the motion
			// direction and the primitive intermediate poses
			boost::regex_match(line, what, costmult, boost::match_extra);
			if (what[0].matched) {
				costMultiplier = atof(what[1].str().c_str());
			}

			// the motion direction -- if exist is right before the primitive intermediate poses
			boost::regex_match(line, what, motiondir, boost::match_extra);
			if (what[0].matched) {
				motionDirection = atoi(what[1].str().c_str());
			}

			// check the number of intermediate poses to read for this particular primitive
			boost::regex_match(line, what, poses, boost::match_extra);
			if (what[0].matched) {
				int intermediatePoints = atoi(what[1].str().c_str());

				// load this primitive and into a MotionPrimitiveData
				std::vector<vehicleSimplePoint*> primitiveTrajectory;
				MotionPrimitiveData* primData = new MotionPrimitiveData;

				// set the primitiveID
				primData->setID(primID);

				// set the cost multiplier (1 by default)
				primData->setCostMultiplier(costMultiplier);

				// set the motion direction (1 -- forward -- by default)
				primData->setMotionDirection(motionDirection);

				for (int i = 0; i < intermediatePoints; i++) {
					getline(f, line);
					vehicleSimplePoint* sp = new vehicleSimplePoint;
					// check the number of intermediate poses to read for this particular primitive
					std::stringstream ss(line);
					// NOTE: we assume already normalized angles
					ss >> sp->x >> sp->y >> sp->orient >> sp->steering >> sp->steeringRear; //Cecchi_add
					primitiveTrajectory.push_back(sp);
				}
				// add the trajectory
				primData->setTrajectory(primitiveTrajectory, orientationAngles_, steeringAnglePartitions_);

				// check where to put this primitive : check if the starting angles (steering and orientation) are present in the lookup table
				motionPrimitivesLookup::iterator it;
				it = modelMotionPrimitivesLTS_[set].find(
						std::pair<uint8_t, uint8_t>(primData->getStartingOrientationID(), primData->getStartingSteeringID()));
				if (it == modelMotionPrimitivesLTS_[set].end()) {
					// no match: create a new entry
					std::vector<MotionPrimitiveData*> primitiveList;
					primitiveList.push_back(primData);
					modelMotionPrimitivesLTS_[set].insert(motionPrimitivesLookupEntry(
							std::pair<uint8_t, uint8_t>(primData->getStartingOrientationID(), primData->getStartingSteeringID()),
							primitiveList));
				} else {
					// append to the right match
					(*it).second.push_back(primData);
				}
			}
		}
	}
	f.close();

	// calculate the total length of the car
	this->length_ = this->carBackLength_ + this->carFrontLength_;
	// get the distance reached by the longest primitive
	double longestPrimitiveReach = 0;

	// now let's see if the file of additional data is there
	f.open(motionPrimitiveAdditionalDataFilenameS_[set].c_str(), std::ifstream::in);
	// if it is not open, let's generate it and open it again
	if (!f.is_open()) {
		this->generatePrimitiveAdditionalData();
	} else {

		while (f.good()) {
			getline(f, line);

			// fail safe for trailing endlines
			if (line.empty()) {
				break;
			}

			unsigned short int primitiveID;
			uint8_t orientID;
			uint8_t steeringID;
			unsigned int input;
			std::istringstream iss(line);
			iss >> input;
			orientID = input;
			iss >> input;
			steeringID = input;
			iss >> primitiveID;

			// check if the primitive is present in the lookup table
			motionPrimitivesLookup::iterator motionit = modelMotionPrimitivesLTS_[set].find(std::pair<uint8_t, uint8_t>(orientID,steeringID));

			if (motionit == modelMotionPrimitivesLTS_[set].end()) {
				std::ostringstream logLine;
				logLine << "WARNING: motion primitive not found in data!! -- " <<
						(unsigned) orientID << "," << (unsigned) steeringID << "," << primitiveID;
				writeLogLine(logLine.str(), "DualSteerModel", WP::LOG_FILE);
			} else {
				std::vector<MotionPrimitiveData*> primvec = motionit->second;
				bool primitiveFound = false;
				
				for (std::vector<MotionPrimitiveData*>::iterator primit = primvec.begin(); primit != primvec.end(); primit++) {
					// we found the primitive to which we want to assign the data
					if ((*primit)->getID() == primitiveID) {
						primitiveFound = true;
						double distance;
						iss >> distance;
						(*primit)->setDistance(distance);

						// calculate the distance reached by the primitive
						double reach = sqrt(pow((*primit)->getXOffset()*this->getModelGranularity(),2) + pow((*primit)->getXOffset()*this->getModelGranularity(),2));
						if (reach > longestPrimitiveReach) {
							longestPrimitiveReach = reach;
						}

						std::vector<cellPosition*> sweptCells;
						std::vector<cellPosition*> occCells;
						bool delimiterFound = false;
						while (iss.good() && !delimiterFound) {
							cellPosition* cell = new cellPosition;
							iss >> cell->x_cell;
							iss >> cell->y_cell;
							// swept cells are over. Now the occupied ones
							
							if (cell->x_cell == CELL_DELIMITER && cell->y_cell == CELL_DELIMITER) {
								delete cell;
								delimiterFound = true;
							} else {
								sweptCells.push_back(cell);
							}
						}
						while (iss.good()) {
							cellPosition* cell = new cellPosition;
							iss >> cell->x_cell;
							iss >> cell->y_cell;
							occCells.push_back(cell);
						}

						(*primit)->setSweptCells(sweptCells);
						(*primit)->setOccCells(occCells);
						(*primit)->setSet(set);
					}
				}
				// line.empty prevents an error from the last endl of the file
				if (!primitiveFound && !line.empty()) {
					std::ostringstream logLine;
					logLine << "WARNING: motion primitive not found in data!! -- " <<
							(unsigned) orientID << "," << (unsigned) steeringID << "," << primitiveID;
					writeLogLine(logLine.str(), "DualSteerModel", WP::LOG_FILE);
				}

			}
		}
	}
	double longDimension = this->getCarBackLength() > this->getCarFrontLength()? this->getCarBackLength() : this->getCarFrontLength();
	interferenceRange_ = longestPrimitiveReach + sqrt(pow(this->getWidth()/2,2) + pow(longDimension,2));
	f.close();

	// once all the primitives have been loaded, we need to prepare the selector table
	
	modelMotionPrimitivesLT_ = modelMotionPrimitivesLTS_[set];
	modelMotionPrimitivesSelectorLT_ = modelMotionPrimitivesSelectorLTS_[set];
	this->prepareMotionPrimitiveSelectorTable();
	modelMotionPrimitivesSelectorLTS_[set] = modelMotionPrimitivesSelectorLT_;
}

void DualSteerModel::generatePrimitiveAdditionalData() {
	int set = currentSet;
	// load the file with the primitives
	if (WP::LOG_LEVEL >= 1) {
		std::ostringstream logLine;
		logLine << "Generating primitives' additional data: " << motionPrimitiveAdditionalDataFilename_;
		writeLogLine(logLine.str(), "DualSteerModel", WP::LOG_FILE);
	}

	int counter = 0;

	// iterate over the existing primitives and generate the additional data  //Cecchi_add_data
	for (motionPrimitivesLookup::iterator it = modelMotionPrimitivesLTS_[set].begin(); it != modelMotionPrimitivesLTS_[set].end(); it++) {
		writeLogLine("addDAta", "\x1B[34mDualSteerModel \034[0m", WP::LOG_FILE);
		counter ++;
		// load the file with the primitives
		if (WP::LOG_LEVEL >= 1) {
			std::ostringstream logLine;
			logLine << "Generating additional data [" << counter << "/" << modelMotionPrimitivesLTS_[set].size() << "]";
			writeLogLine(logLine.str(), "DualSteerModel", WP::LOG_FILE);
		}

		std::vector<MotionPrimitiveData*> mprimdata = (*it).second;
		for (std::vector<MotionPrimitiveData*>::iterator primit = mprimdata.begin(); primit != mprimdata.end(); primit++) {

			// calculate the distance
			(*primit)->setDistance(calculateLength((*primit)->getTrajectory()));

			// now calculate the cells swept and occupied at the end of the motion
			std::vector<cellPosition*> cellsSwept;
			cellsSwept.clear();

			std::vector<vehicleSimplePoint*> traj = (*primit)->getTrajectory();

			for (std::vector<vehicleSimplePoint*>::iterator pointit = traj.begin(); pointit != traj.end(); pointit++) {

				vehicleSimplePoint* p = *pointit;
				std::vector<cellPosition*> cellsOccByPos = this->getCellsOccupiedInPosition(p);

				for (std::vector<cellPosition*>::iterator it = cellsOccByPos.begin(); it != cellsOccByPos.end(); it++ ) {
					bool found = false;
					// is it already registered?
					for (std::vector<cellPosition*>::iterator cellit = cellsSwept.begin(); cellit != cellsSwept.end(); cellit++) {
						if ((*cellit)->x_cell == (*it)->x_cell && (*cellit)->y_cell == (*it)->y_cell) {
							found = true;
							break;
						}
					}
					// add it
					if (!found) {
						cellPosition* cel_pos = new cellPosition;
						cel_pos->x_cell = (*it)->x_cell;
						cel_pos->y_cell = (*it)->y_cell;
						cellsSwept.push_back(cel_pos);
					}
				}
				// cleanup
				for (std::vector<cellPosition*>::iterator it = cellsOccByPos.begin(); it != cellsOccByPos.end(); it++ ) {
					delete (*it);
				}

				// last position: set the final cells occupied
				if (pointit + 1 == traj.end()){
					std::vector<cellPosition*> cellsOccByPos = this->getCellsOccupiedInPosition(p);
					(*primit)->setOccCells(cellsOccByPos);
				}
			}
			(*primit)->setSweptCells(cellsSwept);
		}
	}
	// adjust the distances for the 8-axis symmetry
	modelMotionPrimitivesLT_ = modelMotionPrimitivesLTS_[set];
	this->adjustPrimitiveDistancesWith8AxisSymmetry();
	// save
	std::ofstream f;
	f.open(motionPrimitiveAdditionalDataFilenameS_[set].c_str(), std::ios::app);
	for (motionPrimitivesLookup::iterator it = modelMotionPrimitivesLTS_[set].begin(); it != modelMotionPrimitivesLTS_[set].end(); it++) {
		std::vector<MotionPrimitiveData*> mprimdata = (*it).second;
		for (std::vector<MotionPrimitiveData*>::iterator primit = mprimdata.begin(); primit != mprimdata.end(); primit++) {

			f << (unsigned short int) (*primit)->getStartingOrientationID() << " " <<
					(unsigned short int) (*primit)->getStartingSteeringID() << " " <<
					(*primit)->getID() << " " << (*primit)->getDistanceCovered();

			std::vector<cellPosition*> cells = (*primit)->getSweptCells();
			for (std::vector<cellPosition*>::iterator cellit = cells.begin(); cellit != cells.end(); cellit++) {
				f << " " << (*cellit)->x_cell << " " << (*cellit)->y_cell;
			}
			f << " " << CELL_DELIMITER << " " << CELL_DELIMITER;
			cells.clear();
			cells = (*primit)->getOccCells();
			for (std::vector<cellPosition*>::iterator cellit = cells.begin(); cellit != cells.end(); cellit++) {
				f << " " << (*cellit)->x_cell << " " << (*cellit)->y_cell;
			}

			f << std::endl;
		}
	}
	f.close();
}

double DualSteerModel::getCarFrontLength() {
	return carFrontLength_;
}

double DualSteerModel::getCarBackLength() {
	return carBackLength_;
}

double DualSteerModel::getCarMaxSteeringAngle() {
	return carMaxSteeringAngle_;
}



// std::vector<MotionPrimitiveData*> DualSteerModel::selectApplicablePrimitives(
// 		World* w, short int startXcell, short int startYcell, uint8_t orientationID, uint8_t steeringID) {
// 	std::vector<MotionPrimitiveData*> primitiveSet;
// 	std::string str = "\n";
// 	std::vector<int> setToUse;
// 	setToUse = this ->selectSet();

// 	for (unsigned vec = 0; vec < setToUse.size(); vec++){
// 		int set = setToUse[vec];
		
// 		motionPrimitiveSelectorLookup::iterator it;
// 		it = modelMotionPrimitivesSelectorLTS_[vec] .find(std::pair<uint8_t, uint8_t>(orientationID, steeringID));
// 		if (it == modelMotionPrimitivesSelectorLTS_[vec].end()) {
// 			std::vector<MotionPrimitiveData*> dummy;
// 			std::ostringstream logLine;
// 			logLine << "KEY NOT FOUND [" << (int) orientationID  << "," << (int) steeringID << "]";
// 			writeLogLine(logLine.str(), "VehicleModel", WP::LOG_FILE);
// 			//return dummy;
// 		} else {
// 			char info[150];
// 			//sprintf(info, "set %d : primitives $$ %lu in [%d,%d]\n", set, (*it).second, orientationID,steeringID );
// 			str.append(std::string(info));
// 			//for (unsigned i=0; i<(*it).second; i++){

// 				MotionPrimitiveSelector* s = (*it).second;
// 				std::vector<MotionPrimitiveData*> result = s->getValidPrimitives(w, startXcell, startYcell);
//     			primitiveSet.push_back( result.at(vec) );
// 				std::ostringstream logLine;
// 				logLine << "CHECK!! [" << (int) orientationID  << "," << (int) steeringID << "]";
// 				writeLogLine(logLine.str(), "VehicleModel", WP::LOG_FILE);
			
// 			//}
// 		}
// 	}
// 	char info[150];
// 	sprintf(info, "Total primitives %lu \n", primitiveSet.size() );
// 	str.append(std::string(info));
// 	writeLogLine(str, "DualSteerModel", WP::LOG_FILE);
// 	return primitiveSet;

// }



std::vector<MotionPrimitiveData*>  DualSteerModel::getApplicablePrimitives(uint8_t orientationID, uint8_t steeringID) {
	std::vector<MotionPrimitiveData*> primitiveSet;
	std::string str = "\n";
	std::vector<int> setToUse;
	setToUse = this ->selectSet();
	for (unsigned vec = 0; vec < setToUse.size(); vec++){
		int set = setToUse[vec];
		motionPrimitivesLookup::iterator it;
		it = modelMotionPrimitivesLTS_[set].find(std::pair<uint8_t, uint8_t>(orientationID, steeringID));
		if (it == modelMotionPrimitivesLTS_[set].end()) {
			char info[150];
			sprintf(info, "set %d : KEY NOT FOUND in [%d,%d]\n", set, orientationID,steeringID );
			str.append(std::string(info));
		} else {
			char info[150];
			sprintf(info, "set %d : primitives %lu in [%d,%d]\n", set, (*it).second.size(), orientationID,steeringID );
			str.append(std::string(info));
			for (unsigned i=0; i<(*it).second.size(); i++)
    			primitiveSet.push_back( (*it).second.at(i) );
		}	
	}
	char info[150];
	sprintf(info, "Total primitives %lu \n", primitiveSet.size() );
	str.append(std::string(info));
	//writeLogLine(str, "[DualSteerModel]", WP::LOG_FILE);
	return primitiveSet;
}

std::vector<int> DualSteerModel:: selectSet(){
	std:: vector<int> selectedSet;
	//condition for set 0
	if(sets>0 && true){
		selectedSet.push_back(0);
	}
	//condition for set 1
	if(sets >1 && true){
		selectedSet.push_back(1);
	}
	//condition for set 2
	if(sets >2 && true){
		selectedSet.push_back(2);
	}
	//condition for set 4
	if(sets >3 && true){
		selectedSet.push_back(3);
	}
	//condition for set 5
	if(sets >4 && true){
		selectedSet.push_back(4);
	}
	return selectedSet;
}


