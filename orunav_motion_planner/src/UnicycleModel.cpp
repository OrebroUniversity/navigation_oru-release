/**
 * @file UnicycleModel.cpp
 *
 *  Created on: Mar 29, 2011
 *      Author: marcello
 */

#include "orunav_motion_planner/UnicycleModel.h"

UnicycleModel::UnicycleModel(std::string modelPrimitivesFilename) : 	VehicleModel(modelPrimitivesFilename) {
	this->loadPrimitiveLookupTable();
}

UnicycleModel::~UnicycleModel() {
}

std::vector<cellPosition*> UnicycleModel::getCellsOccupiedInPosition(vehicleSimplePoint* p) {

	double base_orient = addAnglesRadians(p->orient, M_PI_2, WP::DECIMAL_APPROXIMATION);

	double x_bottom_centre = p->x - (this->getLength() / 2) * cos(p->orient);
	double y_bottom_centre = p->y - (this->getLength() / 2) * sin(p->orient);

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


void UnicycleModel::loadPrimitiveLookupTable() {

	// load the file with the primitives
	if (WP::LOG_LEVEL >= 1) {
		std::ostringstream logLine;
		logLine << "Loading motion primitives:  " << motionPrimitivesFilename_;
		writeLogLine(logLine.str(), "UnicycleModel", WP::LOG_FILE);
	}

	std::string line;
	std::ifstream f(motionPrimitivesFilename_.c_str(), std::ifstream::in);

	// the cost multiplier of the primitive (e.g., when moving backwards). Set to 1 by default
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
	// get the number of angles and set the WorldParameter
	static const boost::regex angles("^numberofangles: (.+?)$");
	// get the primitiveID
	static const boost::regex id("^primID: (.+?)$");
	// get the unicycle length
	static const boost::regex length("^unicycle_length: (.+?)$");
	// get the unicycle width
	static const boost::regex width("^unicycle_width: (.+?)$");

	boost::smatch what;

	int primID = -1;

	if (f.is_open()) {
		while (f.good()) {
			getline(f, line);

			// length of the vehicle
			boost::regex_match(line, what, length, boost::match_extra);
			if (what[0].matched) {
				this->length_ = (double) atof(what[1].str().c_str());
			}

			boost::regex_match(line, what, width, boost::match_extra);
			if (what[0].matched) {
				this->width_ = (double) atof(what[1].str().c_str());
			}

			// update the orientationAngles_ of this vehicle
			boost::regex_match(line, what, angles, boost::match_extra);
			if (what[0].matched) {
				orientationAngles_ = atoi(what[1].str().c_str());
			}

			// update the WP::WORLD_SPACE_GRANULARITY
			boost::regex_match(line, what, resolution, boost::match_extra);
			if (what[0].matched) {
				double worldRes = atof(what[1].str().c_str());
				vehicleGranularity_ = worldRes;
				WP::setWorldSpaceGranularity(worldRes);
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
				MotionPrimitiveData* primData = new MotionPrimitiveData();

				// set the primitiveID
				primData->setID(primID);

				// set the cost multiplier (1 by default)
				primData->setCostMultiplier(costMultiplier);

				// set the motion direction (1 -- forward -- by default)
				primData->setMotionDirection(motionDirection);

				for (int i = 0; i < intermediatePoints; i++) {
					getline(f, line);
					vehicleSimplePoint* sp = new vehicleSimplePoint;
					std::stringstream ss(line);
					// NOTE: we assume already normalized angles
					ss >> sp->x >> sp->y >> sp->orient;
					sp->steering = 0;
					primitiveTrajectory.push_back(sp);
				}
				// add the trajectory -- steeringAnglePartitions_ is initialized to 1 by constructor
				primData->setTrajectory(primitiveTrajectory, orientationAngles_, steeringAnglePartitions_);

				// check where to put this primitive : check if the starting angle is present in the lookup table
				motionPrimitivesLookup::iterator it;
				it = modelMotionPrimitivesLT_.find(
						std::pair<uint8_t, uint8_t>(primData->getStartingOrientationID(),primData->getStartingSteeringID()));
				if (it == modelMotionPrimitivesLT_.end()) {
					// no match: create a new entry
					std::vector<MotionPrimitiveData*> primitiveList;
					primitiveList.push_back(primData);

					modelMotionPrimitivesLT_.insert(motionPrimitivesLookupEntry(
							std::pair<uint8_t, uint8_t>(primData->getStartingOrientationID(),primData->getStartingSteeringID()),
							primitiveList));
				} else {
					// append to the right match
					(*it).second.push_back(primData);
				}
			}
		}
	}
	f.close();

	// get the distance reached by the longest primitive
	double longestPrimitiveReach = 0;

	// now let's see if the file of additional data is there
	f.open(motionPrimitiveAdditionalDataFilename_.c_str(), std::ifstream::in);
	// if it is not open, let's generate it
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
			std::pair<uint8_t, uint8_t> p = std::pair<uint8_t, uint8_t>(orientID,steeringID);
			motionPrimitivesLookup::iterator motionit = modelMotionPrimitivesLT_.find(p);

			if (motionit == modelMotionPrimitivesLT_.end()) {
				std::ostringstream logLine;
				logLine << "WARNING: motion primitive not found in data!! -- " <<
						(unsigned) orientID << "," << (unsigned) steeringID << "," << primitiveID;
				writeLogLine(logLine.str(), "UnicycleModel", WP::LOG_FILE);
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
					}
				}
				// line.empty prevents an error from the last endl of the file
				if (!primitiveFound && !line.empty()) {
					std::ostringstream logLine;
					logLine << "WARNING: motion primitive not found in data!! -- " <<
							(unsigned) orientID << "," << (unsigned) steeringID << "," << primitiveID;
					writeLogLine(logLine.str(), "UnicycleModel", WP::LOG_FILE);
				}
			}
		}
	}
	interferenceRange_ = longestPrimitiveReach + sqrt(pow(this->getWidth()/2,2) + pow(this->getLength()/2,2));
	f.close();

	// once all the primitives have been loaded, we need to prepare the selector table
	this->prepareMotionPrimitiveSelectorTable();
}

void UnicycleModel::generatePrimitiveAdditionalData() {

	// load the file with the primitives
	if (WP::LOG_LEVEL >= 1) {
		std::ostringstream logLine;
		logLine << "Generating primitives' additional data: " << motionPrimitiveAdditionalDataFilename_;
		writeLogLine(logLine.str(), "UnicycleModel", WP::LOG_FILE);
	}

	int counter = 0;

	// iterate over the existing primitives and generate the additional data
	for (motionPrimitivesLookup::iterator it = modelMotionPrimitivesLT_.begin(); it != modelMotionPrimitivesLT_.end(); it++) {

		counter ++;
		// load the file with the primitives
		if (WP::LOG_LEVEL >= 1) {
			std::ostringstream logLine;
			logLine << "Generating additional data [" << counter << "/" << modelMotionPrimitivesLT_.size() << "]";
			writeLogLine(logLine.str(), "UnicycleModel", WP::LOG_FILE);
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
	this->adjustPrimitiveDistancesWith8AxisSymmetry();

	// save
	std::ofstream f;
	f.open(motionPrimitiveAdditionalDataFilename_.c_str(), std::ios::app);
	for (motionPrimitivesLookup::iterator it = modelMotionPrimitivesLT_.begin(); it != modelMotionPrimitivesLT_.end(); it++) {
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

