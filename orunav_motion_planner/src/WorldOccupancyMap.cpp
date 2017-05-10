/**
 * @file WorldOccupancyMap.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Apr 6, 2011
 *      Author: marcello
 */

#include "orunav_motion_planner/WorldOccupancyMap.h"

WorldOccupancyMap::WorldOccupancyMap(std::string filename) {

	std::string log = std::string("Loading Map from file ");
	if (WP::LOG_LEVEL >= 1) {
		log.append(filename);
		writeLogLine(log, "WorldOccupancyMap", WP::LOG_FILE);
	}

	std::string line;
	std::ifstream map(filename.c_str(), std::ifstream::in);
	static const boost::regex header("^# (\\w+)\\s(.*)$");
	// static const boost::regex data("^(\\d(.+?)\\s)+$");
	static const boost::regex data("^\\d(.+)$");
	boost::smatch what;

	// init all the private variables
	xcells_ = 0;
	ycells_ = 0;
	originalxcells_ = 0;
	originalycells_ = 0;
	originalOccupancyMap_.clear();

	// two copies of the map are loaded: one copy (originalOccupancyMap)
	// will not change through the lifetime of the object, while
	// the other (occupancyMap) can be modified
	if (map.is_open()) {
		while (map.good()) {
			getline(map, line);

			boost::regex_match(line, what, header, boost::match_extra);
			if (what[0].matched) {
				if (what[1].str().compare("GRID_SIZE") == 0) {
					granularity_ = atof(what[2].str().c_str());
					originalGranularity_ = granularity_;
				}
				if (what[1].str().compare("WIDTH") == 0) {
					xcells_ = atoi(what[2].str().c_str());
				}
				if (what[1].str().compare("HEIGHT") == 0) {
					ycells_ = atoi(what[2].str().c_str());
				}
			}
			// data line
			boost::regex_match(line, what, data, boost::match_extra);
			if (what[0].matched) {
				std::vector<double> x_line = splitMapLine(line, " ");
				occupancyMap_.push_back(x_line);
			}
		}
		map.close();
		// invert the y axis
		reverse(occupancyMap_.begin(), occupancyMap_.end());
	} else {
		if (WP::LOG_LEVEL >= 1) {
			log = std::string("Unable to open file!");
			writeLogLine(log, "WorldOccupancyMap", WP::LOG_FILE);
		}
	}
	if (xcells_ == 0 || ycells_ == 0) {
		if (WP::LOG_LEVEL >= 1) {
			log = std::string("Missing information from file!");
			writeLogLine(log, "WorldOccupancyMap", WP::LOG_FILE);
		}
	}
	// fill in the originalObstacles_ map
	originalObstacles_.resize(ycells_);
	for (unsigned int y = 0; y < originalObstacles_.size(); y++) {
		originalObstacles_[y].resize(xcells_);
		for (unsigned int x = 0; x < originalObstacles_[y].size(); x++) {
			originalObstacles_[y][x] = occupancyMap_[y][x];
		}
	}
	// the map contains obstacles
	containsObstacles_ = true;
}

WorldOccupancyMap::WorldOccupancyMap(unsigned short int xcells, unsigned short int ycells, double mapGranularity){

	// init all the private variables
	xcells_ = xcells;
	ycells_ = ycells;
	originalxcells_ = 0;
	originalycells_ = 0;
	originalOccupancyMap_ .clear();
	originalGranularity_ = mapGranularity;
	granularity_ = mapGranularity;

	// fill in the originalObstacles_ map and the occupancyMap_
	occupancyMap_.resize(ycells_);
	originalObstacles_.resize(ycells_);
	for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
		occupancyMap_[y].resize(xcells_);
		originalObstacles_[y].resize(xcells_);
		for (unsigned int x = 0; x < occupancyMap_[y].size(); x++) {
			occupancyMap_[y][x] = 0;
			originalObstacles_[y][x] = 0;
		}
	}

	// the map does not contain obstacles
	containsObstacles_ = false;
}

WorldOccupancyMap::~WorldOccupancyMap() {

}

double WorldOccupancyMap::getXSize() {
	return xcells_ * granularity_;
}

double WorldOccupancyMap::getYSize() {
	return ycells_ * granularity_;
}

unsigned short int WorldOccupancyMap::getXCells() {
	return xcells_;
}

unsigned short int WorldOccupancyMap::getYCells() {
	return ycells_;
}

double WorldOccupancyMap::getGranularity() {
	return granularity_;
}

std::vector<double> WorldOccupancyMap::splitMapLine(const std::string& s, const std::string& f) {
	std::vector<double> temp;
	if (f.empty()) {
		temp.push_back(atof(s.c_str()));
		return temp;
	}
	typedef std::string::const_iterator iter;
	const iter::difference_type f_size(distance(f.begin(), f.end()));
	iter i(s.begin());
	int count = 0;
	for (iter pos; (pos = search(i, s.end(), f.begin(), f.end())) != s.end();) {
		temp.push_back(atof(std::string(i, pos).c_str()));
		advance(pos, f_size);
		i = pos;
		count++;
	}
	return temp;
}

void WorldOccupancyMap::scaleGranularity(double newGran) {

	// save the original map, if not already done
	if (originalOccupancyMap_.size() == 0) {
		originalOccupancyMap_.resize(occupancyMap_.size());
		for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
			originalOccupancyMap_[y].resize(occupancyMap_[y].size());
			for (unsigned int x = 0; x < occupancyMap_[y].size(); x++) {
				originalOccupancyMap_[y][x] = occupancyMap_[y][x];
			}
		}
		originalxcells_ = xcells_;
		originalycells_ = ycells_;
	}

	double initialGran = granularity_;
	if (fabs(initialGran - newGran) <= WP::CALCULATION_APPROXIMATION_ERROR) {
		return;
	}
	if (initialGran > newGran) {
		this->decreaseGranularity(ceil(initialGran / newGran));
	} else {
		this->increaseGranularity(ceil(newGran / initialGran));
	}

	// clear the current originalObstacles_ map and resize it
	for (unsigned int y = 0; y < originalObstacles_.size(); y++) {
		originalObstacles_[y].clear();
	}
	originalObstacles_.clear();
	// fill in the originalObstacles_ map
	originalObstacles_.resize(ycells_);
	for (unsigned int y = 0; y < originalObstacles_.size(); y++) {
		originalObstacles_[y].resize(xcells_);
		for (unsigned int x = 0; x < originalObstacles_[y].size(); x++) {
			originalObstacles_[y][x] = occupancyMap_[y][x];
		}
	}
}

void WorldOccupancyMap::decreaseGranularity(int times) {
	// copy the current granularity map to a temporary one
	std::vector<std::vector<double> > temp;
	temp.resize(occupancyMap_.size());
	for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
		temp[y].resize(occupancyMap_[y].size());
	}
	for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
		for (unsigned int x = 0; x < occupancyMap_[y].size(); x++) {
			temp[y][x] = occupancyMap_[y][x];
		}
	}
	// clear the current occupancy map and resize it
	for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
		occupancyMap_[y].clear();
	}
	occupancyMap_.clear();
	occupancyMap_.resize(temp.size() * times);
	for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
		occupancyMap_[y].resize(temp[0].size() * times);
	}
	// fill the values using the originalOccupancyMap
	for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
		for (unsigned int x = 0; x < occupancyMap_[y].size(); x++) {
			occupancyMap_[y][x] = temp[y / times][x / times];
		}
	}
	// update resolution and number of cells
	ycells_ = occupancyMap_.size();
	xcells_ = occupancyMap_[0].size();
	granularity_ = granularity_ / times;
}

void WorldOccupancyMap::increaseGranularity(int times) {
	// copy the current granularity map to a temporary one
	std::vector<std::vector<double> > temp;
	temp.resize(occupancyMap_.size());
	for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
		temp[y].resize(occupancyMap_[y].size());
	}
	for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
		for (unsigned int x = 0; x < occupancyMap_[y].size(); x++) {
			temp[y][x] = occupancyMap_[y][x];
		}
	}
	// clear the current occupancy map and resize it (twice the size)
	for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
		occupancyMap_[y].clear();
	}
	occupancyMap_.clear();
	// we may have to approximate the new size
	occupancyMap_.resize(temp.size() / times);
	for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
		occupancyMap_[y].resize(temp[0].size() / times);
	}
	// fill the values using the originalOccupancyMap
	for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
		for (unsigned int x = 0; x < occupancyMap_[y].size(); x++) {
			double val = 0;
			for (unsigned int yo = y * times; yo < y * times + times; yo++) {
				for (unsigned int xo = x * times; xo < x * times + times; xo++) {
					if (yo < temp.size() && xo < temp[yo].size()) {
						val = val > temp[yo][xo] ? val : temp[yo][xo];
					}
				}
			}
			// put the highest value of the corresponding cells in the original
			// occupancy map
			occupancyMap_[y][x] = val;
		}
	}
	// update the resolution
	granularity_ = granularity_ * times;
	xcells_ = occupancyMap_[0].size();
	ycells_ = occupancyMap_.size();
}

std::vector<std::vector<double> > WorldOccupancyMap::getMap() {
	return occupancyMap_;
}

void WorldOccupancyMap::restoreOriginalMap() {
	// clear the current occupancy map and resize it, but only if there is an original map
	if (originalOccupancyMap_.size() > 0) {
		for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
			occupancyMap_[y].clear();
		}
		// resize the occupancyMap_ and fill the values using the originalOccupancyMap_
		occupancyMap_.clear();
		occupancyMap_.resize(originalOccupancyMap_.size());
		for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
			occupancyMap_[y].resize(originalOccupancyMap_[0].size());
			for (unsigned int x = 0; x < occupancyMap_[y].size(); x++) {
				occupancyMap_[y][x] = originalOccupancyMap_[y][x];
			}
		}
		// update the resolution -- no need to change the xSize and ySize
		granularity_ = originalGranularity_;
		xcells_ = originalxcells_;
		ycells_ = originalycells_;

		// clear the current originalObstacles_ map and resize it
		for (unsigned int y = 0; y < originalObstacles_.size(); y++) {
			originalObstacles_[y].clear();
		}
		originalObstacles_.clear();
		// fill in the originalObstacles_ map
		originalObstacles_.resize(ycells_);
		for (unsigned int y = 0; y < originalObstacles_.size(); y++) {
			originalObstacles_[y].resize(xcells_);
			for (unsigned int x = 0; x < originalObstacles_[y].size(); x++) {
				originalObstacles_[y][x] = occupancyMap_[y][x];
			}
		}
	}
}

void WorldOccupancyMap::selectSubMap(double xfrom, double yfrom, double xto, double yto) {

	// save the original map, if not already done
	if (originalOccupancyMap_.size() == 0) {
		originalOccupancyMap_.resize(occupancyMap_.size());
		for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
			originalOccupancyMap_[y].resize(occupancyMap_[y].size());
			for (unsigned int x = 0; x < occupancyMap_[y].size(); x++) {
				originalOccupancyMap_[y][x] = occupancyMap_[y][x];
			}
		}
		originalxcells_ = xcells_;
		originalycells_ = ycells_;
	}

	// copy the current granularity map to a temporary one
	std::vector<std::vector<double> > temp;
	temp.resize(occupancyMap_.size());
	for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
		temp[y].resize(occupancyMap_[y].size());
		for (unsigned int x = 0; x < occupancyMap_[y].size(); x++) {
			temp[y][x] = occupancyMap_[y][x];
		}
	}
	// check the coordinates of the submap and make sure to get whole cells
	xfrom = floor((double) (xfrom / granularity_ + WP::CALCULATION_APPROXIMATION_ERROR)) * granularity_;
	xto = ceil(xto / granularity_) * granularity_;
	yfrom = floor((double) (yfrom / granularity_ + WP::CALCULATION_APPROXIMATION_ERROR)) * granularity_;
	yto = ceil(yto / granularity_) * granularity_;

	xfrom = xfrom < 0 ? 0 : xfrom;
	yfrom = yfrom < 0 ? 0 : yfrom;
	xto = xto > this->getXSize() ? this->getXSize() : xto;
	yto = yto > this->getYSize() ? this->getYSize() : yto;
	// clear the current occupancy map and resize it (twice the size)
	for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
		occupancyMap_[y].clear();
	}
	occupancyMap_.clear();
	// we may have to approximate the new size
	occupancyMap_.resize((yto - yfrom) / granularity_);

	std::cout.precision(20);

	unsigned int xcells = (unsigned int) ((xto - xfrom) / granularity_ + WP::CALCULATION_APPROXIMATION_ERROR);
	for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
		occupancyMap_[y].resize(xcells);
	}
	//fill the values
	unsigned int ybase = (unsigned int) (yfrom / granularity_ + WP::CALCULATION_APPROXIMATION_ERROR);
	unsigned int xbase = (unsigned int) (xfrom / granularity_ + WP::CALCULATION_APPROXIMATION_ERROR);

	for (unsigned int y = 0; y < occupancyMap_.size(); y++) {
		for (unsigned int x = 0; x < occupancyMap_[y].size(); x++) {
			occupancyMap_[y][x] = temp[y + ybase][x + xbase];
		}
	}
	// restore the right number of cells
	xcells_ = occupancyMap_[0].size();
	ycells_ = occupancyMap_.size();

	// clear the current originalObstacles_ map and resize it
	for (unsigned int y = 0; y < originalObstacles_.size(); y++) {
		originalObstacles_[y].clear();
	}
	originalObstacles_.clear();
	// fill in the originalObstacles_ map
	originalObstacles_.resize(ycells_);
	for (unsigned int y = 0; y < originalObstacles_.size(); y++) {
		originalObstacles_[y].resize(xcells_);
		for (unsigned int x = 0; x < originalObstacles_[y].size(); x++) {
			originalObstacles_[y][x] = occupancyMap_[y][x];
		}
	}
}

double WorldOccupancyMap::getOccupancyValueInCell(unsigned short int x_cell, unsigned short int y_cell) {
	if (y_cell < occupancyMap_.size() && x_cell < occupancyMap_[y_cell].size()) {
		return occupancyMap_[y_cell][x_cell];
	}
	return INFINITY;
}

void WorldOccupancyMap::addObstacles(std::vector<cellPosition> obstacles) {
	for(std::vector<cellPosition>::iterator it = obstacles.begin(); it != obstacles.end(); it++) {
		occupancyMap_[(*it).y_cell][(*it).x_cell] = 1;
	}
}

void WorldOccupancyMap::removeObstacles(std::vector<cellPosition> obstacles) {
	for(std::vector<cellPosition>::iterator it = obstacles.begin(); it != obstacles.end(); it++) {
		occupancyMap_[(*it).y_cell][(*it).x_cell] = originalObstacles_[(*it).y_cell][(*it).x_cell];
	}
}

bool WorldOccupancyMap::containsObstacles() {
	return containsObstacles_;
}

void WorldOccupancyMap::initialize(int xcells, int ycells, double granularity, const std::vector<std::vector<double> > &occupancyMap) {
        xcells_ = xcells;
        ycells_ = ycells;
        granularity_ = granularity;
        occupancyMap_ = occupancyMap;

        //Need to scale the granularity to be WP::WORLD_SPACE_GRANULARITY?
        this->scaleGranularity(WP::WORLD_SPACE_GRANULARITY);

        originalOccupancyMap_ = occupancyMap;
        originalxcells_ = xcells;
        originalycells_ = ycells;
        originalGranularity_ = granularity;
}
