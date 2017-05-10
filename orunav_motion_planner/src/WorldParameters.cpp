/**
 * @file WorldParameters.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Apr 4, 2011
 *      Author: marcello
 */
#include "orunav_motion_planner/WorldParameters.h"

// initialization of the world parameters

int WP::LOG_LEVEL = 1;
std::string WP::LOG_FILE = std::string("stdout");
bool WP::SAVE_FINAL_VISUALIZATION_TO_FILE = false;

std::string WP::PRIMITIVES_DIR = std::string("/home/mco/Workspace/ALLO/src/branches/MRTimelessPathFinder/Primitives/");
std::string WP::TABLES_DIR = std::string("/home/mco/Workspace/ALLO/src/branches/MRTimelessPathFinder/LookupTables/");
std::string WP::MAPS_DIR = std::string("/home/mco/Workspace/ALLO/src/branches/MRTimelessPathFinder/Maps/");

unsigned long int WP::EXPANSION_QUEUE_MAX = 10000000;
double WP::COST_CUTOFF = INFINITY;

bool WP::ALLOW_GARBAGE_COLLECTION = false;
bool WP::USE_HEURISTIC_ESTIMATION = true;
double WP::HEURISTIC_VALUE_MULTIPLIER = 1;

double WP::WORLD_SPACE_GRANULARITY = 0.5;
double WP::CALCULATION_APPROXIMATION_ERROR = 0.001;
double WP::OCCUPANCY_THRESHOLD = 0.5;

int WP::DECIMAL_APPROXIMATION = 4;
WP::NodeExpansionMethod WP::NODE_EXPANSION_METHOD = WP::NodeExpansionMethod::EPFSG;


void WP::setExpansionMethod(WP::NodeExpansionMethod method) {
	WP::NODE_EXPANSION_METHOD = method;
}

void WP::setPrimitivesDir(std::string dir) {
	WP::PRIMITIVES_DIR = dir;
}

void WP::setTablesDir(std::string dir) {
	WP::TABLES_DIR = dir;
}

void WP::setMapsDir(std::string dir) {
	WP::MAPS_DIR = dir;
}

void WP::setExpansionQueueMaxSize(unsigned long int size) {
	WP::EXPANSION_QUEUE_MAX = size;
}

void WP::setCostCutoff(double cost) {
	WP::COST_CUTOFF = cost;
}

void WP::enableGarbageCollection(bool enable) {
	WP::ALLOW_GARBAGE_COLLECTION = enable;
}

void WP::enableUseOfHeuristicEstimation(bool enable) {
	WP::USE_HEURISTIC_ESTIMATION = enable;
}

void WP::setHeuristicValueMultiplier(double multiplier) {
	WP::HEURISTIC_VALUE_MULTIPLIER = multiplier;
}

void WP::setLogLevel(int level) {
	WP::LOG_LEVEL = level;
}

void WP::setWorldSpaceGranularity(double granularity) {
	WP::WORLD_SPACE_GRANULARITY = granularity;
}

void WP::setApproximationError(double approx) {
	WP::CALCULATION_APPROXIMATION_ERROR = approx;
}

void WP::setOccupancyThreshold(double threshold) {
	WP::OCCUPANCY_THRESHOLD = threshold;
}

void WP::setLogFile(std::string filename) {
	WP::LOG_FILE = filename;
}

void WP::setSaveFinalVisualizationToFile(bool enable) {
	WP::SAVE_FINAL_VISUALIZATION_TO_FILE = enable;
}
