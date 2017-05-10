/**
 * @file MapGenerators.cpp
 * @author Marcello Cirillo
 *
 * Created on: Jul 15, 2011
 *      Author: marcello
 */

#include "orunav_motion_planner/MapGenerators.h"

void generateRandomMap(double xSize, double ySize, double gran, unsigned int obstacles, double obstRad,
		double xObstMin, double xObstMax, double yObstMin, double yObstMax, std::string filename) {

	int x_cells = (int) (xSize / gran + WP::CALCULATION_APPROXIMATION_ERROR);
	int y_cells = (int) (ySize / gran + WP::CALCULATION_APPROXIMATION_ERROR);
	int x_start_obs = (int) (xObstMin / gran + WP::CALCULATION_APPROXIMATION_ERROR);
	int x_end_obs = (int) (xObstMax / gran + WP::CALCULATION_APPROXIMATION_ERROR);
	int y_start_obs = (int) (yObstMin / gran + WP::CALCULATION_APPROXIMATION_ERROR);
	int y_end_obs = (int) (yObstMax / gran + WP::CALCULATION_APPROXIMATION_ERROR);

	std::vector<std::vector<double> > map;
	map.resize(y_cells);
	for (unsigned int i = 0; i < map.size(); i++) {
		map[i].resize(x_cells);
	}

	// initialize random seed
	srand(time(NULL));
	// generate the obstacles
	for (unsigned int i = 0; i < obstacles; i++) {
		int xObst;
		int yObst;
		xObst = rand() % (int) (x_end_obs - x_start_obs + 1);
		xObst += x_start_obs;
		yObst = rand() % (int) (y_end_obs - y_start_obs + 1);
		yObst += y_start_obs;
		map[yObst][xObst] = 1;
	}

	// Expand the obstacles
	// check how many expansions are needed
	int radius = ceil(obstRad / gran);

	// create a temp map, resize it and copy the old content
	std::vector<std::vector<double> > temp;
	temp.resize(map.size());
	for (unsigned int y = 0; y < map.size(); y++) {
		temp[y].resize(map[y].size());
	}
	for (unsigned int y = 0; y < map.size(); y++) {
		for (unsigned int x = 0; x < map[y].size(); x++) {
			temp[y][x] = map[y][x];
		}
	}
	// parse and expand
	for (unsigned int y = 0; y < temp.size(); y++) {
		for (unsigned int x = 0; x < temp[y].size(); x++) {
			double value = temp[y][x];
			if (value == 1) {
				int y_min = (int) y - radius < 0 ? 0 : y - radius;
				int x_min = (int) x - radius < 0 ? 0 : x - radius;
				for (unsigned int j = y_min; j <= y + radius; j++) {
					for (unsigned int i = x_min; i <= x + radius; i++) {
						// within borders
						double dx = (double) i - (double) x;
						double dy = (double) j - (double) y;
						if ((int) (sqrt(pow(dx, 2) + pow(dy, 2)) + WP::CALCULATION_APPROXIMATION_ERROR)
								<= radius) {
							if (j < temp.size() && i < temp[j].size() && i >= 0 && j >= 0) {
								map[j][i] = value;
							}
						}
					}
				}
			}
		}
	}

	// reverse the map
	reverse(map.begin(), map.end());

	// prepare the files
	std::ofstream mapfile;
	mapfile.open((std::string().append(WP::MAPS_DIR).append(filename).append(".map")).c_str());

	// basic information
	mapfile << "# 2D grid-map representation v.1" << std::endl;
	mapfile << "# GRID_SIZE\t" << gran << std::endl;
	mapfile << "# WIDTH\t" << (int) (xSize / gran + WP::CALCULATION_APPROXIMATION_ERROR) << std::endl;
	mapfile << "# HEIGHT\t" << (int) (ySize / gran + WP::CALCULATION_APPROXIMATION_ERROR) << std::endl;

	// write the map
	for (unsigned int y = 0; y < map.size(); y++) {
		for (unsigned int x = 0; x < map[y].size(); x++) {
			mapfile << map[y][x] << " ";
		}
		mapfile << std::endl;
	}
	mapfile.close();

}

void generateNarrowPassageMap(double xSize, double ySize, double gran, double xLowLeft, double xTopRight,
		double yLowLeft, double yTopRight, double passageSize, std::string filename) {

	int x_cells = (int) (xSize / gran + WP::CALCULATION_APPROXIMATION_ERROR);
	int y_cells = (int) (ySize / gran + WP::CALCULATION_APPROXIMATION_ERROR);
	int y_low_left = (int) (yLowLeft / gran + WP::CALCULATION_APPROXIMATION_ERROR);
	int y_top_right = (int) (yTopRight / gran + WP::CALCULATION_APPROXIMATION_ERROR);
	int passage_size = (int) (passageSize / gran + WP::CALCULATION_APPROXIMATION_ERROR);

	std::vector<std::vector<double> > map;
	map.resize(y_cells);
	for (unsigned int i = 0; i < map.size(); i++) {
		map[i].resize(x_cells);
	}

	for (int i = 0; i < x_cells; i++) {
		map[y_low_left][i] = 1;
	}
	int passage_l = ((y_top_right - y_low_left) / 2) - (passage_size / 2) + y_low_left;
	int passage_r = ((y_top_right - y_low_left) / 2) + (passage_size / 2) + y_low_left;
	for (int i = passage_l; i < passage_r; i++) {
		map[y_low_left][i] = 0;
	}
	for (int i = 0; i < x_cells; i++) {
		map[y_top_right][i] = 1;
	}
	for (int i = passage_l; i < passage_r; i++) {
		map[y_top_right][i] = 0;
	}

	/*// generate the box
	 for (int i = x_low_left; i < x_top_right; i++) {
	 map[y_top_right][i] = 1;
	 }
	 for (int i = x_low_left; i < x_top_right; i++) {
	 map[y_low_left][i] = 1;
	 }
	 for (int i = y_low_left; i < y_top_right; i++) {
	 map[i][x_top_right] = 1;
	 }
	 for (int i = y_low_left; i < y_top_right; i++) {
	 map[i][x_low_left] = 1;
	 }

	 // open the passage
	 int passage_l = ((y_top_right - y_low_left) / 2) - (passage_size / 2) + y_low_left;
	 int passage_r = ((y_top_right - y_low_left) / 2) + (passage_size / 2) + y_low_left;
	 for (int i = passage_l; i < passage_r; i++) {
	 map[i][x_low_left] = 0;
	 }*/

	// reverse the map
	reverse(map.begin(), map.end());

	// prepare the files
	std::ofstream mapfile;
	mapfile.open((std::string().append(WP::MAPS_DIR).append(filename).append(".map")).c_str());

	// basic information
	mapfile << "# 2D grid-map representation v.1" << std::endl;
	mapfile << "# GRID_SIZE\t" << gran << std::endl;
	mapfile << "# WIDTH\t" << (int) (xSize / gran + WP::CALCULATION_APPROXIMATION_ERROR) << std::endl;
	mapfile << "# HEIGHT\t" << (int) (ySize / gran + WP::CALCULATION_APPROXIMATION_ERROR) << std::endl;

	// write the map
	for (unsigned int y = 0; y < map.size(); y++) {
		for (unsigned int x = 0; x < map[y].size(); x++) {
			mapfile << map[y][x] << " ";
		}
		mapfile << std::endl;
	}
	mapfile.close();

}

