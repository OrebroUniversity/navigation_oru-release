/**
 * @file Utils.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Mar 15, 2011
 *      Author: marcello
 */

#include "orunav_motion_planner/Utils.h"

BOOST_GEOMETRY_REGISTER_C_ARRAY_CS(cs::cartesian)

std::vector<cellPosition*> getOccupiedCells(simplePoint p1, simplePoint p2, simplePoint p3, simplePoint p4, double granularity) {

	simplePoint points[] = {p1, p2, p3, p4};
	std::vector<cellPosition*> result;
	result.clear();

	// lowest x,y values and the highest x,y values of the 4 points
	double x_low = p1.x;
	double x_high = p1.x;
	double y_low = p1.y;
	double y_high = p1.y;

	for (int i = 0; i < 4; i++) {
		x_low = x_low < points[i].x ? x_low : points[i].x;
		x_high = x_high > points[i].x ? x_high : points[i].x;
		y_low = y_low < points[i].y ? y_low : points[i].y;
		y_high = y_high > points[i].y ? y_high : points[i].y;
	}

	// calculate the bounds of cells it might occupy -- cells can be negative: do not change
	int x_cell_min = floor((double) (x_low / granularity)) - 1;
	int x_cell_max = floor((double) (x_high / granularity)) + 1;
	int y_cell_min = floor((double) (y_low / granularity)) - 1;
	int y_cell_max = floor((double) (y_high / granularity)) + 1;

	double pointspoly1[][2] = {{p1.x, p1.y}, {p2.x, p2.y}, {p3.x, p3.y}, {p4.x, p4.y}, {p1.x, p1.y}};
	boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > poly1;
	boost::geometry::append(poly1, pointspoly1);

	for (int x = x_cell_min; x <= x_cell_max; x ++) {
		for (int y = y_cell_min; y <= y_cell_max; y++) {
			double x_b_l = x * granularity;
			double y_b_l = y * granularity;

			double pointspoly2[][2] = {{x_b_l, y_b_l}, {x_b_l, y_b_l+granularity},
					{x_b_l+granularity, y_b_l+granularity}, {x_b_l+granularity, y_b_l}, {x_b_l, y_b_l}};
			boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > poly2;
			boost::geometry::append(poly2, pointspoly2);

			if (boost::geometry::intersects(poly1, poly2)) {
				cellPosition* occ_cell = new cellPosition;
				occ_cell->x_cell = x;
				occ_cell->y_cell = y;
				result.push_back(occ_cell);
			}
		}
	}
	return result;
}

double calculateLength(std::vector<vehicleSimplePoint*> path) {
	double cost = 0;
	if (path.size() <= 1) {
		return cost;
	}
	vehicleSimplePoint* start;
	std::vector<vehicleSimplePoint*>::iterator it;
	it = path.begin();
	start = *it;
	it++;
	while (it != path.end()) {
		cost += sqrt(pow(start->x - (*it)->x, 2) + pow(start->y - (*it)->y, 2));
		start = (*it);
		it++;
	}
	return cost;
}

double addDegrees(double d1, double d2, int digitprec) {
	double sum = d1 + d2;
	if (sum >= 0 && sum < 360) {
		return roundNumberToDecimal(sum, digitprec);
	} else if (sum < 0) {
		return roundNumberToDecimal(360 + sum, digitprec);
	} else {
		// sum >= 360
		return roundNumberToDecimal(sum - 360, digitprec);
	}
}

double addAnglesRadians(double a1, double a2, int digitprec) {
	// sum them and normalize
	double z = a1 + a2;
	if (fabs(z) < M_PI) {
		return roundNumberToDecimal(z, digitprec);
	}
	double result = roundNumberToDecimal(atan2(sin(z), cos(z)), digitprec);
	// we want angles (-M_PI,M_PI]
	if (fabsf(result - MINUS_M_PI) < 10e-7) {
		return -result;
	}
	return result;
}

double roundAngle(double a, double base, int digitprec) {
	base = fabs(base);
	double f_mod = fabs(fmod(a, base));
	if (f_mod == 0) {
		return roundNumberToDecimal(a, digitprec);
	}
	if (a < 0) {
		if ((base - f_mod) < f_mod) {
			a = addAnglesRadians(a, -(base - f_mod), digitprec);
		} else {
			a = addAnglesRadians(a, f_mod, digitprec);
		}
	} else { // a > 0
		if ((base - f_mod) < f_mod) {
			a = addAnglesRadians(a, (base - f_mod), digitprec);
		} else {
			a = addAnglesRadians(a, -f_mod, digitprec);
		}
	}
	return a;
}

double calculateAngleDifference(double a, double b, int digitprec) {
	// both positive or both negative
	if ((a >= 0 && b >= 0) || (a < 0 && b < 0)) {
		return roundNumberToDecimal(fabs(fabs(a) - fabs(b)), digitprec);
	}
	// de-normalize the negative angle
	if (a < 0) {
		a = 2 * M_PI + a;
	} else {
		b = 2 * M_PI + b;
	}
	// return the closest result
	if (fabs(a - b) > M_PI) {
		return roundNumberToDecimal(M_PI * 2 - fabs(a - b), digitprec);
	} else {
		return roundNumberToDecimal(fabs(a - b), digitprec);
	}
}

double roundNumberToDecimal(double n, double dec) {
	double mult = pow(10, dec);
	int base = n > 0 ? floor(n * mult) : ceil(n * mult);
	if (fabs(n * mult) - fabs(base) >= 0.5) {
		base = base > 0 ? base + 1 : base - 1;
	}
	return (double) base / mult;
}

double floorPositiveNumber(double n, double base, double tolerance) {
	double f_mod = fmod(n, base);
	if (f_mod < (base - tolerance)) {
		n = n - f_mod;
	} else {
		n = n + (base - f_mod);
	}
	return n;
}

void writeLogLine(std::string line, std::string filename) {
	if (filename.compare(std::string("stdout")) == 0) {
		std::cout << line << std::endl;
	} else {
		std::ofstream logfile;
		logfile.open(filename.c_str(), std::ios::app);
		logfile << line << std::endl;
		logfile.close();
	}
}

void writeLogLine(std::string line, const char* className, std::string filename) {
	if (filename.compare(std::string("stdout")) == 0) {
		std::cout << "[" << className << "] " << line << std::endl;
	} else {
		std::ofstream logfile;
		logfile.open(filename.c_str(), std::ios::app);
		logfile << "[" << className << "] " << line << std::endl;
		logfile.close();
	}
}


#ifndef _WIN32 // linux only
void increaseStackSize(long int newSize) {
	int result;
	struct rlimit rl;
	result = getrlimit(RLIMIT_STACK, &rl);
	rl.rlim_cur = newSize;
	result = setrlimit(RLIMIT_STACK, &rl);
	if (result != 0) {
		std::cout << "[" << "Utils" << "] " << "Unable to increase stack size" << std::endl;
	}
}
#endif

