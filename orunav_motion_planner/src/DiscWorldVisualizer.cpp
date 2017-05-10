/**
 * @file DiscWorldVisualizer.cpp
 * @author Marcello Cirillo
 *
 *  Created on: Mar 2, 2011
 *      Author: marcello
 */

#include "orunav_motion_planner/DiscWorldVisualizer.h"


unsigned short int DiscWorldVisualizer::R[5] = {10, 160, 240,  25,   0};
unsigned short int DiscWorldVisualizer::G[5] = {160, 40,  15,  15, 225};
unsigned short int DiscWorldVisualizer::B[5] = {5,    5,  50, 240, 240};

DiscWorldVisualizer::DiscWorldVisualizer(int xCels, int yCels, int scale) {
	scaleFactor_ = scale;
	xTotCels_ = xCels;
	yTotCels_ = yCels;
	imgXsize_ = xTotCels_ * scaleFactor_;
	imgYsize_ = yTotCels_ * scaleFactor_;
	// create the image that will represent the world
	worldDisplayImg_ = cvCreateImage(cvSize(imgXsize_, imgYsize_), IPL_DEPTH_8U, 3);
	// white background for clarity reasons
	cvRectangle(worldDisplayImg_, cvPoint(0, 0), cvPoint(imgXsize_, imgYsize_), cvScalar(255, 255, 255, 0), CV_FILLED);
	// create the window to display the image
	cvNamedWindow("World", CV_WINDOW_AUTOSIZE);

	// prepare the thread for execution
	threadStop_ = false;

	// start the thread
	visualizedThread_ = boost::thread(DisplayDiscWorld_thread_wrapper, this);

	// lock the mutex before writing
	boost::mutex::scoped_lock lock(update_mutex_);

	for (int x = 0; x <= xTotCels_ * scaleFactor_; x += scaleFactor_) {
		cvLine(worldDisplayImg_, cvPoint(x, 0), cvPoint(x, imgYsize_), cvScalar(211, 211, 211, 0), 1, 8, 0);
	}
	for (int y = 0; y <= yTotCels_ * scaleFactor_; y += scaleFactor_) {
		cvLine(worldDisplayImg_, cvPoint(0, y), cvPoint(imgXsize_, y), cvScalar(211, 211, 211, 0), 1, 8, 0);
	}
}

DiscWorldVisualizer::~DiscWorldVisualizer() {
	// check is saving the final image is required
	stopVisualization();
	if (WP::SAVE_FINAL_VISUALIZATION_TO_FILE) {
		cvSaveImage((WP::LOG_FILE.append(".PNG")).c_str(), worldDisplayImg_);
	}
	visualizedThread_.join();
	cvDestroyWindow("World");
	cvReleaseImage(&worldDisplayImg_);
}

void DiscWorldVisualizer::resetVisualizer() {
	// create the image that will represent the world
	worldDisplayImg_ = cvCreateImage(cvSize(imgXsize_, imgYsize_), IPL_DEPTH_8U, 3);
	cvRectangle(worldDisplayImg_, cvPoint(0, 0), cvPoint(imgXsize_, imgYsize_), cvScalar(255, 255, 255, 0),
			CV_FILLED);

	// lock the mutex before writing
	boost::mutex::scoped_lock lock(update_mutex_);

	for (int x = 0; x <= xTotCels_ * scaleFactor_; x += scaleFactor_) {
		cvLine(worldDisplayImg_, cvPoint(x, 0), cvPoint(x, imgYsize_), cvScalar(211, 211, 211, 0), 1, 8, 0);
	}
	for (int y = 0; y <= yTotCels_ * scaleFactor_; y += scaleFactor_) {
		cvLine(worldDisplayImg_, cvPoint(0, y), cvPoint(imgXsize_, y), cvScalar(211, 211, 211, 0), 1, 8, 0);
	}
}

void DiscWorldVisualizer::drawOccupancy(std::vector<std::vector<double> > map) {
	int y = 0;
	int x = 0;
	std::vector<std::vector<double> >::iterator it_y;
	std::vector<double>::iterator it_x;
	for (it_y = map.begin(); it_y != map.end(); it_y++) {
		x = 0;
		for (it_x = (*it_y).begin(); it_x != (*it_y).end(); it_x++) {
			// draw only the obstacles
			if ((*it_x) > WP::CALCULATION_APPROXIMATION_ERROR) {
				this->drawFilledRectangle(x * scaleFactor_, y * scaleFactor_, x * scaleFactor_ + scaleFactor_,
						y * scaleFactor_ + scaleFactor_, 255 * (1 - (*it_x)), 255 * (1 - (*it_x)),
						255 * (1 - (*it_x)));
			}
			x++;
		}
		y++;
	}
}

void DiscWorldVisualizer::drawStart(Configuration* conf, int vehicleID) {
	unsigned short int ind = vehicleID % AVAIL_COLORS;
	this->drawConfiguration(conf, R[ind], G[ind], B[ind]);
}

void DiscWorldVisualizer::drawGoal(Configuration* conf, int vehicleID) {
	unsigned short int ind = vehicleID % AVAIL_COLORS;
	this->drawConfiguration(conf, R[ind]/2, G[ind]/2, B[ind]/2);
	if (WP::SAVE_FINAL_VISUALIZATION_TO_FILE && WP::LOG_LEVEL >= 10) {
		std::string name = std::string(WP::LOG_FILE);
		cvSaveImage((name.append(".").append(std::to_string(vehicleID)).append(".PNG")).c_str(), worldDisplayImg_);
	}
}

void DiscWorldVisualizer::drawConfigurations(std::vector<Configuration*> confs) {
	for (std::vector<Configuration*>::iterator it = confs.begin(); it != confs.end(); it++) {
		unsigned short int ind = (*it)->getMission()->getVehicleID() % AVAIL_COLORS;
		this->drawPath((*it)->getTrajectory(), R[ind], G[ind], B[ind]);
		this->drawConfiguration(*it, R[ind], G[ind], B[ind]);
	}

}

void DiscWorldVisualizer::display() {
	unsigned long int counter = 0;
	while (threadStop_ == false) {
		{
			boost::mutex::scoped_lock lock(update_mutex_);
			cvShowImage("World", worldDisplayImg_);
		}

		if (WP::SAVE_FINAL_VISUALIZATION_TO_FILE && WP::LOG_LEVEL >= 10) {
			if (counter % 20 == 0) {
				std::string name = std::string(WP::LOG_FILE);
				cvSaveImage((name.append(std::to_string(counter)).append(".PNG")).c_str(), worldDisplayImg_);
			}
			counter ++;
		}

		cvWaitKey(50);
		boost::posix_time::milliseconds workTime(50);
		boost::this_thread::sleep(workTime);
	}
}

void DiscWorldVisualizer::stopVisualization() {
	threadStop_ = true;
}

void DiscWorldVisualizer::drawConfiguration(Configuration* conf, int R, int G, int B) {
	if (dynamic_cast<CarConfiguration*> (conf)) {
		this->drawCarConfiguration(dynamic_cast<CarConfiguration*> (conf), R, G, B);
	} else if (dynamic_cast<LHDConfiguration*> (conf)) {
		this->drawLHDConfiguration(dynamic_cast<LHDConfiguration*> (conf), R, G, B);
	} else {
		this->drawBaseConfiguration(conf, R, G, B);
	}

}

void DiscWorldVisualizer::drawBaseConfiguration(Configuration* conf, int R, int G, int B) {
	// calculate the space
	int radius = 1;
	double x_centre, y_centre, x_top, y_top, x_bottom, y_bottom, x_left, y_left, x_right, y_right, orient,
	base_orient;
	if (scaleFactor_ > 2) {
		radius = scaleFactor_ / 2;
	}
	orient = conf->getOrientation();
	// compute the central point of the triangle
	x_centre = conf->getXCoordinate() * (scaleFactor_ / WP::WORLD_SPACE_GRANULARITY);
	y_centre = conf->getYCoordinate() * (scaleFactor_ / WP::WORLD_SPACE_GRANULARITY);
	// top vertex of the triangle
	x_top = x_centre + radius * cos(orient);
	y_top = y_centre + radius * sin(orient);
	// calculate the bottom points
	x_bottom = x_centre - radius * cos(orient);
	y_bottom = y_centre - radius * sin(orient);
	base_orient = addAnglesRadians(orient, M_PI_2, WP::DECIMAL_APPROXIMATION);
	x_right = x_bottom + (radius * cos(base_orient)) / 3;
	y_right = y_bottom + (radius * sin(base_orient)) / 3;
	x_left = x_bottom - (radius * cos(base_orient)) / 3;
	y_left = y_bottom - (radius * sin(base_orient)) / 3;
	// draw the configuration
	this->drawLine(x_top, y_top, x_right, y_right, R, G, B);
	this->drawLine(x_left, y_left, x_right, y_right, R, G, B);
	this->drawLine(x_left, y_left, x_top, y_top, R, G, B);
}

void DiscWorldVisualizer::drawCarConfiguration(CarConfiguration* conf, int R, int G, int B) {

	CarModel* m = dynamic_cast<CarModel*> (conf->getMission()->getVehicleModel());

	// speed up
	double scale = scaleFactor_ / WP::WORLD_SPACE_GRANULARITY;
	double cosOrient = cos(conf->getOrientation());
	double sinOrient = sin(conf->getOrientation());
	double base_orient = addAnglesRadians(conf->getOrientation(), M_PI_2, WP::DECIMAL_APPROXIMATION);
	double cosBaseOrient = cos(base_orient);
	double sinBaseOrient = sin(base_orient);

	// length of the car: from rear axel to front, and from rear axel to back
	double back_length = m->getCarBackLength() * scale;
	double front_length = m->getCarFrontLength() * scale;
	// width and orientation
	double width = conf->getMission()->getVehicleModel()->getWidth() * scale;

	// compute the central point of the back
	double x_base_centre = (conf->getXCoordinate() * scale) - back_length * cosOrient;
	double y_base_centre = (conf->getYCoordinate() * scale) - back_length * sinOrient;
	// central point of the front
	double x_top_centre = x_base_centre + (back_length + front_length) * cosOrient;
	double y_top_centre = y_base_centre + (back_length + front_length) * sinOrient;

	double half_width = width / 2;
	double x_base_right = x_base_centre + half_width * cosBaseOrient;
	double y_base_right = y_base_centre + half_width * sinBaseOrient;
	double x_base_left = x_base_centre - half_width * cosBaseOrient;
	double y_base_left = y_base_centre - half_width * sinBaseOrient;
	double x_top_right = x_top_centre + half_width * cosBaseOrient;
	double y_top_right = y_top_centre + half_width * sinBaseOrient;
	double x_top_left = x_top_centre - half_width * cosBaseOrient;
	double y_top_left = y_top_centre - half_width * sinBaseOrient;

	// The wheels have the same orientation of the car
	/** @todo check if we need them */
	double wheel_orient = conf->getOrientation();
	double wheel_size = front_length / 6;

	// front axel
	double x_front_axel_centre = x_base_centre + (back_length + front_length * 2 / 3) * cosOrient;
	double y_front_axel_centre = y_base_centre + (back_length + front_length * 2 / 3) * sinOrient;

	double right_wheel_center_x = x_front_axel_centre - width / 3 * cosBaseOrient;
	double right_wheel_center_y = y_front_axel_centre - width / 3 * sinBaseOrient;

	double x_wheel_right_top = right_wheel_center_x + wheel_size * cos(wheel_orient);
	double y_wheel_right_top = right_wheel_center_y + wheel_size * sin(wheel_orient);
	double x_wheel_right_bottom = right_wheel_center_x - wheel_size * cos(wheel_orient);
	double y_wheel_right_bottom = right_wheel_center_y - wheel_size * sin(wheel_orient);

	double left_wheel_center_x = x_front_axel_centre + width / 4 * cosBaseOrient;
	double left_wheel_center_y = y_front_axel_centre + width / 4 * sinBaseOrient;

	double x_wheel_left_top = left_wheel_center_x + wheel_size * cos(wheel_orient);
	double y_wheel_left_top = left_wheel_center_y + wheel_size * sin(wheel_orient);
	double x_wheel_left_bottom = left_wheel_center_x - wheel_size * cos(wheel_orient);
	double y_wheel_left_bottom = left_wheel_center_y - wheel_size * sin(wheel_orient);

	// draw the configuration
	this->drawLine(x_base_right, y_base_right, x_base_left, y_base_left, R, G, B);
	this->drawLine(x_base_right, y_base_right, x_top_right, y_top_right, R, G, B);
	this->drawLine(x_top_left, y_top_left, x_top_right, y_top_right, R, G, B);
	this->drawLine(x_top_left, y_top_left, x_base_left, y_base_left, R, G, B);
	// draw the wheel
	this->drawLine(x_wheel_left_top, y_wheel_left_top, x_wheel_left_bottom, y_wheel_left_bottom, R, G, B);
	this->drawLine(x_wheel_right_top, y_wheel_right_top, x_wheel_right_bottom, y_wheel_right_bottom, R, G, B);
}

void DiscWorldVisualizer::drawLHDConfiguration(LHDConfiguration* conf, int R, int G, int B) {

	LHDModel* model = dynamic_cast<LHDModel*> (conf->getMission()->getVehicleModel());
	double scale = scaleFactor_ / WP::WORLD_SPACE_GRANULARITY;

	double frontBodyOrientation = conf->getOrientation();
	double rearBodyOrientation = addAnglesRadians(frontBodyOrientation, conf->getSteering(), WP::DECIMAL_APPROXIMATION);

	double cosFrontOrient = cos(frontBodyOrientation);
	double sinFrontOrient = sin(frontBodyOrientation);
	double cosRearOrient = cos(rearBodyOrientation);
	double sinRearOrient = sin(rearBodyOrientation);

	double frontBaseOrient = addAnglesRadians(frontBodyOrientation, M_PI_2, WP::DECIMAL_APPROXIMATION);
	double cosFrontBaseOrient = cos(frontBaseOrient);
	double sinFrontBaseOrient = sin(frontBaseOrient);

	double rearBaseOrient = addAnglesRadians(rearBodyOrientation, M_PI_2, WP::DECIMAL_APPROXIMATION);
	double cosRearBaseOrient = cos(rearBaseOrient);
	double sinRearBaseOrient = sin(rearBaseOrient);


	double width = model->getWidth() * scale;
	// front body of the WheelLoader -- from the front to the joint
	double frontLength = (model->getLHDFrontFromAxle() + model->getLHDFrontLength()) * scale;
	// rear body of the WheelLoader -- from the back to the joint
	double rearLength = (model->getLHDBackFromAxle() + model->getLHDBackLength()) * scale;
	// distance of the axles from the central joint
	// double rearAxleToJoint = model->getLHDBackLength() * scale;
	double frontAxleToJoint = model->getLHDFrontLength() * scale;

	// draw the reference point
	this->drawDot((conf->getXCoordinate() * scale), (conf->getYCoordinate() * scale), width/8, R,G,B);

	// compute the coordinates of the joint
	double x_joint = (conf->getXCoordinate() * scale) - frontAxleToJoint * cosFrontOrient;
	double y_joint = (conf->getYCoordinate() * scale) - frontAxleToJoint * sinFrontOrient;

	// detach the two bodies from the joint
	// rear body
	double width_2 = width / 2;
	double rearLength_8 = rearLength / 8;
	double rearLengthDraw = rearLength - rearLength_8;
	double topRearX = (x_joint - rearLength_8 * cosRearOrient);
	double topRearY = (y_joint - rearLength_8 * sinRearOrient);
	double x_rear_top_right = topRearX + (width_2 * cosRearBaseOrient);
	double y_rear_top_right = topRearY + (width_2 * sinRearBaseOrient);
	double x_rear_top_left = topRearX - (width_2 * cosRearBaseOrient);
	double y_rear_top_left = topRearY - (width_2 * sinRearBaseOrient);
	double x_rear_base_right = x_rear_top_right - (rearLengthDraw * cosRearOrient);
	double y_rear_base_right = y_rear_top_right - (rearLengthDraw * sinRearOrient);
	double x_rear_base_left = x_rear_top_left - (rearLengthDraw * cosRearOrient);
	double y_rear_base_left = y_rear_top_left - (rearLengthDraw * sinRearOrient);

	// front body
	double frontLength_8 = frontLength / 8;
	double frontLengthDraw = frontLength - frontLength_8;
	double bottomFrontX = (x_joint + frontLength_8 * cosFrontOrient);
	double bottomFrontY = (y_joint + frontLength_8 * sinFrontOrient);
	double x_front_base_right = bottomFrontX + (width_2 * cosFrontBaseOrient);
	double y_front_base_right = bottomFrontY + (width_2 * sinFrontBaseOrient);
	double x_front_base_left = bottomFrontX - (width_2 * cosFrontBaseOrient);
	double y_front_base_left = bottomFrontY - (width_2 * sinFrontBaseOrient);
	double x_front_top_right = x_front_base_right + (frontLengthDraw * cosFrontOrient);
	double y_front_top_right = y_front_base_right + (frontLengthDraw * sinFrontOrient);
	double x_front_top_left = x_front_base_left + (frontLengthDraw * cosFrontOrient);
	double y_front_top_left = y_front_base_left + (frontLengthDraw * sinFrontOrient);

	// front axle & joint
	//double x_axle_left = conf->getXcoordinate() * scale - ((width / 3) * cosBaseOrient);
	//double y_axle_left = conf->getYcoordinate() * scale - ((width / 3) * sinBaseOrient);
	//double x_axle_right = conf->getXcoordinate() * scale + ((width / 3) * cosBaseOrient);
	//double y_axle_right = conf->getYcoordinate() * scale + ((width / 3) * sinBaseOrient);

	// draw the configuration
	this->drawLine(x_rear_top_right, y_rear_top_right, x_rear_top_left, y_rear_top_left, R, G, B);
	this->drawLine(x_rear_top_right, y_rear_top_right, x_rear_base_right, y_rear_base_right, R, G, B);
	this->drawLine(x_rear_base_left, y_rear_base_left, x_rear_base_right, y_rear_base_right, R, G, B);
	this->drawLine(x_rear_base_left, y_rear_base_left, x_rear_top_left, y_rear_top_left, R, G, B);

	this->drawLine(x_front_top_right, y_front_top_right, x_front_top_left, y_front_top_left, R, G, B);
	this->drawLine(x_front_top_right, y_front_top_right, x_front_base_right, y_front_base_right, R, G, B);
	this->drawLine(x_front_base_left, y_front_base_left, x_front_base_right, y_front_base_right, R, G, B);
	this->drawLine(x_front_base_left, y_front_base_left, x_front_top_left, y_front_top_left, R, G, B);

	this->drawLine(bottomFrontX, bottomFrontY, x_joint, y_joint, R, G, B);
	this->drawLine(topRearX, topRearY, x_joint, y_joint, R, G, B);
	//this->drawLine(x_axle_right, y_axle_right, x_axle_left, y_axle_left, R, G, B);

}

void DiscWorldVisualizer::drawPath(std::vector<vehicleSimplePoint> path, int R, int G, int B) {
	// it applies only if the path is populated
	if (path.size() <= 1) {
		return;
	}
	std::vector<vehicleSimplePoint>::iterator it;
	vehicleSimplePoint start;
	vehicleSimplePoint end;
	it = path.begin();
	start = *it;
	it++;
	while (it != path.end()) {
		end = *it;
		drawLine(start.x * (scaleFactor_ / WP::WORLD_SPACE_GRANULARITY),
				start.y * (scaleFactor_ / WP::WORLD_SPACE_GRANULARITY),
				end.x * (scaleFactor_ / WP::WORLD_SPACE_GRANULARITY),
				end.y * (scaleFactor_ / WP::WORLD_SPACE_GRANULARITY), R, G, B);
		start = end;
		it++;
	}

}

void DiscWorldVisualizer::drawLine(double xfrom, double yfrom, double xto, double yto, int R, int G, int B) {
	// reverse y axis!
	yfrom = imgYsize_ - yfrom;
	yto = imgYsize_ - yto;
	// lock the mutex
	boost::mutex::scoped_lock lock(update_mutex_);
	cvLine(worldDisplayImg_, cvPoint((int) xfrom, (int) yfrom), cvPoint((int) xto, (int) yto),
			cvScalar(R, G, B), 1, 8, 0);
}

void DiscWorldVisualizer::drawDot(double x, double y, double radius, int R, int G, int B) {
	// reverse the y axis for correct visualization
	y = imgYsize_ - y;
	// lock the mutex before writing
	boost::mutex::scoped_lock lock(update_mutex_);
	cvCircle(worldDisplayImg_, cvPoint((int) x, (int) y), radius, cvScalar(R, G, B), CV_FILLED);
}

void DiscWorldVisualizer::drawFilledRectangle(double xfrom, double yfrom, double xto, double yto, int R,
		int G, int B) {
	// reverse the y axis for correct visualization
	yfrom = imgYsize_ - yfrom;
	yto = imgYsize_ - yto;
	// lock the mutex before writing
	boost::mutex::scoped_lock lock(update_mutex_);
	cvRectangle(worldDisplayImg_, cvPoint((int) xfrom, (int) yfrom), cvPoint((int) xto, (int) yto),
			cvScalar(R, G, B), CV_FILLED);
}

void DiscWorldVisualizer::drawRectangle(double xfrom, double yfrom, double xto, double yto, int R, int G,
		int B) {
	// reverse the y axis for correct visualization
	yfrom = imgYsize_ - yfrom;
	yto = imgYsize_ - yto;
	// lock the mutex before writing
	boost::mutex::scoped_lock lock(update_mutex_);
	cvRectangle(worldDisplayImg_, cvPoint((int) xfrom, (int) yfrom), cvPoint((int) xto, (int) yto),
			cvScalar(R, G, B), 1, 8, 0);
}

