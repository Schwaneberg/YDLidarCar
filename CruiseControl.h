/*
 * CruiseControl.h
 * Simple demonstration how to use the scan data and control the car.
 *  Created on: 18.12.2018
 *      Author: Oliver Schwaneberg
 */

#ifndef CRUISECONTROL_H_
#define CRUISECONTROL_H_

#include "CarControl.h"
#include "Lidar.h"

class ObjectOfInterest {
public:
	ObjectOfInterest():
		start_angle(0),
		end_angle(0),
		start_distance(1000000),
		end_distance(1000000),
		start_x(0),
		start_y(0),
		end_x(0),
		end_y(0),
		num_points(0)
	{};

	float start_angle;
	float end_angle;
	float start_distance;
	float end_distance;
	float start_x;
	float start_y;
	float end_x;
	float end_y;
	int num_points;
};


class CruiseControl {
public:
	CruiseControl();
	virtual ~CruiseControl();

	/*
	 * Start cruise control - a simple autonomous driver that tries
	 * not to hit anything.
	 */
	void start();

private:
	enum carState {IDLE, CRUISE, REVERSING, STEERING, STOP};
	static void processScan(std::vector<std::tuple<float, float>> scanData);
	static bool isWithinEllipse(float distance, float angle);
	static void convertToXY(float distance, float angle, float *x, float *y);
#define PI 3.1415926535f
#define CRUISE_SPEED		35
#define STEER_SPEED			35
#define CORNERING_SPEED     20
#define REVERSE_SPEED		-20

	/*
	 * The ELLIPSE_RADIUS_X and _Y define the elliptical shape around
	 * the car which is used as a region of interest for the navigation
	 * algorithm in this class.
	 * Note that the car drives along the y axis.
	 */
#define ELLIPSE_RADIUS_X	0.30
#define ELLIPSE_RADIUS_Y	1.00

	/*
	 * Set true and the program will write all measure points into
	 * a file named "data.csv".
	 * The points are represented as coordinates with prefix
	 * 'F' for data used for navigation in forward direction
	 * 'R' for data used for navigation in backward direction
	 * 'I' for data which is not evaluated by the navigation algorithm.
	 */
#define RECORD_RAW			false
};


#endif /* CRUISECONTROL_H_ */
