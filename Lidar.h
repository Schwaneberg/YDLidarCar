/*
 * Lidar.h
 * Frontend to the ydlidar_driver
 *  Created on: 16.12.2018
 *      Author: Oliver Schwaneberg
 */

#ifndef LIDAR_H_
#define LIDAR_H_

#include <CYdLidar.h>
#include <iostream>
#include <string>
#include <memory>

using namespace ydlidar;

namespace lidar {

class Lidar {
public:
	/*
	 * @return true if Lidar is ready to use
	 */
	bool isReady();

	/*
	 * @return static instance of Lidar
	 */
	static Lidar& getInstance();

	CYdLidar laser;

	/*
	 * Perform a 360 degree scan and return results
	 * @return tuples of angle (0.0 to 359.9 degree, clockwise)
	 *         and distance in meter.
	 */
	// std::vector<std::tuple<float, float>> scan();
	// LaserScan scan;

private:
	Lidar();
	virtual ~Lidar();
	std::string port;
	int baudrate;
};

} /* namespace ydlidar */

#endif /* LIDAR_H_ */
