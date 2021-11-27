/*
 * Lidar.cpp
 *
 *  Created on: 16.12.2018
 *      Author: oliver
 */

#include "Lidar.h"
#include <stdio.h>
#include <iostream>
#include <thread>
#include <chrono>

namespace lidar {

Lidar::Lidar() {
	std::cout << "Init Lidar..." << std::endl;

	/*
	 * Baudrate for ydlidar X4 is always 128000
	 * Note: the baudrate must be altered for
	 * other types.
	 */
	baudrate = 128000;
	std::string intensity;
	ydlidar::init(0, nullptr);
	std::map<std::string, std::string> ports =  ydlidar::YDlidarDriver::lidarPortList();
	std::map<std::string,std::string>::iterator it;
	if(ports.size()==1) {
		it = ports.begin();
		port = it->second;
	} else {
		std::cerr << "Did not detect YDLidar port!" << std::endl;
	}
	std::cout << "Port: " << port << std::endl;

	cylidar.setSerialPort(port);
	cylidar.setSerialBaudrate(baudrate);
	cylidar.setIntensities(false);
	cylidar.setAutoReconnect(true);//hot plug
	cylidar.setReversion(false);
	cylidar.setFixedResolution(false);
	cylidar.setScanFrequency(8);
	cylidar.initialize();
}

Lidar::~Lidar() {}

Lidar& Lidar::getInstance()
{
	static Lidar instance;
	return instance;
}

bool Lidar::isReady()
{
	return ydlidar::ok();
}

std::vector<std::tuple<float, float>> Lidar::scan() {
	/*
	 * Assuming a resolution of 0.5 degree at
	 * standard scan speed of 8 Hz
	 */
	static node_info nodes[720];
	static size_t count = _countof(nodes);
	std::vector<std::tuple<float, float>> scanData;
	//  wait Scan data:
	//uint64_t tim_scan_start = getCurrentTime();
	while (!IS_OK(cylidar.lidarPtr->grabScanData(nodes, count))) {
		std::cout << "sleep" << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	};
	//uint64_t tim_scan_end = getCurrentTime();

	auto op_result = cylidar.lidarPtr->ascendScanData(nodes, count);

	if (IS_OK(op_result)) {
		for (unsigned int i = 0; i < count; i++) {
			if (nodes[i].distance_q2 != 0) {
				float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);
				scanData.push_back(std::make_tuple(angle, nodes[i].distance_q2 / 4000.0f));
			}
		}
	}
	return scanData;
}

}
