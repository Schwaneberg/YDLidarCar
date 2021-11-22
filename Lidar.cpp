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
#include <fstream>

#define LOGGING false

namespace lidar {

Lidar::Lidar() {
	std::cout << "Init Lidar..." << std::endl;

	/*
	 * Baudrate for ydlidar X4 is always 128000
	 * Note: the baudrate must be altered for
	 * other types.
	 */
	ydlidar::os_init();
	baudrate = 128000;

	//////////////////////string property/////////////////
	/// lidar port
	port = "/dev/ttyUSB0";
	laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
	/// ignore array
	std::string ignore_array;
	ignore_array.clear();
	laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
					ignore_array.size());

	//////////////////////int property/////////////////
	/// lidar baudrate
	laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
	/// tof lidar
	int optval = TYPE_TRIANGLE;
	laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
	/// device type
	optval = YDLIDAR_TYPE_SERIAL;
	laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
	/// sample rate
	optval = 5; // or 4?
	laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
	/// abnormal count
	optval = 10;
	laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

	//////////////////////bool property/////////////////
	/// fixed angle resolution
	bool b_optvalue = false;
	laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
	/// rotate 180
	laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
	/// Counterclockwise
	laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
	b_optvalue = true;
	laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
	b_optvalue = false;
	/// one-way communication
	laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
	/// intensity
	laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
	/// Motor DTR
	b_optvalue = true;
	laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
	/// HeartBeat
	b_optvalue = false;
	laser.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));

	//////////////////////float property/////////////////
	/// unit: °
	float f_optvalue = 180.0f;
	float frequency = 5.0;
	laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
	f_optvalue = -180.0f;
	laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
	/// unit: m
	f_optvalue = 10.f;
	laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
	f_optvalue = 0.12f;
	laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
	/// unit: Hz
	laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

	bool ret = laser.initialize();

	if (ret) {
	ret = laser.turnOn();
	} else {
	fprintf(stderr, "%s\n", laser.DescribeError());
	fflush(stderr);
	}

}

Lidar::~Lidar() {}

Lidar& Lidar::getInstance()
{
	static Lidar instance;
	return instance;
}

bool Lidar::isReady()
{
	return ydlidar::os_isOk();
}

}
