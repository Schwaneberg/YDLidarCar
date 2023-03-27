/*
 * CarControl.cpp
 *
 *  Created on: 16.12.2018
 *      Author: oliver
 */

#include "CarControl.h"

namespace car {

CarControl::CarControl() :
	bumper(ev3dev::INPUT_1),
	colorSensor(ev3dev::INPUT_2),
	rightMotor(ev3dev::OUTPUT_A),
	leftMotor(ev3dev::OUTPUT_D)
{
	// Nothing to do
	rightMotor.set_stop_action("hold");
	leftMotor.set_stop_action("hold");
	colorSensor.set_mode("COL-COLOR");
}

CarControl::~CarControl() {
	// TODO Auto-generated destructor stub
}

void CarControl::steerStraight()
{
	auto max_speed = leftMotor.max_speed();
	leftMotor.set_speed_sp(max_speed).run_forever();
	rightMotor.set_speed_sp(max_speed).run_forever();
}

int CarControl::getColor()
{
	return colorSensor.value();
}

/*
 * @brief Steer by degree
 * @param -30° (rightmost) to 30° (leftmost)
 */
void CarControl::steerToAbsDegree(int angle, int percent)
{
	//static double countsPerDegree =  static_cast<double>(angle) * 10.0;
	int timePerDregree;
	auto max_speed = leftMotor.max_speed();
	if (angle > 0)
	{
		// Turn right
		leftMotor.set_speed_sp((leftMotor.max_speed()/100) * -percent);
		rightMotor.set_speed_sp((rightMotor.max_speed()/100) * percent / 2);
		timePerDregree = angle * 20;
	}
	else
	{
		// Turn left
		leftMotor.set_speed_sp((leftMotor.max_speed()/100) * percent / 2);
		rightMotor.set_speed_sp((rightMotor.max_speed()/100) * -percent);
		timePerDregree = angle * -20;
	}
	leftMotor.set_time_sp(timePerDregree).run_timed();
	rightMotor.set_time_sp(timePerDregree).run_timed();
}

void CarControl::setDriveSpeed(int percent)
{
	leftMotor.set_speed_sp(-(leftMotor.max_speed()/100) * percent).run_forever();
	rightMotor.set_speed_sp(-(rightMotor.max_speed()/100) * percent).run_forever();
}

bool CarControl::isRunning()
{
	return leftMotor.state().count("running") || rightMotor.state().count("running");
}

bool CarControl::isDriveOverloaded()
{
	return leftMotor.state().count("stalled") || leftMotor.state().count("overloaded") || rightMotor.state().count("stalled") || rightMotor.state().count("overloaded");
}

bool CarControl::isReady()
{
	return leftMotor.connected() && rightMotor.connected() && colorSensor.connected() && bumper.connected();
}

CarControl& CarControl::getInstance()
{
	static CarControl instance;
	return instance;
}

void CarControl::stop()
{
	std::cout << "STOP!" << std::endl;
	leftMotor.stop();
	rightMotor.stop();
}

bool CarControl::reset()
{
	bool success = true;
	if (leftMotor.connected())
		leftMotor.reset();
	else
	{
		std::cerr << "Left motor not connected!" << std::endl;
		success = false;
	}

	if (rightMotor.connected())
		rightMotor.reset();
	else
	{
		std::cerr << "Right motor not connected!" << std::endl;
		success = false;
	}

	return success;
}

} /* namespace CarControl */
