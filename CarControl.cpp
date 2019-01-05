/*
 * CarControl.cpp
 *
 *  Created on: 16.12.2018
 *      Author: oliver
 */

#include "CarControl.h"

namespace car {

CarControl::CarControl() {
	steeringRange = 0;

	if (reset())
	{
		calibSteering();
	}
}

CarControl::~CarControl() {
	// TODO Auto-generated destructor stub
}

void CarControl::calibSteering()
{
	/*
	 * Check maximum angles, calibrate to middle
	 */
	int endPos, leftMax, rightMax;
	steeringMotor.set_stop_action("hold");
	std::string state;
	steeringMotor.set_position(0);
	steeringMotor.set_polarity("normal").set_speed_sp(STEER_MOTOR_SPEED).run_forever();
	while (steeringMotor.state().count("stalled") == 0 && steeringMotor.state().count("overloaded") == 0);
		    //this_thread::sleep_for(chrono::milliseconds(10));
	steeringMotor.stop();
	endPos = steeringMotor.position();


	leftMax = endPos;

	steeringMotor.set_polarity("inversed").set_speed_sp(STEER_MOTOR_SPEED).run_forever();
	while (steeringMotor.state().count("stalled") == 0);
	    //this_thread::sleep_for(chrono::milliseconds(10));
	steeringMotor.stop();
	endPos = steeringMotor.position()*-1;

	rightMax = endPos;

	std::cout << "leftMax: " << leftMax << " rightMax: " << rightMax << std::endl;

	std::cout << "go to: " <<  (leftMax+rightMax)/2 << std::endl;
	steeringMotor.set_polarity("normal").set_speed_sp(STEER_MOTOR_SPEED).set_position_sp((leftMax+rightMax)/2).run_to_abs_pos();
	while (steeringMotor.position() != (leftMax+rightMax)/2 && steeringMotor.state().count("stalled") == 0)
	{
		//std::cout << steeringMotor.position()<< " != " << (leftMax+rightMax)/2 << std::endl;
		std::cout << ".";
	}
	std::cout << std::endl;
	// Set 0 point to neutral position
	steeringMotor.set_position(0);
	steeringMotor.stop();
	steeringRange = ((rightMax > leftMax ? rightMax - leftMax : leftMax - rightMax) / 2) - 4; // Decrease by 4 counts to avoid stalled state
	std::cout << "range:" << steeringRange << std::endl;
}

void CarControl::steerToPos(int pos)
{
	int curPos = steeringMotor.position();
	if (curPos > pos) {
		steeringMotor.set_speed_sp(-1 * STEER_MOTOR_SPEED).set_position_sp(pos).set_stop_action("hold").run_to_abs_pos();
	} else if (curPos < pos){
		steeringMotor.set_speed_sp(STEER_MOTOR_SPEED).set_position_sp(pos).run_to_abs_pos();
	}
}

int CarControl::getSteerPos()
{
	return steeringMotor.position();
}

void CarControl::steerHardLeft()
{
	steerToPos(steeringRange);
}

void CarControl::steerHardRight()
{
	steerToPos(steeringRange * -1);
}

void CarControl::steerStraight()
{
	steerToPos(0);
}

/*
 * @brief Steer by degree
 * @param -30° (rightmost) to 30° (leftmost)
 */
void CarControl::steerToAbsDegree(int angle)
{
	static double countsPerDegree =  static_cast<double>(steeringRange) / 30.0;
	if (angle > MAX_STEER_ANGLE) {
		angle = MAX_STEER_ANGLE;
	} else if (angle < MIN_STEER_ANGLE) {
		angle = -MIN_STEER_ANGLE;
	}
	int absPos = static_cast<int>(angle * countsPerDegree);
	steerToPos(absPos);
}

int CarControl::getSteerDegree()
{
	static double countsPerDegree =  static_cast<double>(steeringRange) / 30.0;
	return static_cast<int>(static_cast<double>(steeringMotor.position()) / countsPerDegree);
}

CarControl::steerState CarControl::getSteeringState()
{
	if (steeringMotor.state().count("stalled"))
		return STALLED;
	else if (steeringMotor.state().count("running") || steeringMotor.state().count("ramping"))
		return MOVING;
	else
		return HOLD;
}

void CarControl::setDriveSpeed(int percent)
{
	static int previously = 0;
	if (percent != previously) {
		driveMotor.set_speed_sp((driveMotor.max_speed()/100) * percent).run_forever();
		previously = percent;
	}
}

bool CarControl::isBumperPressed()
{
	return bumperSensor.is_pressed();
}

bool CarControl::isDriveOverloaded()
{
	return driveMotor.state().count("stalled") || driveMotor.state().count("overloaded");
}

bool CarControl::isReady()
{
	// Simply check the calibration range against expected range
	bool ready = steeringRange > 68 && steeringRange < 90;
	if (!ready) {
		std::cerr << "Steering range not ok!!" << std::endl;
	}
	return ready;
}

CarControl& CarControl::getInstance()
{
	static CarControl instance;
	return instance;
}

bool CarControl::reset()
{
	bool success = true;
	if (driveMotor.connected())
		driveMotor.reset();
	else
		success = false;

	if (steeringMotor.connected())
		steeringMotor.reset();
	else
		success = false;

	if (!bumperSensor.connected())
		success = false;
	return success;
}

} /* namespace CarControl */
