/*
 * CarControl.h
 * Control the Lego Mindstorms components of the YDLidarCar with abstraction.
 *  Created on: 16.12.2018
 *      Author: Oliver Schwaneberg
 */

#ifndef CARCONTROL_H_
#define CARCONTROL_H_

#include <ev3dev.h>

#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>

namespace car {

class CarControl {
public:
	/*
	 * Define the maximum steering angles
	 * and targeted speed of the car.
	 */
	#define MAX_STEER_ANGLE 30
	#define MIN_STEER_ANGLE	-30
	#define STEER_MOTOR_SPEED 350

	/*
	 * HOLD = Motor holds position
	 * MOVING = Motor steers to new position
	 * STALLED = Motor tries to move but
	 *           steering is mechanically blocked
	 */
	enum steerState{HOLD, MOVING, STALLED};

	/*
	 * @return static instance of CarControl
	 * (Singleton)
	 */
	static CarControl& getInstance();

	/*
	 * Reset steering
	 * @return success
	 */
	bool reset();

	/*
	 * @return true if steering is calibrated.
	 * Note: This function uses hard coded values which are valid for the
	 * YDLidarCar. The test might fail with different steering layouts.
	 */
	bool isReady();

	/*
	 * Calibrate steering
	 */
	void calibSteering();

	/*
	 * Steer to absoulte motor position
	 * @param absolute position
	 */
	void steerToPos(int pos);

	/*
	 * @return current absolute position of steering motor
	 */
	int getSteerPos();

	/*
	 * Preset to steer left at maximum angle.
	 */
	void steerHardLeft();

	/*
	 * Preset to steer right at maximum angle.
	 */
	void steerHardRight();

	/*
	 * Preset to center steering.
	 */
	void steerStraight();

	/*
	 * Set steering to angle
	 * @param angle -30 (right) over 0 (center) to 30 (left) degree
	 */
	void steerToAbsDegree(int angle);

	/*
	 * @return current position of steering in degree
	 */
	int getSteerDegree();

	/*
	 * @param speed in percent.
	 * -100 percent is maximum velocity in backwards direction.
	 * 0 stops the motor.
	 * 100 percent is the maximum velocity.
	 */
	void setDriveSpeed(int percent);

	/*
	 * @return true if the bumper switch is pressed
	 */
	bool isBumperPressed();

	/*
	 * @return true if drive is stalled
	 */
	bool isDriveOverloaded();

	/*
	 * @return current state of steering motor
	 */
	steerState getSteeringState();
private:
	CarControl();
	virtual ~CarControl();
	ev3dev::large_motor driveMotor;
	ev3dev::medium_motor steeringMotor;
	ev3dev::touch_sensor bumperSensor;	// Switch to detect collisions
	int steeringRange;					// Positive or negative range from neutral position
};

} /* namespace CarControl */

#endif /* CARCONTROL_H_ */
