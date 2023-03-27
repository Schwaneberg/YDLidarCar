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
	 * Get the color from the color sensor.
	    - 0: No color
        - 1: Black
        - 2: Blue
        - 3: Green
        - 4: Yellow
        - 5: Red
        - 6: White
        - 7: Brown
	 */
	int getColor();

	/*
	 * Preset to center steering.
	 */
	void steerStraight();

	/*
	 * Set steering to angle
	 * @param angle -30 (right) over 0 (center) to 30 (left) degree
	 */
	void steerToAbsDegree(int angle, int speed);

	/*
	 * @param speed in percent.
	 * -100 percent is maximum velocity in backwards direction.
	 * 0 stops the motor.
	 * 100 percent is the maximum velocity.
	 */
	void setDriveSpeed(int percent);

	/*
	 * Stop the car
	 */
	void stop();
	
	/*
	 * @return true if drive is stalled
	 */
	bool isDriveOverloaded();

	/*
	* @return true if motors are running
	*/
	bool isRunning();

private:
	CarControl();
	virtual ~CarControl();
	ev3dev::large_motor leftMotor;
	ev3dev::large_motor rightMotor;
	ev3dev::touch_sensor bumper;
	ev3dev::color_sensor colorSensor;
};

} /* namespace CarControl */

#endif /* CARCONTROL_H_ */
