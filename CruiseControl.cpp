/*
 * CruiseControl.cpp
 *
 *  Created on: 18.12.2018
 *      Author: oliver
 */

#include "CruiseControl.h"
#include <math.h>
#include <fstream>
using namespace std;
using namespace lidar;
using namespace car;


CruiseControl::CruiseControl() {

}

/*
 * Is coordinate within defined ellipse around car
 */
bool CruiseControl::isWithinEllipse(float x, float y)
{
	bool isWithin = ((pow(x, 2.0) / pow(ELLIPSE_RADIUS_X, 2.0)) + (pow(y, 2.0) / pow(ELLIPSE_RADIUS_Y, 2.0))) < 1.0;
	return isWithin;
}

/*
 * Convert angle and distance to x y coordinates
 */
void CruiseControl::convertToXY(float distance, float angle, float *x, float *y)
{
	*y = distance * cos(angle);
	*x = distance * sin(angle);
}

/*
 * Process scan data and control car.
 */
void CruiseControl::processScan(std::vector<std::tuple<float, float>> scanData)
{
	static auto collisionTime = std::chrono::high_resolution_clock::now() - std::chrono::seconds(5);
	static auto reversionTime = std::chrono::high_resolution_clock::now() - std::chrono::seconds(5);
	static carState newState = CRUISE, prevState = IDLE;
	auto &car = CarControl::getInstance();
	float steerTo = 0; //Straight
	float minDistFront = 10.0, minDistBack = 10.0;
	float angleToMinDistFrontDeg = 0.0, angleToMinDistFront = 0.0;
	static float x, y;
	auto isBumperPressed = car.isBumperPressed();
	auto isDriveOverloaded = car.isDriveOverloaded();
	auto now = std::chrono::high_resolution_clock::now();
	/*static auto prev_time = now;
	std::chrono::duration<double> pausetime = now - prev_time;
	std::cout << "Rate: " << 1.0f / pausetime.count() << std::endl;
	prev_time = now;*/
	std::chrono::duration<double> elapsed = now - collisionTime;
	bool bumperTimeLock = elapsed.count() < 5.0;
	elapsed = now - reversionTime;
	bool reversionTimeLock = elapsed.count() < 3.5;
#if RECORD_RAW
	static uint32_t count = 0;
	std::fstream fs;
	fs.open ("data.csv", std::fstream::in | std::fstream::out | std::fstream::app);
	fs << "NEW DATA " << count++ << endl;
#endif
	if (isBumperPressed) {
		auto curSteerDeg = car.getSteerDegree();
		if (curSteerDeg > 10 || curSteerDeg < -10) {
			car.steerToAbsDegree(curSteerDeg * -1);
		} else {
			car.steerHardLeft();
		}
		car.setDriveSpeed(REVERSE_SPEED);
		newState = REVERSING;
		collisionTime = std::chrono::high_resolution_clock::now();
	} else if (isDriveOverloaded) {
		if (prevState == REVERSING && !bumperTimeLock) {
			newState = STEERING;
			car.steerStraight();
			car.setDriveSpeed(STEER_SPEED);
		} else {
			newState = REVERSING;
			reversionTime = std::chrono::high_resolution_clock::now();
			auto curSteerDeg = car.getSteerDegree();
			if (curSteerDeg > 10 || curSteerDeg < -10) {
				car.steerToAbsDegree(curSteerDeg * -1);
			} else {
				car.steerHardLeft();
			}
			car.setDriveSpeed(REVERSE_SPEED);
		}
	} else if (!bumperTimeLock && !reversionTimeLock) {
		for (auto tuple : scanData) {

			// current angle

			auto angleDeg = std::get<0>(tuple);
			auto distance = std::get<1>(tuple);
			auto angle = angleDeg * (PI / 180.0f);
			//current intensity
			//int intensity = scan.intensities[i];
			convertToXY(distance, angle, &x, &y);


			if (angleDeg < 60 || angleDeg > 300) {
				// Front
#if RECORD_RAW
				fs << "F\t" << x << "\t" << y << "\t:\t" << angleDeg << "\t" << distance << endl;
#endif
				if (isWithinEllipse(x, y) && distance < minDistFront) {
					minDistFront = distance;
					angleToMinDistFrontDeg = angleDeg;
					angleToMinDistFront = angle;
				}
			} else if (angleDeg > 120 && angleDeg < 240) {
				// Back
#if RECORD_RAW
				fs << "B\t" << x << "\t" << y << "\t:\t" << angleDeg << "\t" << distance << endl;
#endif
				if (isWithinEllipse(x, y)
						&& distance < minDistBack) {
					minDistBack = distance;
				}

#if RECORD_RAW
				else {
					fs << "I\t" << x << "\t" << y << "\t:\t" << angleDeg << "\t" << distance << endl;
				}
#endif
			}
		}

		convertToXY(minDistFront, angleToMinDistFront, &x, &y);
		auto weight = 1.33 - (((pow(x, 2.0) / pow(ELLIPSE_RADIUS_X, 2.0)) + (pow(y, 2.0) / pow(ELLIPSE_RADIUS_Y, 2.0))));

		cout << "front: " << minDistFront << "\t" << angleToMinDistFrontDeg
				<< "\tback: " << minDistBack << endl;

		if (newState != REVERSING) {
			/*
			 * The closer the obstacle, the higher the weight.
			 * Maximum is weight 1.33
			 */
			if (weight <= 1.18) {
				if (angleToMinDistFrontDeg > 180.0) {
					steerTo = MIN_STEER_ANGLE * weight;
					cout << "right: " << steerTo << " w: " << weight << endl;
				} else {
					steerTo = MAX_STEER_ANGLE * weight;
				}
				if ( minDistFront < ELLIPSE_RADIUS_Y) {
					car.steerToAbsDegree(steerTo);
					car.setDriveSpeed(STEER_SPEED);
					newState = STEERING;
				} else {
					car.steerStraight();
					car.setDriveSpeed(CRUISE_SPEED);
					newState = CRUISE;
				}
			} else {
				car.setDriveSpeed(REVERSE_SPEED);
				auto curSteerDeg = car.getSteerDegree();
				if (curSteerDeg > 15 || curSteerDeg < -15) {
					car.steerToAbsDegree(curSteerDeg * -1);
				} else {
					if (angleToMinDistFrontDeg > 180.0) {
						car.steerHardLeft();
					} else {
						car.steerHardRight();
					}
				}
				newState = REVERSING;
				reversionTime = std::chrono::high_resolution_clock::now();
			}
		} else if (newState == REVERSING) {
			if (minDistFront > 0.33
					&& !bumperTimeLock
					&& minDistBack > 0.25) {
				car.setDriveSpeed(CRUISE_SPEED);
				car.steerToPos(car.getSteerPos() * -1);
				newState = CRUISE;
			} else if (minDistBack < 0.30) {
				car.setDriveSpeed(0);
				newState = CRUISE;
			}
		}
	}

	//auto end = std::chrono::high_resolution_clock::now();
	//std::chrono::duration<double> elapsed = end - start;
	//cout << "CB rt: " << elapsed.count() << " state = " << newState << endl;
#if RECORD_RAW
	fs.close();
#endif
	prevState = newState;
}

void CruiseControl::start() {
	auto &car = CarControl::getInstance();
	LaserScan scan;
	if (car.isReady()) {
		ev3dev::sound::speak("Car is ready!", true);
		//ev3dev::sound::play("lenkung.wav");
		this_thread::sleep_for(chrono::milliseconds(3000));
		auto &lidar = Lidar::getInstance();
		if (lidar.isReady()) {
			//ev3dev::sound::play("lidar.wav");
			//this_thread::sleep_for(chrono::milliseconds(3000));
			while (1) {
				processScan(lidar.scan());
			}
		} else {
			ev3dev::sound::speak("Lidar is not working!", true);
			//ev3dev::sound::play("lidarKaputt.wav");
		}
	} else {
		ev3dev::sound::speak("Car is not working!", true);
		//ev3dev::sound::play("lenkungKaputt.wav");
	}
}

CruiseControl::~CruiseControl() {

}
