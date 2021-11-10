/*
 * CruiseControl.cpp
 *
 *  Created on: 18.12.2018
 *      Author: oliver
 */

#include "CruiseControl.h"
#include <math.h>
#include <fstream>
#include <sstream>
#include <iomanip>
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
	static carState newState = IDLE, prevState = IDLE;
	static ObjectOfInterest ooi;
	ObjectOfInterest new_ooi;
	static float last_mid_ooi_angle = -1.0;
	static float last_mid_ooi_distance = -100.0;
	auto &car = CarControl::getInstance();
	float steerTo = 0; //Straight
	float minDistFront = 10.0, minDistBack = 10.0;
	float angleToMinDistFrontDeg = 0.0, angleToMinDistFront = 0.0;
	static float x, y;
	auto isDriveOverloaded = car.isDriveOverloaded();
	auto now = std::chrono::high_resolution_clock::now();
	/*static auto prev_time = now;
	std::chrono::duration<double> pausetime = now - prev_time;
	std::cout << "Rate: " << 1.0f / pausetime.count() << std::endl;
	prev_time = now;*/
	std::chrono::duration<double> elapsed = now - collisionTime;
	elapsed = now - reversionTime;
	bool reversionTimeLock = elapsed.count() < 1.5;
#if RECORD_RAW
	static uint32_t count = 0;
	std::fstream fs;
	fs.open ("data.csv", std::fstream::in | std::fstream::out | std::fstream::app);
	fs << "NEW DATA " << count++ << endl;
#endif
	if (isDriveOverloaded) {
		cout << "OVERLOAD" << endl;
		if (prevState == REVERSING) {
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
	} else if (!reversionTimeLock) {
		for (auto tuple : scanData) {

			// current angle

			auto angleDeg = std::get<0>(tuple);
			auto distance = std::get<1>(tuple);
			auto angle = angleDeg * (PI / 180.0f);
			//current intensity
			//int intensity = scan.intensities[i];
			convertToXY(distance, angle, &x, &y);
#if RECORD_RAW
				fs << "F\t" << x << "\t" << y << "\t:\t" << angleDeg << "\t" << distance << endl;
#endif


			if (distance <= 1.0 && distance < new_ooi.start_distance - 0.06)
			{
				// cout << "START " << angle << std::endl;
				// Merke neuen Startpunkt
				new_ooi.start_distance = distance;
				new_ooi.start_angle = angle;
				new_ooi.start_x = x;
				new_ooi.start_y = y;
				new_ooi.num_points = 0;
			} else if (distance < new_ooi.start_distance + 0.06)
			{
				// cout << "ADD " << angle << " np " << new_ooi.num_points << std::endl;
				// Punkt hat bis auf 60mm den gleichen Abstand wie der Startpunkt
				new_ooi.num_points++;
				new_ooi.end_distance = distance;
				new_ooi.end_angle = angle;
				new_ooi.end_x = x;
				new_ooi.end_y = y;
			} else if (new_ooi.num_points >= 3)
			{
				auto ooi_width = sqrt(pow(new_ooi.start_x - new_ooi.end_x, 2)
								+ pow(new_ooi.start_y - new_ooi.end_y, 2));
				// cout << "OBJ at " << new_ooi.start_angle << " / " << ooi.start_distance << " / " << ooi_width << std::endl;
				if (ooi_width >= 0.03 && ooi_width <= 0.1)
				{
					// cout << "Replacing OOI" << endl;
					ooi.start_angle = new_ooi.start_angle;
					ooi.start_distance = new_ooi.start_distance;
					ooi.start_x = new_ooi.start_x;
					ooi.start_y = new_ooi.start_y;
					ooi.end_angle = new_ooi.end_angle;
					ooi.end_distance = new_ooi.end_distance;
					ooi.end_x = new_ooi.end_x;
					ooi.end_y = new_ooi.end_y;
					ooi.num_points = new_ooi.num_points;
				}
			}
		}
		if (ooi.num_points >=3)
		{
			auto mid_ooi_angle = (ooi.start_angle + ooi.end_angle) * 0.5;
			auto mid_ooi_distance = (ooi.start_distance + ooi.end_distance) * 0.5;

			if (mid_ooi_angle > last_mid_ooi_angle + 5
					|| mid_ooi_angle < last_mid_ooi_angle - 5
					|| mid_ooi_distance > last_mid_ooi_distance + 0.08
					|| mid_ooi_distance < last_mid_ooi_distance - 0.08)
			{
				std::ostringstream object_description;
				cout << object_description.str() << endl;
				object_description << setprecision(3) << "Detected object at " << mid_ooi_angle << " degrees and " << mid_ooi_distance << " meters distance.";
				cout << object_description.str() << endl;
				ev3dev::sound::speak(object_description.str(), false);
				last_mid_ooi_angle = mid_ooi_angle;
				last_mid_ooi_distance = mid_ooi_distance;
			}

			if (last_mid_ooi_angle > 3 && last_mid_ooi_angle < 357)
			{
				cout << "a: " << last_mid_ooi_angle << " d: " << last_mid_ooi_distance << endl;
				// Wenden
				if (!reversionTimeLock)
				{
					if (newState != REVERSING)
					{
						cout << "GO" << endl;
						car.setDriveSpeed(REVERSE_SPEED);
						auto curSteerDeg = car.getSteerDegree();
						car.steerHardLeft();
						newState = REVERSING;
						reversionTime = std::chrono::high_resolution_clock::now();
					} else if (newState == REVERSING) {
						cout << "REV" << endl;
						reversionTime = std::chrono::high_resolution_clock::now();
						car.setDriveSpeed(CORNERING_SPEED);
						car.steerToPos(car.getSteerPos() * -1);
						newState = CRUISE;
					}
				}
				else {
					cout << "lock" << endl;
				}
			}
			else{
				if (car.getSteerDegree() != 0)
				{
					car.stop();
					car.steerStraight();
				}
				if (mid_ooi_distance > 0.36)
				{
					car.setDriveSpeed(REVERSE_SPEED);
					newState = REVERSING;
				}
				else
				{
					newState = IDLE;
					car.stop();
					car.closeFork();
				}
			}
		}

		/*convertToXY(minDistFront, angleToMinDistFront, &x, &y);
		auto weight = 1.33 - (((pow(x, 2.0) / pow(ELLIPSE_RADIUS_X, 2.0)) + (pow(y, 2.0) / pow(ELLIPSE_RADIUS_Y, 2.0))));

		cout << "front: " << minDistFront << "\t" << angleToMinDistFrontDeg
				<< "\tback: " << minDistBack << endl;

		if (newState != REVERSING) {
			/*
			 * The closer the obstacle, the higher the weight.
			 * Maximum is weight 1.33
			 */
			/*if (weight <= 1.18) {
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
					&& minDistBack > 0.25) {
				car.setDriveSpeed(CRUISE_SPEED);
				car.steerToPos(car.getSteerPos() * -1);
				newState = CRUISE;
			} else if (minDistBack < 0.30) {
				car.setDriveSpeed(0);
				newState = CRUISE;
			}
		}*/
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
