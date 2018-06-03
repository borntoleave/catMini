/*
 * ServoController.h
 *
 *  Created on: 12 may. 2018
 *      Author: ziash
 */

#ifndef SRC_CONTROLLERS_SERVOCONTROLLER_H_
#define SRC_CONTROLLERS_SERVOCONTROLLER_H_

#include "Adafruit_PWMServoDriver.h"
#include <EEPROM.h>

#define FLAT 5
#define panF 1 / 2.0
#define tF 1.0
#define sXF 2
#define sYF 1/4.0
#define uF 0.8
#define dF -0.8
float coeff[16][2] = {
  { -panF, 0}, {0, -tF}, {0, 0}, {0, 0},
  {sXF, -sYF}, { -sXF, -sYF}, { -sXF, sYF}, {sXF, sYF},
  { uF, 1}, { -uF, 1}, { -uF , -1 }, { uF , -1 },
  { dF, dF}, { -dF, dF}, { -dF, -dF}, { dF, -dF}
};
#define DOF 16

#define SERVO_FREQUENCY 60

class ServoController {
public:
	ServoController();
	void initialize();
	void shutServos();
	void saveEEPROMCalibration(int8_t *var);
	void threadTask();
private:
	Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
	int8_t servoCalibs[DOF] = {};
	char currentAng[DOF] = {};
};

#endif /* SRC_CONTROLLERS_SERVOCONTROLLER_H_ */
