/*
 * Servo.h
 *
 *  Created on: 12 may. 2018
 *      Author: ziash
 */

#ifndef SRC_HARDWARECOMPONENTS_SERVO_H_
#define SRC_HARDWARECOMPONENTS_SERVO_H_

#include "Arduino.h"
#include "EEPROMMemory.h"

#define SERVOMIN  150 //out of 4096
#define SERVOMAX  600 //out of 4096
#define BOUND 1/8
#define RANGE (SERVOMAX - SERVOMIN)

class Servo {

public:
	Servo(byte index);
	byte index;
	int calibratedDuty0;
	byte pin();
	int8_t middleShift();
	byte rangeRatio();
	int8_t rotationDirection();
	int8_t servoCalib();
	void calibrate(byte i, int dt);
};

#endif /* SRC_HARDWARECOMPONENTS_SERVO_H_ */
