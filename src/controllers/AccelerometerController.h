/*
 * AccelerometerController.h
 *
 *  Created on: 27 may. 2018
 *      Author: ziash
 */

#ifndef SRC_CONTROLLERS_ACCELEROMETERCONTROLLER_H_
#define SRC_CONTROLLERS_ACCELEROMETERCONTROLLER_H_

#include "Arduino.h"
#include "utils/Logger.h"
#include "../hardwareComponents/Accelerometer.h"


#define THREAD_UPDATE_TIME 500
#define PIN_SDA 22
#define PIN_SCL 19


class AccelerometerController {
public:
	AccelerometerController();
	static AccelerometerController* getInstance();
	void threadTask();
private:
	static AccelerometerController* accelerometerController;
	float yaw;
	float pitch;
	float roll;
	Accelerometer* acc = new Accelerometer(PIN_SDA, PIN_SCL);
	unsigned long previousTime = millis();
};

#endif /* SRC_CONTROLLERS_ACCELEROMETERCONTROLLER_H_ */
