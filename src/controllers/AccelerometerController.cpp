/*
 *
 * AccelerometerController.cpp
 *
 *  Created on: 27 may. 2018
 *      Author: ziash
 */

#include <controllers/AccelerometerController.h>

AccelerometerController* AccelerometerController::accelerometerController = NULL;

AccelerometerController::AccelerometerController() {}

AccelerometerController* AccelerometerController::getInstance(){
	if(accelerometerController == NULL){
		accelerometerController = new AccelerometerController();
	}
	return accelerometerController;
}

void AccelerometerController::threadTask() {
	if(millis() - previousTime < THREAD_UPDATE_TIME){
		return;
	}

	if(acc->isDataAvailable()){
		acc->getData(&yaw, &pitch, &roll);
	}
	previousTime = millis();
}
