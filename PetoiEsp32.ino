#include <Arduino.h>

#include "controllers/SerialCommunicationController.h"
#include "utils/Logger.h"
#include "controllers/GaitController.h"
#include "controllers/AccelerometerController.h"


#define SERIALPORT_BAUDRATE 115200
#define SERIALPORT_TIMEOUT	10		//ms

GaitController gaitController = GaitController();


void setup() {
	Serial.begin(SERIALPORT_BAUDRATE);
	Serial.setTimeout(SERIALPORT_TIMEOUT);
	Serial.println("************* *************");
	Serial.println("**** Petoi started ********");
	Serial.println("************* *************");
}

void loop() {
	SerialCommunicationController::getInstance()->threadTask();
	AccelerometerController::getInstance()->threadTask();
}
