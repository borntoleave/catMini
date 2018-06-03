/*
 * SerialCommunicationController.cpp
 *
 *  Created on: 12 may. 2018
 *      Author: ziash
 */

#include "SerialCommunicationController.h"

SerialCommunicationController* SerialCommunicationController::serialCommunicationController = NULL;

SerialCommunicationController::SerialCommunicationController() {
	emptyBuffer();
	Logger::log(Logger::Debug, "SerialCommunicationController::SerialCommunicationController - Serial Communication controler created");
}

SerialCommunicationController* SerialCommunicationController::getInstance(){
	if(serialCommunicationController == NULL){
		serialCommunicationController = new SerialCommunicationController();
	}
	return serialCommunicationController;
}


void SerialCommunicationController::threadTask() {
	if(!Serial.available()){
		return;
	}
	String command = Serial.readStringUntil(SERIAL_TERMINATOR_CHARACTER);
	parseCommand(command);
}


void SerialCommunicationController::emptyBuffer(){
	 while (Serial.available() && Serial.read());
}

int SerialCommunicationController::parseCommand(String command){
	if(command == "shutDownServos"){
		return shutDownServos;
	}
	Logger::log(Logger::Error, "SerialCommunicationController::parseCommand - Unknown command " + command);
	return noState;
}


