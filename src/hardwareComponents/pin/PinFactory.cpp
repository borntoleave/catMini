/*
 * PinFactory.cpp
 *
 *  Created on: 30 mar. 2018
 *      Author: root
 */

#include "PinFactory.h"

#define DEFAULT_ANALOG_FREQUENCY 5000
#define DEFAULT_ANALOG_RESOLUTION 8
int ledChannel = 0;


Pin* PinFactory::getDigitalPin(uint8_t pin, uint8_t mode){
	init(pin, mode);
	return new Pin(pin);
}

Pin* PinFactory::getDigitalPin(uint8_t pin){
	return new Pin(pin);
}

void PinFactory::init(uint8_t pin, uint8_t mode){
	pinMode(pin	, mode);
}


Pin* PinFactory::getAnalogPin(uint8_t pin, int analogChannel){
	ledcSetup(analogChannel, DEFAULT_ANALOG_FREQUENCY, DEFAULT_ANALOG_RESOLUTION);
	ledcAttachPin(pin, ledChannel);
	return new Pin(pin, analogChannel);
}
