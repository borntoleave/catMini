/*
 * PinArduino.cpp
 *
 *  Created on: 30 mar. 2018
 *      Author: root
 */

#include "Pin.h"

Pin::Pin() {
}

Pin::Pin(uint8_t pinNumber) {
	this->pinNumber = pinNumber;
}

Pin::Pin(uint8_t pin, int analogChannel){
	this->pinNumber = pinNumber;
	this->analogChannel = analogChannel;
}

int Pin::getPinNumber(){
	return pinNumber;
}

int Pin::getPinValue(){
	return digitalRead(pinNumber);
}

void Pin::setDigitalPinValue(uint8_t pinValue){
	digitalWrite(pinNumber, pinValue);
}

void Pin::setAnalogPinValue(uint8_t pinValue){
	ledcWrite(analogChannel, pinValue);
}
