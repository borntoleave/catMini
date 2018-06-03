/*
 * PinArduino.h
 *
 *  Created on: 30 mar. 2018
 *      Author: root
 */

#ifndef SRC_PIN_PIN_H_
#define SRC_PIN_PIN_H_

#include <Arduino.h>

class Pin {
public:
	Pin();
	Pin(uint8_t pin);
	Pin(uint8_t pin, int analogChannel);
	int getPinValue();
	int getPinNumber();
	void setDigitalPinValue(uint8_t value);
	void setAnalogPinValue(uint8_t pinValue);
private:
	uint8_t pinNumber;
	int analogChannel;
};

#endif /* SRC_PIN_PIN_H_ */
