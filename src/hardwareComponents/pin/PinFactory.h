/*
 * PinFactory.h
 *
 *  Created on: 30 mar. 2018
 *      Author: root
 */

#ifndef SRC_PIN_PINFACTORY_H_
#define SRC_PIN_PINFACTORY_H_

#include <Arduino.h>
#include "Pin.h"

class PinFactory {
public:
	static Pin* getDigitalPin(uint8_t pin, uint8_t mode);
	static Pin* getDigitalPin(uint8_t pin);
	static Pin* getAnalogPin(uint8_t pin, int analogChannel);
private:
	static void init(uint8_t pin, uint8_t mode);
};

#endif /* SRC_PIN_PINFACTORY_H_ */
