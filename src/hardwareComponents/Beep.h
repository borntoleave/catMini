/*
 * Beep.h
 *
 *  Created on: 30 mar. 2018
 *      Author: ziash
 */

#ifndef SRC_UTILS_BEEP_H_
#define SRC_UTILS_BEEP_H_

#include "pin/Pin.h"
#include "pin/PinFactory.h"
#include <utils/Pins.h>

#define BEEP_LOW	0
#define BEEP_HIGH	150
#define BEEP_DEFAULT_LENGTH	100
#define ANALOG_CHANNEL 0

class Beep {
public:
	Beep();
	void playMelody(int eepromLocation);
private:
	Pin* beepPin;
	void playNote(byte note, float duration = 10, int pause = 0, byte repeat = 1);
	void playSound(float delay);
};

#endif /* SRC_UTILS_BEEP_H_ */
