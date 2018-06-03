/*
 * Beep.cpp
 *
 *  Created on: 30 mar. 2018
 *      Author: ziash
 */

#include "Beep.h"

Beep::Beep() {
	this->beepPin = PinFactory::getAnalogPin(BEEP_PIN, ANALOG_CHANNEL);
}


void Beep::playMelody(int eepromLocation) {
	/**
	byte len = (byte)EEPROM.read(eepromLocation) / 2;
	  for (int i = 0; i < len; i++)
		  playNote(EEPROM.read(eepromLocation - 1 - i), 1000 / EEPROM.read(eepromLocation - 1 - len - i), BEEP_DEFAULT_LENGTH);
	**/
}


void Beep::playNote(byte note, float duration, int pause, byte repeat){
	if (note == 0) {
		beepPin->setAnalogPinValue(note);
		delay(duration);
		return;
	}

	int freq = 220 * pow(1.059463, note);
	float period = 1000000.0 / freq / 2.0;
	for (byte r = 0; r < repeat; r++) {
		for (float t = 0; t < duration * 1000; t += period * 2) {
			playSound(t);
		}
		delay(pause);
	}
}

void Beep::playSound(float delay){
	beepPin->setAnalogPinValue(BEEP_HIGH);
	delayMicroseconds(delay);
	beepPin->setAnalogPinValue(BEEP_LOW);
	delayMicroseconds(delay);
}
