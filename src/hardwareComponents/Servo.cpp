/*
 * Servo.cpp
 *
 *  Created on: 12 may. 2018
 *      Author: ziash
 */

#include <hardwareComponents/Servo.h>



Servo::Servo(byte index) {
	this->index = index;
}

byte Servo::pin(){
  return EEPROMMemory::readByte(EEPROM_POSITION_PIN + index);
}

int8_t Servo::middleShift() {
  return EEPROMMemory::readInt8t(EEPROM_POSITION_MIDSHIFT + index);
}
byte Servo::rangeRatio() {
  return EEPROMMemory::readByte(EEPROM_POSITON_RANGE_RATIO + index);
}
int8_t Servo::rotationDirection() {
  return EEPROMMemory::readInt8t(EEPROM_POSITION_ROATION_DIRECTION + index);
}
int8_t Servo::servoCalib() {
  return EEPROMMemory::readInt8t(EEPROM_POSITION_CALIBRATION + index);
}

void Servo::calibrate(byte i, int dt){
	//TODO review servo calibration
	/**
	char duty = max(-128, min(127, dt));
	if (i > 3 && i < 8)
		duty = max(-5, duty);
	//currentAng[i] = duty;
	//dt = calibratedDuty0[i] + float(duty) * RANGE * rotationDirection(i) / rangeRatio(i);
	dt = max(SERVOMIN + RANGE / 2 * BOUND, min(SERVOMAX - RANGE / 2 * BOUND, dt));
	//pwm.setPWM(pin(i), 0, dt);
	 *
	 */
}

