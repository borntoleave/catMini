/*
 * ServoController.cpp
 *
 *  Created on: 12 may. 2018
 *      Author: ziash
 */

#include <controllers/ServoController.h>

ServoController::ServoController() {}

void ServoController::initialize(){
	pwm.begin();
	pwm.setPWMFreq(SERVO_FREQUENCY);
}

void ServoController::shutServos() {
  for (byte i = 0; i < DOF; i++) {
    pwm.setPWM(i, 0, 4096);
  }
}

void ServoController::saveEEPROMCalibration(int8_t *var) {
  for (byte i = 0; i < DOF; i++) {
    //EEPROM.update(CALIB + i, var[i]);
    //TODO calculate calibratedDuty0
    //calibratedDuty0[i] = SERVOMIN + RANGE / 2 + float(middleShift(i) + var[i]) * RANGE * rotationDirection(i) / rangeRatio(i);
  }
}

void ServoController::threadTask(){
	/**
	 {
	    if (token == 'p' || token == 'g') {
	      for (int i = offset; i < DOF; i++) {
	        int dutyIdx =  t * WalkingDOF - offset + i;
	        calibratedPWM(i, (dutyAng[dutyIdx] + adjust(i)) / (1 + sqrt(fabs(ypr[1] * ypr[2])) / M_PI * 2) );
	      }
	      t = (t + 1) % tPeriod;
	      byte pause = 0;
	      if (lastCmd[0] == 'w' && lastCmd[1] == 'k')
	        pause = 5;
	      else
	        pause = 0;
	     // if (WalkingDOF == 8)
	     //   pause *= 2;
	      delay(pause);
	    }
	}
	**/
}
