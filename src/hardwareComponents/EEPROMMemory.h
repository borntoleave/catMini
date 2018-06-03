/*
 * EEPROM.h
 *
 *  Created on: 30 mar. 2018
 *      Author: ziash
 */

#ifndef SRC_UTILS_EEPROMMEMORY_H_
#define SRC_UTILS_EEPROMMEMORY_H_

#include <Arduino.h>
#include "EEPROM.h"

#define EEPROM_POSITION_PIN			 		0
#define EEPROM_POSITION_CALIBRATION 		16
#define EEPROM_POSITION_MIDSHIFT 			32
#define EEPROM_POSITION_ROATION_DIRECTION 	48
#define EEPROM_POSITON_RANGE_RATIO 			64
#define EEPROM_POSITION_MPU_CALIBRATION 	80

#define EEPROM_POSITION_MPU_Z_ACCELERATION_OFFSET	4
#define EEPROM_POSITION_MPU_X_GYRO_OFFSET		 	6
#define EEPROM_POSITION_MPU_Y_GYRO_OFFSET 			8
#define EEPROM_POSITION_MPU_Z_GYRO_OFFSET 			10

class EEPROMMemory {
public:
	EEPROMMemory();
	static void writeInt(int p_address, int p_value);
	static int readInt(int p_address);
	static int8_t readInt8t(byte address);
	static byte readByte(byte address);
};

#endif /* SRC_UTILS_EEPROMMEMORY_H_ */
