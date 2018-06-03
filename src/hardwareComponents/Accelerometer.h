/*
 * Accelerometer.h
 *
 *  Created on: 27 may. 2018
 *      Author: ziash
 */

#ifndef SRC_ACCELEROMETER_H_
#define SRC_ACCELEROMETER_H_

#include <Arduino.h>
#include "MPU6050.h"


#define X_GYRO_OFFSET 				220
#define Y_GYRO_OFFSET 				76
#define Z_GYRO_OFFSET 				-85
#define Z_ACCEL_OFFSET 				1788

#define MAX_FIFO_SIZE 				1024
#define REGISTER_OVERFLOW 			0x10
#define REGISTER_DATA_AVAILABLE 	0x02

#define DEFAULT_PACKET_SIZE 		42
#define DEFAULT_STORAGE_PACKET_SIZE 64

class Accelerometer {
public:
	Accelerometer(int sdaPin, int sclPin);
	boolean isDataAvailable();
	void 	getData(float* yaw, float* pitch, float* roll);
	void readData();
private:
	MPU6050 	mpu = MPU6050();
	void 	i2cInit(int sdaPin, int sclPin);
	void 	configure();
	boolean isOverflowed(uint16_t fifoCount, uint8_t mpuIntStatus);
	void 	readDataPacket(float* yaw, float* pitch, float* roll);
	float 	toDegrees(float radian);

};

#endif /* SRC_ACCELEROMETER_H_ */
