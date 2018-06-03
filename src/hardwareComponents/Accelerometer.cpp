/*
 * Accelerometer.cpp
 *
 *  Created on: 27 may. 2018
 *      Author: ziash
 */

#include "Accelerometer.h"

Accelerometer::Accelerometer(int sdaPin, int sclPin) {
	i2cInit(sdaPin, sclPin);
	configure();
}

void Accelerometer::i2cInit(int sdaPin, int sclPin){
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)sdaPin;
	conf.scl_io_num = (gpio_num_t)sclPin;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

void Accelerometer::configure(){
	mpu.initialize();
	mpu.dmpInitialize();

	mpu.setXGyroOffset(X_GYRO_OFFSET);
	mpu.setYGyroOffset(Y_GYRO_OFFSET);
	mpu.setZGyroOffset(Z_GYRO_OFFSET);
	mpu.setZAccelOffset(Z_ACCEL_OFFSET);

	mpu.setDMPEnabled(true);
}

void Accelerometer::getData(float* yaw, float* pitch, float* roll){
	uint16_t fifoCount = 0;
	while (fifoCount < DEFAULT_PACKET_SIZE) fifoCount = mpu.getFIFOCount();
	readDataPacket(yaw, pitch, roll);
}

boolean Accelerometer::isDataAvailable(){
	uint8_t  mpuIntStatus 	= mpu.getIntStatus();
	uint16_t fifoCount 		= mpu.getFIFOCount();

	if(isOverflowed(mpuIntStatus, fifoCount)){
		mpu.resetFIFO();
	}

	if (mpuIntStatus & REGISTER_DATA_AVAILABLE){
		return true;
	}

	return false;
}

boolean Accelerometer::isOverflowed(uint16_t fifoCount, uint8_t mpuIntStatus){
	 if ((mpuIntStatus & 0x10) || fifoCount == MAX_FIFO_SIZE){
	      return true;
	   }
	 return false;
}

void Accelerometer::readDataPacket(float* yaw, float* pitch, float* roll){
	Quaternion 	quaternion;
	VectorFloat gravity;
	float ypr[3];
	uint8_t fifoBuffer[DEFAULT_STORAGE_PACKET_SIZE];

	mpu.getFIFOBytes(fifoBuffer, DEFAULT_PACKET_SIZE);
	mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &quaternion);
	mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);

	*yaw 	= toDegrees(ypr[0]);
	*pitch 	= toDegrees(ypr[1]);
	*roll 	= toDegrees(ypr[2]);

	printf("YAW: %3.1f, ", ypr[0] * 180/M_PI);
	printf("PITCH: %3.1f, ", ypr[1] * 180/M_PI);
	printf("ROLL: %3.1f \n", ypr[2] * 180/M_PI);
}

float Accelerometer::toDegrees(float radian){
	return radian * 180 /M_PI;
}
