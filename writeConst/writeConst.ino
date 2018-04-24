/*
 *  Save calibration data and constants to EEPROM
 *  
 *  Copyright (c) 2018 Petoi LLC.  All right reserved.
 *  Written by Rongzhong Li
 *  
 *  This sketch also includes others' codes under MIT liscence. 
 *  See those liscences in corresponding sections.
 */

#define INIT true
#define LITE true
#include <EEPROM.h>
#define SPT(s) Serial.print(s)
#define SPTL(s) Serial.println(s)
#define SPTF(s) Serial.print(F(s))

#define MELODY 1023
#define PIN 0
#define CALIB 16
#define MIDSHIFT 32
#define ROTDIRECTION 48
#define RANGERATIO 64
#define MPUCALIB 80

#define DOF 16
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
#define BOUND 1/8
int range = SERVOMAX - SERVOMIN;
// our servo # counter

int8_t servoCalibs[DOF] = {};
char currentAng[DOF] = {};
int calibratedDuty0[DOF];
const char *gaits[] = {"vt", "rest", "calib", "balance",};
const char vt[] PROGMEM = { 30,
                            3, -3, -2,  2, 51, 45, 27, 22, -12,  3,  6, 22,
                            3, -2, -2,  2, 49, 45, 25, 22, -6,  3, 13, 22,
                            3, -2, -2,  1, 45, 45, 22, 22,  3,  3, 22, 22,
                            3, -1, -2,  1, 45, 45, 22, 22,  3,  3, 22, 22,
                            3, -1, -2,  1, 45, 45, 22, 22,  3,  3, 22, 22,
                            2,  1, -1,  1, 45, 46, 22, 22,  3,  1, 22, 20,
                            2,  1, -1, -1, 45, 49, 22, 25,  3, -6, 22, 13,
                            1,  2, -1, -1, 45, 51, 22, 27,  3, -12, 22,  6,
                            1,  2,  1, -1, 45, 54, 22, 29,  3, -19, 22,  1,
                            -1,  3,  1, -2, 45, 56, 22, 31,  3, -25, 22, -6,
                            -1,  3,  1, -2, 45, 59, 22, 33,  3, -33, 22, -12,
                            -2,  3,  1, -2, 45, 59, 22, 32,  3, -31, 22, -11,
                            -2,  3,  2, -2, 45, 56, 22, 31,  3, -25, 22, -6,
                            -3,  3,  2, -2, 45, 54, 22, 29,  3, -19, 22,  1,
                            -3,  3,  2, -2, 45, 51, 22, 27,  3, -12, 22,  6,
                            -3,  3,  2, -2, 45, 49, 22, 25,  3, -6, 22, 13,
                            -3,  3,  2, -2, 45, 45, 22, 22,  3,  3, 22, 22,
                            -2,  3,  2, -2, 45, 45, 22, 22,  3,  3, 22, 22,
                            -2,  3,  1, -2, 45, 45, 22, 22,  3,  3, 22, 22,
                            -1,  2,  1, -1, 45, 45, 22, 22,  3,  3, 22, 22,
                            -1,  2,  1, -1, 45, 45, 22, 22,  3,  3, 22, 22,
                            1,  1,  1, -1, 46, 45, 22, 22,  1,  3, 20, 22,
                            1,  1, -1,  1, 49, 45, 25, 22, -6,  3, 13, 22,
                            2, -1, -1,  1, 51, 45, 27, 22, -12,  3,  6, 22,
                            2, -1, -1,  1, 54, 45, 29, 22, -19,  3,  1, 22,
                            3, -2, -2,  1, 56, 45, 31, 22, -25,  3, -6, 22,
                            3, -2, -2,  2, 59, 45, 33, 22, -33,  3, -12, 22,
                            3, -3, -2,  2, 59, 45, 32, 22, -31,  3, -11, 22,
                            3, -3, -2,  2, 56, 45, 31, 22, -25,  3, -6, 22,
                            3, -3, -2,  2, 54, 45, 29, 22, -19,  3,  1, 22,
                            '\0'
                          };


const char rest[] PROGMEM = { 1,
                              1, 10,  1,  1, -3, -3,  3,  3, 60, 60, 60, 60, -30, -30, -30, -30, '\0'
                            };
const char calib[] PROGMEM = { 1,
                               1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1, '\0'
                             };
const char balance[] PROGMEM = { 1,
                                 1, 20,  1,  1, -3, -3,  3,  3, 30, 30, 30, 30, 30, 30, 30, 30, '\0'
                               };
const char* const gaits_table[] PROGMEM = {vt, rest, calib, balance, };
char *dutyAng = new char [362];

void saveCalib(int8_t *var) {
  for (byte i = 0; i < DOF; i++) {
    EEPROM.update(CALIB + i, var[i]);
    calibratedDuty0[i] = SERVOMIN + range / 2 + float(middleShift(i) + var[i]) * range  * rotationDirection(i) / rangeRatio(i);
  }
}

void saveMPUcalib(int * var) {
  for (byte i = 0; i < 6; i++)
    EEPROM.update(MPUCALIB + i, var[i]);
}

void calibratedPWM(byte i, char dt) {
  currentAng[i] = dt;
  int duty = calibratedDuty0[i] + float(dt) * range  * rotationDirection(i) / rangeRatio(i);
  duty = min(SERVOMAX - range / 2 * BOUND, duty);
  duty = max(SERVOMIN + range / 2 * BOUND, duty);
  pwm.setPWM(pin(i), 0, duty);
}
void allCalibratedPWM(char * dutyAng) {
  for (byte i = 0; i < DOF; i++) {
    calibratedPWM(i, dutyAng[i]);
  }
}

void transform(char * current, char * target, byte offset = 0, float speedRatio = 1) {
  int *diff = new int[DOF - offset], maxDiff = 0;
  for (byte i = offset; i < DOF; i++) {
    diff[i - offset] =   current[i] - target[i - offset];
    maxDiff = max(maxDiff, abs( diff[i - offset]));
  }
  int steps = int(round(maxDiff / 3.0/*degreeStep*/ / speedRatio));
  for (byte s = 0; s < steps; s++)
    for (byte i = offset; i < DOF; i++) {
      int duty = (target[i] + (1 + cos(M_PI * s / steps)) / 2 * diff[i - offset]);
      calibratedPWM(i,  duty);
    }
  delete [] diff;
}

void shutServos() {
  delay(1000);
  for (byte i = 0; i < DOF; i++) {
    pwm.setPWM(i, 0, 4096);
  }
}

int8_t idxOfGait(const char * gait) {
  for (byte i = 0; i < sizeof(gaits) / sizeof(char*); i++)
    if (!strcmp(gaits[i], gait))
      return i;
  return -1;
}

int pgmCpy(char * dutyAng, const char * gait) {
  dutyAng--;
  strcpy_P(dutyAng, (char*)pgm_read_word(&(gaits_table[idxOfGait(gait)])));
  int period = dutyAng[0];
  dutyAng++;
  return period;
}

#include <EEPROM.h> //Needed to access the eeprom read write functions

byte pin(byte idx) {
  return (byte)EEPROM.read(PIN + idx);
}

int8_t middleShift(byte idx) {
  return (int8_t)EEPROM.read( MIDSHIFT + idx);
}
byte rangeRatio(byte idx) {
  return (byte)EEPROM.read(RANGERATIO + idx);
}
int8_t rotationDirection(byte idx) {
  return (int8_t)EEPROM.read(ROTDIRECTION + idx);
}
int8_t servoCalib(byte idx) {
  return (int8_t)EEPROM.read( CALIB + idx);
}

//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value)
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.update(p_address, lowByte);
  EEPROM.update(p_address + 1, highByte);
}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

void writeConst(bool lite = true) {
  // tone: pause,1,  2,  3,  4,  5,  6,  7,  1,  2, 3,
  // code: 0,    1,  3,  5,  6,  8,  10, 12, 13, 15, 17
  byte melody[] = {8, 13, 10, 13, 8,  0,  5,  8,  3,  5, 8,
                   8, 8,  32, 32, 8, 32, 32, 32, 32, 32, 8,
                  };
  EEPROM.update(MELODY, sizeof(melody));
  for (byte i = 0; i < sizeof(melody); i++)
    EEPROM.update(MELODY - 1 - i, melody[i]);

  int8_t calibs[] = {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};

  int8_t middleShifts[] = {0, 45, 0, 0,
                           -45, -45, -45, -45,
                           0, 0, 0, 0,
                           0, 0, 0, 0
                          };
  int8_t rotationDirections[] = {1, -1, 1, 1,
                                 1, -1, 1, -1,
                                 1, -1, 1, -1,
                                 -1, 1, -1, 1
                                };
  byte mg90s = 180 , mg92b =  180;
  byte rangeRatios[] =  {mg92b, mg90s, mg92b, mg92b,
                         mg92b, mg92b, mg92b, mg92b,
                         mg92b, mg92b, mg92b, mg92b,
                         mg92b, mg92b, mg92b, mg92b
                        };

  for (byte i = 0; i < DOF; i++) {
    if (lite) {
      byte pins[] = {0, 0, 0, 0, 0, 0, 0, 0, 5, 4, 10, 11, 7, 6,8, 9};
      //byte pins[] = { 0, 1, 0, 0,  0, 0,  0, 0, 3, 2,  6, 7,  5, 4,  8, 9};
      EEPROM.update(PIN + i, pins[i]);
    }
    else {
      byte pins[] = { 1, 0, 15, 14,  3, 2,  8, 9, 5, 4,  10, 11,  7, 6,  12, 13};
      EEPROM.update(PIN + i, pins[i]);
    }
    if (INIT)
      EEPROM.update(CALIB + i, calibs[i]);
    EEPROM.update(MIDSHIFT + i, middleShifts[i]);
    EEPROM.update(ROTDIRECTION + i, rotationDirections[i]);
    EEPROM.update(RANGERATIO + i, rangeRatios[i]);
    SPT(servoCalib(i));
    SPT(',');
  }
  SPT('\n');
}

#define BUZZER 4
#define MELODY 1023
void beep(byte note, float duration = 10, int pause = 0, byte repeat = 1 ) {
  if (note == 0) {
    analogWrite(BUZZER, 0);
    delay(duration);
    return;
  }
  int freq = 220 * pow(1.059463, note);
  float period = 1000000.0 / freq / 2.0;
  for (byte r = 0; r < repeat; r++) {
    for (float t = 0; t < duration * 1000; t += period * 2) {
      analogWrite(BUZZER, 150);      // Almost any value can be used except 0 and 255
      // experiment to get the best tone
      delayMicroseconds(period);          // wait for a delayms ms
      analogWrite(BUZZER, 0);       // 0 turns it off
      delayMicroseconds(period);          // wait for a delayms ms
    }
    delay(pause);
  }
}

// http://wired.chillibasket.com/2015/01/calibrating-mpu6050/

// Arduino sketch that returns calibration offsets for MPU6050
//   Version 1.1  (31th January 2014)
// Done by Luis Ródenas <luisrodenaslorda@gmail.com>
// Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net>
// Updates (of the library) should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

// These offsets were meant to calibrate MPU6050's internal DMP, but can be also useful for reading sensors.
// The effect of temperature has not been taken into account so I can't promise that it will work if you
// calibrate indoors and then use it outdoors. Best is to calibrate and use at the same room temperature.

/* ==========  LICENSE  ==================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2011 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  =========================================================
*/

// I2Cdev and MPU6050 must be installed as libraries
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int discard = 100;
int buffersize = 1000;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 mpu;
MPU6050 mpu(0x68); // <-- use for AD0 high

int16_t ax, ay, az, gx, gy, gz;

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

int t = 0;
int tPeriod = 1;
byte offset;
char token;
#define CMD_LEN 10
char lastCmd[CMD_LEN] = {};
bool printMPU = false;
int mpuOffset[6];
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(57600);
  Serial.setTimeout(5);
  while (!Serial);

  for (byte i = 0; i < 6; i++)
    SPTL(EEPROMReadInt(MPUCALIB + i * 2));
  // initialize device
  mpu.initialize();

  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  /*while (!Serial.available()){
    Serial.println(F("Send any character to start sketch.\n"));
    delay(1500);
    }
    while (Serial.available() && Serial.read()); // empty buffer again
  */
  SPT(F("* RoPet Writing Constants to EEPROM *\n"));
  writeConst(LITE); // only run for the first time when writing to the board.

  // servo
  { pwm.begin();
    pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
    delay(200);

    strcpy(lastCmd, "rest");
    pgmCpy(dutyAng, "rest");
    for (byte i = 0; i < DOF; i++) {
      servoCalibs[i] = servoCalib(i);
      calibratedDuty0[i] =  SERVOMIN + range / 2 + float(middleShift(i) + servoCalibs[i]) * range   * rotationDirection(i) / rangeRatio(i);
      calibratedPWM(i, dutyAng[i]);
    }
    shutServos();
    token = 'd';
  }
  beep(30);

  // start message
  Serial.println("\nMPU6050 Calibration Sketch");
  delay(2000);
  Serial.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
  delay(3000);
  // verify connection
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(1000);
  // reset offsets
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
}

void loop() {
  if (state == 0) {
    Serial.println("\nReading sensors for first time...");
    meansensors();
    state++;
    delay(1000);
  }

  if (state == 1) {
    Serial.println("\nCalculating offsets...");
    calibration();
    state++;
    delay(1000);
  }

  if (state == 2) {
    meansensors();
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor readings with offsets:\t");
    Serial.print(mean_ax);
    Serial.print("\t");
    Serial.print(mean_ay);
    Serial.print("\t");
    Serial.print(mean_az);
    Serial.print("\t");
    Serial.print(mean_gx);
    Serial.print("\t");
    Serial.print(mean_gy);
    Serial.print("\t");
    Serial.println(mean_gz);
    Serial.print("Your offsets:\t");
    Serial.print(ax_offset);
    Serial.print("\t");
    Serial.print(ay_offset);
    Serial.print("\t");
    Serial.print(az_offset);
    Serial.print("\t");
    Serial.print(gx_offset);
    Serial.print("\t");
    Serial.print(gy_offset);
    Serial.print("\t");
    Serial.println(gz_offset);
    Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
    Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
    Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");

    mpuOffset[0] = ax_offset;
    mpuOffset[1] = ay_offset;
    mpuOffset[2] = az_offset;
    mpuOffset[3] = gx_offset;
    mpuOffset[4] = gy_offset;
    mpuOffset[5] = gz_offset;

    for (byte i = 0; i < 6; i++)
      EEPROMWriteInt(MPUCALIB + i * 2, mpuOffset[i]);

    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
    //while (1);
    state = 3;
  }

  char cmd[CMD_LEN] = {};
  byte newCmd = 0;
  if (t == 0 && Serial.available() > 0) {
    token = Serial.read();
    newCmd = 3;
  }
  if (newCmd) {
    beep(newCmd * 10);
    // this block handles argumentless tokens

    if (token == 'm')
      printMPU = !printMPU;
    if (token == 'h')
      SPTF("** Help Information **");// print the help document

    else if (token == 'd' ) {
      pgmCpy(dutyAng, "rest");
      transform(currentAng, dutyAng);
      SPTF("shut down servos");
      shutServos();
    }
    else if (token == 's') {
      SPT(F("save calibration"));
      saveCalib(servoCalibs);
    }
    else if (token == 'a') {
      SPT(F("abandon calibration"));
      for (byte i = 0; i < DOF; i++) {
        servoCalibs[i] = servoCalib( i);
      }
    }
    // this block handles array like arguments
    else if (token == 'c' || token == 't') {
      int target[2] = {};
      String inBuffer = Serial.readStringUntil('\n');
      strcpy(cmd, inBuffer.c_str());
      char *pch;
      pch = strtok (cmd, " ,");
      for (int c = 0; pch != NULL; c++)
      {
        target[c] = atoi(pch);
        pch = strtok (NULL, " ,");
      }

      if (token == 'c') {
        //SPTF("calibrating [ targetIdx, angle ]: ");
        if (strcmp(lastCmd, "c")) { //first time entering the calibration function
          pgmCpy(dutyAng, "calib");
          transform(currentAng, dutyAng);
          shutServos();
        }
        SPT('\n');
        for (byte i = 0; i < DOF; i++) {
          SPT(i);
          SPT(",\t");
        }
        SPT('\n');
        for (byte i = 0; i < DOF; i++) {
          SPT(servoCalibs[i]);
          SPT(",\t");
        }
        SPT('\n');
        yield();
        servoCalibs[target[0]] = target[1];
      }
      else if (token == 't') {
        //SPTF("moving [ targetIdx, angle ]: ");
        dutyAng[target[0]] = target[1];
      }

      SPT(target[0]);
      SPT(",\t");
      SPT(target[1]);

      int duty = SERVOMIN + range / 2 + float(middleShift(target[0])  + servoCalibs[target[0]] + dutyAng[target[0]]) * range * rotationDirection(target[0]) / rangeRatio(target[0]);
      pwm.setPWM(pin(target[0]), 0,  duty);
      shutServos();
    }

    else if (Serial.available() > 0) {
      String inBuffer = Serial.readStringUntil('\n');
      strcpy(cmd, inBuffer.c_str());
    }
    while (Serial.available() && Serial.read()); //flush the remaining serial buffer in case the commands are parsed incorrectly

    if (strcmp(cmd, "") && strcmp(lastCmd, cmd) ) {
      if (token == 'p' || token == 'g') { //validating key
        if (idxOfGait(cmd) >= 0) {
          tPeriod =  pgmCpy(dutyAng, cmd);
          t = 0;
          if (strcmp(cmd, "balance") )
            strcpy(lastCmd, cmd);
          offset = (tPeriod == 1) ? 0 : 4;
          transform(currentAng, dutyAng, offset, 0.5);
        }
        else
          SPT(F("wrong key!\n"));
      }
      else {
        lastCmd[0] = token;
        memset(lastCmd + 1, '\0', CMD_LEN - 1);
      }
    }
  }

  //motion block
  {
    if (printMPU) {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      SPTF("ax\tay\taz\tgx\tgy\tgz: ");
      SPT(ax);
      SPT('\t');
      SPT(ay);
      SPT('\t');
      SPT(az);
      SPT('\t');
      SPT(gx);
      SPT('\t');
      SPT(gy);
      SPT('\t');
      SPT(gz);
      SPT('\n');
    }
    if (token == 'p' || token == 'g') {
      for (int i = offset; i < DOF; i++) {
        int dutyIdx =  t * 12 - offset + i;
        calibratedPWM(pin(i), dutyAng[dutyIdx]);

      }
      t = (t + 1) % tPeriod;

      delay(30);
    }
  }
}

void meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (buffersize + discard + 1)) {
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > discard && i <= (buffersize + discard)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + discard)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration() {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 8;
  gy_offset = -mean_gy / 8;
  gz_offset = -mean_gz / 8;
  while (1) {
    int ready = 0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax) <= acel_deadzone) {
      Serial.print (1);
      ready++;
    }
    else ax_offset = ax_offset - mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone)  {
      Serial.print (2);
      ready++;
    }
    else ay_offset = ay_offset - mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone)  {
      Serial.print (3);
      ready++;
    }
    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone) {
      Serial.print (4);
      ready++;
    }
    else gx_offset = gx_offset - mean_gx / (giro_deadzone + 2);

    if (abs(mean_gy) <= giro_deadzone) {
      Serial.print (5);
      ready++;
    }
    else gy_offset = gy_offset - mean_gy / (giro_deadzone + 2);

    if (abs(mean_gz) <= giro_deadzone)  {
      Serial.print (6);
      ready++;
    }
    else gz_offset = gz_offset - mean_gz / (giro_deadzone + 2);

    for(int i=0;i<ready;i++){
        beep(30);
        delay(200);
    }
    if (ready == 6) 
      break;
    
  }
}
