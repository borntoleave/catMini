//http://forum.arduino.cc/index.php?topic=396450
#include <EEPROM.h>
#define DOF 16
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define SPT(s) Serial.print(s)
bool blinkState = false;
const byte wakePin = 4;

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

int range = SERVOMAX - SERVOMIN;
// our servo # counter
byte servoIdx = 0;
byte pins[] = { 0, 1, 15, 14,  3, 2,  8, 9, 5, 4,  10, 11,  7, 6,  12, 13};
int8_t servoCalib[DOF] = {};
int8_t middleShifts[DOF] = {0, 45, 0, 0,
                            -45, -45, -45, -45,
                            0, 0, 0, 0,
                            0, 0, 0, 0
                           };

int8_t rotationDirections[DOF] = {1, -1, 1, 1,
                                  1, -1, 1, -1,
                                  1, -1, 1, -1,
                                  -1, 1, -1, 1
                                 };
int8_t current[DOF] = {};
float panF = 1 / 4.0,
      tF = 1.0,
      sXF = 1 / 5.0,
      sYF = 1 / 5.0,
      uF = 1 / 3.0,
      dF = 0.6;
int duty0[DOF];
int calibratedDuty0[DOF];

//movements and postures
#include "modes.h"

int t = 0;
byte tPeriod = 1;
byte cycle = 0;
char lastToken[DOF] = {};

int8_t getCalib(int idx) {
  return (int8_t)EEPROM[ idx ];
}
void saveCalib(int8_t *var) {
  for (byte i = 0; i < DOF; i++) {
    EEPROM.update(i, var[i]);
    yield();
    calibratedDuty0[i] = duty0[i] + float(var[i]) * range / 180.0  * rotationDirections[i];
  }

}
void shutServos() {
  delay(100);
  for (byte servoIdx = 0; servoIdx < DOF; servoIdx++) {
    pwm.setPWM(pins[servoIdx], 0, 4096);
  }
}

int8_t idxOfPose(const char * pose) {
  for (byte i = 0; i < sizeof(poses) / sizeof(char*); i++)
    if (!strcmp(poses[i], pose))
      return i;
  return -1;
}
int8_t idxOfGait(const char * gait) {
  for (byte i = 0; i < sizeof(gaits) / sizeof(char*); i++)
    if (!strcmp(gaits[i], gait))
      return i;
  return -1;
}

void setup() {
  Serial.begin(115200);
  pinMode(wakePin, INPUT);

  Serial.println("16 channel Servo test!");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  yield();

  strcpy_P(dutyAng, (char*)pgm_read_word(&(poses_table[idxOfPose("rest")])));
  for (byte servoIdx = 0; servoIdx < DOF; servoIdx++) {
    duty0[servoIdx] = SERVOMIN + range / 2 + float(middleShifts[servoIdx] ) * range / 180.0  * rotationDirections[servoIdx];
    servoCalib[servoIdx] = getCalib(servoIdx);
    yield();
    SPT((int)servoCalib[servoIdx]);
    SPT(", ");
    calibratedDuty0[servoIdx] = duty0[servoIdx] + float(servoCalib[servoIdx]) * range / 180.0  * rotationDirections[servoIdx];
    int duty = ((float)dutyAng[servoIdx]) * range / 180.0 * rotationDirections[servoIdx];
    pwm.setPWM(pins[servoIdx], 0, calibratedDuty0[servoIdx] + duty);
  }
  delay(1000);
  shutServos();
}

bool calibrating = false;
byte type;
void loop() {
  if (!digitalRead(wakePin)) { //the brain is not awake
    SPT("Brain Sleeping...\n");
    shutServos();
    delay(2000);
  }
  else {
    if (t == 0 && Serial.available() > 0) { //recieves some instructions from the brain
      type = Serial.read();
      if (type < 2) {  //format: [idx,duty]
        char inBuffer[DOF];
        while (Serial.available() < 2);
        Serial.readBytes(inBuffer, 2);
        byte targetIdx = inBuffer[0];

        if (type == 0 )  {//calibration
          servoCalib[targetIdx] = int(inBuffer[1]);
          SPT("calib\n");
          if (calibrating == false) { //first time entering the calibration function
            strcpy_P(dutyAng, (char*)pgm_read_word(&(poses_table[idxOfPose("calib")])));
            for (byte i = 0; i < DOF; i++) {
              int duty = float(dutyAng[i] + servoCalib[i]) * range / 180.0 * rotationDirections[i];
              pwm.setPWM(pins[i], 0, duty0[i] + duty);
            }
            shutServos();
            calibrating = true;
          }
        }
        else if (type == 1)
          dutyAng[targetIdx] = int(inBuffer[1]);

        int duty = float(dutyAng[targetIdx] + servoCalib[targetIdx]) * range / 180.0 * rotationDirections[targetIdx];
        pwm.setPWM(pins[targetIdx], 0, duty0[targetIdx] + duty);
        delay(100);


      }
      else if (type == 2) { //pose by angle array
        char inBuffer[DOF];
        while (Serial.available() < DOF);
        Serial.readBytes(inBuffer, DOF);
        SPT("array\n");
        for (byte i = 0; i < DOF; i++) {
          dutyAng[i] = inBuffer[i];
          calibratedDuty0[i] = duty0[i] + float(servoCalib[i]) * range / 180.0  * rotationDirections[i];
          int duty = ((float)dutyAng[i]) * range / 180.0 * rotationDirections[i];
          pwm.setPWM(pins[i], 0, calibratedDuty0[i] + duty);
        }
        delay(100);
        tPeriod = 1;
      }
      else  { //tokens string
        String inBuffer = Serial.readStringUntil('\n');
        calibrating = false;
        char token[DOF] = {};
        for (int i = 0; i < inBuffer.length() ; i++)
          token[i] = inBuffer[i]; //extract token string
        token[inBuffer.length()] = '\0';

        if (type == '3') {
          SPT("cmd: ");
          if (!strcmp(token, "save")) {
            //save calibration to EEPROM
            saveCalib(servoCalib);
          }
          else if (!strcmp(token, "quit")) {

          }
          else if (!strcmp(token, "shut"))
            shutServos();
        }
        else if (type == '4') {
          SPT("posing: ");
          if (strcmp(lastToken, token)) {
            strcpy_P(dutyAng, (char*)pgm_read_word(&(poses_table[idxOfPose(token)])));
            tPeriod = 1;
            strcpy(lastToken,token);
          }
          for (byte i = 0; i < DOF; i++) {

            int duty = ((float)dutyAng[i]) * range / 180.0 * rotationDirections[i];
            pwm.setPWM(pins[i], 0, calibratedDuty0[i] + duty);
          }
        }
        else if (type == '5') {
          SPT("gait: ");
          if (strcmp(lastToken, token)) {
            strcpy_P(dutyAng, (char*)pgm_read_word(&(gaits_table[idxOfGait(token)])));
            tPeriod = gaits_len[idxOfGait(token)] / (DOF - 4);
            strcpy(lastToken,token );
          }
        }
        Serial.println(token);
        SPT("\n");

      }
      //blink to indicate recieving signals
      /*{ blinkState = !blinkState;
        digitalWrite(LED_PIN, 1);
        delay(1);
        digitalWrite(LED_PIN, 0);
        }*/
    }

    else if ( type == '5') { //keep original gait
      for (int i = 4; i < DOF; i++) {
        int duty = float(dutyAng[t * (DOF - 4) + i - 4]) * range / 180.0 * rotationDirections[i];
        pwm.setPWM(pins[i], 0, calibratedDuty0[i] + duty);
      }
  
      t = (t + 1) % tPeriod;
    }

    delay(30);
  }

}
