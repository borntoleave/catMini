#define SPT(s) Serial.print(s)
#define SPTL(s) Serial.println(s)
#define SPTF(s) Serial.print(F(s))
#define SPLF(s) Serial.println(F(s))

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
//#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3], yprLag[2][3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
bool overFlow = false;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

#include <EEPROM.h>
//#include <avr/eeprom.h>
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
void playMelody(int start) {
  byte len = (byte)EEPROM.read(start) / 2;
  for (int i = 0; i < len; i++)
    beep(EEPROM.read(start - 1 - i), 1000 / EEPROM.read(start - 1 - len - i), 100);
}

// https://brainy-bits.com/blogs/tutorials/ir-remote-arduino
#include "IRremote.h"
int receiver = 5; // Signal Pin of IR receiver to Arduino Digital Pin 5
/*-----( Declare objects )-----*/
IRrecv irrecv(receiver);     // create instance of 'irrecv'
decode_results results;      // create instance of 'decode_results'
String translateIR() // takes action based on IR code received
// describing Remote IR codes. 
{                                     
  switch (results.value) {
                 //abbreviation of gaits      key on IR remote                gait/posture names
    case 0xFF629D: return (F("vt"));        //Serial.println(" FORWARD");   //stepping on spot, "mark time"
    case 0xFF22DD: return (F("sit"));       //Serial.println(" LEFT");      //sit
    case 0xFF02FD: return (F("bd"));        //Serial.println(" -OK-");      //bound
    case 0xFFC23D: return (F("balance"));   //Serial.println(" RIGHT");     //standing
    case 0xFFA857: return (F("d"));         //Serial.println(" REVERSE");   //shut down servos
    case 0xFF6897: return (F("trL"));       //Serial.println(" 1");         //trot left
    case 0xFF9867: return (F("tr"));        //Serial.println(" 2");         //trot 
    case 0xFFB04F: return (F("trR"));       //Serial.println(" 3");         //trot right
    case 0xFF30CF: return (F("wkL"));       //Serial.println(" 4");         //walk left
    case 0xFF18E7: return (F("wk"));        //Serial.println(" 5");         //walk 
    case 0xFF7A85: return (F("wkR"));       //Serial.println(" 6");         //walk right
    case 0xFF10EF: return (F("crL"));       //Serial.println(" 7");         //crawl left
    case 0xFF38C7: return (F("cr"));        //Serial.println(" 8");         //crawl
    case 0xFF5AA5: return (F("crR"));       //Serial.println(" 9");         //crawl right
    case 0xFF42BD: return (F("bkL"));       //Serial.println(" *");         //back left
    case 0xFF4AB5: return (F("bk"));        //Serial.println(" 0");         //back
    case 0xFF52AD: return (F("bkR"));       //Serial.println(" #");         //back right
    case 0xFFFFFFFF: return (""); //Serial.println(" REPEAT");

    default:
      return ("");                      //Serial.println("null");
  }// End Case
  //delay(100); // Do not get immediate repeat //no need because the main loop is slow
  
  // The control could be organized in another way, such as:
  // forward/backward to change the gaits corresponding to different speeds. 
  // left/right key for turning left and right
  // number keys for different postures or behaviors
}

#define DOF 16
const byte wakePin = 6;
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
#define RANGE (SERVOMAX - SERVOMIN)
// our servo # counter

int8_t servoCalibs[DOF] = {};
char currentAng[DOF] = {};
#define FLAT 5
#define panF 1 / 2.0
#define tF 1.0
#define sXF 2
#define sYF 1/4.0
#define uF 0.8
#define dF -0.8
float coeff[16][2] = {
  { -panF, 0}, {0, -tF}, {0, 0}, {0, 0},
  {sXF, -sYF}, { -sXF, -sYF}, { -sXF, sYF}, {sXF, sYF},
  { uF, 1}, { -uF, 1}, { -uF , -1 }, { uF , -1 },
  { dF, dF}, { -dF, dF}, { -dF, -dF}, { dF, -dF}
};

int calibratedDuty0[DOF];

//movements and postures
#include "modes.h"


#define PIN 0
#define CALIB 16
#define MIDSHIFT 32
#define ROTDIRECTION 48
#define RANGERATIO 64
#define MPUCALIB 80


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

void saveCalib(int8_t *var) {
  for (byte i = 0; i < DOF; i++) {
    EEPROM.update(CALIB + i, var[i]);
    calibratedDuty0[i] = SERVOMIN + RANGE / 2 + float(middleShift(i) + var[i]) * RANGE * rotationDirection(i) / rangeRatio(i);
  }
}

void calibratedPWM(byte i, int dt) {
  char duty = max(-128, min(127, dt));
  if (i > 3 && i < 8)
    duty = max(-5, duty);
  currentAng[i] = duty;
  dt = calibratedDuty0[i] + float(duty) * RANGE * rotationDirection(i) / rangeRatio(i);
  dt = max(SERVOMIN + RANGE / 2 * BOUND, min(SERVOMAX - RANGE / 2 * BOUND, dt));
  pwm.setPWM(pin(i), 0, dt);
}
void allCalibratedPWM(char * dutyAng) {
  for (byte i = 0; i < DOF; i++) {
    calibratedPWM(i, dutyAng[i]);
  }
}
void printList(char * arr, byte len = DOF) {
  for (byte i = 0; i < len; i++) {
    SPT(int(arr[i]));
    SPT('\t');
  }
  SPTL();
}
void transform( char * target, byte offset = 0, float speedRatio = 0.5) {
  //  printList(currentAng);

  char *diff = new char[DOF - offset], maxDiff = 0;
  for (byte i = offset; i < DOF; i++) {
    diff[i - offset] =   currentAng[i] - target[i - offset];
    maxDiff = max(maxDiff, abs( diff[i - offset]));
  }
  byte steps = byte(round(maxDiff / 3.0/*degreeStep*/ / speedRatio));
  for (byte s = 0; s < steps; s++)
    for (byte i = offset; i < DOF; i++) {
      char duty = (target[i - offset] + (1 + cos(M_PI * s / steps)) / 2 * diff[i - offset]);
      calibratedPWM(i,  duty);
    }
  delete [] diff;
  //  printList(currentAng);
  //  SPTL();
}

void shutServos() {
  delay(250);
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

int adjust(int i) {
  if (i>7)
    return coeff[i][0] * ypr[2] * 180 / M_PI + coeff[i][1] * ypr[1] * 180 / M_PI;
  else
    return coeff[i][0] * ypr[2] * 180 / M_PI + coeff[i][1] * ypr[1] * 180 / M_PI;
}

int t = 0;
int tPeriod = 1;
byte hold = 0;
byte offset;
char token;
#define CMD_LEN 10
char lastCmd[CMD_LEN] = {};

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(57600);
  Serial.setTimeout(10);
  while (!Serial);
  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  delay(100);
  SPLF("* Starting *");

  SPLF("Initializing I2C");
  mpu.initialize();
  // verify connection
  SPLF("Testing connections...");
  Serial.println(mpu.testConnection() ? F("MPU successful") : F("MPU failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity

  mpu.setZAccelOffset(EEPROMReadInt(MPUCALIB + 4));
  mpu.setXGyroOffset(EEPROMReadInt(MPUCALIB + 6));
  mpu.setYGyroOffset(EEPROMReadInt(MPUCALIB + 8));
  mpu.setZGyroOffset(EEPROMReadInt(MPUCALIB + 10));
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection"));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready!"));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.println(F("DMP failed (code "));
    SPT(devStatus);
    SPLF(")");
  }

  //opening music
  if (WalkingDOF == 8)
  {
    pinMode(BUZZER, OUTPUT);
    playMelody(MELODY);
  }
  //IR
  {
    //Serial.println(F("IR Receiver Button Decode"));
    irrecv.enableIRIn(); // Start the receiver
  }

  // servo
  { pwm.begin();
    pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
    delay(200);
    char cmd[CMD_LEN] = {};
    //    while (1) {
    //      if (irrecv.decode(&results)) {
    //        if (translateIR() != "") {
    //          strcpy(cmd, translateIR().c_str());
    //          if (!strcmp(cmd, "d"))
    //            break;
    //        }
    //        irrecv.resume(); // receive the next value
    //      }
    //    }

    strcpy(lastCmd, "rest");
    pgmCpy(dutyAng, "rest");
    for (byte i = 0; i < DOF; i++) {
      servoCalibs[i] = servoCalib(i);
      calibratedDuty0[i] =  SERVOMIN + RANGE / 2 + float(middleShift(i) + servoCalibs[i]) * RANGE  * rotationDirection(i) / rangeRatio(i);
      calibratedPWM(i, dutyAng[i]);
    }
    //    pgmCpy(dutyAng, "sit");
    //    transform( dutyAng, 0, 0.5);
    //    delay(2000);
    //    pgmCpy(dutyAng, "rest");
    //    transform( dutyAng, 0, 0.5);
    shutServos();
    token = 'd';
  }
  beep(30);
}

void loop() {
  /*if (!digitalRead(wakePin)) {
    Serial.print("Brain Sleeping...\n");
    delay(2000);
    }
    else */
  // MPU block
  {
    if (!dmpReady) return;
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) ;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 256) { //1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      overFlow = true;
      //Serial.println(F("overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      //#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //#endif
    }
    if (overFlow) {
      for (byte g = 0; g < 3; g++) {
        ypr[g] = yprLag[0][g];
        yprLag[1][g] = yprLag[0][g];
      }
      overFlow = false;
    }
    else {
      for (byte g = 0; g < 3; g++) {
        float temp = ypr[g];
        ypr[g] = yprLag[0][g];
        yprLag[0][g] = yprLag[1][g];
        yprLag[1][g] = temp;
      }
    }
  }
  for (byte i = 1; i < 3; i++)
    ypr[i] = ((ypr[i] > 0) - (ypr[i] < 0)) * max(fabs(ypr[i]) - FLAT*M_PI/180.0, 0);
  /*if (fabs(ypr[2] * 180 / M_PI) < FLAT)
    ypr[2] = 0;
  if (fabs(ypr[1] * 180 / M_PI) < FLAT)
    ypr[1] = 0;*/
  ypr[2] = -ypr[2];
  //            Serial.print("ypr\t");
  //  Serial.print(ypr[0] * 180 / M_PI);
  //  Serial.print("\t");
  //  Serial.print(ypr[1] * 180 / M_PI);
  //  Serial.print("\t");
  //  Serial.println(ypr[2] * 180 / M_PI);
  char cmd[CMD_LEN] = {};
  byte newCmd = 0;
  // accident block
  if (fabs(ypr[1] * 180 / M_PI) > 60) {
    if (!hold) {
      token = 'p';
      strcpy(cmd, ypr[1] * 180 / M_PI > 60 ? "lifted" : "dropped");
      newCmd = 1;
    }
    hold = 10;
  }
  // recover
  else if (hold ) {
    if (hold == 10) {
      token = 'p';
      strcpy(cmd, "balance");
      newCmd = 1;
    }
    hold --;
    if (!hold) {
      char temp[CMD_LEN];
      strcpy(temp, cmd);
      strcpy(cmd, lastCmd);
      strcpy(lastCmd, temp);
      newCmd = 1;
    }
  }

  // input block
  //else if (t == 0) {
    if (irrecv.decode(&results)) {
      if (translateIR() != "") {
        strcpy(cmd, translateIR().c_str());
        if (!strcmp(cmd, "d"))
          token = 'd';
        else
          token = 'g';
        newCmd = 2;
      }
      irrecv.resume(); // receive the next value
    }
    if ( Serial.available() > 0) {
      token = Serial.read();
      newCmd = 3;
    }
  //}
  if (newCmd) {
    SPTL(token);
    beep(newCmd * 10);
    // this block handles argumentless tokens
    if (token == 'h')
      SPLF("** Help Information **");// print the help document
    /*else if (token == 'k') {
      SPLF("kill machine");
      delay(60);//replace with actual arduino board shut down code
      }
      else if (token == 'r') {
      SPLF("reset machine");
      //resetFunc();//replace with actual arduino board reset code
      }*/
    else if (token == 'd' ) {
      pgmCpy(dutyAng, "rest");
      transform( dutyAng);
      //SPLF("shut down servos");
      shutServos();
    }
    else if (token == 's') {
      //SPLF("save calibration");
      saveCalib(servoCalibs);
    }
    else if (token == 'a') {
      //SPLF("abandon calibration");
      for (byte i = 0; i < DOF; i++) {
        servoCalibs[i] = servoCalib( i);
      }
    }
    // this block handles array like arguments
    else if (token == 'l' ) {
      int len = Serial.read();
      SPT(len);
      //        //SPLF("receiving 16 angle list in binary [ byte, ..., byte ] ");

      char *inBuffer = new char[len];
      //
      for (byte i = 0; i < len; i++){
        inBuffer[i] = Serial.read();
        SPT(inBuffer[i]);
        SPTL();
      }
      if (len == DOF)
        allCalibratedPWM(inBuffer);
      else
        for (byte i = 0; i < len / 2; i++)
          calibratedPWM(inBuffer[i * 2], inBuffer[i * 2 + 1]);
      //          Serial.readBytes(inBuffer, DOF);
      //          //allCalibratedPWM(dutyAng);
      //          delay(200);
      //
      delete [] inBuffer;
    }
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
        //SPLF("calibrating [ targetIdx, angle ]: ");
        if (strcmp(lastCmd, "c")) { //first time entering the calibration function
          pgmCpy(dutyAng, "calib");
          transform( dutyAng);
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
        //SPF("moving [ targetIdx, angle ]: ");
        dutyAng[target[0]] = target[1];
      }

      SPT(target[0]);
      SPT(",\t");
      SPT(target[1]);

      int duty = SERVOMIN + RANGE / 2 + float(middleShift(target[0])  + servoCalibs[target[0]] + dutyAng[target[0]]) * RANGE * rotationDirection(target[0]) / rangeRatio(target[0]);
      pwm.setPWM(pin(target[0]), 0,  duty);
    }

    else if (Serial.available() > 0) {
      String inBuffer = Serial.readStringUntil('\n');
      strcpy(cmd, inBuffer.c_str());
    }
    //while (Serial.available() && Serial.read());
    //    while (Serial.available())
    //      Serial.read();//flush the remaining serial buffer in case the commands are parsed incorrectly

    if (strcmp(cmd, "") && strcmp(lastCmd, cmd) ) {
      //      SPT("compare lastCmd ");
      //      SPT(lastCmd);
      //      SPT(" with newCmd ");
      //      SPT(token);
      //      SPT(cmd);
      //      SPT("\n");
      if (token == 'w'); //some words for undefined behaviors

      if (token == 'p' || token == 'g') { //validating key
        if (idxOfGait(cmd) >= 0) {
          tPeriod =  pgmCpy(dutyAng, cmd);
          t = 0;
          if (strcmp(cmd, "balance") && strcmp(cmd, "lifted") && strcmp(cmd, "dropped") )
            strcpy(lastCmd, cmd);
          offset = (tPeriod == 1) ? 0 : DOF - WalkingDOF;
          //          char dutyTemp[DOF];
          //          for (int i = offset; i < DOF; i++) {
          //            int dutyIdx =  t * WalkingDOF - offset + i;
          //            dutyTemp[dutyIdx] =(dutyAng[dutyIdx] + adjust(i)) / (1 + sqrt(fabs(ypr[1] * ypr[2])) / M_PI * 2) ;
          //          }

          transform( dutyAng, offset, 1);
          if (!strcmp(cmd, "rest")) {
            shutServos();
            token = 'd';
          }
        }
        else
          SPLF("wrong key!");
      }
      else {
        lastCmd[0] = token;
        memset(lastCmd + 1, '\0', CMD_LEN - 1);
      }
    }
  }

  //motion block
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
}
