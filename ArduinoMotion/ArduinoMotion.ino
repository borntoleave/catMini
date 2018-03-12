#include <helper_3dmath.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


/***************************************************
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

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
#define DOF 16

const byte wakePin = 4;
int range = SERVOMAX - SERVOMIN;
// our servo # counter
uint8_t servonum = 0;
uint8_t pwmPin[DOF] = {  0, 0, 0, 0,  0, 0,  0, 0, 2, 3, 7, 6,  4, 5,  9, 8};
int8_t middleShifts[DOF] = {0, 45, 0, 0,
                            -45, -45, -45, -45,
                            0, 0, 0, 0,
                            0, 0, 0, 0
                           };
int8_t servoCalibs[] = {0, 0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 0
                       };
int8_t rotationDirections[DOF] = {1, -1, 1, 1,
                                  1, -1, 1, -1,
                                  1, -1, 1, -1,
                                  -1, 1, -1, 1
                                 };
float panF = 1 / 4.0,
      tF = 1.0,
      sXF = 1 / 5.0,
      sYF = 1 / 5.0,
      uF = 1 / 3.0,
      dF = 0.6;

//
byte dir = 0;

//movements and postures
#include "modes.h"

#include <EEPROM.h>
int getCalib(int idx) {
  return (int)(char)EEPROM[ idx ];
}
void setCalib(int *var, bool save) {
  for (int i = 0; i < 16; i++) {
    servoCalibs[i] = var[i];
    if (save)
      EEPROM.update(i, var[i]);
  }
}

int *duty0 = new int [DOF];
int t = 0;
int tPeriod = 1;
int cycle = 0;
char lastMode[15] = "";


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  // while (Serial.available() && Serial.read()); // empty buffer
  // while (!Serial.available());                 // wait for data
  // while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  Serial.println("16 channel Servo test!");
  pinMode(wakePin, INPUT);
  pwm.begin();

  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  yield();
  strcpy_P(dutyRad, (char*)pgm_read_word(&(poses_table[0])));

  for (int servonum = 0; servonum < DOF; servonum++) {
    //Serial.println(servonum);
    Serial.print((int)dutyRad[servonum]);
    Serial.print(", ");
    duty0[servonum] = SERVOMIN + range / 2 + (middleShifts[servonum] + getCalib(servonum)) * range / 180.0 * rotationDirections[servonum];
    pwm.setPWM(pwmPin[servonum], 0, duty0[servonum]);
  }
  Serial.print("\n");
  delay(2000);
}


// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period");
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit");
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);

}


void loop() {
  /*if (!digitalRead(wakePin)) {
    Serial.print("Brain Sleeping...\n");
    for (int servonum = 0; servonum < DOF; servonum++) {
      //Serial.println(servonum);
      pwm.setPWM(pwmPin[servonum], 0, 4096);
    }
    delay(2000);
    }
    else*/ {
    //if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      // other program behavior stuff here
      // .
      // .
      // .
      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
      // .
      // .
      // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
      delay(10);
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

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      /*Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180 / M_PI);*/
#endif




      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);

      if (ypr[1] * 180 / M_PI < -60) {
        strcpy_P(dutyRad, (char*)pgm_read_word(&(poses_table[0])));
        t = 0;
        tPeriod = 1;
      }
      else if (ypr[1] * 180 / M_PI > 60) {
        strcpy_P(dutyRad, (char*)pgm_read_word(&(poses_table[9])));
        t = 0;
        tPeriod = 1;
      }
      else if (t == 0) {
        if (Serial.available() > 0) {

          String inBuffer = Serial.readStringUntil('\n');
          String outBuffer = "Arduino: ";
          if (int(inBuffer[0]) == 0)
            Serial.print("calib\n");
          else if (int(inBuffer[0]) == 1)
            Serial.print("pose\n");
          if (int(inBuffer[0]) == 2)
            Serial.print("move\n");
          if (gaits[dir] != lastMode) {
            Serial.print(gaits[dir]);
            strcpy_P(dutyRad, (char*)pgm_read_word(&(gaits_table[dir])));
            tPeriod = gaits_len[dir] / DOF;
            Serial.print (tPeriod);
            Serial.print(" ");
            strcpy(lastMode, gaits[dir]);
          }
        }

      }


      for (int servonum = 8; servonum < DOF; servonum++) {
        //Serial.println(servonum);
        if (abs(ypr[2] * 180 / M_PI) < 5)
          ypr[2] = 0;
        if (abs(ypr[1] * 180 / M_PI) < 5)
          ypr[1] = 0;
        if (servonum > 3 and servonum < 8 ) {
          if (( servonum == 4 or servonum == 7) and ypr[2] * 180 / M_PI > 5
              or (servonum == 5 or servonum == 6) and ypr[2] * 180 / M_PI < -5 )
            sXF = 2.0;
          else
            sXF = -1 / 2.0;
        }
        float coeff[DOF][2] = {
          { -panF, 0}, {0, -tF}, {0, 0}, {0, 0},
          {sXF, -sYF}, { -sXF, -sYF}, { -sXF, sYF}, {sXF, sYF},
          {uF, 1}, { -uF, 1}, { -uF / 2.0, -1 / 2.0}, {uF / 2.0, -1 / 2.0},
          {dF, -dF}, { -dF, -dF}, { -dF, dF}, {dF, dF}
        };

        int duty = (dutyRad[t * DOF + servonum]  + coeff[servonum][0] * ypr[2] * 180 / M_PI - coeff[servonum][1] * ypr[1] * 180 / M_PI) * range / 180.0 * rotationDirections[servonum];
        pwm.setPWM(pwmPin[servonum], 0,4096);// duty0[servonum] + duty);
      }

      t++;
      if (t == tPeriod) {
        t = 0;
        cycle++;
        if (cycle == 5) {
          dir = (dir + 1) % 2;
          cycle = 0;
        }
      }
    }

    // Drive each servo one at a time
  }
}
