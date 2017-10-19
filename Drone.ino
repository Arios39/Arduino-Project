/*
attempt to build drone with arduino
*Drone remote controller ver.(0.5)
**/

#include <PinChangeInt.h>
#include <Servo.h>
#include "I2Cdev.h"
#include <PID_v1.h>

static uint16_t unThrottleIn;
static uint16_t unPitchIn;
static uint16_t unRollIn;

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

int outputTL, outputTR, outputBR, outputBL, auxTL, auxTR, auxBR, auxBL;
int mpuYaw, mpuPitch, mpuRoll;
int motorGain = 70;


int power = 0;
// still having problems with power consumption;(
/*******************************************
* VALador++ *
********************************************/

#define MPU_STABILIZER_ACTIVATION 2

/******************
*   RADIO CONTROL *
*     RX/TX       *
*     PINS        *
*******************/

#define THROTTLE_IN_PIN 3 // Gas
#define PITCH_IN_PIN 4 // Elevator
#define ROLL_IN_PIN 5 // Aileron

/********************************
*  Electronic Speed Controllers *
*            SERVO              *
*            PINS               *
*********************************/

#define MOTORTL_OUT_PIN 8
#define MOTORTR_OUT_PIN 9
#define MOTORBR_OUT_PIN 11
#define MOTORBL_OUT_PIN 13

Servo servoMotorTL;
Servo servoMotorTR;
Servo servoMotorBR;
Servo servoMotorBL;

double PitchaggKp=.40, PitchaggKi=0.02, PitchaggKd=.9;
double PitchconsKp=.53, PitchconsKi=0.02, PitchconsKd=0.12;

double RollaggKp=.40, RollaggKi=0.02, RollaggKd=.9;
double RollconsKp=.53, RollconsKi=0.02, RollconsKd=0.12;


#define OUTPUT_LIMITS 50

double pitchSetpoint, pitchInput, pitchOutput;
double rollSetpoint, rollInput, rollOutput;

/**************
*   X:  *
*   Y:    *
*   Z: roll    *
**************/


bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];


#define THROTTLE_FLAG 1
#define PITCH_FLAG 1
#define ROLL_FLAG 1

volatile uint8_t bUpdateFlagsShared;

volatile uint16_t unThrottleInShared;
volatile uint16_t unPitchInShared;
volatile uint16_t unRollInShared;

uint32_t ulThrottleStart;
uint32_t ulPitchStart;
uint32_t ulRollStart;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup(){

  pitchInput = 0;
  rollInput = 0;

  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(0, OUTPUT_LIMITS);
  rollPID.SetOutputLimits(0, OUTPUT_LIMITS);

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24;
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);
  Serial.println("multiChannels");

  servoMotorTL.attach(MOTORTL_OUT_PIN);
  servoMotorTR.attach(MOTORTR_OUT_PIN);
  servoMotorBL.attach(MOTORBL_OUT_PIN);
  servoMotorBR.attach(MOTORBR_OUT_PIN);

  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle, CHANGE);
  PCintPort::attachInterrupt(PITCH_IN_PIN, calcPitch, CHANGE);
  PCintPort::attachInterrupt(ROLL_IN_PIN, calcRoll, CHANGE);

  arm();

  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}

void loop() {

  static uint8_t bUpdateFlags;

  if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
    }

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  mpuYaw = ypr[0] * 180/M_PI;
  mpuRoll = ypr[1] * 180/M_PI;
  mpuPitch = ypr[2] * 180/M_PI;

void calcRoll() {
  if(digitalRead(ROLL_IN_PIN) == HIGH) {
    ulRollStart = micros();
  } else{
    unRollInShared = (uint16_t)(micros() - ulRollStart);
    bUpdateFlagsShared |= ROLL_FLAG;
  }
}

void initMotors(int tl, int tr, int br, int bl) {

  Serial.println(tl);
  Serial.println(tr);
  Serial.println(br);
  Serial.println(bl);

  servoMotorTL.writeMicroseconds(tl);
  servoMotorTR.writeMicroseconds(tr);
  servoMotorBR.writeMicroseconds(br);
  servoMotorBL.writeMicroseconds(bl);

}

void arm() {
  servoMotorTL.writeMicroseconds(1000);
  servoMotorTR.writeMicroseconds(1000);
  servoMotorBL.writeMicroseconds(1000);
  servoMotorBR.writeMicroseconds(1000);
}
