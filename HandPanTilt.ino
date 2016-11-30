#include <Servo.h>

#include <I2Cdev.h>

//#include <MPU6050.h>
#include <MPU6050_6Axis_MotionApps20.h>
//#include <helper_3dmath.h>
//#include <MPU6050_9Axis_MotionApps41.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

/**************
 *            *
 * MPU CONFIG *
 *            *
 **************/

MPU6050 mpu;

void initMPU();

//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL
//#define OUTPUT_TEAPOT

// MPU control/status variables
bool      dmpReady = false;         // true if init was successful
uint8_t   mpuIntStatus;             // status of MPU interrupt
uint8_t   devStatus;             // device status
uint16_t  packetSize;               // expected Packet size from DMP (def. 42 bytes)
uint16_t  fifoCount;                // count of all bytes in FIFO
uint8_t   fifoBuffer[64];           // FIFO buffer

// orientation
Quaternion q;                      // orientation quarternion
VectorInt16 aa;                     // Acc measurements
VectorInt16 aaReal;                 // gravity-free accel sensor measurements
VectorInt16 aaWorld;                // world-frame accel sensor measurements
VectorFloat gravity;                // gravity vector
float euler[3];                     // Euler angle container
float ypr[3];                       // yaw/pitch/roll container and gravity vector

// INTERRUPT DETECTION

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

/****************
 * SERVO SETUP  *
 ****************/

Servo pan, tilt;

#define PAN_PIN   9
#define TILT_PIN  6

int init_position = 90;
int pos = 90;

void initServos();
void deinitServos();

uint8_t angles[3];

/******************
 * LED (for fun)  *
 ******************/

#define LED_PIN 13
bool blinkState = false;

/******************
 * MISCELLANEOUS  *
 ******************/

static char* usage = "COMMANDS:\n\t\t1: Begin\n\t\t0: Pause";

bool init_p = false;

void setup() {
  Serial.begin(115200);

  initMPU();

  Serial.println(usage);
  while (Serial.available() && Serial.read());  // Empty Buffer
  while (!Serial.available());
  while (Serial.available()) {
    byte d = Serial.read();
    if(d - '1' == 0) break;
  }
  
  initServos();
  
  pinMode(LED_PIN, OUTPUT);
}


void initMPU() {

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

    Serial.println(F("[INIT] I2C Devices"));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    Serial.println(F("[INIT] Verifying I2C Connections"));
    Serial.println(mpu.testConnection() ? F("[INIT] MPU OK") : F("[INIT] FAIL"));

    Serial.println(F("[INIT] DMP"));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {
    Serial.println("[INIT] DMP Enabling");
    mpu.setDMPEnabled(true);

    Serial.println("[INIT] DMP Enabling Interrupt (0)");
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("[INIT] DMP Ready for Interrupt"));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // devStatus = 1 ==> Initial Memory Load Fail
    // devStatus = 2 ==> DMP Configuration Updates Failed
    
    Serial.print("[ERROR][DMP] INIT FAILED! [");
    Serial.print(devStatus);
    Serial.println("]");
  }
  
}


void initServos() {
  if (init_p) return;
  init_p = true;
  pan.attach(PAN_PIN);
  tilt.attach(TILT_PIN);
  pan.write(90); tilt.write(90);
  
}

void deinitServos() {
  if (!init_p) return;
  init_p = false;
  pan.write(90); tilt.write(90);
  pan.detach();
  tilt.detach();
}


void loop() {
  while (Serial.available()) {
    byte data = Serial.read();
    if(data - '1' == 0) {
      initServos();
    } else if (data - '0' == 0) {
      deinitServos();
    }
  }
  if (!init_p || !dmpReady) return;

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
    Serial.println("[ERROR] FIFO overflow!");
    Serial.print("[DEBUG] S: "); Serial.print(mpuIntStatus, HEX); Serial.print(" C: "); Serial.println(fifoCount, DEC);

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // Wait for FIFO to fill up
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // Read FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion (&q, fifoBuffer);

    #ifdef OUTPUT_READABLE_EULER

    mpu.dmpGetEuler       (euler, &q);
    
    angles[0] = euler[0]* 180/M_PI;
    angles[1] = euler[1]* 180/M_PI;
    angles[2] = euler[2]* 180/M_PI;
    
    #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL

    mpu.dmpGetGravity     (&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    angles[0] = ypr[0]* 180/M_PI;
    angles[1] = ypr[1]* 180/M_PI;
    angles[2] = ypr[2]* 180/M_PI;
    
    #endif

    #ifdef OUTPUT_READABLE_REALACCEL

    mpu.dmpGetAccel       (&aa, fifoBuffer);
    mpu.dmpGetGravity     (&gravity, &q);
    mpu.dmpGetLinearAccel (&aaReal, &aa, &gravity);
    
    #endif

    #ifdef OUTPUT_READABLE_WORLDACCEL

    mpu.dmpGetAccel       (&aa, fifoBuffer);
    mpu.dmpGetGravity     (&gravity, &q);
    mpu.dmpGetLinearAccel (&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld (&aaWorld, &aaReal, &q);
    
    #endif

    pan.write(angles[0]);
//    tilt.write(angles[1]);
//    delay(15);

    blinkState = !blinkState;
    digitalWrite  (LED_PIN, blinkState);
  }

}
