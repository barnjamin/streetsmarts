#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <ArduinoJson.h>
#include <MPU6050.h>
#include <NMEAGPS.h>
#include <Wire.h>

MPU6050 mpu;

NMEAGPS gps;
gps_fix fix;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           

float scale =  9.80665/16384.0 ;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup(){
  Serial.begin(115200);

  Serial3.begin(9600);

  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  mpu.initialize();

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(-323);
  mpu.setYAccelOffset(1350);
  mpu.setZAccelOffset(957);

  mpu.setXGyroOffset(156);
  mpu.setYGyroOffset(-95);
  mpu.setZGyroOffset(-5);




  // turn on the DMP, now that it's ready
  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();

  dmpReady = true;

  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
}

void loop()
{
  jsonBuffer.clear();
  JsonObject& root = jsonBuffer.createObject();
  
  if(gps.available( Serial3 )) {
    fix = gps.read();
    if(fix.valid.location){
        JsonObject& gpsdata = root.createNestedObject("gps");
        gpsdata["lat"]      = fix.latitudeL();
        gpsdata["lng"]      = fix.longitudeL();
        if(fix.valid.altitude){
            gpsdata["alt"]      = fix.alt.whole;
        }
        if(fix.valid.heading){
            gpsdata["heading"]  = fix.heading();
        }
        if(fix.valid.date && fix.valid.time){
            gpsdata["stamp"]    = fix.dateTime_ms();
        }
        if(fix.valid.speed){
            gpsdata["speed"]    = fix.spd.whole;
        }
    }
  }
  
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
      if (mpuInterrupt && fifoCount < packetSize) {
        // try to get out of the infinite loop 
        fifoCount = mpu.getFIFOCount();
      }  
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      fifoCount = mpu.getFIFOCount();
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);

      JsonObject& quat = root.createNestedObject("quat");
      quat["x"] = q.x;
      quat["y"] = q.y;
      quat["z"] = q.z;
      quat["w"] = q.w;
      
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      JsonObject& velo = root.createNestedObject("velo");
      velo["x"] = ypr[2];
      velo["y"] = ypr[1];
      velo["z"] = ypr[0];
      
      mpu.dmpGetAccel(&aa, fifoBuffer);
      //mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      JsonObject& accel = root.createNestedObject("accel");
      accel["x"] = (aa.x * scale) * -2;
      accel["y"] = (aa.y * scale) * -2;
      accel["z"] = (aa.z * scale) * -2;

  }

  root["time"] = millis();

  root.printTo(Serial);
  Serial.println();

}
