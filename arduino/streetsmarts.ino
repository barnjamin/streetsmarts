#include <ArduinoJson.h>
#include <MPU6050_tockn.h>
#include <NMEAGPS.h>
#include <Wire.h>

StaticJsonBuffer<200> jsonBuffer;


MPU6050 mpu6050(Wire);
int incomingByte = 0;   // for incoming serial data

NMEAGPS gps;
gps_fix fix;

void setup(){
  Serial.begin(9600);      // open the serial port at 9600 bps:
  Serial3.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);
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
  
  mpu6050.update();
  
  JsonObject& accel = root.createNestedObject("accel");
  accel["x"] = mpu6050.getAccX();
  accel["y"] = mpu6050.getAccY();
  accel["z"] = mpu6050.getAccZ();

  JsonObject& gyro = root.createNestedObject("gyro");
  gyro["x"] = mpu6050.getGyroX();
  gyro["y"] = mpu6050.getGyroY();
  gyro["z"] = mpu6050.getGyroZ();

  root["time"] = millis();

  root.printTo(Serial);
  
  Serial.println();
}
