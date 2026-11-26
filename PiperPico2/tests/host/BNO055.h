#pragma once
#include "Arduino.h"
#include "Wire.h"
struct BNO055CalibrationStatus {uint8_t system=3,gyro=2,accel=1,mag=0;};
enum class BNO055UpdateError:uint8_t {None,ReadFailed,ImplausibleHeading};
struct BNO055 {
 inline static bool present=true,readOK=true,statusOK=true,storedOffset=true;
 inline static int updates=0,loads=0,beginCalls=0;
 inline static float angle=123.4f,rate=-12.5f;
 inline static uint8_t address=0;
 inline static BNO055UpdateError error=BNO055UpdateError::ReadFailed;
 bool begin(uint8_t addr,TwoWire& wire,int,int,uint32_t clock){++beginCalls;address=addr;wire.setClock(clock);return present;}
 bool loadStoredGyroCalibration(){++loads;return storedOffset;}
 bool update(){++updates;return readOK;}
 bool resetAngles() {
  if (!readOK) return false;
  angle = 0.0f;
  return true;
 }
 float yaw(){return angle;} float gyro(char){return rate;}
 bool calibrationStatus(BNO055CalibrationStatus&){return statusOK;}
 BNO055UpdateError lastUpdateError(){return error;}
};
