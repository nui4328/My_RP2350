#pragma once
#include "Arduino.h"
#include "Wire.h"
#define TCS34725_ADDRESS 0x29
#define TCS_IT_101MS 0xD5
#define TCS_GAIN_16X 2
struct my_TCS {
 inline static int starts[2]={},saves[2]={},loads[2]={},captures[2]={};
 inline static int beginCalls[2]={};
 inline static bool available[2]={true,true},readOK=true,captureOK=true,saveOK=true;
 uint8_t index,flags=0,storedFlags=63;
 uint16_t c=100;
 bool saved=false;
 my_TCS(uint16_t offset):index(offset==0x40?1:0){}
 bool begin(uint8_t,TwoWire*){++beginCalls[index];return available[index];}
 bool setConfig(int,int){return true;}
 bool readFast(){return readOK;}
 void startColorCalibration(){++starts[index];flags=0;saved=false;}
 void capture(uint8_t color){++captures[index];if(captureOK)flags|=1u<<color;}
 void calibrateRed(){capture(0);} void calibrateGreen(){capture(1);}
 void calibrateBlue(){capture(2);} void calibrateYellow(){capture(3);}
 void calibrateWhite(){capture(4);} void calibrateBlack(){capture(5);}
 bool allColorsCalibrated(){return flags==63;}
 void calibrateDone(){++saves[index];saved=saveOK;if(saved)storedFlags=flags;}
 bool lastCalibrationSaveSucceeded(){return saved;}
 void loadColorCalibration(){++loads[index];flags=storedFlags;}
 const char* getColor(){return "RED";}
};
