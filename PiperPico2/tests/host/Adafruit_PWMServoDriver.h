#pragma once
#include <cstdint>
#include <cassert>
#include "Arduino.h"
#include <vector>
struct Adafruit_PWMServoDriver {
 struct Write { uint8_t channel; uint16_t on,off; uint32_t at; bool succeeded; };
 inline static std::vector<Write> history;
 inline static uint16_t channelOn[16]={},channelOff[16]={};
 inline static bool works=true;
 inline static uint32_t readyAt=0;
 inline static int beginCalls=0;
 inline static int writes=0,failWrites=0,failChannel=-1;
 Adafruit_PWMServoDriver(int){}
 bool begin(){++beginCalls;return works && clockMs>=readyAt;}
 void setPWMFreq(int){}
 uint8_t setPWM(uint8_t channel,uint16_t on,uint16_t off){
  if(channel<8){
   const int pins[8]={8,8,7,7,9,9,6,6};
   assert(duty[pins[channel]]==0); // Drive must be removed before direction writes.
  }
  ++writes;
  const bool failed=failWrites>0 || channel==failChannel;
  if(failWrites>0) --failWrites;
  history.push_back({channel,on,off,clockMs,!failed});
  if(failed) return 1;
  channelOn[channel]=on;channelOff[channel]=off;
  return 0;
 }
};
