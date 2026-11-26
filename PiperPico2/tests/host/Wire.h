#pragma once
#include <cstdint>
#include <vector>
#include <array>
#include "Arduino.h"
struct TwoWire {
 std::array<uint8_t,512> memory;
 std::vector<uint8_t> tx;
 uint16_t address=0; uint8_t device=0; int remaining=0;
 bool fail=false, shortRead=false; int failAt=-1, requests=0;
 uint32_t clock=0;
 uint32_t deviceReadyAt[128]={};
 uint32_t timeoutMs=0;
 bool resetOnTimeout=false;
 int beginCalls=0,endCalls=0;
 TwoWire(){memory.fill(255);}
 void setSDA(int){} void setSCL(int){} void begin(){++beginCalls;}
 void end(){++endCalls;}
 void setTimeout(uint32_t ms,bool reset){timeoutMs=ms;resetOnTimeout=reset;}
 void setClock(uint32_t hz){clock=hz;}
 void beginTransmission(uint8_t dev){device=dev;tx.clear();}
 int write(uint8_t b){tx.push_back(b);return 1;}
 int endTransmission(){
  if(fail || clockMs<deviceReadyAt[device]) return 4;
  if(device==0x50 && tx.size()>=2){
   address=(tx[0]<<8)|tx[1];
   for(unsigned i=2;i<tx.size();++i) memory[address++]=tx[i];
  }
  return 0;
 }
 uint8_t requestFrom(uint8_t,uint8_t count){
  if(fail || requests++==failAt){remaining=0;return 0;}
  remaining=shortRead?count-1:count;return remaining;
 }
 int available(){return remaining;}
 int read(){--remaining;return memory[address++];}
};
inline TwoWire Wire;

inline TwoWire Wire1;
