#pragma once
#include <cstdint>
#include <cstdlib>
#include <algorithm>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define RISING 3
inline uint32_t clockMs=0;
inline int gpio[32]={};
inline uint32_t pinLowUntil[32]={};
inline int duty[32]={};
inline uint32_t dutyUpdatedAt[32]={};
inline int adcValue=2000;
inline void (*interrupts[32])()={};
inline uint32_t millis(){return clockMs;}
inline void delay(uint32_t n){clockMs+=n;}
inline void delayMicroseconds(unsigned long){}
inline void pinMode(int pin,int mode){if(mode==INPUT_PULLUP) gpio[pin]=HIGH;}
inline void digitalWrite(int pin,int value){gpio[pin]=value;}
inline void digitalWriteFast(int pin,int value){digitalWrite(pin,value);}
inline int digitalRead(int pin){return clockMs<pinLowUntil[pin]?LOW:gpio[pin];}
inline void analogWrite(int pin,int value){duty[pin]=value;dutyUpdatedAt[pin]=clockMs;}
inline void analogWriteFreq(uint32_t){}
inline void analogWriteRange(uint32_t){}
inline void analogReadResolution(int){}
inline int analogRead(int){return adcValue;}
inline int digitalPinToInterrupt(int pin){return pin;}
inline void attachInterrupt(int pin,void (*fn)(),int){interrupts[pin]=fn;}
inline void detachInterrupt(int pin){interrupts[pin]=nullptr;}
inline unsigned int lastToneFrequency=0;
inline unsigned long lastToneDuration=0;
inline void tone(int,unsigned int frequency,unsigned long duration){lastToneFrequency=frequency;lastToneDuration=duration;}
inline void noTone(int){}
template<class T> T constrain(T x,T a,T b){return std::max(a,std::min(x,b));}
inline long map(long x,long a,long b,long c,long d){return (x-a)*(d-c)/(b-a)+c;}

struct MockSerial {
 void begin(unsigned long){}
 template<class... T> void print(T...){}
 template<class... T> void println(T...){}
};
inline MockSerial Serial;
