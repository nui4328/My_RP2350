#pragma once
#include "Wire.h"
#include <cassert>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#define SSD1306_SWITCHCAPVCC 2
#define SSD1306_WHITE 1
#define SSD1306_BLACK 0
struct Adafruit_SSD1306 {
 struct Text {int x,y;std::string value;};
 inline static bool works=true;
 inline static std::vector<Text> texts;
 int cursorX=0,cursorY=0,textSize=1;
 Adafruit_SSD1306(int,int,TwoWire*,int,uint32_t,uint32_t){}
 bool begin(int,int,bool,bool){return works;}
 void clearDisplay(){texts.clear();}
 void setTextSize(int size){textSize=size;}
 void setTextColor(int){} void setTextWrap(bool){}
 void setCursor(int x,int y){cursorX=x;cursorY=y;}
 void fillRect(int x,int y,int w,int h,int){assert(x>=0 && y>=0 && x+w<=128 && y+h<=32);}
 void display(){}
 void output(const std::string& value){
  assert(cursorX>=0 && cursorY>=0 && cursorX+int(value.size())*6*textSize<=128 && cursorY+8*textSize<=32);
  texts.push_back({cursorX,cursorY,value});cursorX+=int(value.size())*6*textSize;
 }
 template<class T>void print(T value){std::ostringstream out;out<<value;output(out.str());}
 void print(uint8_t value){print(unsigned(value));}
 template<class T>void println(T value){print(value);cursorX=0;cursorY+=8*textSize;}
 void print(float value,int digits){std::ostringstream out;out<<std::fixed<<std::setprecision(digits)<<value;output(out.str());}
 template<class... T>void printf(const char* format,T... args){char b[128];std::snprintf(b,sizeof(b),format,args...);output(b);}
protected:
 uint32_t wireClk=100000,restoreClk=100000;
};
