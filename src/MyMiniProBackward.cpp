#include "Pico2_MyMiniPro.h"

bool MyMiniPro::setBLineKd(float kd) {
  if(!isfinite(kd) || kd<0 || kd>1)return false;
  bLineKd_=kd;return true;
}
bool MyMiniPro::setBLineRecoveryKpThreshold(float threshold) {
  if(!isfinite(threshold) || threshold<0 || threshold>100)return false;
  bLineRecoveryKpThreshold_=threshold;return true;
}
bool MyMiniPro::setBLineSensorDebounceMs(uint32_t milliseconds) {
  if(milliseconds>60000)return false;
  bLineSensorDebounceMs_=static_cast<uint16_t>(milliseconds);return true;
}
MyMiniPro::BackwardResult MyMiniPro::B_Line(int l,int r,float kp,float cm) {
  return runForward(l,r,kp,false,cm,LineSensor::B0,nullptr,true);
}
MyMiniPro::BackwardResult MyMiniPro::B_Line(int l,int r,float kp,LineSensor sensor) {
  return runForward(l,r,kp,true,0,sensor,nullptr,true);
}
