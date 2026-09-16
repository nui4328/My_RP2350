#pragma once
#include <stdint.h>
#include <math.h>
#include <stdlib.h>

namespace MyMiniProLinePID {
constexpr uint8_t kCount=16;
constexpr uint16_t kMinWait=1,kMaxWait=2000;
constexpr uint16_t kMinSpan=100; // Provisional calibration quality threshold.
constexpr float kBase=25.0f,kMaximum=40.0f;
constexpr float kP=12.0f,kI=0.0f,kD=0.10f,kDerivativeTau=0.020f;
constexpr float kIntegralLimit=20.0f; // error-seconds; additional safety clamp.
constexpr uint32_t kMaximumRunUs=10000000,kMaximumDtUs=50000;

inline bool validMap(const uint8_t *map) {
  uint16_t mask=0;
  for(uint8_t i=0;i<kCount;++i){if(map[i]>=kCount || (mask&(1u<<map[i])))return false;mask|=1u<<map[i];}
  return mask==65535;
}
constexpr bool validWait(unsigned value){return value>=kMinWait&&value<=kMaxWait;}

struct Calibration {
  uint16_t white[16]{},black[16]{};
  bool haveWhite=false,haveBlack=false;
  void clear(){haveWhite=haveBlack=false;}
  bool valid() const {
    if(!haveWhite||!haveBlack)return false;
    for(uint8_t ch=0;ch<16;++ch)
      if(abs(int(white[ch])-int(black[ch]))<kMinSpan)return false;
    return true;
  }
};
struct Line { bool valid=false; float error=0; uint16_t strength=0; uint8_t active=0; };
inline Line locate(const uint16_t *raw,const Calibration &cal,const uint8_t *map) {
  Line line;
  if(!cal.valid()||!validMap(map))return line;
  float moment=0,total=0;
  for(uint8_t position=0;position<16;++position){
    uint8_t ch=map[position];
    float dark=1000.0f*(int(cal.white[ch])-int(raw[ch]))/(int(cal.white[ch])-int(cal.black[ch]));
    if(dark<0)dark=0;if(dark>1000)dark=1000;
    if(dark>line.strength)line.strength=uint16_t(dark);
    if(dark>=600)++line.active;
    float weight=dark>200?dark-200:0;
    total+=weight;moment+=weight*(2.0f*position/15.0f-1.0f);
  }
  line.valid=line.strength>=600&&line.active>=1&&line.active<=8&&total>=400;
  if(line.valid)line.error=moment/total;
  return line;
}

struct Output {bool valid=false;float left=0,right=0;bool clipped=false;float correction=0;};
class Controller {
 public:
  Controller(float kp=kP,float ki=kI,float kd=kD):kp_(kp),ki_(ki),kd_(kd){}
  bool setSpeed(float base,float maximum){
    if(!isfinite(base)||!isfinite(maximum)||base<15||base>35||maximum<base||maximum>50)return false;
    base_=base;maximum_=maximum;reset();return true;
  }
  float base()const{return base_;}
  float maximum()const{return maximum_;}
  bool setGains(float p,float i,float d){
    if(!isfinite(p)||!isfinite(i)||!isfinite(d)||p<0||p>30||i<0||i>10||d<0||d>1)return false;
    kp_=p;ki_=i;kd_=d;reset();return true;
  }
  void reset(){initialized_=false;derivative_=integral_=0;}
  // Small adaptive updates keep derivative/time history. Preserve the current
  // integral contribution when Ki changes, rather than erasing curve bias.
  bool retuneGains(float p,float i,float d){
    if(!isfinite(p)||!isfinite(i)||!isfinite(d)||p<0||p>30||i<0||i>10||d<0||d>1)return false;
    if(i==0)integral_=0;
    else if(ki_>0)integral_=fmaxf(-kIntegralLimit,fminf(kIntegralLimit,integral_*ki_/i));
    kp_=p;ki_=i;kd_=d;return true;
  }
  float integralState()const{return integral_;}
  Output stepLine(const Line &line,uint32_t now){if(!line.valid){reset();return {};}return step(line.error,now);}
  Output step(float error,uint32_t now){
    Output out;
    if(!isfinite(error)||fabsf(error)>1.001f)return out;
    float dt=0;
    if(initialized_){
      uint32_t elapsed=now-previousTime_;
      if(!elapsed||elapsed>kMaximumDtUs)return out;
      dt=elapsed*1e-6f;
      float slope=(error-previousError_)/dt;
      derivative_+=dt/(kDerivativeTau+dt)*(slope-derivative_);
    }
    initialized_=true;previousError_=error;previousTime_=now;
    float limit=base_<maximum_-base_?base_:maximum_-base_;
    float candidate=integral_+error*dt;
    if(candidate>kIntegralLimit)candidate=kIntegralLimit;
    if(candidate<-kIntegralLimit)candidate=-kIntegralLimit;
    float requested=kp_*error+ki_*candidate+kd_*derivative_;
    // Conditional integration: never accumulate further into saturation.
    // Permit unwinding when error opposes the saturated correction.
    if(ki_==0)integral_=0;
    else if(fabsf(requested)<=limit || (requested>limit&&error<0) || (requested<-limit&&error>0))integral_=candidate;
    float correction=kp_*error+ki_*integral_+kd_*derivative_;
    out.correction=correction; // Exact controller result before wheel clipping.
    // Positive error: line is on robot's right. Left wheel goes faster.
    out.left=base_+correction;out.right=base_-correction;
    out.clipped=out.left<0||out.right<0||out.left>maximum_||out.right>maximum_;
    if(out.left<0)out.left=0;if(out.right<0)out.right=0;
    if(out.left>maximum_)out.left=maximum_;if(out.right>maximum_)out.right=maximum_;
    out.valid=true;return out;
  }
 private:
  bool initialized_=false;
  uint32_t previousTime_=0;
  float previousError_=0,derivative_=0;
  float integral_=0,kp_,ki_,kd_,base_=kBase,maximum_=kMaximum;
};

// Hardware Start cannot start anything until a command explicitly arms it.
// A HIGH after arming is required, so holding Start during arm cannot launch.
class StartGate {
 public:
  void arm(){armed_=true;released_=false;pressing_=false;}
  void cancel(){armed_=released_=pressing_=false;}
  bool poll(bool pressed,uint32_t nowMs){
    if(!armed_)return false;
    if(!pressed){released_=true;pressing_=false;return false;}
    if(!released_)return false;
    if(!pressing_){pressing_=true;pressedAt_=nowMs;return false;}
    if(uint32_t(nowMs-pressedAt_)<30)return false;
    cancel();return true;
  }
 private:
  bool armed_=false,released_=false,pressing_=false;
  uint32_t pressedAt_=0;
};

struct Ready {
  bool mapping=false,calibration=false,leftCheck=false,rightCheck=false,motors=false;
  bool valid()const{return mapping&&calibration&&motors;}
};
// Parser owns a fixed buffer; overlong/invalid lines are rejected as a whole.
class CommandLine {
 public:
  enum Event {None,Complete,Error};
  Event feed(char c){
    if(c=='\r'||c=='\n'){
      text_[used_]=0;bool bad=invalid_;bool empty=used_==0;
      used_=0;invalid_=false;return bad?Error:empty?None:Complete;
    }
    if(c<32||c>126||used_>=sizeof(text_)-1)invalid_=true;
    else if(!invalid_)text_[used_++]=c;
    return None;
  }
  char *text(){return text_;}
 private:char text_[96]{};uint8_t used_=0;bool invalid_=false;
};
inline bool unsignedNumber(const char *p,unsigned &out){
  if(!p||!*p)return false;out=0;
  for(;*p;++p){if(*p<'0'||*p>'9')return false;out=out*10+(*p-'0');if(out>65535)return false;}
  return true;
}
}
