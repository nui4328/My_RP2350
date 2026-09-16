#pragma once
#include "MyMiniProLinePID.h"
// Bounded empirical tuning policy, not a convergence-guaranteed controller.
namespace MyMiniProAdaptivePID {
struct Gains {float p=12,i=0,d=.1f;};
struct Window {
  unsigned n=0,clips=0,flips=0;
  uint32_t maxPeriod=0,duration=0;
  float mean=0,rms=0,variation=0,score=0;
  bool stable=false,changed=false;
  const char *reason="collecting";
  Gains measured,applied;
};
class Tuner {
 public:
  Gains current,best;
  Window last;
  bool hasBest=false;
  float bestScore=0;
  unsigned windows=0;
  static bool allowed(Gains g){return isfinite(g.p)&&isfinite(g.i)&&isfinite(g.d)&&g.p>=0&&g.p<=30&&g.i>=0&&g.i<=2&&g.d>=0&&g.d<=.5f;}
  void reset(Gains seed,uint32_t now){current=seed;best={};hasBest=false;bestScore=0;windows=0;last={};clear(now);}
  bool observe(float error,bool clipped,uint32_t now,uint32_t period,float headroom){
    if(uint32_t(now-warmup_)<250000)return false;
    if(!isfinite(error)||fabsf(error)>1||period>50000){clear(now);last.reason="pause_invalid";return false;}
    if(!n_)start_=now;
    if(n_)variation_+=fabsf(error-previous_);
    int sign=error>.03f?1:error<-.03f?-1:0;
    if(sign&&sign_&&sign!=sign_)++flips_;
    if(sign)sign_=sign;
    previous_=error;sum_+=error;squares_+=error*error;++n_;if(clipped)++clips_;
    if(period>maxPeriod_)maxPeriod_=period;
    uint32_t elapsed=now-start_;if(elapsed<2000000)return false;
    last={};last.n=n_;last.clips=clips_;last.flips=flips_;last.maxPeriod=maxPeriod_;last.duration=elapsed;
    last.mean=sum_/n_;last.rms=sqrtf(squares_/n_);last.variation=variation_*1000000/elapsed;
    last.score=last.rms+.03f*last.variation;last.measured=current;
    if(n_<80||maxPeriod_>50000||elapsed>2100000)last.reason="pause_samples";
    else if(clips_)last.reason="pause_saturation";
    else if(headroom<2)last.reason="pause_headroom";
    else {
      last.stable=last.rms<=.5f&&last.variation<.8f&&flips_<=2;
      if(last.stable&&(!hasBest||last.score<bestScore)){hasBest=true;best=current;bestScore=last.score;}
      if(last.rms>.08f&&fabsf(last.mean)/last.rms>.85f&&last.variation<.8f){
        float next=fminf(2,current.i+.1f);last.reason=next>current.i?"increase_i":"gain_limit";current.i=next;
      }else if(last.rms>.05f&&flips_>=3&&last.variation>=.8f){
        current.p=fmaxf(0,current.p-fminf(.5f,current.p*.05f));current.d=fminf(.5f,current.d+.01f);last.reason="damp_oscillation";
      }else last.reason=last.stable?"hold_stable":"hold_unclear";
    }
    last.applied=current;last.changed=current.p!=last.measured.p||current.i!=last.measured.i||current.d!=last.measured.d;
    ++windows;clear(now);return true;
  }
 private:
  uint32_t warmup_=0,start_=0,maxPeriod_=0;
  unsigned n_=0,clips_=0,flips_=0;
  int sign_=0;
  float sum_=0,squares_=0,variation_=0,previous_=0;
  void clear(uint32_t now){warmup_=now;start_=0;maxPeriod_=0;n_=clips_=flips_=0;sign_=0;sum_=squares_=variation_=previous_=0;}
};
}
