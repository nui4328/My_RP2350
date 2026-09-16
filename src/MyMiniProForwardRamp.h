#pragma once
#include <stdint.h>

namespace MyMiniProForward {
inline float distanceRampFactor(float remainingCm,float windowCm,
                                int endPercent,int leftTarget,int rightTarget) {
  if(windowCm<=0 || remainingCm>=windowCm)return 1;
  const int peak=leftTarget>rightTarget ? leftTarget : rightTarget;
  const float requestedFactor=float(endPercent)/100;
  const float resolvableFactor=1.0f/peak;
  const float endFactor=requestedFactor>resolvableFactor ? requestedFactor : resolvableFactor;
  // Hold the configured low speed in the last20% of the window, rather than
  // approaching zero asymptotically or only reaching the low speed at the target.
  const float holdCm=windowCm*.2f;
  if(remainingCm<=holdCm)return endFactor;
  return endFactor+(1-endFactor)*((remainingCm-holdCm)/(windowCm-holdCm));
}
// Per-call translation envelope. First command is zero; completion is latched
// so a later millis() rollover cannot restart the ramp during a long sensor run.
class StartupRamp {
 public:
  explicit StartupRamp(uint32_t durationMs) : durationMs_(durationMs) {}
  float factor(uint32_t now) {
    if(!durationMs_ || complete_)return 1;
    if(!started_){started_=true;startedAt_=now;return 0;}
    const uint32_t elapsed=now-startedAt_;
    if(elapsed>=durationMs_){complete_=true;return 1;}
    return float(elapsed)/float(durationMs_);
  }
 private:
  const uint32_t durationMs_;
  uint32_t startedAt_=0;
  bool started_=false,complete_=false;
};
}
