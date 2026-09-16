#pragma once
#include "MyMiniProLinePID.h"
#include <float.h>

namespace MyMiniProForward {
// F_Line error/history/derivative use [-50,+50] position units. Gains are per
// position unit; time is seconds, output is PWM percentage points. The paused
// B_Line draft continues to supply its legacy normalized units and gains.
// Reuses the PID tuner's20ms derivative filter and50ms maximum sample gap.
class PD {
 public:
  float step(float error, float kp, float kd, uint32_t sampleUs, bool unboundedGains=false) {
    const uint32_t elapsed=sampleUs-previousUs_;
    const float p=unboundedGains ? finiteCorrection(double(kp)*error) : kp*error;
    if (!initialized_ || kd==0 || elapsed<100 ||
        elapsed>MyMiniProLinePID::kMaximumDtUs) {
      initialized_=true; previousUs_=sampleUs; previousError_=error;
      filteredDerivative_=0;
      return p; // no startup/recovery kick, division by zero, or stale D
    }
    const float dt=elapsed*1e-6f;
    const float slope=(error-previousError_)/dt;
    const float alpha=dt/(MyMiniProLinePID::kDerivativeTau+dt);
    filteredDerivative_+=alpha*(slope-filteredDerivative_);
    previousUs_=sampleUs; previousError_=error;
    // F_Line accepts every finite signed float gain. Sum in double before
    // bounding to finite float output, so opposing huge terms cannot form inf-inf.
    if(unboundedGains)return finiteCorrection(double(kp)*error+double(kd)*filteredDerivative_);
    return p+kd*filteredDerivative_;
  }
 private:
  static float finiteCorrection(double value) {
    return value>FLT_MAX ? FLT_MAX : value<-FLT_MAX ? -FLT_MAX : static_cast<float>(value);
  }
  bool initialized_=false;
  uint32_t previousUs_=0;
  float previousError_=0,filteredDerivative_=0;
};
}
