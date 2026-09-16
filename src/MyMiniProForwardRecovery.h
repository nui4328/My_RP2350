#pragma once
#include <math.h>

namespace MyMiniProForward {
// The tracker reports only visible lines; recovery never invents a branch candidate.
class Recovery {
 public:
  explicit Recovery(float fullScale=1.0f) : fullScale_(fullScale) {}
  float error(bool visible, float measured, float kp, int left, int right, float threshold) {
    if(visible) {
      previous_=fmaxf(-fullScale_,fminf(fullScale_,measured));
      if(fabsf(previous_)>0.0001f*fullScale_)direction_=previous_>0 ? 1 : -1;
      return previous_;
    }
    if(kp<=threshold || !direction_)return previous_;
    // Produce at least one base speed of correction. The caller determines
    // wheel bounds: F_Line permits reverse, B_Line retains its legacy limits.
    // Bound the synthetic value even for an extremely small positive KP.
    return direction_*fmaxf(fullScale_,fmaxf(left,right)/fmaxf(fabsf(kp),0.0001f/fullScale_));
  }
 private:
  const float fullScale_;
  float previous_=0;
  int direction_=0;
};
}
