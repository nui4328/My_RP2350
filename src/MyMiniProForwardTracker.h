#pragma once
#include "MyMiniProLinePID.h"

namespace MyMiniProForward {
// Forward-only junction tracking; the PID tuner's original locator is unchanged.
class Tracker {
 public:
  MyMiniProLinePID::Line locate(const uint16_t *raw,
                               const MyMiniProLinePID::Calibration &cal,
                               const uint8_t *map) {
    using namespace MyMiniProLinePID;
    Line best;
    if (!cal.valid() || !validMap(map)) return best;
    float dark[16];
    for (uint8_t pos=0; pos<16; ++pos) {
      const uint8_t ch=map[pos];
      dark[pos]=fmaxf(0, fminf(1000,
          1000.0f*(int(cal.white[ch])-int(raw[ch])) /
          (int(cal.white[ch])-int(cal.black[ch]))));
    }
    const float reference=havePrevious_ ? previousError_ : 0;
    float bestDistance=3, bestCenterDistance=3;
    uint16_t visibleMask=0, bestMask=0;
    uint8_t bestWidth=0;
    bool bestHeld=false;
    for (uint8_t begin=0; begin<16;) {
      if (dark[begin]<=200) { ++begin; continue; }
      uint8_t end=begin;
      while (end+1<16 && dark[end+1]>200) ++end;
      const uint8_t width=end-begin+1;
      uint16_t isolated[16], mask=0;
      for (uint8_t ch=0; ch<16; ++ch) isolated[ch]=cal.white[ch];
      for (uint8_t pos=begin; pos<=end; ++pos) {
        isolated[map[pos]]=raw[map[pos]];
        mask|=uint16_t(1u<<pos);
      }
      // Reuse calibrated strength/centroid/noise thresholds per group.
      Line candidate=MyMiniProLinePID::locate(isolated,cal,map);
      const float low=2.0f*begin/15.0f-1.0f;
      const float high=2.0f*end/15.0f-1.0f;
      const bool broad=candidate.active>8;
      if (!candidate.valid && broad && havePrevious_) candidate.valid=true;
      if (candidate.valid) {
        visibleMask|=mask;
        // A previously separate branch can join through weak (>200) pixels
        // before nine pixels become strong. Do not follow that merged centroid.
        const bool merged=havePrevious_ && (mask&previousSelectedMask_) &&
                          (mask&previousOtherMask_);
        const bool held=havePrevious_ &&
            (merged || broad || (holding_ && width>pathWidth_+1));
        if (held) candidate.error=fmaxf(low,fminf(high,reference));
        float distance=reference<low ? low-reference :
                       reference>high ? reference-high : 0;
        // One sensor-spacing switching hysteresis favors the existing group.
        if (havePrevious_ && !(mask&previousSelectedMask_)) distance+=2.0f/15.0f;
        const float centerDistance=fabsf(candidate.error-reference);
        if (!best.valid || distance<bestDistance ||
            (distance==bestDistance && centerDistance<bestCenterDistance)) {
          best=candidate; bestDistance=distance; bestCenterDistance=centerDistance;
          bestMask=mask; bestWidth=width; bestHeld=held;
        }
      }
      begin=end+1;
    }
    if (best.valid) {
      previousError_=best.error;
      previousSelectedMask_=bestMask;
      previousOtherMask_=visibleMask & uint16_t(~bestMask);
      // Retain the pre-merge width until a narrow path is visible again.
      if (!bestHeld) pathWidth_=bestWidth;
      holding_=bestHeld;
      havePrevious_=true;
    }
    return best; // No visible group: caller applies recovery; never invent a branch.
  }
 private:
  bool havePrevious_=false, holding_=false;
  float previousError_=0;
  uint16_t previousSelectedMask_=0, previousOtherMask_=0;
  uint8_t pathWidth_=0;
};
}
