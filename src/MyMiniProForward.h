#pragma once
#include <stdint.h>
#include <math.h>

// Reserve these names for rear sensors; use standard 0b literals for binary numbers.
#ifdef B0
#undef B0
#endif
#ifdef B1
#undef B1
#endif
#ifdef B10
#undef B10
#endif
#ifdef B11
#undef B11
#endif

namespace MyMiniProForward {
// F_Line gains use position units [-50,+50]; tracker geometry stays normalized.
constexpr float kPositionErrorScale=50.0f;
// F/B numbers are MUX channel IDs, independent of the steering frontMap.
enum class Sensor : uint8_t {
  F0, F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12, F13, F14, F15,
  CL, CR,
  B0, B1, B2, B3, B4, B5, B6, B7, B8, B9, B10, B11, B12, B13, B14, B15,
  // Synonyms keep Sensor::B0 etc. valid after the public compatibility macros.
  MyMiniPro_B0=B0, MyMiniPro_B1=B1, MyMiniPro_B10=B10, MyMiniPro_B11=B11,
  // Lowercase names are typed aliases; existing numeric IDs stay unchanged.
  f0=F0, f1=F1, f2=F2, f3=F3, f4=F4, f5=F5, f6=F6, f7=F7,
  f8=F8, f9=F9, f10=F10, f11=F11, f12=F12, f13=F13, f14=F14, f15=F15,
  cl=CL, cr=CR,
  b0=B0, b1=B1, b2=B2, b3=B3, b4=B4, b5=B5, b6=B6, b7=B7,
  b8=B8, b9=B9, b10=B10, b11=B11, b12=B12, b13=B13, b14=B14, b15=B15
};

enum class Result : uint8_t {
  DistanceReached, SensorDetected, InvalidArgument, CalibrationNotReady,
  DistanceNotCalibrated, LineLost, SensorError, Timeout, Stopped, MotorsNotReady,
  TurnCompleted, StopCompleted, CrossCompleted, InvalidAction, InvalidTurnSensor,
  InvalidCrossSensor
};
enum class Action : uint8_t {
  FL, FR, CL, CR, NL, NR, STOP, CROSS, NS, CS, CP, FS, FP, Invalid,
  fl=FL, fr=FR, cl=CL, cr=CR, nl=NL, nr=NR, stop=STOP, cross=CROSS, ns=NS, cs=CS, cp=CP, fs=FS, fp=FP
};
struct ActionChoice {
  Action value;
  constexpr ActionChoice(Action action) : value(action) {}
  constexpr ActionChoice(Sensor sensor) : value(sensor==Sensor::CL ? Action::CL :
      sensor==Sensor::CR ? Action::CR : Action::Invalid) {}
};
struct BrakePulse { int left, right; uint16_t durationMs; };
inline int clampPercent(int value) { return value<0 ? 0 : value>100 ? 100 : value; }
inline BrakePulse brakePulse(int left, int right, int level) {
  level=clampPercent(level);
  if(!level || (!left && !right))return {0,0,0};
  const auto counter=[level](int speed) {
    const int magnitude=clampPercent(speed<0 ? -speed : speed);
    const int command=(magnitude*level+99)/100;
    return speed>0 ? -command : speed<0 ? command : 0;
  };
  return {counter(left),counter(right),uint16_t(5+(25*level+99)/100)};
}
inline const char *resultName(Result result) {
  switch (result) {
    case Result::DistanceReached: return "DistanceReached";
    case Result::SensorDetected: return "SensorDetected";
    case Result::InvalidArgument: return "InvalidArgument";
    case Result::CalibrationNotReady: return "CalibrationNotReady";
    case Result::DistanceNotCalibrated: return "DistanceNotCalibrated";
    case Result::LineLost: return "LineLost";
    case Result::SensorError: return "SensorError";
    case Result::Timeout: return "Timeout";
    case Result::Stopped: return "Stopped";
    case Result::MotorsNotReady: return "MotorsNotReady";
    case Result::TurnCompleted: return "TurnCompleted";
    case Result::StopCompleted: return "StopCompleted";
    case Result::CrossCompleted: return "CrossCompleted";
    case Result::InvalidAction: return "InvalidAction";
    case Result::InvalidTurnSensor: return "InvalidTurnSensor";
    case Result::InvalidCrossSensor: return "InvalidCrossSensor";
  }
  return "Unknown";
}
// Pure unit conversion from a measured run; never infers RPM from wheel size.
// Returns NAN for invalid inputs/results. meanNominalPercent is before voltage gain.
inline float distanceCoefficientFromMeasurement(float centimeters, uint32_t elapsedMs,
                                                 float meanNominalPercent) {
  if (!isfinite(centimeters) || centimeters<=0 || !elapsedMs ||
      !isfinite(meanNominalPercent) || meanNominalPercent<=0 || meanNominalPercent>100)
    return NAN;
  const float coefficient=centimeters/(elapsedMs*0.001f*meanNominalPercent);
  return isfinite(coefficient) && coefficient>0 ? coefficient : NAN;
}
struct Settings {
  // cm/s per one percent command, measured with compensation enabled.
  float centimetersPerSecondPerPercent = 0;
  uint32_t maximumRunMs = 3000; // distance-mode safety limit
  uint32_t sensorMaximumRunMs = 0; // optional sensor safety limit; 0 disables
  uint16_t debounceMs = 20; // clear-surface arming time; line confirmation is separate
  bool frontLineHigh = false;
  bool rearLineHigh = false;
  bool centerLineHigh = false;
  // Steering order only; NEVER remaps the selected exit sensor.
  uint8_t frontMap[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
  // Called each iteration; true requests an immediate stop. Must not block.
  bool (*stopRequested)() = nullptr;
  // Rear steering order in chassis-left to chassis-right order; B_Line only.
  uint8_t rearMap[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
  // F_Line only: linear startup envelope, ms from first translation command.
  // Zero restores immediate output. Snapshotted per call; ignored by B_Line.
  // A wheel with requested translation speed below25 bypasses both profiles.
  uint32_t startupRampMs = 300;
  // F_Line numeric Follow only; zero disables distance deceleration.
  float distanceDecelerationCm = 5.0f;
  // Percent OF requested speed (1..100); held over final fifth of decel window.
  // Example40 target -> approximately8 at20; integer motor resolution still applies.
  uint8_t distanceEndSpeedPercent = 20;
};
// A stable clear surface is required before a stable line, including at boot.
class Edge {
 public:
  bool update(uint16_t strength, uint32_t now, uint16_t debounce) {
    return update(strength,now,debounce,debounce);
  }
  bool update(uint16_t strength, uint32_t now, uint16_t clearMs, uint16_t lineMs) {
    if (triggered_) return true;
    const bool qualifying = armed_ ? strength >= 600 : strength <= 400;
    if (!qualifying) { timing_ = false; return false; }
    if (!timing_) { since_ = now; timing_ = true; }
    if (uint32_t(now - since_) < (armed_ ? lineMs : clearMs)) return false;
    if (armed_) { triggered_=true; return true; }
    armed_ = true; timing_ = false; return false;
  }
 private:
  bool armed_ = false, timing_ = false, triggered_ = false;
  uint32_t since_ = 0;
};
}






