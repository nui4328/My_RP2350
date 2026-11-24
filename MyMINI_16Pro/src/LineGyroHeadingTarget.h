#pragma once

#include <math.h>
#include <stdint.h>

// Tracks an absolute heading referenced to the mission's gyro zero. The raw
// yaw stays continuous; either direction may cross the same modulo-360 target.
class LineGyroHeadingTarget {
 public:
  bool begin(float startRawYaw, float targetHeadingDeg) {
    if (!isfinite(startRawYaw) || !isfinite(targetHeadingDeg)) return false;
    headingDeg_ = fmodf(targetHeadingDeg, 360.0f);
    if (headingDeg_ < 0.0f) headingDeg_ += 360.0f;
    previousRawYaw_ = startRawYaw;
    stopRawYaw_ = startRawYaw;
    armed_ = fabsf(headingError(startRawYaw)) >= kDepartureDeg;
    hit_ = false;
    hitFrames_ = 0;
    return true;
  }

  // Confirm two near frames, or confirm a crossing on the next frame.
  bool sample(float rawYaw) {
    if (!isfinite(rawYaw)) return false;
    const float error = headingError(rawYaw);
    if (!armed_) {
      // A command starting at its target must leave that heading before the
      // same heading can end this line-following leg.
      if (fabsf(error) >= kDepartureDeg) armed_ = true;
      previousRawYaw_ = rawYaw;
      return false;
    }

    const float previousError = headingError(previousRawYaw_);
    const bool crossed = crossedTarget(previousRawYaw_, rawYaw) ||
        (fabsf(previousError) <= 0.25f &&
         fabsf(error) > fabsf(previousError));
    if (crossed) {
      hit_ = true;
      stopRawYaw_ = rawYaw + error;
    }
    previousRawYaw_ = rawYaw;
    if (hit_) {
      if (hitFrames_ < 2u) ++hitFrames_;
    } else if (fabsf(error) <= kToleranceDeg) {
      if (hitFrames_ < 2u) ++hitFrames_;
      stopRawYaw_ = rawYaw + error;
    } else {
      hitFrames_ = 0;
    }
    return hitFrames_ >= 2u;
  }

  float stopRawYaw() const { return stopRawYaw_; }

 private:
  static constexpr float kDepartureDeg = 5.0f;
  static constexpr float kToleranceDeg = 2.0f;

  float headingError(float rawYaw) const {
    float error = fmodf(headingDeg_ - fmodf(rawYaw, 360.0f), 360.0f);
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;
    return error;
  }

  bool crossedTarget(float previousRawYaw, float rawYaw) const {
    const float previousTurns = (previousRawYaw - headingDeg_) / 360.0f;
    const float currentTurns = (rawYaw - headingDeg_) / 360.0f;
    if (rawYaw > previousRawYaw)
      return floorf(currentTurns) > floorf(previousTurns);
    if (rawYaw < previousRawYaw)
      return ceilf(currentTurns) < ceilf(previousTurns);
    return false;
  }

  float headingDeg_ = 0.0f;
  float previousRawYaw_ = 0.0f;
  float stopRawYaw_ = 0.0f;
  uint8_t hitFrames_ = 0;
  bool armed_ = false;
  bool hit_ = false;
};