#pragma once

#include <math.h>
#include <stdint.h>

// Heading target validation used by turn_gyro(). Angles are absolute to the
// wait_button() start reference; positive travel is a right turn.
enum class TurnGyroResult : unsigned char {
  Running,
  Reached,
  InvalidTarget,
  InvalidSample,
  WrongDirection
};

class TurnGyroTarget {
 public:
  static bool timedOut(uint32_t nowMs, uint32_t startedAtMs,
                       uint16_t timeoutMs) {
    return static_cast<uint32_t>(nowMs - startedAtMs) >= timeoutMs;
  }

  static float wrap180(float angle) {
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
  }

  TurnGyroResult begin(float currentDeg, float targetDeg, int direction,
                       float toleranceDeg = 1.5f) {
    if (!isfinite(targetDeg) || targetDeg < -180.0f || targetDeg > 180.0f ||
        !isfinite(toleranceDeg) || toleranceDeg <= 0.0f) {
      return TurnGyroResult::InvalidTarget;
    }
    if (!isfinite(currentDeg)) return TurnGyroResult::InvalidSample;
    if (direction != -1 && direction != 1) {
      return TurnGyroResult::WrongDirection;
    }
    const float rawError = targetDeg - currentDeg;
    if (!isfinite(rawError)) return TurnGyroResult::InvalidSample;
    const float error = wrap180(rawError);
    if (fabsf(error) <= toleranceDeg) return TurnGyroResult::Reached;
    const float required = static_cast<float>(direction) * error;
    if (required <= 0.0f) return TurnGyroResult::WrongDirection;
    direction_ = direction;
    requiredDeg_ = required;
    travelledDeg_ = 0.0f;
    previousDeg_ = currentDeg;
    lastStepDeg_ = 0.0f;
    return TurnGyroResult::Running;
  }

  // Legacy early-stop tracker, kept for callers of this internal helper.
  // turn_gyro() now uses rate-based braking and settled-heading correction.
  TurnGyroResult beginWithBrakeLead(float currentDeg, float targetDeg,
                                    int direction, float leadDeg,
                                    float brakeToleranceDeg = 0.05f) {
    if (!isfinite(leadDeg) || leadDeg < 0.0f ||
        !isfinite(brakeToleranceDeg) || brakeToleranceDeg <= 0.0f) {
      return TurnGyroResult::InvalidTarget;
    }
    const TurnGyroResult result = begin(currentDeg, targetDeg, direction);
    if (result != TurnGyroResult::Running) return result;
    const float appliedLead = fminf(leadDeg, requiredDeg_ * 0.5f);
    const float brakeHeading = wrap180(
        currentDeg + static_cast<float>(direction) *
                         (requiredDeg_ - appliedLead));
    return begin(currentDeg, brakeHeading, direction, brakeToleranceDeg);
  }

  TurnGyroResult sample(float currentDeg, float toleranceDeg = 1.5f) {
    if (!isfinite(currentDeg)) return TurnGyroResult::InvalidSample;
    const float rawDelta = currentDeg - previousDeg_;
    if (!isfinite(rawDelta)) return TurnGyroResult::InvalidSample;
    const float delta = wrap180(rawDelta);
    previousDeg_ = currentDeg;
    lastStepDeg_ = static_cast<float>(direction_) * delta;
    travelledDeg_ += lastStepDeg_;
    // A substantial opposite yaw means the motor ratio or physical polarity
    // disagrees with the selected turn mode.
    if (travelledDeg_ < -5.0f) return TurnGyroResult::WrongDirection;
    return travelledDeg_ >= requiredDeg_ - toleranceDeg
        ? TurnGyroResult::Reached : TurnGyroResult::Running;
  }

  float remainingDeg() const { return requiredDeg_ - travelledDeg_; }
  float lastStepDeg() const { return lastStepDeg_; }

 private:
  int direction_ = 0;
  float requiredDeg_ = 0.0f;
  float travelledDeg_ = 0.0f;
  float previousDeg_ = 0.0f;
  float lastStepDeg_ = 0.0f;
};
