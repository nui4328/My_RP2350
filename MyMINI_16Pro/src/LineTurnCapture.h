#pragma once

#include <math.h>
#include <stdint.h>

// One-shot permission to receive a successful, unbraked sensor turn. A later
// motor command (including an explicit stop), another motion, or a deliberate
// pause invalidates it. Unsigned subtraction also handles micros() wraparound.
class TurnLineHandoff {
 public:
  void arm(uint32_t motorSequence, uint32_t nowUs) {
    sequence_ = motorSequence;
    completedAtUs_ = nowUs;
    pending_ = true;
  }

  void clear() { pending_ = false; }

  bool consume(uint32_t motorSequence, uint32_t nowUs, bool forward) {
    const bool accepted = pending_ && forward && sequence_ == motorSequence &&
        static_cast<uint32_t>(nowUs - completedAtUs_) <= 250000UL;
    clear();
    return accepted;
  }

 private:
  uint32_t sequence_ = 0u;
  uint32_t completedAtUs_ = 0u;
  bool pending_ = false;
};

// Applies only to the next forward line command after turn(..., stopPull=0).
// Error is the accepted, un-rate-limited line position minus the requested
// target, in the library's -50..50 position units. No IMU read is needed.
class LineTurnCapture {
 public:
  explicit LineTurnCapture(uint16_t rampMs) : rampMs_(rampMs) {}

  float update(uint32_t nowUs, bool lineFound, float error) {
    if (rampMs_ == 0u || finished_) return 1.0f;

    if (!lineFound || !isfinite(error)) {
      // Leave steering to the existing recovery; do not push forward blind.
      ramping_ = false;
      factor_ = 0.0f;
      return factor_;
    }

    if (ramping_ && fabsf(error) > 20.0f) {
      ramping_ = false;
    }

    if (!ramping_) {
      // Smoothly interpolate: <=10 -> 80%, 20 -> 55%, >=30 -> 30%.
      const float x = clamp((fabsf(error) - 10.0f) / 20.0f);
      const float desired = 0.80f - 0.50f * smooth(x);
      factor_ = desired;
      // Begin on the first accepted frame, including after recapture.
      // No centered-frame count, position-rate or group-width startup gate.
      ramping_ = true;
      rampStartedAtUs_ = nowUs;
      rampStartFactor_ = factor_;
      return factor_;
    }

    const uint32_t elapsedUs = nowUs - rampStartedAtUs_;
    // Half the configured ramp, retaining 0.5 ms precision for odd values.
    const uint32_t durationUs = static_cast<uint32_t>(rampMs_) * 500UL;
    factor_ = rampStartFactor_ + (1.0f - rampStartFactor_) *
        smooth(static_cast<float>(elapsedUs) / durationUs);
    if (elapsedUs >= durationUs) finished_ = true;
    return factor_;
  }

 private:
  static float clamp(float x) {
    return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
  }
  static float smooth(float x) {
    x = clamp(x);
    return x * x * (3.0f - 2.0f * x);
  }

  uint16_t rampMs_;
  bool ramping_ = false;
  bool finished_ = false;
  uint32_t rampStartedAtUs_ = 0u;
  float rampStartFactor_ = 0.80f;
  float factor_ = 0.30f;
};
