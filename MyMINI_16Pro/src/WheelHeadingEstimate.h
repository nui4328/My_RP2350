#pragma once
#include <stdint.h>
#include <math.h>

// Open-loop yaw, positive for a right turn. Commands are the actual logical
// motor outputs after sentinel handling; no encoder feedback is implied.
class WheelHeadingEstimate {
 public:
  void configure(uint16_t spinMs, uint16_t pivotMs, uint8_t speed) {
    spin90Ms_ = spinMs; pivot90Ms_ = pivotMs; calibrationSpeed_ = speed;
  }
  void reset(uint32_t nowUs, float yaw = 0.0f) {
    yaw_ = yaw; lastUs_ = nowUs; initialized_ = true;
  }
  void advance(uint32_t nowUs) {
    if (!initialized_) { reset(nowUs); return; }
    yaw_ += rate() * (static_cast<uint32_t>(nowUs - lastUs_) * 1.0e-6f);
    lastUs_ = nowUs;
  }
  void command(int left, int right, uint32_t nowUs) {
    advance(nowUs);
    left_ = absCommand(left); right_ = absCommand(right);
  }
  float yaw() const { return yaw_; }
  float rate() const {
    const float peak = fmaxf(fabsf(left_), fabsf(right_));
    if (peak == 0.0f || left_ == right_) return 0.0f;
    const bool opposed = left_ * right_ < 0.0f;
    const float other = fminf(fabsf(left_), fabsf(right_));
    const float blend = opposed ? other / peak : 0.0f;
    const float ms90 = pivot90Ms_ + (spin90Ms_ - pivot90Ms_) * blend;
    const float turningCommand = opposed ? peak : fabsf(left_ - right_);
    return (left_ > right_ ? 1.0f : -1.0f) * turningCommand * 90000.0f /
        (ms90 * calibrationSpeed_);
  }
 private:
  static float absCommand(int command) {
    // 0 is coast, +/-1 are electrical brake sentinels.
    return command >= -1 && command <= 1 ? 0.0f : static_cast<float>(command);
  }
  float yaw_ = 0.0f, left_ = 0.0f, right_ = 0.0f;
  uint32_t lastUs_ = 0;
  uint16_t spin90Ms_ = 300, pivot90Ms_ = 550;
  uint8_t calibrationSpeed_ = 50;
  bool initialized_ = false;
};
