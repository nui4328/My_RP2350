#include "line_arm.h"

#include <atomic>
#include <limits.h>
#include <math.h>

// Keep both Pico 2 cores at an 8 KB stack when setup1()/loop1() are present.
bool core1_separate_stack = true;

namespace {
constexpr int32_t kNoArmRequest = INT32_MIN;
constexpr uint32_t kHomeTimeoutMs = 3000;
constexpr uint32_t kHomeSettleMs = 200;
constexpr uint32_t kCoreResponseTimeoutMs = 250;
constexpr uint32_t kMaxTimedArmMs = 10000;

std::atomic<int32_t> requestedArmMs{kNoArmRequest};
std::atomic<bool> armStarted{false};
std::atomic<bool> armDone{true};
std::atomic<bool> armSucceeded{false};
std::atomic<bool> cancelArm{false};
std::atomic<uint32_t> armStoppedAtMs{0};

void finishArm(bool succeeded) {
  servo(22, 90);
  armStoppedAtMs.store(millis(), std::memory_order_relaxed);
  armSucceeded.store(succeeded, std::memory_order_relaxed);
  armDone.store(true, std::memory_order_release);
}
}  // namespace

// Arduino-Pico runs these functions on core 1. Only core 1 operates servo 22
// while a six-argument line command is active; no Serial or I2C runs here.
void setup1() {}

void loop1() {
  static bool active = false;
  static int32_t motionMs = 0;
  static uint32_t startedAtMs = 0;

  if (!active) {
    const int32_t request = requestedArmMs.exchange(
        kNoArmRequest, std::memory_order_acq_rel);
    if (request == kNoArmRequest) {
      delay(1);
      return;
    }
    motionMs = request;
    startedAtMs = millis();
    active = true;
    if (motionMs == 0 && digitalRead(1) == LOW) {
      finishArm(true);
      active = false;
    } else {
      servo(22, motionMs > 0 ? 180 : 0);
    }
    armStarted.store(true, std::memory_order_release);
  }

  if (active) {
    const uint32_t elapsedMs = millis() - startedAtMs;
    if (cancelArm.load(std::memory_order_acquire)) {
      finishArm(false);
      active = false;
    } else if (motionMs == 0) {
      if (digitalRead(1) == LOW) {
        finishArm(true);
        active = false;
      } else if (elapsedMs >= kHomeTimeoutMs) {
        finishArm(false);
        active = false;
      }
    } else {
      const uint32_t durationMs = static_cast<uint32_t>(
          motionMs < 0 ? -motionMs : motionMs);
      if (elapsedMs >= durationMs) {
        finishArm(true);
        active = false;
      }
    }
  }
  delay(1);
}

static void runLineWithArm(bool backward, int sl, int sr, float kp,
                           float distanceCm, uint8_t stopPull,
                           int armMotionMs) {
  if (!isfinite(distanceCm) || distanceCm <= 0.0f ||
      armMotionMs < -static_cast<int>(kMaxTimedArmMs) ||
      armMotionMs > static_cast<int>(kMaxTimedArmMs)) {
    Serial.println(backward ? F("B_LINE ARM: invalid distance or arm time") :
                              F("F_LINE ARM: invalid distance or arm time"));
    motor(1, 1);
    servo(22, 90);
    return;
  }

  pinMode(1, INPUT_PULLUP);  // GP1 limit switch is active LOW.
  armDone.store(false, std::memory_order_relaxed);
  armStarted.store(false, std::memory_order_relaxed);
  armSucceeded.store(false, std::memory_order_relaxed);
  cancelArm.store(false, std::memory_order_relaxed);
  requestedArmMs.store(armMotionMs, std::memory_order_release);

  const uint32_t requestAtMs = millis();
  while (!armStarted.load(std::memory_order_acquire) &&
         millis() - requestAtMs < kCoreResponseTimeoutMs)
    delay(1);
  if (!armStarted.load(std::memory_order_acquire)) {
    cancelArm.store(true, std::memory_order_release);
    requestedArmMs.store(kNoArmRequest, std::memory_order_release);
    motor(1, 1);
    servo(22, 90);
    Serial.println(backward ? F("B_LINE ARM: core 1 did not start") :
                              F("F_LINE ARM: core 1 did not start"));
    return;
  }

  // Keep the library's original distance follower on core 0.
  if (backward)
    b_line(sl, sr, kp, distanceCm, stopPull);
  else
    f_line(sl, sr, kp, distanceCm, stopPull);

  if (!armDone.load(std::memory_order_acquire))
    cancelArm.store(true, std::memory_order_release);
  const uint32_t stopRequestedAtMs = millis();
  // stopPull == 0 hands the moving wheels to the next command. Do not leave
  // them running while waiting for a slow core-1 response.
  const uint32_t stopWaitMs = stopPull == 0u ? 10u : kCoreResponseTimeoutMs;
  while (!armDone.load(std::memory_order_acquire) &&
         millis() - stopRequestedAtMs < stopWaitMs)
    delay(1);
  if (!armDone.load(std::memory_order_acquire)) {
    motor(1, 1);
    servo(22, 90);
    Serial.println(backward ? F("B_LINE ARM: core 1 did not stop the arm") :
                              F("F_LINE ARM: core 1 did not stop the arm"));
    return;
  }

  if (!armSucceeded.load(std::memory_order_acquire)) {
    Serial.println(backward ?
        (armMotionMs == 0 ?
            F("B_LINE ARM: lower limit not reached before timeout or line end") :
            F("B_LINE ARM: timed motion interrupted at line end")) :
        (armMotionMs == 0 ?
            F("F_LINE ARM: lower limit not reached before timeout or line end") :
            F("F_LINE ARM: timed motion interrupted at line end")));
    return;
  }
  if (armMotionMs == 0 && stopPull != 0u) {
    const uint32_t elapsedSinceStop = millis() -
        armStoppedAtMs.load(std::memory_order_relaxed);
    if (elapsedSinceStop < kHomeSettleMs)
      delay(kHomeSettleMs - elapsedSinceStop);
  }
}

void f_line(int sl, int sr, float kp, float distanceCm,
            uint8_t stopPull, int armMotionMs) {
  runLineWithArm(false, sl, sr, kp, distanceCm, stopPull, armMotionMs);
}

void b_line(int sl, int sr, float kp, float distanceCm,
            uint8_t stopPull, int armMotionMs) {
  runLineWithArm(true, sl, sr, kp, distanceCm, stopPull, armMotionMs);
}
