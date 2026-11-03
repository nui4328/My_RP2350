#include "MotionHeading.h"
#include "WheelHeadingEstimate.h"
#include "MyMINI_ProInternal.h"

namespace {
WheelHeadingEstimate estimate;
bool referenceReady = false;
bool estimated = false;
float currentRate = 0.0f;
}

bool gyro_estimation_active() { return referenceReady && estimated; }
bool motion_heading_ready() {
  return referenceReady || (imuReady && headingReferenceReady);
}
void motion_heading_zero(bool useGyro) {
  estimate.reset(micros());
  estimated = !useGyro;
  referenceReady = true;
  currentRate = 0.0f;
}
void motion_heading_command(int left, int right) {
  estimate.command(left, right, micros());
}
void motion_heading_calibrate(uint16_t spinMs, uint16_t pivotMs, uint8_t speed) {
  estimate.advance(micros());
  estimate.configure(spinMs, pivotMs, speed);
}
bool motion_heading_update() {
  if (!motion_heading_ready()) return false;
  estimate.advance(micros());
  // No IMU probes or recovery in estimation mode. Only explicit reset/set
  // selects the physical gyro again.
  if (!estimated) {
    if (imuReady && headingReferenceReady && imu.update() &&
        isfinite(imu.yawRaw()) && isfinite(imu.gyro('z'))) {
      estimate.reset(micros(), imu.yawRaw());
      currentRate = imu.gyro('z');
      referenceReady = true;
      return true;
    }
    // Continue from the last measured heading plus driven wheel time.
    estimated = true;
    referenceReady = true;
    headingReferenceReady = false;
    Serial.println(F("GYRO_ESTIMATED: read failed; explicit gyro_reset/set required"));
  }
  estimate.advance(micros());
  currentRate = estimate.rate();
  return isfinite(estimate.yaw());
}
float motion_heading_yaw() { return estimate.yaw(); }
float motion_heading_rate() { return currentRate; }
