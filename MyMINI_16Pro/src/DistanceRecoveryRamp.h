#pragma once

// Preserve distance deceleration while removing only startup-ramp attenuation
// from the steering command during a confirmed remembered-edge recovery.
inline float distanceRecoverySteeringFactor(bool directDistanceTracking,
    bool edgeRecoveryActive, bool anyBlack, bool afterTurnEntry,
    float accelFactor, float decelFactor, float motionFactor) {
  if (directDistanceTracking && edgeRecoveryActive && !anyBlack &&
      !afterTurnEntry && accelFactor < 1.0f) return decelFactor;
  return afterTurnEntry ? decelFactor : motionFactor;
}
