#include "LineFollower.h"

#include <math.h>

#include <BNO055.h>

#include "CalibrationManager.h"
#include "DualMuxSensors.h"
#include "I2CDevices.h"
#include "MyMINI_ProInternal.h"
#include "MotorDriver.h"
#include "RobotConfig.h"
#include "TurnGyroTarget.h"

// -----------------------------------------------------------------------------
// Constants, internal types, sensor helpers, PID, turn, gyro, and rotate helpers
// -----------------------------------------------------------------------------

namespace {

constexpr int8_t LINE_WEIGHT[MyMINIConfig::SENSOR_COUNT] = {
    -50, -43, -37, -30, -23, -17, -10, -3,
      3,  10,  17,  23,  30,  37,  43, 50
};
constexpr uint16_t LINE_BLACK_NOISE_THRESHOLD = 200;
constexpr uint32_t LINE_ALL_WHITE_TIMEOUT_US = 3000000UL;
constexpr uint16_t EXIT_WHITE_RELEASE = 800;
// f_line()/b_line() only: black is below the calibrated midpoint (~500).
// Leave a 100-point hysteresis margin while allowing a white floor that
// normalizes around 630-660 to arm the exit detector.
constexpr uint16_t LINE_EXIT_WHITE_RELEASE = 600;
constexpr uint32_t EXIT_ARM_DELAY_US = 100000;
constexpr uint32_t LINE_LOST_TIMEOUT_US = 100000;
constexpr float PID_MIN_DT_SECONDS = 0.001f;
constexpr float PID_MAX_DT_SECONDS = 0.05f;
constexpr float DERIVATIVE_FILTER_ALPHA = 0.70f;
constexpr float INTEGRAL_LIMIT = 1000.0f;
constexpr uint32_t TURN_STOP_PULL_DURATION_US = 35000;
constexpr float FW_GYRO_CORRECTION_SIGN = -1.0f;
constexpr float ROTATE_GYRO_KP = 0.90f;
constexpr float ROTATE_TOLERANCE_DEG = 1.5f;
constexpr float TURN_GYRO_BRAKE_LEAD_DEG = 20.0f;
constexpr float TURN_GYRO_BRAKE_TOLERANCE_DEG = 0.05f;
constexpr uint8_t ROTATE_MIN_SPEED = 15;
constexpr uint16_t ROTATE_LOOP_DELAY_MS = 5;
constexpr uint16_t ROTATE_BRAKE_MS = 35;
uint16_t rotateSpin90Ms = 300;
uint16_t rotatePivot90Ms = 550;
uint8_t rotateCalibrationSpeed = 50;

struct TurnMotorRatio {
  int8_t left;
  int8_t right;
};

TurnMotorRatio turnMotorRatios[] = {
    {-5, 100},   // TurnMode::fl
    {100, -5},   // TurnMode::fr
    {-100, 100},  // TurnMode::cl
    {100, -100},  // TurnMode::cr
    {-100, 100},  // TurnMode::l
    {100, -100}   // TurnMode::r
};
uint16_t turnOvershootMs = 20;                 // à¹€à¸”à¸´à¸™à¸«à¸™à¹‰à¸²à¸•à¹ˆà¸­à¸«à¸¥à¸±à¸‡à¸‚à¹‰à¸²à¸¡à¹€à¸ªà¹‰à¸™ à¸à¹ˆà¸­à¸™à¹€à¸šà¸£à¸à¹à¸¥à¸°à¸«à¸¡à¸¸à¸™ (ms)
uint16_t turnTimeoutMs = 3000;                 // à¹€à¸§à¸¥à¸²à¸ªà¸¹à¸‡à¸ªà¸¸à¸”à¸‚à¸­à¸‡ turn() à¸à¹ˆà¸­à¸™à¸«à¸¢à¸¸à¸”à¸‰à¸¸à¸à¹€à¸‰à¸´à¸™ (ms)
float turnApproachKp = 0.150f;                 // Kp à¸Šà¹ˆà¸§à¸‡à¹€à¸”à¸´à¸™à¸«à¸™à¹‰à¸² PID à¹€à¸‚à¹‰à¸²à¸«à¸²à¹€à¸ªà¹‰à¸™
uint8_t turnSensorExitSpeed = 10;
uint8_t turnDistanceExitSpeed = 30;
uint8_t turnTouchBrake = 15;                   // à¸à¸³à¸¥à¸±à¸‡à¹€à¸šà¸£à¸à¸¢à¹‰à¸­à¸™à¸«à¸¥à¸±à¸‡à¸‚à¹‰à¸²à¸¡à¹€à¸ªà¹‰à¸™ (0â€“100)
uint16_t turnTouchBrakeMs = 20;                 // Reverse brake duration before rotation (ms).
uint8_t turnLineSearchSpeed = 70;              // à¸„à¸§à¸²à¸¡à¹€à¸£à¹‡à¸§à¸«à¸¡à¸¸à¸™à¸Šà¹‰à¸²à¸Šà¹ˆà¸§à¸‡à¸„à¹‰à¸™à¸«à¸²à¹€à¸ªà¹‰à¸™à¸«à¸¢à¸¸à¸” (1â€“100)
uint16_t turnFastTimeMs = 60;                  // à¹€à¸§à¸¥à¸²à¸«à¸¡à¸¸à¸™à¹€à¸£à¹‡à¸§ à¸à¹ˆà¸­à¸™à¸¥à¸”à¹€à¸›à¹‡à¸™à¸„à¸§à¸²à¸¡à¹€à¸£à¹‡à¸§à¸„à¹‰à¸™à¸«à¸²à¹€à¸ªà¹‰à¸™ (ms)

enum class LastMotionType : uint8_t {
  none,
  fLine,
  bLine,
  turn
};

enum class LineExitSource : uint8_t {
  none,
  front,
  rear,
  center,
  distance
};

enum class TravelDirection : uint8_t {
  none,
  forward,
  backward
};

LastMotionType lastMotionType = LastMotionType::none;
LineExitSource lastLineExitSource = LineExitSource::none;
TravelDirection lastLineDirection = TravelDirection::none;
bool lastLineEndedNormally = false;
bool pendingLineHandoff = false;
bool previousLineCommandContinuous = false;
bool lineDiagnosticsEnabled = false;
bool lineErrorMonitorEnabled = false;
uint16_t lineRampMs = LINE_ACCEL_TIME_MS;
float lineKpScale = 1.0f;
float lineKi = LINE_KI;
float lineKd = LINE_KD;

bool lineDebug() { return LINE_DEBUG || lineDiagnosticsEnabled; }

// Computed PID and turn outputs must not become the public brake sentinel.
void motorMotion(int left, int right) {
  if (left == 1) left = 2;
  else if (left == -1) left = -2;
  if (right == 1) right = 2;
  else if (right == -1) right = -2;
  motor(left, right);
}

void printLineReason(bool reverseDirection,
                      const __FlashStringHelper* reason, uint8_t stopPull) {
  if (!lineDebug()) return;
  Serial.print(reverseDirection ? F("b_line reason=") : F("f_line reason="));
  Serial.print(reason);
  Serial.print(F(" offset="));
  Serial.println(stopPull);
}

void stopLineWithError(bool reverseDirection,
                       const __FlashStringHelper* reason, uint8_t stopPull) {
  motor(1, 1);
  tone(MyMINIConfig::PIN_BUZZER, 3500, 400);
  printLineReason(reverseDirection, reason, stopPull);
}

struct TurnExitState {
  bool initialized = false;
  bool exitWasBlackAtStart = false;
  bool exitHasSeenWhite = false;
  bool exitArmed = false;
  uint8_t consecutiveWhiteSamples = 0;
  uint32_t lastFrameSequence = 0;
};

// CalibrationManager::begin() loads all EEPROM min/max values into data_ once
// during robot_begin(). A later successful calibration updates that same RAM
// data. Compare each fresh ADC sample against its own channel midpoint.
bool rawBelowCalibrationMidpoint(int32_t raw, int32_t minValue,
                                  int32_t maxValue) {
  return raw * 2 < minValue + maxValue;
}

// The exit sensor is always the requested channel. Its companion is the
// immediately adjacent channel toward the centre of the 16-sensor row.
uint8_t inwardExitCompanion(uint8_t selectedIndex) {
  return selectedIndex <= 7u ? selectedIndex + 1u : selectedIndex - 1u;
}

bool normalizedFront(uint8_t channel, uint16_t& normalized,
                     bool* isBlack = nullptr) {
  if (!calibrationManager.frontValid() ||
      channel >= MyMINIConfig::SENSOR_COUNT) {
    return false;
  }

  const CalibrationData& calibration = calibrationManager.data();
  const uint16_t minValue = calibration.frontMin[channel];
  const uint16_t maxValue = calibration.frontMax[channel];
  if (maxValue <= minValue) return false;

  const int32_t raw = sensorArrays.frontFiltered()[channel];
  if (isBlack) *isBlack = rawBelowCalibrationMidpoint(raw, minValue, maxValue);
  const int32_t mapped = static_cast<int32_t>(map(
      raw, static_cast<int32_t>(minValue), static_cast<int32_t>(maxValue),
      0L, 1000L));
  normalized = static_cast<uint16_t>(constrain(mapped, 0L, 1000L));
  return true;
}

bool normalizedRear(uint8_t channel, uint16_t& normalized,
                    bool* isBlack = nullptr) {
  if (!calibrationManager.rearValid() ||
      channel >= MyMINIConfig::SENSOR_COUNT) {
    return false;
  }

  const CalibrationData& calibration = calibrationManager.data();
  const uint16_t minValue = calibration.rearMin[channel];
  const uint16_t maxValue = calibration.rearMax[channel];
  if (maxValue <= minValue) return false;

  const int32_t raw = sensorArrays.rearFiltered()[channel];
  if (isBlack) *isBlack = rawBelowCalibrationMidpoint(raw, minValue, maxValue);
  const int32_t mapped = static_cast<int32_t>(map(
      raw, static_cast<int32_t>(minValue), static_cast<int32_t>(maxValue),
      0L, 1000L));
  normalized = static_cast<uint16_t>(constrain(mapped, 0L, 1000L));
  return true;
}

bool normalizedTrackingSensor(bool useRearSensor, uint8_t channel,
                              uint16_t& normalized,
                              bool* isBlack = nullptr) {
  return useRearSensor ? normalizedRear(channel, normalized, isBlack)
                       : normalizedFront(channel, normalized, isBlack);
}

bool normalizedCenter(uint8_t channel, uint16_t& normalized,
                      bool* isBlack = nullptr) {
  if (!calibrationManager.centerValid() || channel > 1u) return false;

  const CalibrationData& calibration = calibrationManager.data();
  const int16_t minValue = calibration.centerMin[channel];
  const int16_t maxValue = calibration.centerMax[channel];
  if (maxValue <= minValue) return false;

  const uint8_t adsChannel = channel == 0u
      ? MyMINIConfig::ADS_CHANNEL_CENTER_LEFT
      : MyMINIConfig::ADS_CHANNEL_CENTER_RIGHT;
  int16_t raw = 0;
  if (!i2cDevices.readAds1115SingleEnded(adsChannel, raw)) return false;
  if (isBlack) *isBlack = rawBelowCalibrationMidpoint(raw, minValue, maxValue);

  const int32_t mapped = static_cast<int32_t>(map(
      static_cast<int32_t>(raw), static_cast<int32_t>(minValue),
      static_cast<int32_t>(maxValue), 0L, 1000L));
  normalized = static_cast<uint16_t>(constrain(mapped, 0L, 1000L));
  return true;
}

void printPairExitDebug(bool rearExit, uint8_t selected, uint8_t pairFirst,
                        uint8_t pairSecond, uint16_t firstValue,
                        uint16_t secondValue, uint8_t blackSampleCount) {
  const char prefix = rearExit ? 'b' : 'f';
  Serial.print(F("exit="));
  Serial.print(prefix);
  Serial.print(selected);
  Serial.print(F(" pair="));
  Serial.print(prefix);
  Serial.print(pairFirst);
  Serial.print(',');
  Serial.print(prefix);
  Serial.print(pairSecond);
  Serial.print(F(" value1="));
  Serial.print(firstValue);
  Serial.print(F(" value2="));
  Serial.print(secondValue);
  Serial.print(F(" samples="));
  Serial.print(blackSampleCount);
  Serial.println(F("/3"));
}

void printCenterExitDebug(LineExitSensor exitSensor, uint16_t value,
                          uint8_t blackSampleCount) {
  Serial.print(F("exit="));
  Serial.print(exitSensor == cl ? F("cl") : F("cr"));
  Serial.print(F(" value="));
  Serial.print(value);
  Serial.print(F(" samples="));
  Serial.print(blackSampleCount);
  Serial.println(F("/3"));
}

bool selectTrackedGroup(const uint16_t* blackStrength, float referenceError,
                        bool trackingInitialized, float& measuredError,
                        uint8_t& outsideBlack) {
  outsideBlack = 0;
  if (trackingInitialized) {
    for (uint8_t i = 0; i < MyMINIConfig::SENSOR_COUNT; ++i) {
      if (blackStrength[i] > 0 &&
          fabsf(LINE_WEIGHT[i] - referenceError) > TRACK_WINDOW_RADIUS) {
        ++outsideBlack;
      }
    }
  }

  bool foundGroup = false;
  float closestGroupDistance = 1000.0f;
  for (uint8_t start = 0; start < MyMINIConfig::SENSOR_COUNT;) {
    while (start < MyMINIConfig::SENSOR_COUNT && blackStrength[start] == 0) {
      ++start;
    }
    if (start >= MyMINIConfig::SENSOR_COUNT) break;

    uint8_t end = start;
    while (end + 1u < MyMINIConfig::SENSOR_COUNT &&
           blackStrength[end + 1u] > 0) {
      ++end;
    }

    const bool wideGroup = (end - start + 1u) > 5u;
    int32_t weightedSum = 0;
    uint32_t strengthSum = 0;
    for (uint8_t i = start; i <= end; ++i) {
      const bool outsideWindow =
          fabsf(LINE_WEIGHT[i] - referenceError) > TRACK_WINDOW_RADIUS;
      if ((trackingInitialized || wideGroup) && outsideWindow) continue;
      weightedSum += static_cast<int32_t>(blackStrength[i]) * LINE_WEIGHT[i];
      strengthSum += blackStrength[i];
    }

    if (strengthSum > 0) {
      const float candidate = static_cast<float>(weightedSum) / strengthSum;
      const float candidateDistance = fabsf(candidate - referenceError);
      if (!foundGroup || candidateDistance < closestGroupDistance) {
        foundGroup = true;
        closestGroupDistance = candidateDistance;
        measuredError = candidate;
      }
    }
    start = end + 1u;
  }
  return foundGroup;
}

// If the line jumps beyond the tracking window, the normal selector can see
// black sensors yet return "no line" forever because its reference error is
// frozen. Recover only from a persistent group of black sensors
// and choose the nearest group, without changing turn()'s branch selection.
bool selectRecoveryGroup(const uint16_t* blackStrength, float referenceError,
                         float& measuredError, uint8_t& groupSize) {
  bool found = false;
  float closestDistance = 1000.0f;
  for (uint8_t start = 0; start < MyMINIConfig::SENSOR_COUNT;) {
    while (start < MyMINIConfig::SENSOR_COUNT && blackStrength[start] == 0u) {
      ++start;
    }
    if (start >= MyMINIConfig::SENSOR_COUNT) break;

    uint8_t end = start;
    int32_t weightedSum = blackStrength[start] * LINE_WEIGHT[start];
    uint32_t strengthSum = blackStrength[start];
    while (end + 1u < MyMINIConfig::SENSOR_COUNT &&
           blackStrength[end + 1u] > 0u) {
      ++end;
      weightedSum += static_cast<int32_t>(blackStrength[end]) * LINE_WEIGHT[end];
      strengthSum += blackStrength[end];
    }
    const float candidate = static_cast<float>(weightedSum) / strengthSum;
    const float distance = fabsf(candidate - referenceError);
    if (!found || distance < closestDistance) {
      found = true;
      closestDistance = distance;
      measuredError = candidate;
      groupSize = end - start + 1u;
    }
    start = end + 1u;
  }
  return found;
}

// Keep the correction calculation shared by line() and turn() so that an
// approach into a turn follows exactly the same PID convention as f_line()
// and b_line().
float calculateLinePidCorrection(float kp, float error, float dt,
                                  bool lineFound, int8_t correctionDirection,
                                  float minimumCommand, float baseLeft,
                                  float baseRight,
                                  float& previousError, float& integral,
                                  float& filteredDerivative,
                                  float ki = LINE_KI,
                                  bool integrateOnlyWhenLineFound = false,
                                  float kd = LINE_KD) {
  const float rawDerivative = lineFound
      ? (error - previousError) / dt : 0.0f;
  filteredDerivative = DERIVATIVE_FILTER_ALPHA * filteredDerivative +
      (1.0f - DERIVATIVE_FILTER_ALPHA) * rawDerivative;

  const float proposedIntegral =
      (!integrateOnlyWhenLineFound || lineFound) ? integral + error * dt : integral;
  const float proportional = kp * error;
  const float derivativeTerm = kd * filteredDerivative;
  const float proposedCorrection = proportional +
      ki * proposedIntegral + derivativeTerm;
  const float proposedLeft = baseLeft +
      correctionDirection * proposedCorrection;
  const float proposedRight = baseRight -
      correctionDirection * proposedCorrection;
  if (proposedLeft >= minimumCommand && proposedLeft <= 100.0f &&
      proposedRight >= minimumCommand && proposedRight <= 100.0f) {
    integral = constrain(proposedIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  }

  return proportional + ki * integral + derivativeTerm;
}

float smoothstep(float value) {
  value = constrain(value, 0.0f, 1.0f);
  return value * value * (3.0f - 2.0f * value);
}

float wrapAngle180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

bool validRotateArguments(float angleDeg, uint8_t speed, uint8_t stopPull,
                          bool allowZeroAngle = false) {
  return isfinite(angleDeg) && (allowZeroAngle || angleDeg != 0.0f) &&
      fabsf(angleDeg) <= 360.0f && speed >= 1u && speed <= 100u &&
      stopPull <= 100u;
}

void driveRotation(bool spinMode, bool backwardPivot, bool turnRight,
                   uint8_t commandSpeed) {
  const int command = static_cast<int>(commandSpeed);
  if (spinMode) {
    motorMotion(turnRight ? command : -command, turnRight ? -command : command);
  } else if (backwardPivot) {
    motorMotion(turnRight ? 0 : -command, turnRight ? -command : 0);
  } else {
    motorMotion(turnRight ? command : 0, turnRight ? 0 : command);
  }
}

bool completeRotation(bool spinMode, bool backwardPivot, bool turnRight,
                      uint8_t stopPull) {
  if (stopPull == 0u) {
    motor(1, 1);
    return true;
  }

  const int pull = static_cast<int>(stopPull);
  if (spinMode) {
    motorMotion(turnRight ? -pull : pull, turnRight ? pull : -pull);
  } else if (backwardPivot) {
    motorMotion(turnRight ? 0 : pull, turnRight ? pull : 0);
  } else {
    motorMotion(turnRight ? -pull : 0, turnRight ? 0 : -pull);
  }
  delay(ROTATE_BRAKE_MS);
  motor(1, 1);
  return true;
}

bool rotateWithGyro(bool spinMode, bool backwardPivot, float angleDeg,
                    uint8_t speed, uint8_t stopPull) {
  if (!imu.update()) {
    motor(1, 1);
    return false;
  }

  float lastYaw = imu.yaw();
  if (!isfinite(lastYaw)) {
    motor(1, 1);
    return false;
  }
  float turnedDegrees = 0.0f;
  const float targetDegrees = fabsf(angleDeg);
  const bool turnRight = angleDeg > 0.0f;
  const uint32_t startedAtMs = millis();
  uint32_t expectedMs = static_cast<uint32_t>(
      (spinMode ? rotateSpin90Ms : rotatePivot90Ms) *
      (targetDegrees / 90.0f) *
      (static_cast<float>(rotateCalibrationSpeed) /
       static_cast<float>(speed)));
  uint32_t timeoutMs = expectedMs * 3UL + 500UL;
  if (timeoutMs < 2000UL) timeoutMs = 2000UL;
  uint32_t lastProgressAtMs = startedAtMs;
  float lastProgressDegrees = 0.0f;

  for (;;) {
    const uint32_t nowMs = millis();
    if (static_cast<uint32_t>(nowMs - startedAtMs) >= timeoutMs) {
      motor(1, 1);
      return false;
    }
    if (!imu.update()) {
      motor(1, 1);
      return false;
    }

    const float currentYaw = imu.yaw();
    if (!isfinite(currentYaw)) {
      motor(1, 1);
      return false;
    }
    const float deltaYaw = wrapAngle180(currentYaw - lastYaw);
    lastYaw = currentYaw;
    if (fabsf(deltaYaw) <= 30.0f) {
      turnedDegrees += fabsf(deltaYaw);
    }

    const float remainingDegrees = targetDegrees - turnedDegrees;
    if (remainingDegrees <= ROTATE_TOLERANCE_DEG) {
      return completeRotation(spinMode, backwardPivot, turnRight, stopPull);
    }

    if (turnedDegrees - lastProgressDegrees >= 0.25f) {
      lastProgressDegrees = turnedDegrees;
      lastProgressAtMs = nowMs;
    } else if (static_cast<uint32_t>(nowMs - lastProgressAtMs) >= 700UL) {
      motor(1, 1);
      return false;
    }

    const float proportionalSpeed = remainingDegrees * ROTATE_GYRO_KP;
    const uint8_t minimumSpeed = speed < ROTATE_MIN_SPEED
        ? speed : ROTATE_MIN_SPEED;
    const uint8_t commandSpeed = static_cast<uint8_t>(constrain(
        static_cast<int>(proportionalSpeed), static_cast<int>(minimumSpeed),
        static_cast<int>(speed)));
    driveRotation(spinMode, backwardPivot, turnRight, commandSpeed);
    delay(ROTATE_LOOP_DELAY_MS);
  }
}

bool rotatePivotToHeadingWithGyro(bool backwardPivot, float targetYaw,
                                  uint8_t speed, uint8_t stopPull) {
  if (!imu.update()) {
    motor(1, 1);
    return false;
  }

  const float initialRawYaw = imu.yaw();
  if (!isfinite(initialRawYaw)) {
    motor(1, 1);
    return false;
  }

  targetYaw = wrapAngle180(targetYaw);
  const float initialYaw = wrapAngle180(initialRawYaw);
  const float initialError = wrapAngle180(targetYaw - initialYaw);
  if (fabsf(initialError) <= ROTATE_TOLERANCE_DEG) {
    motor(1, 1);
    return true;
  }

  const uint32_t startedAtMs = millis();
  uint32_t expectedMs = static_cast<uint32_t>(
      rotatePivot90Ms * (fabsf(initialError) / 90.0f) *
      (static_cast<float>(rotateCalibrationSpeed) /
       static_cast<float>(speed)));
  uint32_t timeoutMs = expectedMs * 3UL + 500UL;
  if (timeoutMs < 2000UL) timeoutMs = 2000UL;
  uint32_t lastProgressAtMs = startedAtMs;
  float lastProgressRemaining = fabsf(initialError);
  bool hasDriven = false;
  bool lastTurnRight = initialError > 0.0f;

  for (;;) {
    const uint32_t nowMs = millis();
    if (static_cast<uint32_t>(nowMs - startedAtMs) >= timeoutMs) {
      motor(1, 1);
      return false;
    }
    if (!imu.update()) {
      motor(1, 1);
      return false;
    }

    const float rawYaw = imu.yaw();
    if (!isfinite(rawYaw)) {
      motor(1, 1);
      return false;
    }
    const float currentYaw = wrapAngle180(rawYaw);
    const float error = wrapAngle180(targetYaw - currentYaw);
    const float remainingDegrees = fabsf(error);
    if (remainingDegrees <= ROTATE_TOLERANCE_DEG) {
      if (!hasDriven) {
        motor(1, 1);
        return true;
      }
      return completeRotation(false, backwardPivot, lastTurnRight, stopPull);
    }

    const bool turnRight = error > 0.0f;
    lastTurnRight = turnRight;
    if (lastProgressRemaining - remainingDegrees >= 0.25f) {
      lastProgressRemaining = remainingDegrees;
      lastProgressAtMs = nowMs;
    } else if (static_cast<uint32_t>(nowMs - lastProgressAtMs) >= 700UL) {
      motor(1, 1);
      return false;
    }

    const float proportionalSpeed = remainingDegrees * ROTATE_GYRO_KP;
    const uint8_t minimumSpeed = speed < ROTATE_MIN_SPEED
        ? speed : ROTATE_MIN_SPEED;
    const uint8_t commandSpeed = static_cast<uint8_t>(constrain(
        static_cast<int>(proportionalSpeed), static_cast<int>(minimumSpeed),
        static_cast<int>(speed)));
    driveRotation(false, backwardPivot, turnRight, commandSpeed);
    hasDriven = true;
    delay(ROTATE_LOOP_DELAY_MS);
  }
}

bool rotateWithFallback(bool spinMode, bool backwardPivot, float angleDeg,
                        uint8_t speed, uint8_t stopPull) {
  const float reference90Ms = spinMode
      ? static_cast<float>(rotateSpin90Ms)
      : static_cast<float>(rotatePivot90Ms);
  const uint32_t rotateTimeMs = static_cast<uint32_t>(
      reference90Ms * (fabsf(angleDeg) / 90.0f) *
      (static_cast<float>(rotateCalibrationSpeed) /
       static_cast<float>(speed)));
  const bool turnRight = angleDeg > 0.0f;
  const uint32_t startedAtMs = millis();
  while (static_cast<uint32_t>(millis() - startedAtMs) < rotateTimeMs) {
    driveRotation(spinMode, backwardPivot, turnRight, speed);
  }
  return completeRotation(spinMode, backwardPivot, turnRight, stopPull);
}

void stopAfterLineComplete(int sl, int sr, uint8_t stopPull,
                            bool reverseDirection) {
  if (stopPull == 0u) {
    return;
  }

  motorMotion(reverseDirection ? sl : -sl, reverseDirection ? sr : -sr);
  delay(stopPull);
  motor(1, 1);
}

bool turnModeIndex(TurnMode mode, uint8_t& index) {
  switch (mode) {
    case TurnMode::fl: index = 0; return true;
    case TurnMode::fr: index = 1; return true;
    case TurnMode::cl: index = 2; return true;
    case TurnMode::cr: index = 3; return true;
    case TurnMode::l: index = 4; return true;
    case TurnMode::r: index = 5; return true;
    default: return false;
  }
}

int turnHeadingDirection(TurnMode mode) {
  switch (mode) {
    case TurnMode::fl: case TurnMode::cl: case TurnMode::l: return -1;
    case TurnMode::fr: case TurnMode::cr: case TurnMode::r: return 1;
    default: return 0;
  }
}

void failTurnGyro(const __FlashStringHelper* reason) {
  motor(1, 1);
  Serial.print(F("turn_gyro failure: "));
  Serial.println(reason);
  tone(MyMINIConfig::PIN_BUZZER, 3500, 400);
}

bool turnModeUsesApproach(TurnMode mode) {
  return mode == TurnMode::fl || mode == TurnMode::fr ||
         mode == TurnMode::cl || mode == TurnMode::cr;
}

bool turnModeUsesTimedApproach(TurnMode mode) {
  return mode == TurnMode::fl || mode == TurnMode::fr;
}

int turnMotorCommand(uint8_t speed, int8_t ratio) {
  return static_cast<int>(static_cast<int16_t>(speed) * ratio / 100);
}

bool readTurnExit(LineExitSensor exitSensor, bool& white, bool& black) {
  white = false;
  black = false;
  const bool frontExit = exitSensor >= f0 && exitSensor <= f15;
  const bool rearExit = exitSensor >= b0 && exitSensor <= b15;

  if (frontExit || rearExit) {
    const uint8_t selectedIndex = frontExit
        ? static_cast<uint8_t>(exitSensor) - static_cast<uint8_t>(f0)
        : static_cast<uint8_t>(exitSensor) - static_cast<uint8_t>(b0);
    const uint8_t pairFirst = selectedIndex;
    const uint8_t pairSecond = inwardExitCompanion(selectedIndex);
    uint16_t firstValue = 0;
    uint16_t secondValue = 0;
    bool firstBlack = false;
    bool secondBlack = false;
    const bool valuesValid = frontExit
        ? normalizedFront(pairFirst, firstValue, &firstBlack) &&
              normalizedFront(pairSecond, secondValue, &secondBlack)
        : normalizedRear(pairFirst, firstValue, &firstBlack) &&
              normalizedRear(pairSecond, secondValue, &secondBlack);
    if (!valuesValid) return false;
    white = firstValue >= EXIT_WHITE_RELEASE &&
            secondValue >= EXIT_WHITE_RELEASE;
    black = firstBlack || secondBlack;
    return true;
  }

  if (exitSensor == cl || exitSensor == cr) {
    uint16_t centerValue = 0;
    bool centerBlack = false;
    const uint8_t channel = exitSensor == cl ? 0u : 1u;
    if (!normalizedCenter(channel, centerValue, &centerBlack)) return false;
    white = centerValue >= EXIT_WHITE_RELEASE;
    black = centerBlack;
    return true;
  }

  return false;
}

void resetTurnExit(TurnExitState& state) {
  state = TurnExitState{};
  state.lastFrameSequence = sensorArrays.frameSequence();
}

bool updateTurnExit(TurnExitState& state, bool white, bool black) {
  if (!state.initialized) {
    state.initialized = true;
    state.exitWasBlackAtStart = black;
  }

  if (!state.exitArmed) {
    state.consecutiveWhiteSamples = white
        ? static_cast<uint8_t>(state.consecutiveWhiteSamples + 1u)
        : 0;
    if (state.consecutiveWhiteSamples < 2u) return false;
    state.exitHasSeenWhite = true;
    state.exitArmed = true;
    return false;
  }

  return black;
}

bool updateTurnApproachExit(bool black, bool history[3], uint8_t& index) {
  history[index] = black;
  index = (index + 1u) % 3u;
  const uint8_t blackSamples = static_cast<uint8_t>(history[0]) +
      static_cast<uint8_t>(history[1]) + static_cast<uint8_t>(history[2]);
  return blackSamples >= 2u;
}

void stopAfterTurnComplete(int leftCommand, int rightCommand,
                           uint8_t stopPull) {
  if (stopPull == 0u) {
    motor(1, 1);
    return;
  }

  const int pull = constrain(static_cast<int>(stopPull), 0, 100);
  const int brakeLeft = leftCommand > 0 ? -pull
                      : leftCommand < 0 ? pull : 0;
  const int brakeRight = rightCommand > 0 ? -pull
                       : rightCommand < 0 ? pull : 0;
  motorMotion(brakeLeft, brakeRight);
  const uint32_t startedAtUs = micros();
  while (static_cast<uint32_t>(micros() - startedAtUs) <
         TURN_STOP_PULL_DURATION_US) {
    sensorArrays.update(micros());
  }
  motor(1, 1);
}

void applyTurnTouchBrake(bool backwardApproach) {
  if (turnTouchBrake == 0u) {
    motor(1, 1);
    return;
  }

  const int brakeCommand = backwardApproach
      ? static_cast<int>(turnTouchBrake) : -static_cast<int>(turnTouchBrake);
  motorMotion(brakeCommand, brakeCommand);
  const uint32_t startedAtUs = micros();
  while (static_cast<uint32_t>(micros() - startedAtUs) <
         static_cast<uint32_t>(turnTouchBrakeMs) * 1000UL) {
    sensorArrays.update(micros());
  }
  motor(1, 1);
}

LineExitSource exitSourceFor(LineExitSensor exitSensor) {
  switch (exitSensor) {
    case f0: case f1: case f2: case f3: case f4: case f5: case f6: case f7:
    case f8: case f9: case f10: case f11: case f12: case f13: case f14:
    case f15:
      return LineExitSource::front;
    case b0: case b1: case b2: case b3: case b4: case b5: case b6: case b7:
    case b8: case b9: case b10: case b11: case b12: case b13: case b14:
    case b15:
      return LineExitSource::rear;
    case cl:
    case cr:
      return LineExitSource::center;
    default:
      return LineExitSource::none;
  }
}

void clearLastFLineState(LastMotionType motionType) {
  lastMotionType = motionType;
  lastLineExitSource = LineExitSource::none;
  lastLineDirection = TravelDirection::none;
  lastLineEndedNormally = false;
  pendingLineHandoff = false;
  previousLineCommandContinuous = false;
}

void recordLineNormalExit(LastMotionType motionType, LineExitSource source,
                          TravelDirection direction, uint8_t stopPull) {
  lastMotionType = motionType;
  lastLineExitSource = source;
  lastLineDirection = direction;
  lastLineEndedNormally = source != LineExitSource::none;
  pendingLineHandoff = stopPull == 0u;
}

void runLine(int sl, int sr, float kp, bool distanceMode,
             float targetDistanceMm, LineExitSensor exitSensor,
             uint8_t stopPull, bool reverseDirection,
             TravelDirection completedDirection, bool continuousEntry) {
  sl = constrain(sl, -100, 100);
  sr = constrain(sr, -100, 100);
  if (reverseDirection ? !calibrationManager.rearValid()
                        : !calibrationManager.frontValid()) {
    stopLineWithError(reverseDirection, F("INVALID_SENSOR"), stopPull);
    return;
  }

  const uint32_t startedAtUs = micros();
  uint32_t previousPidAtUs = startedAtUs;
  float lastTrackedError = 0.0f;
  float lastPidError = 0.0f;
  bool trackingInitialized = false;
  bool lineWasLost = false;
  bool allWhiteActive = false;
  uint32_t allWhiteSinceUs = 0;
  int8_t lastConfirmedEdgeSide = 0;
  float previousRecoveryCandidate = 0.0f;
  uint8_t recoveryFrameCount = 0;
  float integral = 0.0f;
  float filteredDerivative = 0.0f;
  float estimatedDistanceMm = 0.0f;
  bool continuousPidInitialized = !continuousEntry;

  const bool isFrontExit = exitSensor >= f0 && exitSensor <= f15;
  const bool isRearExit = exitSensor >= b0 && exitSensor <= b15;
  const bool isCenterLeftExit = exitSensor == cl;
  const bool isCenterRightExit = exitSensor == cr;
  uint8_t selectedIndex = 0;
  uint8_t pairFirst = 0;
  uint8_t pairSecond = 0;
  bool rearExit = false;
  bool centerExit = false;
  uint8_t centerChannel = 0;
  bool armed = false;
  uint8_t consecutiveWhiteSamples = 0;
  bool pairBlackHistory[3] = {false, false, false};
  uint8_t pairHistoryIndex = 0;
  uint32_t lastTraceAtUs = startedAtUs;
  uint32_t lastPidTraceAtUs = startedAtUs;
  uint32_t lastErrorMonitorAtUs = startedAtUs;
  if (!distanceMode) {
    if (isFrontExit) {
      selectedIndex = static_cast<uint8_t>(exitSensor) -
                      static_cast<uint8_t>(f0);
    } else if (isRearExit) {
      if (!calibrationManager.rearValid()) {
        stopLineWithError(reverseDirection, F("INVALID_SENSOR"), stopPull);
        return;
      }
      rearExit = true;
      selectedIndex = static_cast<uint8_t>(exitSensor) -
                      static_cast<uint8_t>(b0);
    } else if (isCenterLeftExit) {
      centerExit = true;
      centerChannel = 0;
    } else if (isCenterRightExit) {
      centerExit = true;
      centerChannel = 1;
    } else {
      stopLineWithError(reverseDirection, F("INVALID_SENSOR"), stopPull);
      return;
    }

    if (reverseDirection &&
        ((isFrontExit && !calibrationManager.frontValid()) ||
          (isCenterLeftExit || isCenterRightExit) &&
              !calibrationManager.centerValid())) {
      stopLineWithError(reverseDirection, F("INVALID_SENSOR"), stopPull);
      return;
    }

    if (isFrontExit || isRearExit) {
      pairFirst = selectedIndex;
      pairSecond = inwardExitCompanion(selectedIndex);
    }
  }

  if (continuousEntry) {
    // Do not wait for a new sensor frame while retaining the previous line
    // command: immediately apply this command's own base speeds instead.
    motorMotion(reverseDirection ? -sl : sl, reverseDirection ? -sr : sr);
  }

  for (;;) {
    const uint32_t nowUs = micros();

    if (static_cast<uint32_t>(nowUs - startedAtUs) >=
        LINE_TIMEOUT_MS * 1000UL) {
      stopLineWithError(reverseDirection, F("TIMEOUT"), stopPull);
      return;
    }

    if (!sensorArrays.update(nowUs)) continue;

    uint16_t normalized[MyMINIConfig::SENSOR_COUNT] = {};
    uint16_t blackStrength[MyMINIConfig::SENSOR_COUNT] = {};
    bool anyConfirmedBlack = false;
    for (uint8_t channel = 0; channel < MyMINIConfig::SENSOR_COUNT; ++channel) {
      if (!normalizedTrackingSensor(reverseDirection, channel,
                                   normalized[channel])) {
        stopLineWithError(reverseDirection, F("INVALID_SENSOR"), stopPull);
        return;
      }
      // Use the calibrated black midpoint for both line position and the
      // all-white decision; weak reflections on white must not invent a line.
      if (normalized[channel] < 500u) {
        anyConfirmedBlack = true;
        blackStrength[channel] = static_cast<uint16_t>(1000u - normalized[channel]);
      }
    }
    if (anyConfirmedBlack) {
      allWhiteActive = false;
      const bool leftEdgeBlack = blackStrength[0] != 0u;
      const bool rightEdgeBlack =
          blackStrength[MyMINIConfig::SENSOR_COUNT - 1u] != 0u;
      // Only the latest black frame may establish the escape side. A line
      // that moves back inward, or spans both edges, cancels the old side.
      lastConfirmedEdgeSide = leftEdgeBlack == rightEdgeBlack
          ? 0 : (leftEdgeBlack ? -1 : 1);
    } else {
      if (!allWhiteActive) {
        allWhiteActive = true;
        allWhiteSinceUs = nowUs;
      } else if (static_cast<uint32_t>(nowUs - allWhiteSinceUs) >=
                 LINE_ALL_WHITE_TIMEOUT_US) {
        stopLineWithError(reverseDirection, F("LINE_LOST_TIMEOUT"), stopPull);
        return;
      }
    }

    if (!distanceMode) {
      uint16_t centerValue = 0;
      uint16_t rearFirstValue = 0;
      uint16_t rearSecondValue = 0;
      uint16_t pairFirstValue = 0;
      uint16_t pairSecondValue = 0;
      bool centerBlack = false;
      bool pairFirstBlack = false;
      bool pairSecondBlack = false;
      bool exitWhite = false;
      bool exitBlackSample = false;
      if (centerExit) {
        if (!normalizedCenter(centerChannel, centerValue, &centerBlack)) {
          stopLineWithError(reverseDirection, F("INVALID_SENSOR"), stopPull);
          return;
        }
        exitWhite = centerValue >= LINE_EXIT_WHITE_RELEASE;
        exitBlackSample = centerBlack;
      } else if (rearExit) {
        if (!normalizedRear(pairFirst, rearFirstValue, &pairFirstBlack) ||
            !normalizedRear(pairSecond, rearSecondValue, &pairSecondBlack)) {
          stopLineWithError(reverseDirection, F("INVALID_SENSOR"), stopPull);
          return;
        }
        pairFirstValue = rearFirstValue;
        pairSecondValue = rearSecondValue;
        exitWhite = pairFirstValue >= LINE_EXIT_WHITE_RELEASE &&
                    pairSecondValue >= LINE_EXIT_WHITE_RELEASE;
        exitBlackSample = pairFirstBlack || pairSecondBlack;
      } else {
        if (!normalizedFront(pairFirst, pairFirstValue, &pairFirstBlack) ||
            !normalizedFront(pairSecond, pairSecondValue, &pairSecondBlack)) {
          stopLineWithError(reverseDirection, F("INVALID_SENSOR"), stopPull);
          return;
        }
        exitWhite = pairFirstValue >= LINE_EXIT_WHITE_RELEASE &&
                    pairSecondValue >= LINE_EXIT_WHITE_RELEASE;
        exitBlackSample = pairFirstBlack || pairSecondBlack;
      }
      if (lineDebug() &&
          static_cast<uint32_t>(nowUs - lastTraceAtUs) >= 100000UL) {
        lastTraceAtUs = nowUs;
        Serial.print(reverseDirection ? F("b_line ") : F("f_line "));
        Serial.print(F("frame="));
        Serial.print(sensorArrays.frameSequence());
        Serial.print(F(" exit="));
        if (centerExit) {
          Serial.print(centerChannel == 0 ? F("cl") : F("cr"));
          Serial.print(F(" norm="));
          Serial.print(centerValue);
        } else {
          const uint16_t* raw = rearExit ? sensorArrays.rearRaw()
                                         : sensorArrays.frontRaw();
          Serial.print(rearExit ? 'b' : 'f');
          Serial.print(pairFirst);
          Serial.print('/');
          Serial.print(rearExit ? 'b' : 'f');
          Serial.print(pairSecond);
          Serial.print(F(" raw="));
          Serial.print(raw[pairFirst]);
          Serial.print('/');
          Serial.print(raw[pairSecond]);
          Serial.print(F(" norm="));
          Serial.print(pairFirstValue);
          Serial.print('/');
          Serial.print(pairSecondValue);
        }
        Serial.print(F(" black="));
        Serial.print(exitBlackSample);
        Serial.print(F(" white="));
        Serial.print(exitWhite);
        Serial.print(F(" armed="));
        Serial.println(armed);
      }
      if (!armed) {
        if (static_cast<uint32_t>(nowUs - startedAtUs) >= EXIT_ARM_DELAY_US) {
          consecutiveWhiteSamples = exitWhite
              ? static_cast<uint8_t>(consecutiveWhiteSamples + 1u)
              : 0;
          if (consecutiveWhiteSamples >= 2u) {
            armed = true;
            pairBlackHistory[0] = false;
            pairBlackHistory[1] = false;
            pairBlackHistory[2] = false;
            pairHistoryIndex = 0;
          }
        }
      } else {
        pairBlackHistory[pairHistoryIndex] = exitBlackSample;
        pairHistoryIndex = (pairHistoryIndex + 1u) % 3u;
        const uint8_t blackSampleCount =
            static_cast<uint8_t>(pairBlackHistory[0]) +
            static_cast<uint8_t>(pairBlackHistory[1]) +
            static_cast<uint8_t>(pairBlackHistory[2]);
        const bool selectedStopSensorIsBlack = blackSampleCount >= 2u;
        if (selectedStopSensorIsBlack) {
          if (lineDebug()) {
            if (centerExit) {
              printCenterExitDebug(exitSensor, centerValue, blackSampleCount);
            } else {
              printPairExitDebug(rearExit, selectedIndex, pairFirst,
                                 pairSecond, pairFirstValue, pairSecondValue,
                                 blackSampleCount);
            }
          }
          recordLineNormalExit(
              reverseDirection ? LastMotionType::bLine : LastMotionType::fLine,
              exitSourceFor(exitSensor), completedDirection, stopPull);
          previousLineCommandContinuous = stopPull == 0u;
          stopAfterLineComplete(sl, sr, stopPull, reverseDirection);
          printLineReason(reverseDirection, F("SENSOR_BLACK"), stopPull);
          return;
        }
      }
    }

    float dt = static_cast<float>(nowUs - previousPidAtUs) * 0.000001f;
    dt = constrain(dt, PID_MIN_DT_SECONDS, PID_MAX_DT_SECONDS);
    previousPidAtUs = nowUs;

    float measuredError = lastTrackedError;
    uint8_t outsideBlack = 0;
    bool lineFound = selectTrackedGroup(
        blackStrength, lastTrackedError, trackingInitialized, measuredError,
        outsideBlack);
    bool recoveredOutsideWindow = false;
    if (!lineFound && trackingInitialized && outsideBlack > 0u) {
      float recoveryCandidate = lastTrackedError;
      uint8_t recoveryGroupSize = 0;
      if (selectRecoveryGroup(blackStrength, lastTrackedError,
                              recoveryCandidate, recoveryGroupSize)) {
        recoveryFrameCount = recoveryFrameCount > 0u &&
                fabsf(recoveryCandidate - previousRecoveryCandidate) <= 12.0f
            ? static_cast<uint8_t>(recoveryFrameCount + 1u) : 1u;
        previousRecoveryCandidate = recoveryCandidate;
        recoveredOutsideWindow = recoveryFrameCount >=
            (recoveryGroupSize >= 2u ? 2u : 3u);
        if (recoveredOutsideWindow) {
          measuredError = recoveryCandidate;
          lineFound = true;
        }
      } else {
        recoveryFrameCount = 0;
      }
    } else {
      recoveryFrameCount = 0;
    }
    if (lineFound) {
      if (recoveredOutsideWindow) {
        // Move the reference into the newly detected group. A rate-limited
        // reference would remain outside the window and lose it next frame.
        lastTrackedError = constrain(measuredError, -50.0f, 50.0f);
        integral = 0.0f;
        recoveryFrameCount = 0;
      } else if (!trackingInitialized) {
        lastTrackedError = constrain(measuredError, -50.0f, 50.0f);
        trackingInitialized = true;
      } else {
        float maxErrorChange = MAX_ERROR_RATE_PER_SECOND * dt;
        maxErrorChange = constrain(maxErrorChange, 1.0f, 10.0f);
        float errorChange = measuredError - lastTrackedError;
        errorChange = constrain(errorChange, -maxErrorChange, maxErrorChange);
        lastTrackedError = constrain(lastTrackedError + errorChange,
                                     -50.0f, 50.0f);
      }
    }
    // Kp is the value supplied to f_line()/b_line(), before kpScale.
    // At low Kp, cross the white gap straight. At high Kp, search only if
    // the last confirmed black frame reached exactly one outer edge.
    const float error = !anyConfirmedBlack
        ? (kp > 0.5f && lastConfirmedEdgeSide != 0
            ? static_cast<float>(lastConfirmedEdgeSide) * 50.0f
            : 0.0f)
        : lastTrackedError;
    if (!anyConfirmedBlack && !lineWasLost) {
      integral = 0.0f;
      filteredDerivative = 0.0f;
    }
    if (lineFound && lineWasLost) {
      // The first fresh line frame is a new D baseline, not a one-frame jump
      // across the whole period when no line position was measurable.
      lastPidError = error;
      filteredDerivative = 0.0f;
    }
    lineWasLost = !lineFound;
    if (!continuousPidInitialized && lineFound) {
      // The new command keeps its own base speed, but starts its derivative
      // sample from the first measured error to avoid a handoff-only D kick.
      lastPidError = error;
      continuousPidInitialized = true;
    }

    const float elapsedMs = static_cast<float>(nowUs - startedAtUs) * 0.001f;
    const float accelFactor = continuousEntry
        ? 1.0f
        : (lineRampMs == 0u ? 1.0f
                           : smoothstep(elapsedMs / static_cast<float>(lineRampMs)));
    float decelFactor = 1.0f;
    if (distanceMode) {
      if (estimatedDistanceMm >= targetDistanceMm) {
        recordLineNormalExit(
            reverseDirection ? LastMotionType::bLine : LastMotionType::fLine,
            LineExitSource::distance, completedDirection, stopPull);
        stopAfterLineComplete(sl, sr, stopPull, reverseDirection);
        printLineReason(reverseDirection, F("DISTANCE"), stopPull);
        return;
      }
      const float remainingMm = max(0.0f, targetDistanceMm - estimatedDistanceMm);
      const float decelRangeMm = min(LINE_DECEL_DISTANCE_MM,
                                     targetDistanceMm * 0.40f);
      if (remainingMm < decelRangeMm) {
        const float x = remainingMm / decelRangeMm;
        decelFactor = LINE_MIN_DECEL_FACTOR +
            (1.0f - LINE_MIN_DECEL_FACTOR) * smoothstep(x);
      }
    }
    const float motionFactor = distanceMode
                                   ? min(accelFactor, decelFactor)
                                   : accelFactor;

    const float correction = calculateLinePidCorrection(
        kp * lineKpScale, error, dt, lineFound,
        reverseDirection ? -1 : 1, reverseDirection ? 0.0f : -100.0f,
        static_cast<float>(sl), static_cast<float>(sr), lastPidError,
        integral, filteredDerivative, lineKi, true, lineKd);
    const float effectiveSL = sl * motionFactor;
    const float effectiveSR = sr * motionFactor;
    const float effectiveCorrection = correction * motionFactor;
    int leftCommand = 0;
    int rightCommand = 0;
    if (reverseDirection) {
      leftCommand = static_cast<int>(constrain(
          lroundf(effectiveSL - effectiveCorrection), 0L, 100L));
      rightCommand = static_cast<int>(constrain(
          lroundf(effectiveSR + effectiveCorrection), 0L, 100L));
      motorMotion(-leftCommand, -rightCommand);
    } else {
      leftCommand = static_cast<int>(constrain(
          lroundf(effectiveSL + effectiveCorrection), -100L, 100L));
      rightCommand = static_cast<int>(constrain(
          lroundf(effectiveSR - effectiveCorrection), -100L, 100L));
      motorMotion(leftCommand, rightCommand);
    }

    // runLine() blocks its caller, so expose its live signed error here.
    // During all-white frames, found=0 and error shows the selected recovery.
    if (lineDebug() &&
        static_cast<uint32_t>(nowUs - lastPidTraceAtUs) >= 200000UL) {
      lastPidTraceAtUs = nowUs;
      Serial.print(reverseDirection ? F("b_line") : F("f_line"));
      Serial.print(F(" error="));
      Serial.print(error, 2);
      Serial.print(F(" found="));
      Serial.print(lineFound ? 1 : 0);
      Serial.print(F(" outside="));
      Serial.print(outsideBlack);
      Serial.print(F(" correction="));
      Serial.print(correction, 2);
      Serial.print(F(" motor="));
      Serial.print(reverseDirection ? -leftCommand : leftCommand);
      Serial.print(',');
      Serial.println(reverseDirection ? -rightCommand : rightCommand);
    }
    if (lineErrorMonitorEnabled &&
        static_cast<uint32_t>(nowUs - lastErrorMonitorAtUs) >= 20000UL) {
      lastErrorMonitorAtUs = nowUs;
      Serial.println(error, 2);
    }

    if (distanceMode) {
      const float forwardCommand = max(0.0f,
          (static_cast<float>(leftCommand) + rightCommand) * 0.5f);
      estimatedDistanceMm += LINE_MM_PER_SECOND_AT_100 *
          (forwardCommand / 100.0f) * dt;
      if (estimatedDistanceMm >= targetDistanceMm) {
        recordLineNormalExit(
            reverseDirection ? LastMotionType::bLine : LastMotionType::fLine,
            LineExitSource::distance, completedDirection, stopPull);
        stopAfterLineComplete(sl, sr, stopPull, reverseDirection);
        printLineReason(reverseDirection, F("DISTANCE"), stopPull);
        return;
      }
    }

    if (lineFound) lastPidError = error;
  }
}

// Absolute-heading variant of turn(). Keep its approach phases independent of
// the sensor-exit implementation above so the existing turn() path is intact.
void runTurnGyro(TurnMode mode, uint8_t speed, float targetDeg,
                 uint8_t stopPull) {
  uint8_t modeIndex = 0;
  const bool flFrMode = turnModeUsesTimedApproach(mode);
  const TravelDirection approachDirection = lastLineEndedNormally
      ? lastLineDirection : TravelDirection::forward;
  const LineExitSource previousExitSource = lastLineExitSource;
  const bool hasHandoff = pendingLineHandoff;
  const bool approachByDistance = previousExitSource == LineExitSource::distance;
  const uint8_t approachSpeed = approachByDistance
      ? turnDistanceExitSpeed : turnSensorExitSpeed;
  const bool rotateImmediately = flFrMode && lastLineEndedNormally &&
      ((approachDirection == TravelDirection::forward &&
        previousExitSource == LineExitSource::front) ||
       (approachDirection == TravelDirection::backward &&
        previousExitSource == LineExitSource::rear));
  clearLastFLineState(LastMotionType::turn);

  if (!turnModeIndex(mode, modeIndex) || speed == 0u || speed > 100u ||
      turnTimeoutMs == 0u || stopPull > 100u) {
    failTurnGyro(F("INVALID_MODE_SPEED_OR_TIMEOUT"));
    return;
  }
  if (!imuReady || !headingReferenceReady) {
    failTurnGyro(F("GYRO_OR_START_REFERENCE_NOT_READY"));
    return;
  }
  if (!isfinite(targetDeg) || targetDeg < -180.0f || targetDeg > 180.0f) {
    failTurnGyro(F("INVALID_TARGET_DEG"));
    return;
  }
  if (!imu.update() || !isfinite(imu.yaw())) {
    failTurnGyro(F("GYRO_READ_FAILED"));
    return;
  }

  const int gyroDirection = turnHeadingDirection(mode);
  TurnGyroTarget initialTarget;
  const TurnGyroResult initialResult = initialTarget.begin(
      imu.yaw(), targetDeg, gyroDirection);
  if (initialResult == TurnGyroResult::WrongDirection) {
    failTurnGyro(F("TARGET_OPPOSES_MODE"));
    return;
  }
  if (initialResult != TurnGyroResult::Running &&
      initialResult != TurnGyroResult::Reached) {
    failTurnGyro(F("INVALID_GYRO_TARGET"));
    return;
  }

  const TurnMotorRatio ratio = turnMotorRatios[modeIndex];
  if (ratio.left < -100 || ratio.left > 100 || ratio.right < -100 ||
      ratio.right > 100) {
    failTurnGyro(F("INVALID_MOTOR_RATIO"));
    return;
  }
  const int initialLeftCommand = turnMotorCommand(speed, ratio.left);
  const int initialRightCommand = turnMotorCommand(speed, ratio.right);
  if (gyroDirection * (initialLeftCommand - initialRightCommand) <= 0) {
    failTurnGyro(F("MOTOR_RATIO_OPPOSES_MODE"));
    return;
  }

  const bool approach = turnModeUsesApproach(mode) && !rotateImmediately;
  const bool centerTurnMode = mode == TurnMode::cl || mode == TurnMode::cr;
  const bool backwardApproach = approachDirection == TravelDirection::backward &&
      (flFrMode || centerTurnMode);
  const int approachMotorCommand = backwardApproach
      ? -static_cast<int>(approachSpeed) : static_cast<int>(approachSpeed);
  const bool approachTriggerCalibrationValid = centerTurnMode
      ? calibrationManager.centerValid()
      : (backwardApproach ? calibrationManager.rearValid()
                          : calibrationManager.frontValid());
  const bool approachTrackingCalibrationValid = backwardApproach
      ? calibrationManager.rearValid() : calibrationManager.frontValid();
  if (approach && (!approachTriggerCalibrationValid ||
                   !approachTrackingCalibrationValid)) {
    failTurnGyro(F("APPROACH_CALIBRATION_NOT_READY"));
    return;
  }

  enum class TurnPhase : uint8_t {
    SearchOuter, CrossOuter, Overshoot, ApproachExit, Rotate
  };
  const uint32_t startedAtMs = millis();
  uint32_t phaseStartedAtMs = startedAtMs;
  const uint32_t approachStartedAtUs = micros();
  uint32_t previousPidAtUs = approachStartedAtUs;
  uint32_t lastLineSeenAtUs = approachStartedAtUs;
  float lastTrackedError = 0.0f;
  float lastPidError = 0.0f;
  float integral = 0.0f;
  float filteredDerivative = 0.0f;
  bool trackingInitialized = false;
  bool approachLineWasLost = false;
  bool outerBlackHistory[3] = {false, false, false};
  uint8_t outerBlackHistoryIndex = 0;
  bool outerWhiteHistory[3] = {false, false, false};
  uint8_t outerWhiteHistoryIndex = 0;
  bool outerLineWasDetected = false;
  TurnGyroTarget gyroTarget;
  bool gyroArmed = false;

  TurnPhase phase = TurnPhase::Rotate;
  if (flFrMode && !rotateImmediately) phase = TurnPhase::SearchOuter;
  else if (approach) phase = TurnPhase::ApproachExit;

  for (;;) {
    const uint32_t nowMs = millis();
    const uint32_t nowUs = micros();
    const uint32_t timeoutStartedAtMs = flFrMode ? phaseStartedAtMs : startedAtMs;
    if (TurnGyroTarget::timedOut(nowMs, timeoutStartedAtMs, turnTimeoutMs)) {
      failTurnGyro(F("TIMEOUT"));
      return;
    }

    const bool sensorFrame = sensorArrays.update(nowUs);
    if (phase == TurnPhase::Rotate) {
      if (!imu.update() || !isfinite(imu.yaw())) {
        failTurnGyro(F("GYRO_READ_FAILED"));
        return;
      }
      const TurnGyroResult result = gyroArmed
          ? gyroTarget.sample(imu.yaw(), TURN_GYRO_BRAKE_TOLERANCE_DEG)
          : gyroTarget.beginWithBrakeLead(
                imu.yaw(), targetDeg, gyroDirection,
                TURN_GYRO_BRAKE_LEAD_DEG, TURN_GYRO_BRAKE_TOLERANCE_DEG);
      gyroArmed = true;
      if (result == TurnGyroResult::WrongDirection) {
        failTurnGyro(F("TARGET_OR_YAW_OPPOSES_MODE"));
        return;
      }
      if (result == TurnGyroResult::InvalidTarget ||
          result == TurnGyroResult::InvalidSample) {
        failTurnGyro(F("INVALID_GYRO_SAMPLE"));
        return;
      }
      if (result == TurnGyroResult::Reached) {
        motor(1, 1);
        return;
      }
      motorMotion(initialLeftCommand, initialRightCommand);
      delay(ROTATE_LOOP_DELAY_MS);
      continue;
    }
    if (!sensorFrame) continue;
    if (!imu.update() || !isfinite(imu.yaw())) {
      failTurnGyro(F("GYRO_READ_FAILED"));
      return;
    }

    if (phase == TurnPhase::CrossOuter) {
      uint16_t f0Value = 0;
      uint16_t f1Value = 0;
      uint16_t f14Value = 0;
      uint16_t f15Value = 0;
      if (!normalizedTrackingSensor(backwardApproach, 0, f0Value) ||
          !normalizedTrackingSensor(backwardApproach, 1, f1Value) ||
          !normalizedTrackingSensor(backwardApproach, 14, f14Value) ||
          !normalizedTrackingSensor(backwardApproach, 15, f15Value)) {
        failTurnGyro(F("APPROACH_SENSOR_READ_FAILED"));
        return;
      }
      const bool outerEdgesWhite =
          f0Value >= EXIT_WHITE_RELEASE && f1Value >= EXIT_WHITE_RELEASE &&
          f14Value >= EXIT_WHITE_RELEASE && f15Value >= EXIT_WHITE_RELEASE;
      if (outerLineWasDetected && updateTurnApproachExit(
              outerEdgesWhite, outerWhiteHistory, outerWhiteHistoryIndex)) {
        phase = TurnPhase::Overshoot;
        phaseStartedAtMs = nowMs;
      }
      motorMotion(approachMotorCommand, approachMotorCommand);
      continue;
    }

    if (phase == TurnPhase::Overshoot) {
      motorMotion(approachMotorCommand, approachMotorCommand);
      if (static_cast<uint32_t>(nowMs - phaseStartedAtMs) < turnOvershootMs) {
        continue;
      }
      applyTurnTouchBrake(backwardApproach);
      phase = TurnPhase::Rotate;
      phaseStartedAtMs = nowMs;
      continue;
    }

    if (phase == TurnPhase::ApproachExit && centerTurnMode) {
      const uint8_t centerChannel = mode == TurnMode::cl ? 0u : 1u;
      uint16_t centerValue = 0;
      bool centerBlack = false;
      if (!normalizedCenter(centerChannel, centerValue, &centerBlack)) {
        failTurnGyro(F("CENTER_SENSOR_READ_FAILED"));
        return;
      }
      if (centerBlack) {
        applyTurnTouchBrake(backwardApproach);
        phase = TurnPhase::Rotate;
        phaseStartedAtMs = nowMs;
        continue;
      }
    }

    uint16_t normalized[MyMINIConfig::SENSOR_COUNT] = {};
    bool blackByChannel[MyMINIConfig::SENSOR_COUNT] = {};
    uint16_t blackStrength[MyMINIConfig::SENSOR_COUNT] = {};
    for (uint8_t channel = 0; channel < MyMINIConfig::SENSOR_COUNT; ++channel) {
      if (!normalizedTrackingSensor(backwardApproach, channel,
                                    normalized[channel], &blackByChannel[channel])) {
        failTurnGyro(F("APPROACH_SENSOR_READ_FAILED"));
        return;
      }
      int32_t strength = 1000 - normalized[channel];
      if (strength < LINE_BLACK_NOISE_THRESHOLD) strength = 0;
      blackStrength[channel] = static_cast<uint16_t>(strength);
    }

    float dt = static_cast<float>(nowUs - previousPidAtUs) * 0.000001f;
    dt = constrain(dt, PID_MIN_DT_SECONDS, PID_MAX_DT_SECONDS);
    previousPidAtUs = nowUs;

    float measuredError = lastTrackedError;
    uint8_t outsideBlack = 0;
    const bool lineFound = selectTrackedGroup(
        blackStrength, lastTrackedError, trackingInitialized, measuredError,
        outsideBlack);
    if (lineFound) {
      if (!trackingInitialized) {
        lastTrackedError = constrain(measuredError, -50.0f, 50.0f);
        trackingInitialized = true;
      } else {
        float maxErrorChange = MAX_ERROR_RATE_PER_SECOND * dt;
        maxErrorChange = constrain(maxErrorChange, 1.0f, 10.0f);
        float errorChange = measuredError - lastTrackedError;
        errorChange = constrain(errorChange, -maxErrorChange, maxErrorChange);
        lastTrackedError = constrain(lastTrackedError + errorChange,
                                     -50.0f, 50.0f);
      }
      lastLineSeenAtUs = nowUs;
    } else {
      if (approachByDistance) {
        approachLineWasLost = true;
        motorMotion(approachMotorCommand, approachMotorCommand);
        continue;
      }
      if (static_cast<uint32_t>(nowUs - lastLineSeenAtUs) >
          LINE_LOST_TIMEOUT_US) {
        failTurnGyro(F("APPROACH_LINE_LOST"));
        return;
      }
    }

    const float error = lastTrackedError;
    const float approachSpeedFloat = static_cast<float>(approachSpeed);
    if (approachLineWasLost) {
      lastPidError = error;
      filteredDerivative = 0.0f;
      approachLineWasLost = false;
    }
    const float correction = calculateLinePidCorrection(
        turnApproachKp, error, dt, lineFound, backwardApproach ? -1 : 1,
        backwardApproach ? 0.0f : -100.0f, approachSpeedFloat,
        approachSpeedFloat, lastPidError, integral, filteredDerivative);
    const float elapsedMs = static_cast<float>(nowUs - approachStartedAtUs) *
        0.001f;
    const float motionFactor = hasHandoff
        ? 1.0f
        : smoothstep(elapsedMs / static_cast<float>(LINE_ACCEL_TIME_MS));
    const int leftCommand = static_cast<int>(constrain(lroundf(
        (backwardApproach ? approachSpeedFloat - correction
                          : approachSpeedFloat + correction) * motionFactor),
        backwardApproach ? 0L : -100L, 100L));
    const int rightCommand = static_cast<int>(constrain(lroundf(
        (backwardApproach ? approachSpeedFloat + correction
                          : approachSpeedFloat - correction) * motionFactor),
        backwardApproach ? 0L : -100L, 100L));
    motorMotion(backwardApproach ? -leftCommand : leftCommand,
                backwardApproach ? -rightCommand : rightCommand);

    if (phase == TurnPhase::SearchOuter) {
      const bool outerEdgeBlack =
          blackByChannel[0] || blackByChannel[1] ||
          blackByChannel[14] || blackByChannel[15];
      if (updateTurnApproachExit(outerEdgeBlack, outerBlackHistory,
                                 outerBlackHistoryIndex)) {
        outerLineWasDetected = true;
        phase = TurnPhase::CrossOuter;
        phaseStartedAtMs = nowMs;
        outerWhiteHistory[0] = false;
        outerWhiteHistory[1] = false;
        outerWhiteHistory[2] = false;
        outerWhiteHistoryIndex = 0;
        motorMotion(approachMotorCommand, approachMotorCommand);
      }
    }
    if (lineFound) lastPidError = error;
  }
}

} // namespace

void set_line_diagnostics(bool enabled) {
  lineDiagnosticsEnabled = enabled;
  if (enabled) lineErrorMonitorEnabled = false;
}

void set_line_error_monitor(bool enabled) {
  lineErrorMonitorEnabled = enabled;
  if (enabled) lineDiagnosticsEnabled = false;
}

bool set_line_ramp_ms(uint16_t rampMs) {
  if (rampMs > LINE_TIMEOUT_MS) return false;
  lineRampMs = rampMs;
  return true;
}

bool set_line_pid_tuning(float kd) {
  if (!isfinite(kd) || kd < 0.0f || kd > 10.0f) return false;
  lineKpScale = 1.0f;
  lineKi = LINE_KI;
  lineKd = kd;
  return true;
}

bool set_line_pid_tuning(float kpScale, float ki) {
  if (!isfinite(kpScale) || !isfinite(ki) || kpScale < 0.0f ||
      kpScale > 10.0f || ki < 0.0f || ki > 10.0f) {
    return false;
  }
  lineKpScale = kpScale;
  lineKi = ki;
  return true;
}

// -----------------------------------------------------------------------------
// Public f_line API
// -----------------------------------------------------------------------------

void f_line(int sl, int sr, float kp, LineExitSensor exitSensor,
            uint8_t stopPull) {
  const bool continuousEntry = previousLineCommandContinuous;
  clearLastFLineState(LastMotionType::fLine);
  runLine(sl, sr, kp, false, 0.0f, exitSensor, stopPull, false,
          TravelDirection::forward, continuousEntry);
}

void f_line(int sl, int sr, float kp, float distanceCm,
            uint8_t stopPull) {
  const bool continuousEntry = previousLineCommandContinuous;
  clearLastFLineState(LastMotionType::fLine);
  if (distanceCm <= 0.0f) {
    stopLineWithError(false, F("INVALID_ARGUMENT"), stopPull);
    return;
  }
  runLine(sl, sr, kp, true, distanceCm * 10.0f, f0, stopPull, false,
          TravelDirection::forward, continuousEntry);
}

// -----------------------------------------------------------------------------
// Public b_line API
// -----------------------------------------------------------------------------

void b_line(int sl, int sr, float kp, LineExitSensor exitSensor,
            uint8_t stopPull) {
  const bool continuousEntry = previousLineCommandContinuous;
  clearLastFLineState(LastMotionType::bLine);
  if (sl < 0 || sr < 0 || sl > 100 || sr > 100) {
    stopLineWithError(true, F("INVALID_ARGUMENT"), stopPull);
    return;
  }
  if (!continuousEntry) motor(1, 1);
  runLine(sl, sr, kp, false, 0.0f, exitSensor, stopPull, true,
          TravelDirection::backward, continuousEntry);
}

void b_line(int sl, int sr, float kp, float distanceCm,
            uint8_t stopPull) {
  const bool continuousEntry = previousLineCommandContinuous;
  clearLastFLineState(LastMotionType::bLine);
  if (sl < 0 || sr < 0 || sl > 100 || sr > 100 || distanceCm <= 0.0f) {
    stopLineWithError(true, F("INVALID_ARGUMENT"), stopPull);
    return;
  }
  if (!continuousEntry) motor(1, 1);
  runLine(sl, sr, kp, true, distanceCm * 10.0f, f0, stopPull, true,
          TravelDirection::backward, continuousEntry);
}

// -----------------------------------------------------------------------------
// Public fw_gyro API
// -----------------------------------------------------------------------------

void fw_gyro(float targetAngle, uint8_t speed, float kp, float distanceCm,
             uint8_t stopPull) {
  const auto fail = []() {
    clearLastFLineState(LastMotionType::none);
    motor(1, 1);
  };

  if (!imuReady) {
    motor(1, 1);
    return;
  }

  if (speed == 0u || speed > 100u || !(kp >= 0.0f) ||
      !(distanceCm > 0.0f) || stopPull > 100u || !isfinite(targetAngle)) {
    fail();
    return;
  }

  targetAngle = wrapAngle180(targetAngle);
  if (!imu.update()) {
    fail();
    return;
  }

  if (!isfinite(imu.yaw())) {
    fail();
    return;
  }

  clearLastFLineState(LastMotionType::fLine);
  const float targetDistanceMm = distanceCm * 10.0f;
  const uint32_t startedAtUs = micros();
  uint32_t previousAtUs = startedAtUs;
  float estimatedDistanceMm = 0.0f;

  for (;;) {
    const uint32_t nowUs = micros();
    if (static_cast<uint32_t>(nowUs - startedAtUs) >=
        LINE_TIMEOUT_MS * 1000UL) {
      fail();
      return;
    }

    if (!imu.update()) {
      fail();
      return;
    }

    const float rawYaw = imu.yaw();
    if (!isfinite(rawYaw)) {
      fail();
      return;
    }
    const float currentYaw = wrapAngle180(rawYaw);
    const float error = wrapAngle180(targetAngle - currentYaw);

    float dt = static_cast<float>(nowUs - previousAtUs) * 0.000001f;
    dt = constrain(dt, PID_MIN_DT_SECONDS, PID_MAX_DT_SECONDS);
    previousAtUs = nowUs;

    if (estimatedDistanceMm >= targetDistanceMm) {
      recordLineNormalExit(LastMotionType::fLine, LineExitSource::distance,
                           TravelDirection::forward, stopPull);
      if (stopPull == 0u) return;
      motorMotion(-static_cast<int>(stopPull), -static_cast<int>(stopPull));
      delay(35);
      motor(1, 1);
      return;
    }

    const float elapsedMs = static_cast<float>(nowUs - startedAtUs) * 0.001f;
    const float accelFactor = smoothstep(elapsedMs /
        static_cast<float>(LINE_ACCEL_TIME_MS));
    const float remainingMm = max(0.0f, targetDistanceMm - estimatedDistanceMm);
    const float decelRangeMm = min(LINE_DECEL_DISTANCE_MM,
                                   targetDistanceMm * 0.40f);
    float decelFactor = 1.0f;
    if (remainingMm < decelRangeMm) {
      const float x = remainingMm / decelRangeMm;
      decelFactor = LINE_MIN_DECEL_FACTOR +
          (1.0f - LINE_MIN_DECEL_FACTOR) * smoothstep(x);
    }

    const float currentBaseSpeed = static_cast<float>(speed) *
        min(accelFactor, decelFactor);
    const float correction = FW_GYRO_CORRECTION_SIGN * kp * error;
    const float leftCommand = constrain(currentBaseSpeed - correction,
                                        0.0f, 100.0f);
    const float rightCommand = constrain(currentBaseSpeed + correction,
                                         0.0f, 100.0f);
    const int leftOutput = static_cast<int>(leftCommand);
    const int rightOutput = static_cast<int>(rightCommand);
    motorMotion(leftOutput, rightOutput);

    const float forwardCommand =
        (static_cast<float>(leftOutput) + rightOutput) * 0.5f;
    estimatedDistanceMm += LINE_MM_PER_SECOND_AT_100 *
        (forwardCommand / 100.0f) * dt;
    if (estimatedDistanceMm >= targetDistanceMm) {
      recordLineNormalExit(LastMotionType::fLine, LineExitSource::distance,
                           TravelDirection::forward, stopPull);
      if (stopPull == 0u) return;
      motorMotion(-static_cast<int>(stopPull), -static_cast<int>(stopPull));
      delay(35);
      motor(1, 1);
      return;
    }
  }
}

// -----------------------------------------------------------------------------
// Public bw_gyro API
// -----------------------------------------------------------------------------

void bw_gyro(float targetAngle, uint8_t speed, float kp, float distanceCm,
             uint8_t stopPull) {
  const auto fail = []() {
    clearLastFLineState(LastMotionType::none);
    motor(1, 1);
  };

  if (!imuReady) {
    motor(1, 1);
    return;
  }

  if (speed == 0u || speed > 100u || !(kp >= 0.0f) ||
      !(distanceCm > 0.0f) || stopPull > 100u || !isfinite(targetAngle)) {
    fail();
    return;
  }

  targetAngle = wrapAngle180(targetAngle);
  if (!imu.update()) {
    fail();
    return;
  }

  if (!isfinite(imu.yaw())) {
    fail();
    return;
  }

  clearLastFLineState(LastMotionType::bLine);
  const float targetDistanceMm = distanceCm * 10.0f;
  const uint32_t startedAtUs = micros();
  uint32_t previousAtUs = startedAtUs;
  float estimatedDistanceMm = 0.0f;

  for (;;) {
    const uint32_t nowUs = micros();
    if (static_cast<uint32_t>(nowUs - startedAtUs) >=
        LINE_TIMEOUT_MS * 1000UL) {
      fail();
      return;
    }

    if (!imu.update()) {
      fail();
      return;
    }

    const float rawYaw = imu.yaw();
    if (!isfinite(rawYaw)) {
      fail();
      return;
    }
    const float currentYaw = wrapAngle180(rawYaw);
    const float error = wrapAngle180(targetAngle - currentYaw);

    float dt = static_cast<float>(nowUs - previousAtUs) * 0.000001f;
    dt = constrain(dt, PID_MIN_DT_SECONDS, PID_MAX_DT_SECONDS);
    previousAtUs = nowUs;

    if (estimatedDistanceMm >= targetDistanceMm) {
      recordLineNormalExit(LastMotionType::bLine, LineExitSource::distance,
                           TravelDirection::backward, stopPull);
      if (stopPull == 0u) return;
      motorMotion(static_cast<int>(stopPull), static_cast<int>(stopPull));
      delay(35);
      motor(1, 1);
      return;
    }

    const float elapsedMs = static_cast<float>(nowUs - startedAtUs) * 0.001f;
    const float accelFactor = smoothstep(elapsedMs /
        static_cast<float>(LINE_ACCEL_TIME_MS));
    const float remainingMm = max(0.0f, targetDistanceMm - estimatedDistanceMm);
    const float decelRangeMm = min(LINE_DECEL_DISTANCE_MM,
                                   targetDistanceMm * 0.40f);
    float decelFactor = 1.0f;
    if (remainingMm < decelRangeMm) {
      const float x = remainingMm / decelRangeMm;
      decelFactor = LINE_MIN_DECEL_FACTOR +
          (1.0f - LINE_MIN_DECEL_FACTOR) * smoothstep(x);
    }

    const float currentBaseSpeed = -static_cast<float>(speed) *
        min(accelFactor, decelFactor);
    const float correction = FW_GYRO_CORRECTION_SIGN * kp * error;
    const float leftCommand = constrain(currentBaseSpeed - correction,
                                        -100.0f, 0.0f);
    const float rightCommand = constrain(currentBaseSpeed + correction,
                                         -100.0f, 0.0f);
    const int leftOutput = static_cast<int>(leftCommand);
    const int rightOutput = static_cast<int>(rightCommand);
    motorMotion(leftOutput, rightOutput);

    const float backwardCommand =
        -(static_cast<float>(leftOutput) + rightOutput) * 0.5f;
    estimatedDistanceMm += LINE_MM_PER_SECOND_AT_100 *
        (backwardCommand / 100.0f) * dt;
    if (estimatedDistanceMm >= targetDistanceMm) {
      recordLineNormalExit(LastMotionType::bLine, LineExitSource::distance,
                           TravelDirection::backward, stopPull);
      if (stopPull == 0u) return;
      motorMotion(static_cast<int>(stopPull), static_cast<int>(stopPull));
      delay(35);
      motor(1, 1);
      return;
    }
  }
}

// -----------------------------------------------------------------------------
// Public rotate API
// -----------------------------------------------------------------------------

bool rotate_spin(float angleDeg, uint8_t speed, uint8_t stopPull) {
  if (!validRotateArguments(angleDeg, speed, stopPull)) {
    motor(1, 1);
    return false;
  }
  return imuReady
      ? rotateWithGyro(true, false, angleDeg, speed, stopPull)
      : rotateWithFallback(true, false, angleDeg, speed, stopPull);
}

bool rotateFW_pivot(float angleDeg, uint8_t speed, uint8_t stopPull) {
  if (!validRotateArguments(angleDeg, speed, stopPull, true)) {
    motor(1, 1);
    return false;
  }
  if (imuReady) {
    return rotatePivotToHeadingWithGyro(false, angleDeg, speed, stopPull);
  }
  if (angleDeg == 0.0f) {
    motor(1, 1);
    return true;
  }
  return rotateWithFallback(false, false, angleDeg, speed, stopPull);
}

bool rotateBW_pivot(float angleDeg, uint8_t speed, uint8_t stopPull) {
  if (!validRotateArguments(angleDeg, speed, stopPull, true)) {
    motor(1, 1);
    return false;
  }
  if (imuReady) {
    return rotatePivotToHeadingWithGyro(true, angleDeg, speed, stopPull);
  }
  if (angleDeg == 0.0f) {
    motor(1, 1);
    return true;
  }
  return rotateWithFallback(false, true, angleDeg, speed, stopPull);
}

bool set_rotate_fallback(uint16_t spin90Ms, uint16_t pivot90Ms,
                         uint8_t calibrationSpeed) {
  if (spin90Ms == 0u || pivot90Ms == 0u || calibrationSpeed == 0u ||
      calibrationSpeed > 100u) {
    return false;
  }
  rotateSpin90Ms = spin90Ms;
  rotatePivot90Ms = pivot90Ms;
  rotateCalibrationSpeed = calibrationSpeed;
  return true;
}

// -----------------------------------------------------------------------------
// Public configuration API
// -----------------------------------------------------------------------------

bool set_turn_motor(TurnMode mode, int8_t leftRatio, int8_t rightRatio) {
  uint8_t index = 0;
  if (!turnModeIndex(mode, index) || leftRatio < -100 || leftRatio > 100 ||
      rightRatio < -100 || rightRatio > 100) {
    return false;
  }
  turnMotorRatios[index] = {leftRatio, rightRatio};
  return true;
}

void set_turn_overshoot(uint16_t overshootMs) {
  turnOvershootMs = overshootMs;
}

void set_turn_timeout(uint16_t timeoutMs) {
  turnTimeoutMs = timeoutMs;
}

void set_turn_approach_kp(float kp) {
  turnApproachKp = kp;
}

bool set_turn_approach(uint8_t forwardSpeed, uint8_t touchBrake) {
  return set_turn_approach(forwardSpeed, forwardSpeed, touchBrake);
}

bool set_turn_approach(uint8_t sensorExitSpeed, uint8_t distanceExitSpeed,
                        uint8_t touchBrake) {
  if (sensorExitSpeed == 0u || sensorExitSpeed > 100u ||
      distanceExitSpeed == 0u || distanceExitSpeed > 100u ||
      touchBrake > 100u) {
    return false;
  }
  turnSensorExitSpeed = sensorExitSpeed;
  turnDistanceExitSpeed = distanceExitSpeed;
  turnTouchBrake = touchBrake;
  return true;
}

void set_turn_touch_brake_ms(uint16_t durationMs) {
  turnTouchBrakeMs = durationMs;
}

bool set_turn_line_search(uint8_t searchSpeed, uint16_t fastTurnMs) {
  if (searchSpeed == 0u || searchSpeed > 100u) return false;
  turnLineSearchSpeed = searchSpeed;
  turnFastTimeMs = fastTurnMs;
  return true;
}

// -----------------------------------------------------------------------------
// Public turn API
// -----------------------------------------------------------------------------

void turn_gyro(TurnMode mode, uint8_t speed, float targetDeg,
               uint8_t stopPull) {
  runTurnGyro(mode, speed, targetDeg, stopPull);
}

void turn(TurnMode mode, uint8_t speed, LineExitSensor exitSensor,
          uint8_t stopPull) {
  uint8_t modeIndex = 0;
  const bool flFrMode = turnModeUsesTimedApproach(mode);
  const TravelDirection approachDirection = lastLineEndedNormally
      ? lastLineDirection : TravelDirection::forward;
  const LineExitSource previousExitSource = lastLineExitSource;
  const bool hasHandoff = pendingLineHandoff;
  const bool approachByDistance =
      previousExitSource == LineExitSource::distance;
  const uint8_t approachSpeed =
      previousExitSource == LineExitSource::distance
          ? turnDistanceExitSpeed : turnSensorExitSpeed;
  const bool rotateImmediately = flFrMode &&
      lastLineEndedNormally &&
      ((approachDirection == TravelDirection::forward &&
        previousExitSource == LineExitSource::front) ||
       (approachDirection == TravelDirection::backward &&
        previousExitSource == LineExitSource::rear));
  clearLastFLineState(LastMotionType::turn);

  if (!turnModeIndex(mode, modeIndex) || speed == 0u || speed > 100u ||
      turnTimeoutMs == 0u) {
    motor(1, 1);
    return;
  }

  const TurnMotorRatio ratio = turnMotorRatios[modeIndex];
  if (ratio.left < -100 || ratio.left > 100 || ratio.right < -100 ||
      ratio.right > 100) {
    motor(1, 1);
    return;
  }

  const bool approach = turnModeUsesApproach(mode) && !rotateImmediately;
  const bool centerTurnMode = mode == TurnMode::cl || mode == TurnMode::cr;
  const bool backwardApproach = approachDirection == TravelDirection::backward &&
      (flFrMode || centerTurnMode);
  const int approachMotorCommand = backwardApproach
      ? -static_cast<int>(approachSpeed)
      : static_cast<int>(approachSpeed);
  const bool approachTriggerCalibrationValid = centerTurnMode
      ? calibrationManager.centerValid()
      : (backwardApproach ? calibrationManager.rearValid()
                          : calibrationManager.frontValid());
  const bool approachTrackingCalibrationValid = backwardApproach
      ? calibrationManager.rearValid() : calibrationManager.frontValid();
  if (approach && (!approachTriggerCalibrationValid ||
                   !approachTrackingCalibrationValid)) {
    motor(1, 1);
    return;
  }
  if (centerTurnMode) {
    const LineExitSource exitSource = exitSourceFor(exitSensor);
    const bool exitCalibrationValid =
        (exitSource == LineExitSource::front && calibrationManager.frontValid()) ||
        (exitSource == LineExitSource::rear && calibrationManager.rearValid()) ||
        (exitSource == LineExitSource::center && calibrationManager.centerValid());
    if (!calibrationManager.centerValid() || !exitCalibrationValid) {
      motor(1, 1);
      return;
    }
  }

  enum class TurnPhase : uint8_t {
    SearchOuter,
    CrossOuter,
    Overshoot,
    ApproachExit,
    Rotate
  };

  const uint32_t startedAtMs = millis();
  uint32_t phaseStartedAtMs = startedAtMs;
  uint32_t approachStartedAtUs = micros();
  uint32_t previousPidAtUs = approachStartedAtUs;
  uint32_t lastLineSeenAtUs = approachStartedAtUs;
  float lastTrackedError = 0.0f;
  float lastPidError = 0.0f;
  float integral = 0.0f;
  float filteredDerivative = 0.0f;
  bool trackingInitialized = false;
  bool approachLineWasLost = false;
  TurnExitState turnExitState;
  bool outerBlackHistory[3] = {false, false, false};
  uint8_t outerBlackHistoryIndex = 0;
  bool outerWhiteHistory[3] = {false, false, false};
  uint8_t outerWhiteHistoryIndex = 0;
  bool outerLineWasDetected = false;

  TurnPhase phase = TurnPhase::Rotate;
  if (flFrMode && !rotateImmediately) {
    phase = TurnPhase::SearchOuter;
  } else if (approach) {
    phase = TurnPhase::ApproachExit;
  }

  if (phase == TurnPhase::Rotate) {
    resetTurnExit(turnExitState);
  }

  for (;;) {
    const uint32_t nowMs = millis();
    const uint32_t nowUs = micros();
    const uint32_t timeoutStartedAtMs = flFrMode
        ? phaseStartedAtMs : startedAtMs;
    if (static_cast<uint32_t>(nowMs - timeoutStartedAtMs) >=
        turnTimeoutMs) {
      motor(1, 1);
      return;
    }

    if (!sensorArrays.update(nowUs)) continue;

    if (phase == TurnPhase::Rotate) {
      const uint32_t frameSequence = sensorArrays.frameSequence();
      if (frameSequence == turnExitState.lastFrameSequence) continue;
      turnExitState.lastFrameSequence = frameSequence;
      const uint32_t elapsedTurnMs = nowMs - phaseStartedAtMs;
      const uint8_t activeTurnSpeed = elapsedTurnMs < turnFastTimeMs
          ? speed : min(speed, turnLineSearchSpeed);
      const int turnLeftCommand = turnMotorCommand(activeTurnSpeed, ratio.left);
      const int turnRightCommand = turnMotorCommand(activeTurnSpeed, ratio.right);
      bool exitWhite = false;
      bool exitBlack = false;
      if (!readTurnExit(exitSensor, exitWhite, exitBlack)) {
        motor(1, 1);
        return;
      }
      if (updateTurnExit(turnExitState, exitWhite, exitBlack)) {
        stopAfterTurnComplete(turnLeftCommand, turnRightCommand, stopPull);
        return;
      }
      motorMotion(turnLeftCommand, turnRightCommand);
      continue;
    }

    if (phase == TurnPhase::CrossOuter) {
      uint16_t f0Value = 0;
      uint16_t f1Value = 0;
      uint16_t f14Value = 0;
      uint16_t f15Value = 0;
      if (!normalizedTrackingSensor(backwardApproach, 0, f0Value) ||
          !normalizedTrackingSensor(backwardApproach, 1, f1Value) ||
          !normalizedTrackingSensor(backwardApproach, 14, f14Value) ||
          !normalizedTrackingSensor(backwardApproach, 15, f15Value)) {
        motor(1, 1);
        return;
      }
      const bool outerEdgesWhite =
          f0Value >= EXIT_WHITE_RELEASE && f1Value >= EXIT_WHITE_RELEASE &&
          f14Value >= EXIT_WHITE_RELEASE && f15Value >= EXIT_WHITE_RELEASE;
      if (outerLineWasDetected && updateTurnApproachExit(
              outerEdgesWhite, outerWhiteHistory, outerWhiteHistoryIndex)) {
        phase = TurnPhase::Overshoot;
        phaseStartedAtMs = nowMs;
      }
      motorMotion(approachMotorCommand, approachMotorCommand);
      continue;
    }

    if (phase == TurnPhase::Overshoot) {
      motorMotion(approachMotorCommand, approachMotorCommand);
      if (static_cast<uint32_t>(nowMs - phaseStartedAtMs) <
          turnOvershootMs) {
        continue;
      }
      applyTurnTouchBrake(backwardApproach);
      phase = TurnPhase::Rotate;
      phaseStartedAtMs = nowMs;
      resetTurnExit(turnExitState);
      motorMotion(turnMotorCommand(speed, ratio.left),
                  turnMotorCommand(speed, ratio.right));
      continue;
    }

    // cl/cr use their selected center sensor as the approach trigger.  Check
    // it before any line-lost handling so a valid turn trigger always wins.
    if (phase == TurnPhase::ApproachExit && centerTurnMode) {
      const uint8_t centerChannel = mode == TurnMode::cl ? 0u : 1u;
      uint16_t centerValue = 0;
      bool centerBlack = false;
      if (!normalizedCenter(centerChannel, centerValue, &centerBlack)) {
        motor(1, 1);
        return;
      }
      if (centerBlack) {
        applyTurnTouchBrake(backwardApproach);
        phase = TurnPhase::Rotate;
        phaseStartedAtMs = nowMs;
        resetTurnExit(turnExitState);
        motorMotion(turnMotorCommand(speed, ratio.left),
                    turnMotorCommand(speed, ratio.right));
        continue;
      }
    }

    uint16_t normalized[MyMINIConfig::SENSOR_COUNT] = {};
    bool blackByChannel[MyMINIConfig::SENSOR_COUNT] = {};
    uint16_t blackStrength[MyMINIConfig::SENSOR_COUNT] = {};
    for (uint8_t channel = 0; channel < MyMINIConfig::SENSOR_COUNT; ++channel) {
      if (!normalizedTrackingSensor(backwardApproach, channel,
                                    normalized[channel], &blackByChannel[channel])) {
        motor(1, 1);
        return;
      }
      int32_t strength = 1000 - normalized[channel];
      if (strength < LINE_BLACK_NOISE_THRESHOLD) strength = 0;
      blackStrength[channel] = static_cast<uint16_t>(strength);
    }

    float dt = static_cast<float>(nowUs - previousPidAtUs) * 0.000001f;
    dt = constrain(dt, PID_MIN_DT_SECONDS, PID_MAX_DT_SECONDS);
    previousPidAtUs = nowUs;

    float measuredError = lastTrackedError;
    uint8_t outsideBlack = 0;
    const bool lineFound = selectTrackedGroup(
        blackStrength, lastTrackedError, trackingInitialized, measuredError,
        outsideBlack);
    if (lineFound) {
      if (!trackingInitialized) {
        lastTrackedError = constrain(measuredError, -50.0f, 50.0f);
        trackingInitialized = true;
      } else {
        float maxErrorChange = MAX_ERROR_RATE_PER_SECOND * dt;
        maxErrorChange = constrain(maxErrorChange, 1.0f, 10.0f);
        float errorChange = measuredError - lastTrackedError;
        errorChange = constrain(errorChange, -maxErrorChange, maxErrorChange);
        lastTrackedError = constrain(lastTrackedError + errorChange,
                                     -50.0f, 50.0f);
      }
      lastLineSeenAtUs = nowUs;
    } else {
      if (approachByDistance) {
        // A distance-ended line command can legitimately leave the line
        // before cl/cr.  Continue toward the center trigger open-loop, but
        // do not integrate an error while there is no line to measure.
        approachLineWasLost = true;
        motorMotion(approachMotorCommand, approachMotorCommand);
        continue;
      }
      if (static_cast<uint32_t>(nowUs - lastLineSeenAtUs) >
          LINE_LOST_TIMEOUT_US) {
        motor(1, 1);
        return;
      }
    }

    const float error = lastTrackedError;
    const float approachSpeedFloat = static_cast<float>(approachSpeed);
    if (approachLineWasLost) {
      // Reacquisition starts a fresh derivative sample to avoid a D spike.
      lastPidError = error;
      filteredDerivative = 0.0f;
      approachLineWasLost = false;
    }
    const float correction = calculateLinePidCorrection(
        turnApproachKp, error, dt, lineFound, backwardApproach ? -1 : 1,
        backwardApproach ? 0.0f : -100.0f, approachSpeedFloat,
        approachSpeedFloat, lastPidError, integral, filteredDerivative);
    const float elapsedMs = static_cast<float>(nowUs - approachStartedAtUs) *
        0.001f;
    const float motionFactor = hasHandoff
        ? 1.0f
        : smoothstep(elapsedMs / static_cast<float>(LINE_ACCEL_TIME_MS));
    const int leftCommand = static_cast<int>(constrain(lroundf(
        (backwardApproach ? approachSpeedFloat - correction
                          : approachSpeedFloat + correction) * motionFactor),
        backwardApproach ? 0L : -100L, 100L));
    const int rightCommand = static_cast<int>(constrain(lroundf(
        (backwardApproach ? approachSpeedFloat + correction
                          : approachSpeedFloat - correction) * motionFactor),
        backwardApproach ? 0L : -100L, 100L));
    motorMotion(backwardApproach ? -leftCommand : leftCommand,
                backwardApproach ? -rightCommand : rightCommand);

    if (phase == TurnPhase::SearchOuter) {
      const bool outerEdgeBlack =
          blackByChannel[0] || blackByChannel[1] ||
          blackByChannel[14] || blackByChannel[15];
      if (updateTurnApproachExit(outerEdgeBlack, outerBlackHistory,
                                 outerBlackHistoryIndex)) {
        outerLineWasDetected = true;
        phase = TurnPhase::CrossOuter;
        phaseStartedAtMs = nowMs;
        outerWhiteHistory[0] = false;
        outerWhiteHistory[1] = false;
        outerWhiteHistory[2] = false;
        outerWhiteHistoryIndex = 0;
        motorMotion(approachMotorCommand, approachMotorCommand);
      }
    }

    if (lineFound) lastPidError = error;
  }
}


