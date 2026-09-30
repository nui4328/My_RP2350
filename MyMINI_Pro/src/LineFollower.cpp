#include "LineFollower.h"

#include <math.h>

#include <BNO055.h>

#include "CalibrationManager.h"
#include "DualMuxSensors.h"
#include "I2CDevices.h"
#include "MyMINI_ProInternal.h"
#include "MotorDriver.h"
#include "RobotConfig.h"

// -----------------------------------------------------------------------------
// Constants, internal types, sensor helpers, PID, turn, gyro, and rotate helpers
// -----------------------------------------------------------------------------

namespace {

constexpr int8_t LINE_WEIGHT[MyMINIConfig::SENSOR_COUNT] = {
    -50, -43, -37, -30, -23, -17, -10, -3,
      3,  10,  17,  23,  30,  37,  43, 50
};
constexpr uint16_t LINE_BLACK_NOISE_THRESHOLD = 200;
constexpr uint16_t EXIT_BLACK_ENTER = 650;
constexpr uint16_t EXIT_WHITE_RELEASE = 800;
constexpr uint32_t EXIT_ARM_DELAY_US = 100000;
constexpr uint32_t LINE_LOST_TIMEOUT_US = 100000;
constexpr float PID_MIN_DT_SECONDS = 0.001f;
constexpr float PID_MAX_DT_SECONDS = 0.05f;
constexpr float DERIVATIVE_FILTER_ALPHA = 0.70f;
constexpr float INTEGRAL_LIMIT = 1000.0f;
constexpr uint32_t LINE_STOP_PULL_DURATION_US = 35000;
constexpr uint32_t TURN_STOP_PULL_DURATION_US = 35000;
constexpr float FW_GYRO_CORRECTION_SIGN = -1.0f;
constexpr float ROTATE_GYRO_KP = 0.90f;
constexpr float ROTATE_TOLERANCE_DEG = 1.5f;
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
constexpr uint16_t TURN_TOUCH_BRAKE_MS = 20;   // à¸£à¸°à¸¢à¸°à¹€à¸§à¸¥à¸²à¹€à¸šà¸£à¸à¸¢à¹‰à¸­à¸™à¸à¹ˆà¸­à¸™à¸«à¸¡à¸¸à¸™ (ms)
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

struct TurnExitState {
  bool initialized = false;
  bool exitWasBlackAtStart = false;
  bool exitHasSeenWhite = false;
  bool exitArmed = false;
  uint8_t consecutiveWhiteSamples = 0;
  uint32_t lastFrameSequence = 0;
};

bool normalizedFront(uint8_t channel, uint16_t& normalized) {
  if (!calibrationManager.frontValid() ||
      channel >= MyMINIConfig::SENSOR_COUNT) {
    return false;
  }

  const CalibrationData& calibration = calibrationManager.data();
  const uint16_t minValue = calibration.frontMin[channel];
  const uint16_t maxValue = calibration.frontMax[channel];
  if (maxValue <= minValue) return false;

  const int32_t raw = sensorArrays.frontRaw()[channel];
  const int32_t mapped = static_cast<int32_t>(map(
      raw, static_cast<int32_t>(minValue), static_cast<int32_t>(maxValue),
      0L, 1000L));
  normalized = static_cast<uint16_t>(constrain(mapped, 0L, 1000L));
  return true;
}

bool normalizedRear(uint8_t channel, uint16_t& normalized) {
  if (!calibrationManager.rearValid() ||
      channel >= MyMINIConfig::SENSOR_COUNT) {
    return false;
  }

  const CalibrationData& calibration = calibrationManager.data();
  const uint16_t minValue = calibration.rearMin[channel];
  const uint16_t maxValue = calibration.rearMax[channel];
  if (maxValue <= minValue) return false;

  const int32_t raw = sensorArrays.rearRaw()[channel];
  const int32_t mapped = static_cast<int32_t>(map(
      raw, static_cast<int32_t>(minValue), static_cast<int32_t>(maxValue),
      0L, 1000L));
  normalized = static_cast<uint16_t>(constrain(mapped, 0L, 1000L));
  return true;
}

bool normalizedTrackingSensor(bool useRearSensor, uint8_t channel,
                              uint16_t& normalized) {
  return useRearSensor ? normalizedRear(channel, normalized)
                       : normalizedFront(channel, normalized);
}

bool normalizedCenter(uint8_t channel, uint16_t& normalized) {
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

// Keep the correction calculation shared by line() and turn() so that an
// approach into a turn follows exactly the same PID convention as f_line()
// and b_line().
float calculateLinePidCorrection(float kp, float error, float dt,
                                 bool lineFound, int8_t correctionDirection,
                                 float minimumCommand, float baseLeft,
                                 float baseRight,
                                 float& previousError, float& integral,
                                 float& filteredDerivative) {
  const float rawDerivative = lineFound
      ? (error - previousError) / dt : 0.0f;
  filteredDerivative = DERIVATIVE_FILTER_ALPHA * filteredDerivative +
      (1.0f - DERIVATIVE_FILTER_ALPHA) * rawDerivative;

  const float proposedIntegral = integral + error * dt;
  const float proportional = kp * error;
  const float derivativeTerm = LINE_KD * filteredDerivative;
  const float proposedCorrection = proportional +
      LINE_KI * proposedIntegral + derivativeTerm;
  const float proposedLeft = baseLeft +
      correctionDirection * proposedCorrection;
  const float proposedRight = baseRight -
      correctionDirection * proposedCorrection;
  if (proposedLeft >= minimumCommand && proposedLeft <= 100.0f &&
      proposedRight >= minimumCommand && proposedRight <= 100.0f) {
    integral = constrain(proposedIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  }

  return proportional + LINE_KI * integral + derivativeTerm;
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
    motor(turnRight ? command : -command, turnRight ? -command : command);
  } else if (backwardPivot) {
    motor(turnRight ? 0 : -command, turnRight ? -command : 0);
  } else {
    motor(turnRight ? command : 0, turnRight ? 0 : command);
  }
}

bool completeRotation(bool spinMode, bool backwardPivot, bool turnRight,
                      uint8_t stopPull) {
  if (stopPull == 0u) {
    motor(0, 0);
    return true;
  }

  const int pull = static_cast<int>(stopPull);
  if (spinMode) {
    motor(turnRight ? -pull : pull, turnRight ? pull : -pull);
  } else if (backwardPivot) {
    motor(turnRight ? 0 : pull, turnRight ? pull : 0);
  } else {
    motor(turnRight ? -pull : 0, turnRight ? 0 : -pull);
  }
  delay(ROTATE_BRAKE_MS);
  motor(0, 0);
  return true;
}

bool rotateWithGyro(bool spinMode, bool backwardPivot, float angleDeg,
                    uint8_t speed, uint8_t stopPull) {
  if (!imu.update()) {
    motor(0, 0);
    return false;
  }

  float lastYaw = imu.yaw();
  if (!isfinite(lastYaw)) {
    motor(0, 0);
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
      motor(0, 0);
      return false;
    }
    if (!imu.update()) {
      motor(0, 0);
      return false;
    }

    const float currentYaw = imu.yaw();
    if (!isfinite(currentYaw)) {
      motor(0, 0);
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
      motor(0, 0);
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
    motor(0, 0);
    return false;
  }

  const float initialRawYaw = imu.yaw();
  if (!isfinite(initialRawYaw)) {
    motor(0, 0);
    return false;
  }

  targetYaw = wrapAngle180(targetYaw);
  const float initialYaw = wrapAngle180(initialRawYaw);
  const float initialError = wrapAngle180(targetYaw - initialYaw);
  if (fabsf(initialError) <= ROTATE_TOLERANCE_DEG) {
    motor(0, 0);
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
      motor(0, 0);
      return false;
    }
    if (!imu.update()) {
      motor(0, 0);
      return false;
    }

    const float rawYaw = imu.yaw();
    if (!isfinite(rawYaw)) {
      motor(0, 0);
      return false;
    }
    const float currentYaw = wrapAngle180(rawYaw);
    const float error = wrapAngle180(targetYaw - currentYaw);
    const float remainingDegrees = fabsf(error);
    if (remainingDegrees <= ROTATE_TOLERANCE_DEG) {
      if (!hasDriven) {
        motor(0, 0);
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
      motor(0, 0);
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

  const int pullCommand = constrain(static_cast<int>(stopPull), 0, 100);
  const int reversePull = reverseDirection ? pullCommand : -pullCommand;
  motor(reversePull, reversePull);
  const uint32_t startedAtUs = micros();
  while (static_cast<uint32_t>(micros() - startedAtUs) <
         LINE_STOP_PULL_DURATION_US) {
    sensorArrays.update(micros());
  }
  motor(0, 0);
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
    const uint8_t pairFirst = selectedIndex & 0xFEu;
    const uint8_t pairSecond = pairFirst + 1u;
    uint16_t firstValue = 0;
    uint16_t secondValue = 0;
    const bool valuesValid = frontExit
        ? normalizedFront(pairFirst, firstValue) &&
              normalizedFront(pairSecond, secondValue)
        : normalizedRear(pairFirst, firstValue) &&
              normalizedRear(pairSecond, secondValue);
    if (!valuesValid) return false;
    white = firstValue >= EXIT_WHITE_RELEASE &&
            secondValue >= EXIT_WHITE_RELEASE;
    black = firstValue <= EXIT_BLACK_ENTER ||
            secondValue <= EXIT_BLACK_ENTER;
    return true;
  }

  if (exitSensor == cl || exitSensor == cr) {
    uint16_t centerValue = 0;
    const uint8_t channel = exitSensor == cl ? 0u : 1u;
    if (!normalizedCenter(channel, centerValue)) return false;
    white = centerValue >= EXIT_WHITE_RELEASE;
    black = centerValue <= EXIT_BLACK_ENTER;
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
    motor(0, 0);
    return;
  }

  const int pull = constrain(static_cast<int>(stopPull), 0, 100);
  const int brakeLeft = leftCommand > 0 ? -pull
                      : leftCommand < 0 ? pull : 0;
  const int brakeRight = rightCommand > 0 ? -pull
                       : rightCommand < 0 ? pull : 0;
  motor(brakeLeft, brakeRight);
  const uint32_t startedAtUs = micros();
  while (static_cast<uint32_t>(micros() - startedAtUs) <
         TURN_STOP_PULL_DURATION_US) {
    sensorArrays.update(micros());
  }
  motor(0, 0);
}

void applyTurnTouchBrake(bool backwardApproach) {
  if (turnTouchBrake == 0u) {
    motor(0, 0);
    return;
  }

  const int brakeCommand = backwardApproach
      ? static_cast<int>(turnTouchBrake) : -static_cast<int>(turnTouchBrake);
  motor(brakeCommand, brakeCommand);
  const uint32_t startedAtUs = micros();
  while (static_cast<uint32_t>(micros() - startedAtUs) <
         static_cast<uint32_t>(TURN_TOUCH_BRAKE_MS) * 1000UL) {
    sensorArrays.update(micros());
  }
  motor(0, 0);
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
    motor(0, 0);
    return;
  }

  const uint32_t startedAtUs = micros();
  uint32_t previousPidAtUs = startedAtUs;
  uint32_t lastLineSeenAtUs = startedAtUs;
  float lastTrackedError = 0.0f;
  float lastPidError = 0.0f;
  bool trackingInitialized = false;
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
  // A zero stopPull keeps the existing no-brake handoff, and additionally
  // requires the selected exit sensor to cross a black line before completing.
  const bool crossLineBeforeStop = !distanceMode && stopPull == 0u;
  bool stopSensorSawBlack = false;
  if (!distanceMode) {
    if (isFrontExit) {
      selectedIndex = static_cast<uint8_t>(exitSensor) -
                      static_cast<uint8_t>(f0);
    } else if (isRearExit) {
      if (!calibrationManager.rearValid()) {
        motor(0, 0);
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
      motor(0, 0);
      return;
    }

    if (reverseDirection &&
        ((isFrontExit && !calibrationManager.frontValid()) ||
         (isCenterLeftExit || isCenterRightExit) &&
             !calibrationManager.centerValid())) {
      motor(0, 0);
      return;
    }

    if (isFrontExit || isRearExit) {
      pairFirst = selectedIndex & 0xFEu;
      pairSecond = pairFirst + 1u;
    }
  }

  if (continuousEntry) {
    // Do not wait for a new sensor frame while retaining the previous line
    // command: immediately apply this command's own base speeds instead.
    motor(reverseDirection ? -sl : sl, reverseDirection ? -sr : sr);
  }

  for (;;) {
    const uint32_t nowUs = micros();

    if (static_cast<uint32_t>(nowUs - startedAtUs) >=
        LINE_TIMEOUT_MS * 1000UL) {
      motor(0, 0);
      return;
    }

    if (!sensorArrays.update(nowUs)) continue;

    uint16_t normalized[MyMINIConfig::SENSOR_COUNT] = {};
    uint16_t blackStrength[MyMINIConfig::SENSOR_COUNT] = {};
    for (uint8_t channel = 0; channel < MyMINIConfig::SENSOR_COUNT; ++channel) {
      if (!normalizedTrackingSensor(reverseDirection, channel,
                                   normalized[channel])) {
        motor(0, 0);
        return;
      }
      int32_t strength = 1000 - normalized[channel];
      if (strength < LINE_BLACK_NOISE_THRESHOLD) strength = 0;
      blackStrength[channel] = static_cast<uint16_t>(strength);
    }

    if (!distanceMode) {
      uint16_t centerValue = 0;
      uint16_t rearFirstValue = 0;
      uint16_t rearSecondValue = 0;
      uint16_t pairFirstValue = 0;
      uint16_t pairSecondValue = 0;
      bool exitWhite = false;
      bool exitBlackSample = false;
      if (centerExit) {
        if (!normalizedCenter(centerChannel, centerValue)) {
          motor(0, 0);
          return;
        }
        exitWhite = centerValue >= EXIT_WHITE_RELEASE;
        exitBlackSample = centerValue <= EXIT_BLACK_ENTER;
      } else if (rearExit) {
        if (!normalizedRear(pairFirst, rearFirstValue) ||
            !normalizedRear(pairSecond, rearSecondValue)) {
          motor(0, 0);
          return;
        }
        pairFirstValue = rearFirstValue;
        pairSecondValue = rearSecondValue;
        exitWhite = pairFirstValue >= EXIT_WHITE_RELEASE &&
                    pairSecondValue >= EXIT_WHITE_RELEASE;
        exitBlackSample = pairFirstValue <= EXIT_BLACK_ENTER ||
                          pairSecondValue <= EXIT_BLACK_ENTER;
      } else {
        if (!normalizedFront(pairFirst, pairFirstValue) ||
            !normalizedFront(pairSecond, pairSecondValue)) {
          motor(0, 0);
          return;
        }
        exitWhite = pairFirstValue >= EXIT_WHITE_RELEASE &&
                    pairSecondValue >= EXIT_WHITE_RELEASE;
        exitBlackSample = pairFirstValue <= EXIT_BLACK_ENTER ||
                          pairSecondValue <= EXIT_BLACK_ENTER;
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
        if (crossLineBeforeStop) {
          if (!stopSensorSawBlack) {
            if (selectedStopSensorIsBlack) {
              stopSensorSawBlack = true;
            }
          } else if (exitWhite) {
            if (LINE_DEBUG) {
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
            previousLineCommandContinuous = true;
            return;
          }
        } else if (selectedStopSensorIsBlack) {
          if (LINE_DEBUG) {
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
          stopAfterLineComplete(sl, sr, stopPull, reverseDirection);
          return;
        }
      }
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
    } else if (static_cast<uint32_t>(nowUs - lastLineSeenAtUs) >
               LINE_LOST_TIMEOUT_US) {
      motor(0, 0);
      return;
    }
    const float error = lastTrackedError;
    if (!continuousPidInitialized && lineFound) {
      // The new command keeps its own base speed, but starts its derivative
      // sample from the first measured error to avoid a handoff-only D kick.
      lastPidError = error;
      continuousPidInitialized = true;
    }

    const float elapsedMs = static_cast<float>(nowUs - startedAtUs) * 0.001f;
    const float accelFactor = continuousEntry
        ? 1.0f
        : smoothstep(elapsedMs / static_cast<float>(LINE_ACCEL_TIME_MS));
    float decelFactor = 1.0f;
    if (distanceMode) {
      if (estimatedDistanceMm >= targetDistanceMm) {
        recordLineNormalExit(
            reverseDirection ? LastMotionType::bLine : LastMotionType::fLine,
            LineExitSource::distance, completedDirection, stopPull);
        stopAfterLineComplete(sl, sr, stopPull, reverseDirection);
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
        kp, error, dt, lineFound, 1, -100.0f, static_cast<float>(sl),
        static_cast<float>(sr), lastPidError, integral, filteredDerivative);
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
      motor(-leftCommand, -rightCommand);
    } else {
      leftCommand = static_cast<int>(constrain(
          lroundf(effectiveSL + effectiveCorrection), -100L, 100L));
      rightCommand = static_cast<int>(constrain(
          lroundf(effectiveSR - effectiveCorrection), -100L, 100L));
      motor(leftCommand, rightCommand);
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
        return;
      }
    }

    if (lineFound) lastPidError = error;
  }
}

} // namespace

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
    motor(0, 0);
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
    motor(0, 0);
    return;
  }
  if (!continuousEntry) motor(0, 0);
  runLine(sl, sr, kp, false, 0.0f, exitSensor, stopPull, true,
          TravelDirection::backward, continuousEntry);
}

void b_line(int sl, int sr, float kp, float distanceCm,
            uint8_t stopPull) {
  const bool continuousEntry = previousLineCommandContinuous;
  clearLastFLineState(LastMotionType::bLine);
  if (sl < 0 || sr < 0 || sl > 100 || sr > 100 || distanceCm <= 0.0f) {
    motor(0, 0);
    return;
  }
  if (!continuousEntry) motor(0, 0);
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
    motor(0, 0);
  };

  if (!imuReady) {
    motor(0, 0);
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
      motor(-static_cast<int>(stopPull), -static_cast<int>(stopPull));
      delay(35);
      motor(0, 0);
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
    motor(leftOutput, rightOutput);

    const float forwardCommand =
        (static_cast<float>(leftOutput) + rightOutput) * 0.5f;
    estimatedDistanceMm += LINE_MM_PER_SECOND_AT_100 *
        (forwardCommand / 100.0f) * dt;
    if (estimatedDistanceMm >= targetDistanceMm) {
      recordLineNormalExit(LastMotionType::fLine, LineExitSource::distance,
                           TravelDirection::forward, stopPull);
      if (stopPull == 0u) return;
      motor(-static_cast<int>(stopPull), -static_cast<int>(stopPull));
      delay(35);
      motor(0, 0);
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
    motor(0, 0);
  };

  if (!imuReady) {
    motor(0, 0);
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
      motor(static_cast<int>(stopPull), static_cast<int>(stopPull));
      delay(35);
      motor(0, 0);
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
    motor(leftOutput, rightOutput);

    const float backwardCommand =
        -(static_cast<float>(leftOutput) + rightOutput) * 0.5f;
    estimatedDistanceMm += LINE_MM_PER_SECOND_AT_100 *
        (backwardCommand / 100.0f) * dt;
    if (estimatedDistanceMm >= targetDistanceMm) {
      recordLineNormalExit(LastMotionType::bLine, LineExitSource::distance,
                           TravelDirection::backward, stopPull);
      if (stopPull == 0u) return;
      motor(static_cast<int>(stopPull), static_cast<int>(stopPull));
      delay(35);
      motor(0, 0);
      return;
    }
  }
}

// -----------------------------------------------------------------------------
// Public rotate API
// -----------------------------------------------------------------------------

bool rotate_spin(float angleDeg, uint8_t speed, uint8_t stopPull) {
  if (!validRotateArguments(angleDeg, speed, stopPull)) {
    motor(0, 0);
    return false;
  }
  return imuReady
      ? rotateWithGyro(true, false, angleDeg, speed, stopPull)
      : rotateWithFallback(true, false, angleDeg, speed, stopPull);
}

bool rotateFW_pivot(float angleDeg, uint8_t speed, uint8_t stopPull) {
  if (!validRotateArguments(angleDeg, speed, stopPull, true)) {
    motor(0, 0);
    return false;
  }
  if (imuReady) {
    return rotatePivotToHeadingWithGyro(false, angleDeg, speed, stopPull);
  }
  if (angleDeg == 0.0f) {
    motor(0, 0);
    return true;
  }
  return rotateWithFallback(false, false, angleDeg, speed, stopPull);
}

bool rotateBW_pivot(float angleDeg, uint8_t speed, uint8_t stopPull) {
  if (!validRotateArguments(angleDeg, speed, stopPull, true)) {
    motor(0, 0);
    return false;
  }
  if (imuReady) {
    return rotatePivotToHeadingWithGyro(true, angleDeg, speed, stopPull);
  }
  if (angleDeg == 0.0f) {
    motor(0, 0);
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

bool set_turn_line_search(uint8_t searchSpeed, uint16_t fastTurnMs) {
  if (searchSpeed == 0u || searchSpeed > 100u) return false;
  turnLineSearchSpeed = searchSpeed;
  turnFastTimeMs = fastTurnMs;
  return true;
}

// -----------------------------------------------------------------------------
// Public turn API
// -----------------------------------------------------------------------------

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
    motor(0, 0);
    return;
  }

  const TurnMotorRatio ratio = turnMotorRatios[modeIndex];
  if (ratio.left < -100 || ratio.left > 100 || ratio.right < -100 ||
      ratio.right > 100) {
    motor(0, 0);
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
    motor(0, 0);
    return;
  }
  if (centerTurnMode) {
    const LineExitSource exitSource = exitSourceFor(exitSensor);
    const bool exitCalibrationValid =
        (exitSource == LineExitSource::front && calibrationManager.frontValid()) ||
        (exitSource == LineExitSource::rear && calibrationManager.rearValid()) ||
        (exitSource == LineExitSource::center && calibrationManager.centerValid());
    if (!calibrationManager.centerValid() || !exitCalibrationValid) {
      motor(0, 0);
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
      motor(0, 0);
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
        motor(0, 0);
        return;
      }
      if (updateTurnExit(turnExitState, exitWhite, exitBlack)) {
        stopAfterTurnComplete(turnLeftCommand, turnRightCommand, stopPull);
        return;
      }
      motor(turnLeftCommand, turnRightCommand);
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
        motor(0, 0);
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
      motor(approachMotorCommand, approachMotorCommand);
      continue;
    }

    if (phase == TurnPhase::Overshoot) {
      motor(approachMotorCommand, approachMotorCommand);
      if (static_cast<uint32_t>(nowMs - phaseStartedAtMs) <
          turnOvershootMs) {
        continue;
      }
      applyTurnTouchBrake(backwardApproach);
      phase = TurnPhase::Rotate;
      phaseStartedAtMs = nowMs;
      resetTurnExit(turnExitState);
      motor(turnMotorCommand(speed, ratio.left),
            turnMotorCommand(speed, ratio.right));
      continue;
    }

    // cl/cr use their selected center sensor as the approach trigger.  Check
    // it before any line-lost handling so a valid turn trigger always wins.
    if (phase == TurnPhase::ApproachExit && centerTurnMode) {
      const uint8_t centerChannel = mode == TurnMode::cl ? 0u : 1u;
      uint16_t centerValue = 0;
      if (!normalizedCenter(centerChannel, centerValue)) {
        motor(0, 0);
        return;
      }
      if (centerValue <= EXIT_BLACK_ENTER) {
        applyTurnTouchBrake(backwardApproach);
        phase = TurnPhase::Rotate;
        phaseStartedAtMs = nowMs;
        resetTurnExit(turnExitState);
        motor(turnMotorCommand(speed, ratio.left),
              turnMotorCommand(speed, ratio.right));
        continue;
      }
    }

    uint16_t normalized[MyMINIConfig::SENSOR_COUNT] = {};
    uint16_t blackStrength[MyMINIConfig::SENSOR_COUNT] = {};
    for (uint8_t channel = 0; channel < MyMINIConfig::SENSOR_COUNT; ++channel) {
      if (!normalizedTrackingSensor(backwardApproach, channel,
                                    normalized[channel])) {
        motor(0, 0);
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
        motor(approachMotorCommand, approachMotorCommand);
        continue;
      }
      if (static_cast<uint32_t>(nowUs - lastLineSeenAtUs) >
          LINE_LOST_TIMEOUT_US) {
        motor(0, 0);
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
    motor(backwardApproach ? -leftCommand : leftCommand,
          backwardApproach ? -rightCommand : rightCommand);

    if (phase == TurnPhase::SearchOuter) {
      const bool outerEdgeBlack =
          normalized[0] <= EXIT_BLACK_ENTER ||
          normalized[1] <= EXIT_BLACK_ENTER ||
          normalized[14] <= EXIT_BLACK_ENTER ||
          normalized[15] <= EXIT_BLACK_ENTER;
      if (updateTurnApproachExit(outerEdgeBlack, outerBlackHistory,
                                 outerBlackHistoryIndex)) {
        outerLineWasDetected = true;
        phase = TurnPhase::CrossOuter;
        phaseStartedAtMs = nowMs;
        outerWhiteHistory[0] = false;
        outerWhiteHistory[1] = false;
        outerWhiteHistory[2] = false;
        outerWhiteHistoryIndex = 0;
        motor(approachMotorCommand, approachMotorCommand);
      }
    }

    if (lineFound) lastPidError = error;
  }
}


