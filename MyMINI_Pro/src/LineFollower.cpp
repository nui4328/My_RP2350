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

int positoin_error = 50;

// -----------------------------------------------------------------------------
// Constants, internal types, sensor helpers, PID, turn, gyro, and rotate helpers
// -----------------------------------------------------------------------------

namespace {

struct ResetLinePositionOnReturn {
  ~ResetLinePositionOnReturn() { positoin_error = 50; }
};

constexpr int8_t LINE_WEIGHT[MyMINIConfig::SENSOR_COUNT] = {
    -50, -43, -37, -30, -23, -17, -10, -3,
      3,  10,  17,  23,  30,  37,  43, 50
};
constexpr uint16_t LINE_BLACK_NOISE_THRESHOLD = 200;
constexpr uint32_t LINE_ALL_WHITE_TIMEOUT_US = 3000000UL;
constexpr uint8_t INTERSECTION_STABLE_FRAMES = 2;
constexpr uint8_t INTERSECTION_CONFIRM_FRAMES = 2;
constexpr uint8_t INTERSECTION_CLEAR_FRAMES = 2;
constexpr uint8_t INTERSECTION_MAX_NARROW_BLACK = 4;
constexpr int16_t INTERSECTION_ANCHOR_RADIUS = 10;
constexpr uint8_t INTERSECTION_MIN_OUTSIDE_BLACK = 2;
constexpr uint8_t INTERSECTION_MIN_SPAN = 6;
constexpr uint8_t INTERSECTION_MIN_NEAR_WING_BLACK = 2;
constexpr uint8_t INTERSECTION_MIN_NEAR_SPAN = 3;
constexpr uint32_t INTERSECTION_MIN_HOLD_US = 80000UL;
constexpr uint32_t INTERSECTION_MAX_HOLD_US = 500000UL;
// f_line()/b_line(), turn() exit arming, and pre-turn crossing use this threshold.
// Black is below the calibrated midpoint (~500); the 100-point hysteresis
// still accepts a white floor that normalizes around 630-660.
constexpr uint16_t LINE_EXIT_WHITE_RELEASE = 600;
constexpr uint32_t EXIT_ARM_DELAY_US = 100000;
constexpr uint32_t LINE_LOST_TIMEOUT_US = 100000;
constexpr float SIDE_LINE_MAX_KP = 0.85f;
constexpr float SIDE_LINE_STEER_CORRECTION = 10.0f;
constexpr uint8_t SIDE_LINE_CONFIRM_FRAMES = 2;
constexpr uint8_t SIDE_LINE_CLEAR_FRAMES = 2;
constexpr uint32_t SIDE_LINE_MAX_CROSS_US = 500000UL;
constexpr float PID_MIN_DT_SECONDS = 0.001f;
constexpr float PID_MAX_DT_SECONDS = 0.05f;
constexpr float DERIVATIVE_FILTER_ALPHA = 0.70f;
constexpr float INTEGRAL_LIMIT = 1000.0f;
constexpr uint16_t TURN_EXIT_MIN_ROTATION_MS = 40;
constexpr float FW_GYRO_CORRECTION_SIGN = -1.0f;
constexpr float DEFAULT_GYRO_KD = 0.035f;
constexpr float TURN_GYRO_KP = 0.90f;
constexpr float TURN_GYRO_BRAKE_LEAD_DEG = 20.0f;
constexpr uint16_t ROTATE_LOOP_DELAY_MS = 5;
constexpr float GYRO_SETTLE_TOLERANCE_DEG = 1.5f;
constexpr uint16_t GYRO_SETTLE_MS = 110;
constexpr uint8_t GYRO_MAX_CORRECTIONS = 4;
uint16_t rotateSpin90Ms = 300;
uint16_t rotatePivot90Ms = 550;
uint8_t rotateCalibrationSpeed = 50;
float rotateGyroKp = 0.90f;
float rotateGyroKd = 0.035f;
uint16_t rotateSpinSlowdownMs = 0;
uint8_t rotateSpinSlowSpeed = 20;

struct TurnMotorRatio {
  int8_t left;
  int8_t right;
};

TurnMotorRatio turnMotorRatios[] = {
    {-10, 100},  // TurnMode::fl
    {100, -10},  // TurnMode::fr
    {-100, 100},  // TurnMode::cl
    {100, -100},  // TurnMode::cr
    {-100, 100},  // TurnMode::l
    {100, -100}   // TurnMode::r
};
uint16_t turnOvershootMs = 20;                 // à¹€à¸”à¸´à¸™à¸«à¸™à¹‰à¸²à¸•à¹ˆà¸­à¸«à¸¥à¸±à¸‡à¸‚à¹‰à¸²à¸¡à¹€à¸ªà¹‰à¸™ à¸à¹ˆà¸­à¸™à¹€à¸šà¸£à¸à¹à¸¥à¸°à¸«à¸¡à¸¸à¸™ (ms)
uint16_t turnTimeoutMs = 3000;                 // à¹€à¸§à¸¥à¸²à¸ªà¸¹à¸‡à¸ªà¸¸à¸”à¸‚à¸­à¸‡ turn() à¸à¹ˆà¸­à¸™à¸«à¸¢à¸¸à¸”à¸‰à¸¸à¸à¹€à¸‰à¸´à¸™ (ms)
float turnApproachKp = 0.150f;                 // Kp à¸Šà¹ˆà¸§à¸‡à¹€à¸”à¸´à¸™à¸«à¸™à¹‰à¸² PID à¹€à¸‚à¹‰à¸²à¸«à¸²à¹€à¸ªà¹‰à¸™
uint8_t turnSensorExitSpeed = 25;
uint8_t turnDistanceExitSpeed = 25;
uint8_t turnTouchBrake = 15;                   // à¸à¸³à¸¥à¸±à¸‡à¹€à¸šà¸£à¸à¸¢à¹‰à¸­à¸™à¸«à¸¥à¸±à¸‡à¸‚à¹‰à¸²à¸¡à¹€à¸ªà¹‰à¸™ (0â€“100)
uint16_t turnTouchBrakeMs = 30;                 // Reverse brake duration before rotation (ms).
uint8_t turnCenterForwardBrakePower = 15;
uint16_t turnCenterForwardBrakeMs = 10;
uint8_t turnCenterBackwardBrakePower = 15;
uint16_t turnCenterBackwardBrakeMs = 10;
bool turnCenterBrakeConfigured = false;
uint8_t turnLineSearchSpeed = 40;               // Search speed after the initial fast turn (1-100).
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
uint16_t lastFrontExitSensorMask = 0u;
TravelDirection lastLineDirection = TravelDirection::none;
bool lastLineEndedNormally = false;
bool pendingLineHandoff = false;
bool previousLineCommandContinuous = false;
bool lineDiagnosticsEnabled = false;
bool lineErrorMonitorEnabled = false;
constexpr uint16_t LINE_FOLLOW_DEFAULT_RAMP_MS = 200;
constexpr int LINE_FOLLOW_NO_RAMP_BELOW_SPEED = 40;
uint16_t lineRampMs = LINE_FOLLOW_DEFAULT_RAMP_MS;
int lineRampStartSpeed = 0;
float lineDistanceScale = 1.0f;
float lineDecelRampMm = 20.0f;  // 200 units in set_line_decel_ramp_cm().
float lineKpScale = 1.0f;
float lineKi = LINE_KI;
float lineKd = LINE_KD;
float forwardGyroKd = DEFAULT_GYRO_KD;
float backwardGyroKd = DEFAULT_GYRO_KD;

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
  bool exitArmed = false;
  uint8_t consecutiveWhiteSamples = 0;
  bool blackHistory[3] = {false, false, false};
  uint8_t blackHistoryIndex = 0;
  uint32_t lastFrameSequence = 0;
};

// CalibrationManager::begin() loads all EEPROM min/max values into data_ once
// during robot_begin(). A later successful calibration updates that same RAM
// data. Compare each fresh ADC sample against its own channel midpoint.
bool rawBelowCalibrationMidpoint(int32_t raw, int32_t minValue,
                                  int32_t maxValue) {
  return raw * 2 < minValue + maxValue;
}

// The adjacent channel helps confirm that the selected exit sensor has
// cleared the previous line; only the requested channel can stop the turn.
uint8_t inwardExitCompanion(uint8_t selectedIndex) {
  return selectedIndex <= 7u ? selectedIndex + 1u : selectedIndex - 1u;
}

bool normalizedFront(uint8_t channel, uint16_t& normalized,
                     bool* isBlack = nullptr, bool useRaw = false) {
  if (!calibrationManager.frontValid() ||
      channel >= MyMINIConfig::SENSOR_COUNT) {
    return false;
  }

  const CalibrationData& calibration = calibrationManager.data();
  const uint16_t minValue = calibration.frontMin[channel];
  const uint16_t maxValue = calibration.frontMax[channel];
  if (maxValue <= minValue) return false;

  const int32_t raw = useRaw ? sensorArrays.frontRaw()[channel]
                             : sensorArrays.frontFiltered()[channel];
  if (isBlack) *isBlack = rawBelowCalibrationMidpoint(raw, minValue, maxValue);
  const int32_t mapped = static_cast<int32_t>(map(
      raw, static_cast<int32_t>(minValue), static_cast<int32_t>(maxValue),
      0L, 1000L));
  normalized = static_cast<uint16_t>(constrain(mapped, 0L, 1000L));
  return true;
}

bool normalizedRear(uint8_t channel, uint16_t& normalized,
                    bool* isBlack = nullptr, bool useRaw = false) {
  if (!calibrationManager.rearValid() ||
      channel >= MyMINIConfig::SENSOR_COUNT) {
    return false;
  }

  const CalibrationData& calibration = calibrationManager.data();
  const uint16_t minValue = calibration.rearMin[channel];
  const uint16_t maxValue = calibration.rearMax[channel];
  if (maxValue <= minValue) return false;

  const int32_t raw = useRaw ? sensorArrays.rearRaw()[channel]
                             : sensorArrays.rearFiltered()[channel];
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
                         uint8_t& outsideBlack,
                         uint8_t* selectedStart = nullptr,
                         uint8_t* selectedEnd = nullptr) {
  outsideBlack = 0;
  if (selectedStart) *selectedStart = 255u;
  if (selectedEnd) *selectedEnd = 255u;
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
        if (selectedStart) *selectedStart = start;
        if (selectedEnd) *selectedEnd = end;
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

float lineLostRecoveryError(float kp, bool trackingInitialized,
                            int8_t lastEdgeSide, float lastMeasuredError) {
  if (kp <= 0.5f || !trackingInitialized) return 0.0f;
  if (lastEdgeSide != 0) return static_cast<float>(lastEdgeSide) * 50.0f;
  return lastMeasuredError;
}

struct SideLineCrossing {
  int8_t activeSide = 0;  // -1 left, +1 right.
  int8_t lastVisibleSide = 0;
  int8_t pendingSide = 0;
  uint8_t pendingFrames = 0;
  uint8_t clearFrames = 0;
  uint32_t startedAtUs = 0;
};

bool sideBlackRange(const uint16_t* normalized, uint8_t first, uint8_t last) {
  for (uint8_t i = first; i <= last; ++i) {
    if (normalized[i] >= 500u) return false;
  }
  return true;
}

bool sideWhitePair(const uint16_t* normalized, uint8_t first) {
  return normalized[first] >= LINE_EXIT_WHITE_RELEASE &&
         normalized[first + 1u] >= LINE_EXIT_WHITE_RELEASE;
}

// While cl/cr approaches its trigger, a fully white row has no usable line
// error. A nearly black half-row is a wide side branch, not a line to steer
// toward. Keep the approach straight until the center trigger or a narrow
// track returns.
bool centerApproachShouldDriveStraight(const uint16_t* normalized,
                                       const bool* blackByChannel) {
  bool allWhite = true;
  uint8_t leftBlack = 0;
  uint8_t rightBlack = 0;
  for (uint8_t i = 0; i < MyMINIConfig::SENSOR_COUNT; ++i) {
    if (normalized[i] < LINE_EXIT_WHITE_RELEASE) allWhite = false;
    if (blackByChannel[i]) {
      if (i < MyMINIConfig::SENSOR_COUNT / 2u) {
        ++leftBlack;
      } else {
        ++rightBlack;
      }
    }
  }
  return allWhite || leftBlack >= 6u || rightBlack >= 6u;
}

struct IntersectionGuard {
  uint8_t stableFrames = 0;
  uint8_t narrowStart = 255;
  uint8_t narrowEnd = 255;
  uint8_t narrowCount = 0;
  uint8_t candidateFrames = 0;
  uint8_t clearFrames = 0;
  int8_t candidateSide = 0;
  bool confirmed = false;
  uint32_t startedAtUs = 0;
  float heldError = 0.0f;
};

enum class IntersectionGuardResult : uint8_t { Follow, Hold, Released };

struct IntersectionGuardObservation {
  uint8_t blackCount = 0;
  uint8_t anchorCount = 0;
  uint8_t leftOutside = 0;
  uint8_t rightOutside = 0;
  int8_t branchSide = 0;
  bool narrowMainLine = false;
  bool branchShape = false;
};

// Capture control-loop values in RAM. Serial output happens only when the
// caller prints the trace after a blocking line/turn command has returned.
constexpr uint16_t LINE_INTERSECTION_TRACE_CAPACITY = 192;
constexpr uint8_t LINE_INTERSECTION_TRACE_POST_FRAMES = 48;
struct LineIntersectionTraceFrame {
  uint32_t frameSequence = 0;
  uint32_t frameAtUs = 0;
  uint32_t frameDeltaUs = 0;
  uint32_t pidDeltaUs = 0;
  uint32_t guardAgeUs = 0;
  uint16_t normalized[MyMINIConfig::SENSOR_COUNT] = {};
  uint16_t blackStrength[MyMINIConfig::SENSOR_COUNT] = {};
  uint16_t guardStrength[MyMINIConfig::SENSOR_COUNT] = {};
  float referenceError = 0.0f;
  float measuredError = 0.0f;
  float trackedError = 0.0f;
  float error = 0.0f;
  float pidPreviousBefore = 0.0f;
  float pidPreviousUsed = 0.0f;
  float proportional = 0.0f;
  float integral = 0.0f;
  float derivative = 0.0f;
  float correction = 0.0f;
  float motionFactor = 0.0f;
  float heldError = 0.0f;
  int16_t leftMotor = 0;
  int16_t rightMotor = 0;
  uint8_t command = 0;
  uint8_t groupStart = 255;
  uint8_t groupEnd = 255;
  uint8_t outsideBlack = 0;
  uint8_t selectedLineFound = 0;
  uint8_t finalLineFound = 0;
  uint8_t guardResult = 0;
  uint8_t stableBefore = 0;
  uint8_t candidateBefore = 0;
  uint8_t confirmedBefore = 0;
  uint8_t stableAfter = 0;
  uint8_t candidateAfter = 0;
  uint8_t confirmedAfter = 0;
  uint8_t clearAfter = 0;
  IntersectionGuardObservation observation;
};

bool lineIntersectionTraceEnabled = false;
bool lineIntersectionTraceFrozen = false;
bool lineIntersectionTraceTriggered = false;
uint8_t lineIntersectionTracePostFrames = 0;
uint16_t lineIntersectionTraceNext = 0;
uint16_t lineIntersectionTraceCount = 0;
bool lineIntersectionTraceHasPriorError = false;
float lineIntersectionTracePriorError = 0.0f;
LineIntersectionTraceFrame lineIntersectionTrace[LINE_INTERSECTION_TRACE_CAPACITY];

LineIntersectionTraceFrame* nextLineIntersectionTraceFrame(
    uint8_t command, uint32_t frameSequence, uint32_t frameAtUs,
    uint32_t frameDeltaUs, uint32_t pidDeltaUs, const uint16_t* normalized,
    const uint16_t* blackStrength, const uint16_t* guardStrength) {
  if (!lineIntersectionTraceEnabled || lineIntersectionTraceFrozen) return nullptr;
  LineIntersectionTraceFrame& frame =
      lineIntersectionTrace[lineIntersectionTraceNext];
  lineIntersectionTraceNext = static_cast<uint16_t>(
      (lineIntersectionTraceNext + 1u) % LINE_INTERSECTION_TRACE_CAPACITY);
  if (lineIntersectionTraceCount < LINE_INTERSECTION_TRACE_CAPACITY)
    ++lineIntersectionTraceCount;
  frame.command = command;
  frame.frameSequence = frameSequence;
  frame.frameAtUs = frameAtUs;
  frame.frameDeltaUs = frameDeltaUs;
  frame.pidDeltaUs = pidDeltaUs;
  for (uint8_t i = 0; i < MyMINIConfig::SENSOR_COUNT; ++i) {
    frame.normalized[i] = normalized[i];
    frame.blackStrength[i] = blackStrength[i];
    frame.guardStrength[i] = guardStrength[i];
  }
  return &frame;
}

void finishLineIntersectionTraceFrame(LineIntersectionTraceFrame& frame) {
  const bool outputJump = lineIntersectionTraceHasPriorError &&
      fabsf(frame.error - lineIntersectionTracePriorError) >= 6.0f;
  const bool measuredJump =
      fabsf(frame.measuredError - frame.referenceError) >= 6.0f;
  const bool wideOutside = frame.observation.blackCount >= 4u &&
      (frame.observation.leftOutside >= 2u ||
       frame.observation.rightOutside >= 2u);
  if (!lineIntersectionTraceTriggered && lineIntersectionTraceCount >= 3u &&
      (outputJump || measuredJump || wideOutside)) {
    lineIntersectionTraceTriggered = true;
    lineIntersectionTracePostFrames = LINE_INTERSECTION_TRACE_POST_FRAMES;
  } else if (lineIntersectionTraceTriggered &&
             lineIntersectionTracePostFrames > 0u &&
             --lineIntersectionTracePostFrames == 0u) {
    lineIntersectionTraceFrozen = true;
  }
  lineIntersectionTracePriorError = frame.error;
  lineIntersectionTraceHasPriorError = true;
}

uint32_t intersectionHoldLimitUs(int leftSpeed, int rightSpeed) {
  const float requestedSpeed = max(10.0f,
      (static_cast<float>(abs(leftSpeed)) + abs(rightSpeed)) * 0.5f);
  const float millimetersPerSecond = LINE_MM_PER_SECOND_AT_100 *
      requestedSpeed / 100.0f;
  const uint32_t travelTimeUs = static_cast<uint32_t>(
      60.0f * 1000000.0f / millimetersPerSecond);
  if (travelTimeUs < INTERSECTION_MIN_HOLD_US) return INTERSECTION_MIN_HOLD_US;
  if (travelTimeUs > INTERSECTION_MAX_HOLD_US) return INTERSECTION_MAX_HOLD_US;
  return travelTimeUs;
}

IntersectionGuardResult updateIntersectionGuard(
    IntersectionGuard& guard, const uint16_t* blackStrength,
    float referenceError, float measuredError, bool lineFound,
    uint8_t outsideBlack, uint32_t nowUs, uint32_t holdLimitUs,
    IntersectionGuardObservation* observation = nullptr) {
  uint8_t blackCount = 0;
  uint8_t anchorCount = 0;
  uint8_t leftOutside = 0;
  uint8_t rightOutside = 0;
  uint8_t leftWing = 0;
  uint8_t rightWing = 0;
  uint8_t firstBlack = MyMINIConfig::SENSOR_COUNT;
  uint8_t lastBlack = 0;
  for (uint8_t i = 0; i < MyMINIConfig::SENSOR_COUNT; ++i) {
    if (blackStrength[i] == 0u) continue;
    if (blackCount == 0u) firstBlack = i;
    lastBlack = i;
    ++blackCount;
    const float offset = LINE_WEIGHT[i] - referenceError;
    if (fabsf(offset) <= INTERSECTION_ANCHOR_RADIUS) ++anchorCount;
    if (offset < -INTERSECTION_ANCHOR_RADIUS) ++leftWing;
    if (offset > INTERSECTION_ANCHOR_RADIUS) ++rightWing;
    if (offset < -TRACK_WINDOW_RADIUS) ++leftOutside;
    if (offset > TRACK_WINDOW_RADIUS) ++rightOutside;
  }

  const bool narrowMainLine = lineFound && blackCount > 0u &&
      blackCount <= INTERSECTION_MAX_NARROW_BLACK &&
      leftOutside == 0u && rightOutside == 0u &&
      fabsf(measuredError - referenceError) <= INTERSECTION_ANCHOR_RADIUS;
  const uint8_t leftExtension = guard.narrowCount > 0u &&
      firstBlack < guard.narrowStart ? guard.narrowStart - firstBlack : 0u;
  const uint8_t rightExtension = guard.narrowCount > 0u &&
      lastBlack > guard.narrowEnd ? lastBlack - guard.narrowEnd : 0u;
  // A branch can begin with four black sensors, still inside the ordinary
  // tracking window. Compare it with the previously stable narrow footprint
  // before updating that footprint or allowing its centroid into the PID.
  const bool nearWingShape = guard.stableFrames >= INTERSECTION_STABLE_FRAMES &&
      guard.narrowCount > 0u &&
      blackCount >= guard.narrowCount + INTERSECTION_MIN_NEAR_WING_BLACK &&
      lastBlack - firstBlack >= INTERSECTION_MIN_NEAR_SPAN &&
      (leftExtension >= INTERSECTION_MIN_NEAR_WING_BLACK ||
       rightExtension >= INTERSECTION_MIN_NEAR_WING_BLACK);
  const bool farWingShape = blackCount >= INTERSECTION_MAX_NARROW_BLACK &&
      lastBlack - firstBlack >= INTERSECTION_MIN_SPAN &&
      outsideBlack >= INTERSECTION_MIN_OUTSIDE_BLACK;
  const bool leftBranch = leftWing >= INTERSECTION_MIN_NEAR_WING_BLACK ||
      (nearWingShape && leftExtension >= INTERSECTION_MIN_NEAR_WING_BLACK);
  const bool rightBranch = rightWing >= INTERSECTION_MIN_NEAR_WING_BLACK ||
      (nearWingShape && rightExtension >= INTERSECTION_MIN_NEAR_WING_BLACK);
  const int8_t branchSide = leftBranch && rightBranch ? 2 :
      leftBranch ? -1 : rightBranch ? 1 : 0;
  const bool branchShape = lineFound && anchorCount > 0u &&
      branchSide != 0 && (nearWingShape || farWingShape);
  if (observation) {
    observation->blackCount = blackCount;
    observation->anchorCount = anchorCount;
    observation->leftOutside = leftOutside;
    observation->rightOutside = rightOutside;
    observation->branchSide = branchSide;
    observation->narrowMainLine = narrowMainLine;
    observation->branchShape = branchShape;
  }

  if (guard.confirmed || guard.candidateFrames > 0u) {
    if (narrowMainLine && !branchShape &&
        blackCount <= guard.narrowCount &&
        fabsf(measuredError - guard.heldError) <= INTERSECTION_ANCHOR_RADIUS) {
      if (++guard.clearFrames >= INTERSECTION_CLEAR_FRAMES) {
        guard = IntersectionGuard{};
        guard.stableFrames = INTERSECTION_STABLE_FRAMES;
        guard.narrowStart = firstBlack;
        guard.narrowEnd = lastBlack;
        guard.narrowCount = blackCount;
        return IntersectionGuardResult::Released;
      }
      return IntersectionGuardResult::Hold;
    }
    guard.clearFrames = 0;
    if (anchorCount == 0u ||
        static_cast<uint32_t>(nowUs - guard.startedAtUs) >= holdLimitUs) {
      guard = IntersectionGuard{};
      return IntersectionGuardResult::Released;
    }
    if (guard.confirmed) return IntersectionGuardResult::Hold;
    if (!branchShape) {
      guard = IntersectionGuard{};
      return IntersectionGuardResult::Released;
    }
    // A one-sided arm may fill the opposite side on the next frame as the
    // robot crosses the junction; keep the original line anchor in that case.
    if (branchSide == guard.candidateSide || branchSide == 2 ||
        guard.candidateSide == 2) {
      if (++guard.candidateFrames >= INTERSECTION_CONFIRM_FRAMES)
        guard.confirmed = true;
      return IntersectionGuardResult::Hold;
    }
    guard = IntersectionGuard{};
    return IntersectionGuardResult::Released;
  }

  if (branchShape && guard.stableFrames >= INTERSECTION_STABLE_FRAMES) {
    guard.candidateFrames = 1u;
    guard.candidateSide = branchSide;
    guard.startedAtUs = nowUs;
    guard.heldError = referenceError;
    return IntersectionGuardResult::Hold;
  }
  guard.stableFrames = narrowMainLine
      ? min(static_cast<uint8_t>(guard.stableFrames + 1u),
            INTERSECTION_STABLE_FRAMES) : 0u;
  if (narrowMainLine) {
    guard.narrowStart = firstBlack;
    guard.narrowEnd = lastBlack;
    guard.narrowCount = blackCount;
  } else {
    guard.narrowStart = 255u;
    guard.narrowEnd = 255u;
    guard.narrowCount = 0u;
  }
  return IntersectionGuardResult::Follow;
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

float lineStartupFactor(float elapsedMs, int sl, int sr,
                        bool continuousEntry, bool recoveringKnownSide) {
  const bool lowSpeedStart = abs(sl) < LINE_FOLLOW_NO_RAMP_BELOW_SPEED &&
                             abs(sr) < LINE_FOLLOW_NO_RAMP_BELOW_SPEED;
  if (continuousEntry || lowSpeedStart || recoveringKnownSide ||
      lineRampMs == 0u) return 1.0f;

  const int requestedPeakSpeed = max(abs(sl), abs(sr));
  const float startFactor = min(1.0f, static_cast<float>(lineRampStartSpeed) /
                                      static_cast<float>(requestedPeakSpeed));
  return startFactor + (1.0f - startFactor) *
      smoothstep(elapsedMs / static_cast<float>(lineRampMs));
}

float estimatedLineDistanceStepMm(int leftCommand, int rightCommand,
                                  uint32_t elapsedUs) {
  const float forwardCommand = max(0.0f,
      (static_cast<float>(leftCommand) + rightCommand) * 0.5f);
  return LINE_MM_PER_SECOND_AT_100 * lineDistanceScale *
      (forwardCommand / 100.0f) *
      (static_cast<float>(elapsedUs) * 0.000001f);
}

float distanceLineDecelFactor(float remainingMm, float targetDistanceMm,
                              int sl, int sr, uint8_t stopPull) {
  const float decelRangeMm = lineDecelRampMm >= 0.0f
      ? min(lineDecelRampMm, targetDistanceMm)
      : min(LINE_DECEL_DISTANCE_MM, targetDistanceMm * 0.40f);
  if (decelRangeMm <= 0.0f || remainingMm >= decelRangeMm) return 1.0f;

  float terminalFactor = LINE_MIN_DECEL_FACTOR;
  if (stopPull == 0u) {
    // A distance-ended line command hands its motion to turn() without braking.
    // Match the configured distance-exit approach speed at the handoff.
    const float requestedSpeed =
        (static_cast<float>(abs(sl)) + abs(sr)) * 0.5f;
    if (requestedSpeed > 0.0f) {
      terminalFactor = min(1.0f,
          static_cast<float>(turnDistanceExitSpeed) / requestedSpeed);
    }
  }
  const float x = remainingMm / decelRangeMm;
  return terminalFactor + (1.0f - terminalFactor) * smoothstep(x);
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

void rotationMotorCommands(bool spinMode, bool backwardPivot, bool turnRight,
                           uint8_t commandSpeed, int& left, int& right) {
  const int command = static_cast<int>(commandSpeed);
  if (spinMode) {
    left = turnRight ? command : -command;
    right = -left;
  } else if (backwardPivot) {
    left = turnRight ? 0 : -command;
    right = turnRight ? -command : 0;
  } else {
    left = turnRight ? command : 0;
    right = turnRight ? 0 : command;
  }
}

void driveRotation(bool spinMode, bool backwardPivot, bool turnRight,
                   uint8_t commandSpeed) {
  int left = 0;
  int right = 0;
  rotationMotorCommands(spinMode, backwardPivot, turnRight, commandSpeed,
                        left, right);
  motorMotion(left, right);
}

bool completeRotation(bool spinMode, bool backwardPivot, bool turnRight,
                      uint8_t lastCommandSpeed, uint8_t stopPullMs) {
  if (stopPullMs > 0u && lastCommandSpeed > 0u) {
    int left = 0;
    int right = 0;
    rotationMotorCommands(spinMode, backwardPivot, turnRight, lastCommandSpeed,
                          left, right);
    motorMotion(-left, -right);
    delay(stopPullMs);
  }
  motor(1, 1);
  return true;
}

// Closed-loop controller shared by turn_gyro and the rotate APIs.
// targetRawYaw is unwrapped so crossings of +/-180 degrees remain continuous.
bool rotateToRawYaw(float targetRawYaw, uint8_t speed, int8_t leftRatio,
                      int8_t rightRatio, uint32_t timeoutMs,
                      float maxBrakeLeadDeg, uint8_t stopPullMs,
                      uint8_t pivotKind, float kp, float kd,
                      bool settleAfterStop, uint16_t slowdownMs = 0,
                      uint8_t slowSpeed = 0) {
  if (!imu.update() || !isfinite(imu.yawRaw()) ||
      !isfinite(targetRawYaw) || timeoutMs == 0u) {
    motor(1, 1);
    return false;
  }
  const int baseDirection = leftRatio > rightRatio ? 1 : -1;
  if (leftRatio == rightRatio) {
    motor(1, 1);
    return false;
  }
  const uint32_t startedAtMs = millis();
  uint32_t lastProgressAtMs = startedAtMs;
  uint32_t previousAtUs = micros();
  float previousYaw = imu.yawRaw();
  const int initialDirection = targetRawYaw >= previousYaw ? 1 : -1;
  float filteredRate = 0.0f;
  float previousError = fabsf(targetRawYaw - previousYaw);
  uint8_t corrections = 0;
  uint8_t lastCommandSpeed = 0;
  int lastLeftCommand = 0;
  int lastRightCommand = 0;
  bool firstBrake = true;
  bool slowdownActive = false;
  uint32_t slowdownStartedAtMs = 0;

  for (;;) {
    const uint32_t nowMs = millis();
    if (static_cast<uint32_t>(nowMs - startedAtMs) >= timeoutMs ||
        !imu.update() || !isfinite(imu.yawRaw())) {
      motor(1, 1);
      return false;
    }
    const float yaw = imu.yawRaw();
    const uint32_t nowUs = micros();
    const float dt = static_cast<float>(nowUs - previousAtUs) * 1.0e-6f;
    const float delta = yaw - previousYaw;
    if (fabsf(delta) > 30.0f) {
      motor(1, 1);
      return false;
    }
    if (dt >= 0.002f && dt < 0.2f) {
      filteredRate = 0.60f * filteredRate + 0.40f * (delta / dt);
    }
    previousYaw = yaw;
    previousAtUs = nowUs;
    const float error = targetRawYaw - yaw;
    const float absError = fabsf(error);
    if (previousError - absError >= 0.2f) {
      lastProgressAtMs = nowMs;
      previousError = absError;
    } else if (static_cast<uint32_t>(nowMs - lastProgressAtMs) >= 900u) {
      motor(1, 1);
      return false;
    }

    // Stopping distance depends on the measured rate; the caller's brake lead
    // is a cap, not a fixed early-stop angle.
    const float measuredRate = fmaxf(fabsf(filteredRate),
                                     fabsf(imu.gyro('z')));
    const float lead = fminf(maxBrakeLeadDeg, measuredRate * 0.035f);
    const float stopThreshold = !settleAfterStop && lastCommandSpeed == 0u
        ? GYRO_SETTLE_TOLERANCE_DEG
        : fmaxf(GYRO_SETTLE_TOLERANCE_DEG, lead);
    // Immediate-finish rotations are one-way. A gyro sample can jump across
    // the target without landing inside the stop window; brake instead of
    // reversing the motors and visibly swinging back.
    const bool crossedTarget = !settleAfterStop &&
        initialDirection * error <= 0.0f;
    if (crossedTarget || absError <= stopThreshold) {
      if (firstBrake && stopPullMs > 0u && lastCommandSpeed > 0u) {
        // Apply the opposite of the last motor command for the requested time.
        motorMotion(-lastLeftCommand, -lastRightCommand);
        delay(stopPullMs);
      }
      firstBrake = false;
      motor(1, 1);
      if (!settleAfterStop) return true;
      delay(GYRO_SETTLE_MS);
      if (!imu.update() || !isfinite(imu.yawRaw())) return false;
      const float settledError = targetRawYaw - imu.yawRaw();
      if (fabsf(settledError) <= GYRO_SETTLE_TOLERANCE_DEG) return true;
      if (++corrections >= GYRO_MAX_CORRECTIONS) return false;
      filteredRate = 0.0f;
      previousYaw = imu.yawRaw();
      previousAtUs = micros();
      previousError = fabsf(settledError);
      lastProgressAtMs = millis();
      continue;
    }

    // Begin the timed speed ramp when the measured rate predicts that the
    // target is within slowdownMs. Latch the phase so noisy rate samples
    // cannot raise the motor cap again near the target.
    const float rateTowardTarget = initialDirection * filteredRate;
    if (!slowdownActive && slowdownMs > 0u && lastCommandSpeed > 0u &&
        rateTowardTarget > 1.0f &&
        absError <= lead + rateTowardTarget * slowdownMs * 0.001f) {
      slowdownActive = true;
      slowdownStartedAtMs = nowMs;
    }

    const int direction = error > 0.0f ? 1 : -1;
    const int polarity = direction * baseDirection;
    uint8_t cappedSpeed = corrections == 0u ? speed : min(speed, (uint8_t)25);
    if (slowdownActive) {
      const float progress = smoothstep(
          static_cast<float>(nowMs - slowdownStartedAtMs) / slowdownMs);
      const uint8_t finalCap = min(speed, slowSpeed);
      const float rampCap = speed +
          (static_cast<float>(finalCap) - speed) * progress;
      cappedSpeed = min(cappedSpeed, static_cast<uint8_t>(lroundf(rampCap)));
    }
    const uint8_t minimumSpeed = min(cappedSpeed, (uint8_t)18);
    // P starts at the requested speed for a large error. D reduces the
    // command as measured yaw rate carries the robot toward its target.
    const float pdSpeed = kp * absError - kd * direction * filteredRate;
    const uint8_t commandSpeed = static_cast<uint8_t>(fminf(
        static_cast<float>(cappedSpeed),
        fmaxf(static_cast<float>(minimumSpeed), pdSpeed)));
    const int leftCommand = pivotKind == 1u
        ? (direction > 0 ? commandSpeed : 0)
        : (pivotKind == 2u ? (direction < 0 ? -commandSpeed : 0)
                            : polarity * commandSpeed * leftRatio / 100);
    const int rightCommand = pivotKind == 1u
        ? (direction < 0 ? commandSpeed : 0)
        : (pivotKind == 2u ? (direction > 0 ? -commandSpeed : 0)
                            : polarity * commandSpeed * rightRatio / 100);
    motorMotion(leftCommand, rightCommand);
    lastLeftCommand = leftCommand;
    lastRightCommand = rightCommand;
    lastCommandSpeed = commandSpeed;
    delay(ROTATE_LOOP_DELAY_MS);
  }
}

bool rotateSpinToHeadingWithGyro(float targetYaw, uint8_t speed,
                                 uint8_t stopPull) {
  if (!imu.update() || !isfinite(imu.yawRaw())) {
    motor(1, 1);
    return false;
  }
  const float startYaw = imu.yawRaw();
  float error = wrapAngle180(targetYaw - wrapAngle180(startYaw));
  if (fabsf(fabsf(error) - 180.0f) < 0.01f) {
    error = targetYaw < 0.0f ? -180.0f : 180.0f;
  }
  if (fabsf(error) <= GYRO_SETTLE_TOLERANCE_DEG) {
    motor(1, 1);
    return true;
  }
  const uint32_t expectedMs = static_cast<uint32_t>(
      rotateSpin90Ms * (fabsf(error) / 90.0f) *
      (static_cast<float>(rotateCalibrationSpeed) / speed));
  const uint32_t timeoutMs = max(2000UL, expectedMs * 3UL + 500UL);
  return rotateToRawYaw(startYaw + error, speed, 100, -100,
                        timeoutMs, TURN_GYRO_BRAKE_LEAD_DEG, stopPull,
                        0u, rotateGyroKp, rotateGyroKd, false,
                        rotateSpinSlowdownMs, rotateSpinSlowSpeed);
}

bool rotatePivotToHeadingWithGyro(bool backwardPivot, float targetYaw,
                                  uint8_t speed, uint8_t stopPull) {
  if (!imu.update() || !isfinite(imu.yawRaw())) {
    motor(1, 1);
    return false;
  }
  const float startYaw = imu.yawRaw();
  const float error = wrapAngle180(wrapAngle180(targetYaw) -
                                   wrapAngle180(startYaw));
  const uint32_t expectedMs = static_cast<uint32_t>(
      rotatePivot90Ms * (fabsf(error) / 90.0f) *
      (static_cast<float>(rotateCalibrationSpeed) / speed));
  const uint32_t timeoutMs = max(2000UL, expectedMs * 3UL + 500UL);
  return rotateToRawYaw(startYaw + error, speed,
                        backwardPivot ? 0 : 100,
                        backwardPivot ? -100 : 0,
                        timeoutMs, TURN_GYRO_BRAKE_LEAD_DEG, stopPull,
                         backwardPivot ? 2u : 1u,
                          rotateGyroKp, rotateGyroKd, backwardPivot);
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
  bool hasDriven = false;
  while (static_cast<uint32_t>(millis() - startedAtMs) < rotateTimeMs) {
    driveRotation(spinMode, backwardPivot, turnRight, speed);
    hasDriven = true;
  }
  return completeRotation(spinMode, backwardPivot, turnRight,
                          hasDriven ? speed : 0u, stopPull);
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
    const bool valuesValid = frontExit
        ? normalizedFront(pairFirst, firstValue, &firstBlack, true) &&
              normalizedFront(pairSecond, secondValue, nullptr, true)
        : normalizedRear(pairFirst, firstValue, &firstBlack, true) &&
              normalizedRear(pairSecond, secondValue, nullptr, true);
    if (!valuesValid) return false;
    white = firstValue >= LINE_EXIT_WHITE_RELEASE &&
            secondValue >= LINE_EXIT_WHITE_RELEASE;
    black = firstBlack;
    return true;
  }

  if (exitSensor == cl || exitSensor == cr) {
    uint16_t centerValue = 0;
    bool centerBlack = false;
    const uint8_t channel = exitSensor == cl ? 0u : 1u;
    if (!normalizedCenter(channel, centerValue, &centerBlack)) return false;
    white = centerValue >= LINE_EXIT_WHITE_RELEASE;
    black = centerBlack;
    return true;
  }

  return false;
}

// The approach trigger uses the same selected channel that will stop the turn,
// but reads the filtered value so all three approach channels share a frame.
bool readTurnApproachSelected(LineExitSensor exitSensor,
                              uint16_t& normalized, bool& black) {
  if (exitSensor >= f0 && exitSensor <= f15) {
    return normalizedFront(static_cast<uint8_t>(exitSensor) -
                               static_cast<uint8_t>(f0), normalized, &black);
  }
  if (exitSensor >= b0 && exitSensor <= b15) {
    return normalizedRear(static_cast<uint8_t>(exitSensor) -
                              static_cast<uint8_t>(b0), normalized, &black);
  }
  if (exitSensor == cl || exitSensor == cr) {
    return normalizedCenter(exitSensor == cl ? 0u : 1u, normalized, &black);
  }
  return false;
}

void resetTurnExit(TurnExitState& state) {
  state = TurnExitState{};
  state.lastFrameSequence = sensorArrays.frameSequence();
}

bool updateTurnExit(TurnExitState& state, bool white, bool black) {
  if (!state.exitArmed) {
    state.consecutiveWhiteSamples = white
        ? static_cast<uint8_t>(state.consecutiveWhiteSamples + 1u)
        : 0;
    if (state.consecutiveWhiteSamples < 2u) return false;
    state.exitArmed = true;
    return false;
  }

  state.blackHistory[state.blackHistoryIndex] = black;
  state.blackHistoryIndex = (state.blackHistoryIndex + 1u) % 3u;
  const uint8_t blackSamples = static_cast<uint8_t>(state.blackHistory[0]) +
      static_cast<uint8_t>(state.blackHistory[1]) +
      static_cast<uint8_t>(state.blackHistory[2]);
  return blackSamples >= 2u;
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
  if (stopPull == 0u || (leftCommand == 0 && rightCommand == 0)) {
    motor(1, 1);
    return;
  }

  // stopPull is the reverse-brake duration in milliseconds. Reverse the
  // actual command of each wheel, including its turn motor ratio.
  motorMotion(-leftCommand, -rightCommand);
  const uint32_t startedAtUs = micros();
  while (static_cast<uint32_t>(micros() - startedAtUs) <
         static_cast<uint32_t>(stopPull) * 1000UL) {
    sensorArrays.update(micros());
  }
  motor(1, 1);
}

void applyReverseBrake(bool backwardApproach, uint8_t power,
                       uint16_t durationMs, bool sampleSensors) {
  if (power == 0u || durationMs == 0u) {
    motor(1, 1);
    return;
  }

  const int brakeCommand = backwardApproach
      ? static_cast<int>(power) : -static_cast<int>(power);
  motorMotion(brakeCommand, brakeCommand);
  const uint32_t startedAtUs = micros();
  while (static_cast<uint32_t>(micros() - startedAtUs) <
         static_cast<uint32_t>(durationMs) * 1000UL) {
    if (sampleSensors) sensorArrays.update(micros());
  }
  motor(1, 1);
}

void applyTurnTouchBrake(bool backwardApproach, bool sampleSensors = true) {
  applyReverseBrake(backwardApproach, turnTouchBrake, turnTouchBrakeMs,
                    sampleSensors);
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
  lastFrontExitSensorMask = 0u;
  lastLineDirection = TravelDirection::none;
  lastLineEndedNormally = false;
  pendingLineHandoff = false;
  previousLineCommandContinuous = false;
}

void recordLineNormalExit(LastMotionType motionType, LineExitSource source,
                          TravelDirection direction, uint8_t stopPull,
                          uint16_t frontExitSensorMask = 0u) {
  lastMotionType = motionType;
  lastLineExitSource = source;
  lastFrontExitSensorMask = frontExitSensorMask;
  lastLineDirection = direction;
  lastLineEndedNormally = source != LineExitSource::none;
  pendingLineHandoff = stopPull == 0u;
}

void runLine(int sl, int sr, float kp, bool distanceMode,
             float targetDistanceMm, LineExitSensor exitSensor,
             uint8_t stopPull, bool reverseDirection,
             TravelDirection completedDirection, bool continuousEntry,
             bool requireBothExitSensors = false,
             LineExitSensor secondExitSensor = f0) {
  // Keep the selected target fixed throughout this blocking command.
  const int requestedPosition = positoin_error;
  const float targetPosition = requestedPosition >= 0 && requestedPosition <= 100
      ? static_cast<float>(requestedPosition - 50) : 0.0f;
  sl = constrain(sl, -100, 100);
  sr = constrain(sr, -100, 100);
  const uint32_t intersectionLimitUs = intersectionHoldLimitUs(sl, sr);
  if (reverseDirection ? !calibrationManager.rearValid()
                        : !calibrationManager.frontValid()) {
    stopLineWithError(reverseDirection, F("INVALID_SENSOR"), stopPull);
    return;
  }

  const uint32_t startedAtUs = micros();
  uint32_t previousPidAtUs = startedAtUs;
  float lastTrackedError = 0.0f;
  float lastMeasuredLineError = 0.0f;
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
  IntersectionGuard intersectionGuard;
  SideLineCrossing sideLineCrossing;
  float estimatedDistanceMm = 0.0f;
  bool continuousPidInitialized = !continuousEntry;

  const bool isFrontExit = exitSensor >= f0 && exitSensor <= f15;
  const bool isRearExit = exitSensor >= b0 && exitSensor <= b15;
  const bool isCenterLeftExit = exitSensor == cl;
  const bool isCenterRightExit = exitSensor == cr;
  if (requireBothExitSensors &&
      (distanceMode || exitSensor == secondExitSensor ||
       !(isFrontExit && secondExitSensor >= f0 && secondExitSensor <= f15) &&
       !(isRearExit && secondExitSensor >= b0 && secondExitSensor <= b15))) {
    stopLineWithError(reverseDirection, F("INVALID_SENSOR_PAIR"), stopPull);
    return;
  }
  uint8_t selectedIndex = 0;
  uint8_t pairFirst = 0;
  uint8_t pairSecond = 0;
  bool rearExit = false;
  bool centerExit = false;
  uint8_t centerChannel = 0;
  // A two-sensor AND is specific enough to check from the first frame.
  bool armed = requireBothExitSensors;
  bool firstExitFrame = true;
  uint8_t consecutiveWhiteSamples = 0;
  bool pairBlackHistory[3] = {false, false, false};
  uint8_t pairHistoryIndex = 0;
  bool crossingAfterExit = false;
  bool crossingWhiteHistory[3] = {false, false, false};
  uint8_t crossingWhiteHistoryIndex = 0;
  uint32_t lastTraceAtUs = startedAtUs;
  uint32_t lastPidTraceAtUs = startedAtUs;
  uint32_t lastErrorMonitorAtUs = startedAtUs;
  uint32_t previousTraceFrameAtUs = 0;
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
      pairSecond = requireBothExitSensors
          ? static_cast<uint8_t>(secondExitSensor) -
                static_cast<uint8_t>(isRearExit ? b0 : f0)
          : inwardExitCompanion(selectedIndex);
    }
  }

  // In a continuous handoff the motors are already moving. The last complete
  // frame can arm a white exit pair before the next sensor frame arrives.
  if (!armed && !reverseDirection && !distanceMode &&
      continuousEntry && sensorArrays.frameSequence() != 0u) {
    uint16_t firstValue = 0;
    uint16_t secondValue = 0;
    bool previousExitWhite = false;
    if (centerExit) {
      previousExitWhite = normalizedCenter(centerChannel, firstValue) &&
          firstValue >= LINE_EXIT_WHITE_RELEASE;
    } else if (rearExit) {
      previousExitWhite = normalizedRear(pairFirst, firstValue) &&
          normalizedRear(pairSecond, secondValue) &&
          firstValue >= LINE_EXIT_WHITE_RELEASE &&
          secondValue >= LINE_EXIT_WHITE_RELEASE;
    } else {
      previousExitWhite = normalizedFront(pairFirst, firstValue) &&
          normalizedFront(pairSecond, secondValue) &&
          firstValue >= LINE_EXIT_WHITE_RELEASE &&
          secondValue >= LINE_EXIT_WHITE_RELEASE;
    }
    armed = previousExitWhite;
    firstExitFrame = false;
  }

  const uint16_t frontExitSensorMask = isFrontExit && !reverseDirection
      ? static_cast<uint16_t>((1u << pairFirst) |
          (requireBothExitSensors ? (1u << pairSecond) : 0u))
      : 0u;

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
    const uint32_t traceFrameAtUs = lineIntersectionTraceEnabled
        ? sensorArrays.lastFrameMicros() : 0u;
    const uint32_t traceFrameDeltaUs = previousTraceFrameAtUs != 0u
        ? static_cast<uint32_t>(traceFrameAtUs - previousTraceFrameAtUs) : 0u;
    if (lineIntersectionTraceEnabled) previousTraceFrameAtUs = traceFrameAtUs;

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
      // An outer edge gives an unambiguous escape side. Otherwise keep the
      // last measured line position for recovery if the next frame is white.
      lastConfirmedEdgeSide = leftEdgeBlack == rightEdgeBlack
          ? 0 : (leftEdgeBlack ? -1 : 1);
      if (leftEdgeBlack && rightEdgeBlack) lastMeasuredLineError = 0.0f;
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
        exitBlackSample = requireBothExitSensors
            ? pairFirstBlack && pairSecondBlack
            : pairFirstBlack || pairSecondBlack;
      } else {
        if (!normalizedFront(pairFirst, pairFirstValue, &pairFirstBlack) ||
            !normalizedFront(pairSecond, pairSecondValue, &pairSecondBlack)) {
          stopLineWithError(reverseDirection, F("INVALID_SENSOR"), stopPull);
          return;
        }
        exitWhite = pairFirstValue >= LINE_EXIT_WHITE_RELEASE &&
                    pairSecondValue >= LINE_EXIT_WHITE_RELEASE;
        exitBlackSample = requireBothExitSensors
            ? pairFirstBlack && pairSecondBlack
            : pairFirstBlack || pairSecondBlack;
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
      if (crossingAfterExit) {
        // Keep the last tracking motor command until the selected exit pair
        // has cleared the line; offset zero must remain a continuous handoff.
        if (updateTurnApproachExit(exitWhite, crossingWhiteHistory,
                                   crossingWhiteHistoryIndex)) {
          recordLineNormalExit(LastMotionType::fLine,
                               exitSourceFor(exitSensor), completedDirection,
                               stopPull, frontExitSensorMask);
          previousLineCommandContinuous = true;
          printLineReason(false, F("SENSOR_BLACK"), stopPull);
          return;
        }
        continue;
      }
      if (!armed) {
        // f_line can arm on the first white frame before a normal start.
        // b_line keeps its existing 100 ms delay and two-white release.
        const bool mayArm = !reverseDirection ||
            static_cast<uint32_t>(nowUs - startedAtUs) >=
                EXIT_ARM_DELAY_US;
        if (mayArm) {
          consecutiveWhiteSamples = exitWhite
              ? static_cast<uint8_t>(consecutiveWhiteSamples + 1u)
              : 0u;
        }
        if (mayArm && exitWhite &&
            ((!reverseDirection && firstExitFrame) ||
             consecutiveWhiteSamples >= 2u)) {
          armed = true;
          pairBlackHistory[0] = false;
          pairBlackHistory[1] = false;
          pairBlackHistory[2] = false;
          pairHistoryIndex = 0;
        }
        firstExitFrame = false;
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
          if (!reverseDirection && isFrontExit && stopPull == 0u) {
            crossingAfterExit = true;
            continue;
          }
          recordLineNormalExit(
              reverseDirection ? LastMotionType::bLine : LastMotionType::fLine,
              exitSourceFor(exitSensor), completedDirection, stopPull,
              frontExitSensorMask);
          previousLineCommandContinuous = stopPull == 0u;
          stopAfterLineComplete(sl, sr, stopPull, reverseDirection);
          printLineReason(reverseDirection, F("SENSOR_BLACK"), stopPull);
          return;
        }
      }
    }

    // The chosen stop sensor was checked above. A two-sensor exit can protect
    // both sides; distance mode has no selected stop side.
    const bool checksLeftSide = !distanceMode &&
        ((centerExit && centerChannel == 0u) ||
         (!centerExit && (selectedIndex < 8u ||
                         (requireBothExitSensors && pairSecond < 8u))));
    const bool checksRightSide = !distanceMode &&
        ((centerExit && centerChannel == 1u) ||
         (!centerExit && (selectedIndex >= 8u ||
                         (requireBothExitSensors && pairSecond >= 8u))));
    const bool leftPattern = kp < SIDE_LINE_MAX_KP &&
        sideBlackRange(normalized, 2u, 7u) && sideWhitePair(normalized, 14u);
    const bool rightPattern = kp < SIDE_LINE_MAX_KP &&
        sideBlackRange(normalized, 9u, 15u) && sideWhitePair(normalized, 0u);
    const int8_t detectedSide =
        leftPattern && !checksLeftSide && !(rightPattern && checksRightSide)
            ? -1
            : rightPattern && !checksRightSide && !(leftPattern && checksLeftSide)
                ? 1 : 0;
    if (sideLineCrossing.activeSide == 0) {
      if (detectedSide == 0) {
        sideLineCrossing.pendingSide = 0;
        sideLineCrossing.pendingFrames = 0;
      } else {
        sideLineCrossing.pendingFrames =
            detectedSide == sideLineCrossing.pendingSide
                ? static_cast<uint8_t>(sideLineCrossing.pendingFrames + 1u)
                : 1u;
        sideLineCrossing.pendingSide = detectedSide;
        if (sideLineCrossing.pendingFrames >= SIDE_LINE_CONFIRM_FRAMES) {
          sideLineCrossing.activeSide = detectedSide;
          sideLineCrossing.lastVisibleSide = detectedSide;
          sideLineCrossing.startedAtUs = nowUs;
          sideLineCrossing.clearFrames = 0;
          intersectionGuard = IntersectionGuard{};
          integral = 0.0f;
          filteredDerivative = 0.0f;
        }
      }
    } else if (static_cast<uint32_t>(
                   nowUs - sideLineCrossing.startedAtUs) >=
               SIDE_LINE_MAX_CROSS_US) {
      stopLineWithError(reverseDirection, F("SIDE_LINE_TIMEOUT"), stopPull);
      return;
    }

    const uint32_t pidDeltaUs = static_cast<uint32_t>(nowUs - previousPidAtUs);
    const float distanceDt = static_cast<float>(pidDeltaUs) * 0.000001f;
    const float dt = constrain(distanceDt, PID_MIN_DT_SECONDS,
                               PID_MAX_DT_SECONDS);
    previousPidAtUs = nowUs;

    const float referenceError = lastTrackedError;
    const bool hadTrackedLine = trackingInitialized;
    const float pidPreviousBefore = lastPidError;
    float measuredError = lastTrackedError;
    uint8_t outsideBlack = 0;
    uint8_t selectedStart = 255u;
    uint8_t selectedEnd = 255u;
    const bool selectedLineFound = selectTrackedGroup(
        blackStrength, lastTrackedError, trackingInitialized, measuredError,
        outsideBlack,
        lineIntersectionTraceEnabled ? &selectedStart : nullptr,
        lineIntersectionTraceEnabled ? &selectedEnd : nullptr);
    bool lineFound = selectedLineFound;
    const float selectedMeasuredError = measuredError;
    bool sideLineReleased = false;
    bool activeSidePattern = false;
    if (sideLineCrossing.activeSide != 0) {
      uint8_t blackCount = 0;
      uint8_t leftBlackCount = 0;
      uint8_t rightBlackCount = 0;
      for (uint8_t i = 0; i < MyMINIConfig::SENSOR_COUNT; ++i) {
        if (blackStrength[i] == 0u) continue;
        ++blackCount;
        if (i < MyMINIConfig::SENSOR_COUNT / 2u) ++leftBlackCount;
        else ++rightBlackCount;
      }
      if (leftBlackCount > rightBlackCount) {
        sideLineCrossing.lastVisibleSide = -1;
      } else if (rightBlackCount > leftBlackCount) {
        sideLineCrossing.lastVisibleSide = 1;
      }
      activeSidePattern = sideLineCrossing.activeSide < 0
          ? leftPattern : rightPattern;
      const bool mainLineFound = selectedLineFound && blackCount > 0u &&
          blackCount <= INTERSECTION_MAX_NARROW_BLACK &&
          fabsf(measuredError - lastTrackedError) <= TRACK_WINDOW_RADIUS;
      sideLineCrossing.clearFrames = !activeSidePattern && mainLineFound
          ? static_cast<uint8_t>(sideLineCrossing.clearFrames + 1u) : 0u;
      if (sideLineCrossing.clearFrames >= SIDE_LINE_CLEAR_FRAMES) {
        sideLineCrossing = SideLineCrossing{};
        intersectionGuard = IntersectionGuard{};
        sideLineReleased = true;
      }
    }
    const bool crossingSideLine = sideLineCrossing.activeSide != 0;
    const bool recoveringSideLine = crossingSideLine && !anyConfirmedBlack;
    const IntersectionGuard guardBefore = intersectionGuard;
    IntersectionGuardObservation guardObservation;
    const IntersectionGuardResult intersectionResult = crossingSideLine
        ? IntersectionGuardResult::Follow
        : updateIntersectionGuard(
              intersectionGuard, blackStrength, lastTrackedError, measuredError,
              lineFound, outsideBlack, nowUs, intersectionLimitUs,
              lineIntersectionTraceEnabled ? &guardObservation : nullptr);
    const bool crossingIntersection =
        intersectionResult == IntersectionGuardResult::Hold;
    if (crossingSideLine) {
      measuredError = lastTrackedError;
      lineFound = anyConfirmedBlack;
      recoveryFrameCount = 0;
    } else if (crossingIntersection) {
      measuredError = intersectionGuard.heldError;
      lineFound = true;
      recoveryFrameCount = 0;
    }
    bool recoveredOutsideWindow = false;
    if (!crossingIntersection && !lineFound && trackingInitialized &&
        outsideBlack > 0u) {
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
      if (!crossingIntersection &&
          !(blackStrength[0] &&
            blackStrength[MyMINIConfig::SENSOR_COUNT - 1u])) {
        lastMeasuredLineError = measuredError;
      }
      if (crossingIntersection) {
        lastTrackedError = intersectionGuard.heldError;
      } else if (recoveredOutsideWindow) {
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
    // Search toward the last measured line side when all sensors turn white.
    // An outer edge takes priority even if the tracked group was rejected.
    const float linePosition = recoveringSideLine
        ? static_cast<float>(sideLineCrossing.lastVisibleSide) * 50.0f
        : crossingSideLine ? lastTrackedError : !anyConfirmedBlack
        ? lineLostRecoveryError(kp, trackingInitialized,
                                lastConfirmedEdgeSide, lastMeasuredLineError)
        : (!lineFound && kp > 0.5f && lastConfirmedEdgeSide != 0
            ? static_cast<float>(lastConfirmedEdgeSide) * 50.0f
            : lastTrackedError);
    // Tracking and recovery keep using the sensor's real -50..50 coordinate.
    // Only the normal PID setpoint moves; a lost line still steers toward the
    // last side that saw black with its original recovery strength.
    const float error = lineFound && !crossingSideLine && !crossingIntersection
        ? linePosition - targetPosition : linePosition;
    if (crossingSideLine || sideLineReleased || crossingIntersection ||
        (intersectionResult == IntersectionGuardResult::Released && lineFound)) {
      lastPidError = error;
      filteredDerivative = 0.0f;
    }
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
    if (lineFound && !hadTrackedLine && targetPosition != 0.0f) {
      // The setpoint offset must not cause a D kick on the first line frame.
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
    const bool recoveringKnownSide = kp > 0.5f && trackingInitialized &&
        fabsf(error) >= 10.0f && (!lineFound || lastConfirmedEdgeSide != 0);
    // Do not weaken a known-direction recovery during the startup ramp.
    const float accelFactor = lineStartupFactor(
        elapsedMs, sl, sr, continuousEntry, recoveringKnownSide);
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
      decelFactor = distanceLineDecelFactor(
          remainingMm, targetDistanceMm, sl, sr, stopPull);
    }
    const float motionFactor = distanceMode
                                   ? min(accelFactor, decelFactor)
                                   : accelFactor;

    const float pidPreviousUsed = lastPidError;
    // On all-white ground, pivot toward the last side that saw black.
    // Otherwise countersteer only while the side pattern remains visible.
    const float correction = recoveringSideLine
        ? static_cast<float>(sideLineCrossing.lastVisibleSide) *
              static_cast<float>(max(abs(sl), abs(sr)))
        : crossingSideLine
        ? (activeSidePattern
               ? (sideLineCrossing.activeSide < 0
                      ? SIDE_LINE_STEER_CORRECTION
                      : -SIDE_LINE_STEER_CORRECTION)
               : 0.0f)
        : calculateLinePidCorrection(
              kp * lineKpScale, error, dt, lineFound && !crossingIntersection,
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
    if (lineIntersectionTraceEnabled) {
      LineIntersectionTraceFrame* trace = nextLineIntersectionTraceFrame(
          reverseDirection ? 'B' : 'F', sensorArrays.frameSequence(),
          traceFrameAtUs, traceFrameDeltaUs, pidDeltaUs, normalized,
          blackStrength, blackStrength);
      if (trace) {
        trace->referenceError = referenceError;
        trace->measuredError = selectedMeasuredError;
        trace->trackedError = lastTrackedError;
        trace->error = error;
        trace->pidPreviousBefore = pidPreviousBefore;
        trace->pidPreviousUsed = pidPreviousUsed;
        trace->proportional = kp * lineKpScale * error;
        trace->integral = lineKi * integral;
        trace->derivative = lineKd * filteredDerivative;
        trace->correction = correction;
        trace->motionFactor = motionFactor;
        trace->groupStart = selectedStart;
        trace->groupEnd = selectedEnd;
        trace->outsideBlack = outsideBlack;
        trace->selectedLineFound = selectedLineFound;
        trace->finalLineFound = lineFound;
        trace->guardResult = static_cast<uint8_t>(intersectionResult);
        trace->stableBefore = guardBefore.stableFrames;
        trace->candidateBefore = guardBefore.candidateFrames;
        trace->confirmedBefore = guardBefore.confirmed;
        trace->stableAfter = intersectionGuard.stableFrames;
        trace->candidateAfter = intersectionGuard.candidateFrames;
        trace->confirmedAfter = intersectionGuard.confirmed;
        trace->clearAfter = intersectionGuard.clearFrames;
        trace->heldError = crossingIntersection
            ? intersectionGuard.heldError : guardBefore.heldError;
        const uint32_t guardStartedAtUs = intersectionGuard.startedAtUs != 0u
            ? intersectionGuard.startedAtUs : guardBefore.startedAtUs;
        trace->guardAgeUs = guardStartedAtUs != 0u
            ? static_cast<uint32_t>(nowUs - guardStartedAtUs) : 0u;
        trace->observation = guardObservation;
        const int signedLeft = reverseDirection ? -leftCommand : leftCommand;
        const int signedRight = reverseDirection ? -rightCommand : rightCommand;
        trace->leftMotor = signedLeft == 1 ? 2 : signedLeft == -1 ? -2 : signedLeft;
        trace->rightMotor = signedRight == 1 ? 2 : signedRight == -1 ? -2 : signedRight;
        finishLineIntersectionTraceFrame(*trace);
      }
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
      // Distance uses real elapsed time; the bounded PID dt would undercount
      // travel whenever a sensor frame takes longer than 50 ms.
      estimatedDistanceMm += estimatedLineDistanceStepMm(
          leftCommand, rightCommand, pidDeltaUs);
      if (estimatedDistanceMm >= targetDistanceMm) {
        recordLineNormalExit(
            reverseDirection ? LastMotionType::bLine : LastMotionType::fLine,
            LineExitSource::distance, completedDirection, stopPull);
        stopAfterLineComplete(sl, sr, stopPull, reverseDirection);
        printLineReason(reverseDirection, F("DISTANCE"), stopPull);
        return;
      }
    }

    if (lineFound && !crossingSideLine) lastPidError = error;
  }
}

// Absolute-heading variant of turn(). Keep its approach phases independent of
// the sensor-exit implementation above so the existing turn() path is intact.
void runTurnGyro(TurnMode mode, uint8_t speed, float targetDeg,
                 uint8_t brakeLeadDeg) {
  uint8_t modeIndex = 0;
  const bool flFrMode = turnModeUsesTimedApproach(mode);
  const TravelDirection approachDirection = lastLineEndedNormally
      ? lastLineDirection : TravelDirection::forward;
  const LineExitSource previousExitSource = lastLineExitSource;
  const bool hasHandoff = pendingLineHandoff;
  const bool approachByDistance = previousExitSource == LineExitSource::distance;
  const uint8_t approachSpeed = approachByDistance
      ? turnDistanceExitSpeed : turnSensorExitSpeed;
  const bool stoppedOnFrontExit = flFrMode && lastLineEndedNormally &&
      approachDirection == TravelDirection::forward &&
      previousExitSource == LineExitSource::front && !hasHandoff;
  const bool rotateImmediately = flFrMode && lastLineEndedNormally &&
      ((approachDirection == TravelDirection::forward &&
        previousExitSource == LineExitSource::front && hasHandoff) ||
        (approachDirection == TravelDirection::backward &&
         previousExitSource == LineExitSource::rear));
  clearLastFLineState(LastMotionType::turn);

  if (!turnModeIndex(mode, modeIndex) || speed == 0u || speed > 100u ||
      turnTimeoutMs == 0u || brakeLeadDeg > 100u) {
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
  if (!imu.update() || !isfinite(imu.yawRaw())) {
    failTurnGyro(F("GYRO_READ_FAILED"));
    return;
  }

  const int gyroDirection = turnHeadingDirection(mode);
  const float brakeLead = brakeLeadDeg == 0u
      ? TURN_GYRO_BRAKE_LEAD_DEG : static_cast<float>(brakeLeadDeg);
  TurnGyroTarget initialTarget;
  const TurnGyroResult initialResult = initialTarget.begin(
      imu.yawRaw(), targetDeg, gyroDirection);
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
  if (approach && !stoppedOnFrontExit &&
      (!approachTriggerCalibrationValid ||
       !approachTrackingCalibrationValid)) {
    failTurnGyro(F("APPROACH_CALIBRATION_NOT_READY"));
    return;
  }

  enum class TurnPhase : uint8_t {
    SearchOuter, Overshoot, ApproachExit, Rotate
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

  TurnPhase phase = TurnPhase::Rotate;
  if (flFrMode && !rotateImmediately)
    phase = stoppedOnFrontExit ? TurnPhase::Overshoot : TurnPhase::SearchOuter;
  else if (approach) phase = TurnPhase::ApproachExit;

  for (;;) {
    const uint32_t nowMs = millis();
    const uint32_t nowUs = micros();
    const uint32_t timeoutStartedAtMs = flFrMode ? phaseStartedAtMs : startedAtMs;
    if (TurnGyroTarget::timedOut(nowMs, timeoutStartedAtMs, turnTimeoutMs)) {
      failTurnGyro(F("TIMEOUT"));
      return;
    }

    if (phase == TurnPhase::Rotate) {
      if (!imu.update() || !isfinite(imu.yawRaw())) {
        failTurnGyro(F("GYRO_READ_FAILED"));
        return;
      }
      const float current = imu.yawRaw();
      float error = wrapAngle180(targetDeg - wrapAngle180(current));
      if (fabsf(fabsf(error) - 180.0f) < 0.01f)
        error = static_cast<float>(gyroDirection) * 180.0f;
      const uint32_t elapsed = static_cast<uint32_t>(nowMs - timeoutStartedAtMs);
      const uint32_t remainingMs = turnTimeoutMs - elapsed;
      if (!rotateToRawYaw(current + error, speed, ratio.left, ratio.right,
                           remainingMs, brakeLead, 0, 0u,
                           TURN_GYRO_KP, 0.0f, true)) {
        failTurnGyro(F("HEADING_NOT_REACHED"));
      }
      return;
    }
    if (phase == TurnPhase::Overshoot) {
      motorMotion(approachMotorCommand, approachMotorCommand);
      if (static_cast<uint32_t>(nowMs - phaseStartedAtMs) < turnOvershootMs) {
        continue;
      }
      applyTurnTouchBrake(backwardApproach, false);
      phase = TurnPhase::Rotate;
      phaseStartedAtMs = nowMs;
      continue;
    }

    const bool sensorFrame = sensorArrays.update(nowUs);
    if (!sensorFrame) continue;
    if (!imu.update() || !isfinite(imu.yawRaw())) {
      failTurnGyro(F("GYRO_READ_FAILED"));
      return;
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
        applyTurnTouchBrake(backwardApproach, false);
        phase = TurnPhase::Rotate;
        phaseStartedAtMs = nowMs;
        continue;
      }
    }

    uint16_t normalized[MyMINIConfig::SENSOR_COUNT] = {};
    bool blackByChannel[MyMINIConfig::SENSOR_COUNT] = {};
    uint16_t blackStrength[MyMINIConfig::SENSOR_COUNT] = {};
    bool approachSensorsReady = true;
    for (uint8_t channel = 0; channel < MyMINIConfig::SENSOR_COUNT; ++channel) {
      if (!normalizedTrackingSensor(backwardApproach, channel,
                                    normalized[channel], &blackByChannel[channel])) {
        approachSensorsReady = false;
        break;
      }
      int32_t strength = 1000 - normalized[channel];
      if (strength < LINE_BLACK_NOISE_THRESHOLD) strength = 0;
      blackStrength[channel] = static_cast<uint16_t>(strength);
    }
    if (!approachSensorsReady) {
      // Keep the heading-based turn available when an approach channel fails.
      phase = TurnPhase::Rotate;
      phaseStartedAtMs = nowMs;
      continue;
    }
    if (phase == TurnPhase::SearchOuter) {
      const bool outerEdgeBlack =
          blackByChannel[0] || blackByChannel[1] ||
          blackByChannel[14] || blackByChannel[15];
      if (updateTurnApproachExit(outerEdgeBlack, outerBlackHistory,
                                 outerBlackHistoryIndex)) {
        // Confirm the crossing before PID can skip a frame with no selected line.
        phase = TurnPhase::Overshoot;
        phaseStartedAtMs = nowMs;
        motorMotion(approachMotorCommand, approachMotorCommand);
        continue;
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
    const float motionFactor = 1.0f;  // Approach at the configured speed now.
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

    if (lineFound) lastPidError = error;
  }
}

} // namespace

void set_line_intersection_trace(bool enabled) {
  lineIntersectionTraceEnabled = enabled;
  if (!enabled) return;
  lineDiagnosticsEnabled = false;
  lineErrorMonitorEnabled = false;
  lineIntersectionTraceFrozen = false;
  lineIntersectionTraceTriggered = false;
  lineIntersectionTracePostFrames = 0;
  lineIntersectionTraceNext = 0;
  lineIntersectionTraceCount = 0;
  lineIntersectionTraceHasPriorError = false;
  lineIntersectionTracePriorError = 0.0f;
}

void print_line_intersection_trace(Print& output) {
  output.print(F("MYMINI_TRACE v=1 count="));
  output.print(lineIntersectionTraceCount);
  output.print(F(" triggered="));
  output.print(lineIntersectionTraceTriggered ? 1 : 0);
  output.print(F(" frozen="));
  output.println(lineIntersectionTraceFrozen ? 1 : 0);
  output.println(F("guard_result: 0=Follow 1=Hold 2=Released; group 255:255=none; command F=f_line B=b_line L=tcl R=tcr"));
  output.println(F("pre=stable:candidate:confirmed post=stable:candidate:confirmed:clear pattern=black:anchor:leftOutside:rightOutside:side(-1=left,1=right,2=both):narrow:branch bs=tracked gbs=guard"));
  const uint16_t first = static_cast<uint16_t>(
      (lineIntersectionTraceNext + LINE_INTERSECTION_TRACE_CAPACITY -
       lineIntersectionTraceCount) % LINE_INTERSECTION_TRACE_CAPACITY);
  for (uint16_t row = 0; row < lineIntersectionTraceCount; ++row) {
    const LineIntersectionTraceFrame& f = lineIntersectionTrace[
        (first + row) % LINE_INTERSECTION_TRACE_CAPACITY];
    output.print(F("TRACE cmd=")); output.print(static_cast<char>(f.command));
    output.print(F(" seq=")); output.print(f.frameSequence);
    output.print(F(" frame_us=")); output.print(f.frameAtUs);
    output.print(F(" frame_dt_us=")); output.print(f.frameDeltaUs);
    output.print(F(" pid_dt_us=")); output.print(f.pidDeltaUs);
    output.print(F(" group=")); output.print(f.groupStart);
    output.print(':'); output.print(f.groupEnd);
    output.print(F(" outside=")); output.print(f.outsideBlack);
    output.print(F(" selected=")); output.print(f.selectedLineFound);
    output.print(F(" final=")); output.print(f.finalLineFound);
    output.print(F(" ref=")); output.print(f.referenceError, 3);
    output.print(F(" measured=")); output.print(f.measuredError, 3);
    output.print(F(" tracked=")); output.print(f.trackedError, 3);
    output.print(F(" error=")); output.print(f.error, 3);
    output.print(F(" pid_prev_before=")); output.print(f.pidPreviousBefore, 3);
    output.print(F(" pid_prev_used=")); output.print(f.pidPreviousUsed, 3);
    output.print(F(" P=")); output.print(f.proportional, 3);
    output.print(F(" I=")); output.print(f.integral, 3);
    output.print(F(" D=")); output.print(f.derivative, 3);
    output.print(F(" correction=")); output.print(f.correction, 3);
    output.print(F(" factor=")); output.print(f.motionFactor, 3);
    output.print(F(" motor=")); output.print(f.leftMotor);
    output.print(','); output.print(f.rightMotor);
    output.print(F(" guard=")); output.print(f.guardResult);
    output.print(F(" pre=")); output.print(f.stableBefore);
    output.print(':'); output.print(f.candidateBefore);
    output.print(':'); output.print(f.confirmedBefore);
    output.print(F(" post=")); output.print(f.stableAfter);
    output.print(':'); output.print(f.candidateAfter);
    output.print(':'); output.print(f.confirmedAfter);
    output.print(':'); output.print(f.clearAfter);
    output.print(F(" held=")); output.print(f.heldError, 3);
    output.print(F(" guard_age_us=")); output.print(f.guardAgeUs);
    output.print(F(" pattern=")); output.print(f.observation.blackCount);
    output.print(':'); output.print(f.observation.anchorCount);
    output.print(':'); output.print(f.observation.leftOutside);
    output.print(':'); output.print(f.observation.rightOutside);
    output.print(':'); output.print(f.observation.branchSide);
    output.print(':'); output.print(f.observation.narrowMainLine ? 1 : 0);
    output.print(':'); output.print(f.observation.branchShape ? 1 : 0);
    output.print(F(" n="));
    for (uint8_t i = 0; i < MyMINIConfig::SENSOR_COUNT; ++i) {
      if (i) output.print(';');
      output.print(f.normalized[i]);
    }
    output.print(F(" bs="));
    for (uint8_t i = 0; i < MyMINIConfig::SENSOR_COUNT; ++i) {
      if (i) output.print(';');
      output.print(f.blackStrength[i]);
    }
    output.print(F(" gbs="));
    for (uint8_t i = 0; i < MyMINIConfig::SENSOR_COUNT; ++i) {
      if (i) output.print(';');
      output.print(f.guardStrength[i]);
    }
    output.println();
  }
  output.println(F("MYMINI_TRACE_END"));
}

void set_line_diagnostics(bool enabled) {
  lineDiagnosticsEnabled = enabled;
  if (enabled) {
    lineErrorMonitorEnabled = false;
    lineIntersectionTraceEnabled = false;
  }
}

void set_line_error_monitor(bool enabled) {
  lineErrorMonitorEnabled = enabled;
  if (enabled) {
    lineDiagnosticsEnabled = false;
    lineIntersectionTraceEnabled = false;
  }
}

bool set_line_ramp_ms(uint16_t rampMs) {
  if (rampMs > LINE_TIMEOUT_MS) return false;
  lineRampMs = rampMs;
  return true;
}

bool set_line_ramp_start_speed(int startSpeed) {
  if (startSpeed < 0 || startSpeed > 100) return false;
  lineRampStartSpeed = startSpeed;
  return true;
}

bool set_line_distance_scale(float scale) {
  if (!isfinite(scale) || scale <= 0.0f || scale > 5.0f) return false;
  lineDistanceScale = scale;
  return true;
}

bool set_line_decel_ramp_cm(int distanceCmTimes100) {
  if (distanceCmTimes100 < 0) return false;
  // One integer unit is 0.01 cm, or 0.1 mm.
  lineDecelRampMm = static_cast<float>(distanceCmTimes100) * 0.1f;
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

bool set_gyro_kd(float forwardKd, float backwardKd) {
  if (!isfinite(forwardKd) || !isfinite(backwardKd) ||
      forwardKd < 0.0f || forwardKd > 10.0f ||
      backwardKd < 0.0f || backwardKd > 10.0f) return false;
  forwardGyroKd = forwardKd;
  backwardGyroKd = backwardKd;
  return true;
}

bool set_gyro_kd(float kd) {
  return set_gyro_kd(kd, kd);
}

// -----------------------------------------------------------------------------
// Public f_line API
// -----------------------------------------------------------------------------

void f_line(int sl, int sr, float kp, LineExitSensor exitSensor,
             uint8_t stopPull) {
  ResetLinePositionOnReturn resetPosition;
  const bool continuousEntry = previousLineCommandContinuous;
  clearLastFLineState(LastMotionType::fLine);
  runLine(sl, sr, kp, false, 0.0f, exitSensor, stopPull, false,
          TravelDirection::forward, continuousEntry);
}

void f_line(int sl, int sr, float kp, LineExitPair exitSensors,
             uint8_t stopPull) {
  ResetLinePositionOnReturn resetPosition;
  const bool continuousEntry = previousLineCommandContinuous;
  clearLastFLineState(LastMotionType::fLine);
  runLine(sl, sr, kp, false, 0.0f, exitSensors.first, stopPull, false,
          TravelDirection::forward, continuousEntry, true, exitSensors.second);
}

void f_line(int sl, int sr, float kp, float distanceCm,
             uint8_t stopPull) {
  ResetLinePositionOnReturn resetPosition;
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
  ResetLinePositionOnReturn resetPosition;
  const bool continuousEntry = previousLineCommandContinuous;
  clearLastFLineState(LastMotionType::bLine);
  if (sl < 0 || sr < 0 || sl > 100 || sr > 100) {
    stopLineWithError(true, F("INVALID_ARGUMENT"), stopPull);
    return;
  }
  runLine(sl, sr, kp, false, 0.0f, exitSensor, stopPull, true,
          TravelDirection::backward, continuousEntry);
}

void b_line(int sl, int sr, float kp, float distanceCm,
             uint8_t stopPull) {
  ResetLinePositionOnReturn resetPosition;
  const bool continuousEntry = previousLineCommandContinuous;
  clearLastFLineState(LastMotionType::bLine);
  if (sl < 0 || sr < 0 || sl > 100 || sr > 100 || distanceCm <= 0.0f) {
    stopLineWithError(true, F("INVALID_ARGUMENT"), stopPull);
    return;
  }
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

  if (!imuReady || !headingReferenceReady) {
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

  if (!isfinite(imu.yawRaw())) {
    fail();
    return;
  }

  clearLastFLineState(LastMotionType::fLine);
  const float targetDistanceMm = distanceCm * 10.0f;
  const uint32_t startedAtUs = micros();
  uint32_t previousAtUs = startedAtUs;
  float estimatedDistanceMm = 0.0f;
  float previousYaw = imu.yawRaw();
  float filteredYawRate = 0.0f;

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

    const float rawYaw = imu.yawRaw();
    if (!isfinite(rawYaw)) {
      fail();
      return;
    }
    const float currentYaw = wrapAngle180(rawYaw);
    const float error = wrapAngle180(targetAngle - currentYaw);

    const float dt = static_cast<float>(nowUs - previousAtUs) * 0.000001f;
    previousAtUs = nowUs;
    if (dt >= PID_MIN_DT_SECONDS && dt <= PID_MAX_DT_SECONDS) {
      filteredYawRate = 0.7f * filteredYawRate +
          0.3f * (wrapAngle180(rawYaw - previousYaw) / dt);
    }
    previousYaw = rawYaw;

    if (estimatedDistanceMm >= targetDistanceMm) {
      recordLineNormalExit(LastMotionType::fLine, LineExitSource::distance,
                           TravelDirection::forward, stopPull);
      if (stopPull == 0u) return;
      motorMotion(-static_cast<int>(speed), -static_cast<int>(speed));
      delay(stopPull);
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
    const float correction = FW_GYRO_CORRECTION_SIGN *
        (kp * error - forwardGyroKd * filteredYawRate);
    const float leftCommand = constrain(currentBaseSpeed - correction,
                                        0.0f, 100.0f);
    const float rightCommand = constrain(currentBaseSpeed + correction,
                                         0.0f, 100.0f);
    const int leftOutput = static_cast<int>(leftCommand);
    const int rightOutput = static_cast<int>(rightCommand);
    motorMotion(leftOutput, rightOutput);

    const float forwardCommand =
        (static_cast<float>(leftOutput) + rightOutput) * 0.5f;
    estimatedDistanceMm += LINE_MM_PER_SECOND_AT_100 * lineDistanceScale *
        (forwardCommand / 100.0f) * dt;
    if (estimatedDistanceMm >= targetDistanceMm) {
      recordLineNormalExit(LastMotionType::fLine, LineExitSource::distance,
                           TravelDirection::forward, stopPull);
      if (stopPull == 0u) return;
      motorMotion(-static_cast<int>(speed), -static_cast<int>(speed));
      delay(stopPull);
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

  if (!imuReady || !headingReferenceReady) {
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

  if (!isfinite(imu.yawRaw())) {
    fail();
    return;
  }

  clearLastFLineState(LastMotionType::bLine);
  const float targetDistanceMm = distanceCm * 10.0f;
  const uint32_t startedAtUs = micros();
  uint32_t previousAtUs = startedAtUs;
  float estimatedDistanceMm = 0.0f;
  float previousYaw = imu.yawRaw();
  float filteredYawRate = 0.0f;

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

    const float rawYaw = imu.yawRaw();
    if (!isfinite(rawYaw)) {
      fail();
      return;
    }
    const float currentYaw = wrapAngle180(rawYaw);
    const float error = wrapAngle180(targetAngle - currentYaw);

    const float dt = static_cast<float>(nowUs - previousAtUs) * 0.000001f;
    previousAtUs = nowUs;
    if (dt >= PID_MIN_DT_SECONDS && dt <= PID_MAX_DT_SECONDS) {
      filteredYawRate = 0.7f * filteredYawRate +
          0.3f * (wrapAngle180(rawYaw - previousYaw) / dt);
    }
    previousYaw = rawYaw;

    if (estimatedDistanceMm >= targetDistanceMm) {
      recordLineNormalExit(LastMotionType::bLine, LineExitSource::distance,
                           TravelDirection::backward, stopPull);
      if (stopPull == 0u) return;
      motorMotion(static_cast<int>(speed), static_cast<int>(speed));
      delay(stopPull);
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
    const float correction = FW_GYRO_CORRECTION_SIGN *
        (kp * error - backwardGyroKd * filteredYawRate);
    const float leftCommand = constrain(currentBaseSpeed - correction,
                                        -100.0f, 0.0f);
    const float rightCommand = constrain(currentBaseSpeed + correction,
                                         -100.0f, 0.0f);
    const int leftOutput = static_cast<int>(leftCommand);
    const int rightOutput = static_cast<int>(rightCommand);
    motorMotion(leftOutput, rightOutput);

    const float backwardCommand =
        -(static_cast<float>(leftOutput) + rightOutput) * 0.5f;
    estimatedDistanceMm += LINE_MM_PER_SECOND_AT_100 * lineDistanceScale *
        (backwardCommand / 100.0f) * dt;
    if (estimatedDistanceMm >= targetDistanceMm) {
      recordLineNormalExit(LastMotionType::bLine, LineExitSource::distance,
                           TravelDirection::backward, stopPull);
      if (stopPull == 0u) return;
      motorMotion(static_cast<int>(speed), static_cast<int>(speed));
      delay(stopPull);
      motor(1, 1);
      return;
    }
  }
}

// -----------------------------------------------------------------------------
// Public rotate API
// -----------------------------------------------------------------------------

bool rotate_spin(uint8_t speed, float angleDeg, uint8_t stopPull) {
  if (!validRotateArguments(angleDeg, speed, stopPull, true) ||
      fabsf(angleDeg) > 180.0f || !imuReady || !headingReferenceReady) {
    motor(1, 1);
    return false;
  }
  return rotateSpinToHeadingWithGyro(angleDeg, speed, stopPull);
}

bool rotateFW_pivot(uint8_t speed, float angleDeg, uint8_t stopPull) {
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

bool rotateBW_pivot(uint8_t speed, float angleDeg, uint8_t stopPull) {
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

bool set_turn_center_brake(uint8_t forwardPower, uint16_t forwardMs,
                           uint8_t backwardPower, uint16_t backwardMs) {
  if (forwardPower > 100u || backwardPower > 100u) return false;
  turnCenterForwardBrakePower = forwardPower;
  turnCenterForwardBrakeMs = forwardMs;
  turnCenterBackwardBrakePower = backwardPower;
  turnCenterBackwardBrakeMs = backwardMs;
  turnCenterBrakeConfigured = true;
  return true;
}

bool set_rotate_pid(float kp, float kd) {
  if (!isfinite(kp) || !isfinite(kd) || kp <= 0.0f || kp > 10.0f ||
      kd < 0.0f || kd > 10.0f) {
    return false;
  }
  rotateGyroKp = kp;
  rotateGyroKd = kd;
  return true;
}

bool set_rotate_spin_slowdown(uint16_t slowdownMs, uint8_t slowSpeed) {
  if (slowSpeed < 2u || slowSpeed > 100u) return false;
  rotateSpinSlowdownMs = slowdownMs;
  rotateSpinSlowSpeed = slowSpeed;
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

void turn_gyro(TurnMode mode, uint8_t speed, float targetDeg,
               uint8_t brakeLeadDeg) {
  runTurnGyro(mode, speed, targetDeg, brakeLeadDeg);
}

void turn(TurnMode mode, uint8_t speed, LineExitSensor exitSensor,
          uint8_t stopPull) {
  uint8_t modeIndex = 0;
  const bool flFrMode = turnModeUsesTimedApproach(mode);
  const TravelDirection approachDirection = lastLineEndedNormally
      ? lastLineDirection : TravelDirection::forward;
  const LineExitSource previousExitSource = lastLineExitSource;
  const uint16_t previousFrontExitSensorMask = lastFrontExitSensorMask;
  const bool hasHandoff = pendingLineHandoff;
  const bool approachByDistance =
      previousExitSource == LineExitSource::distance;
  const uint8_t approachSpeed =
      previousExitSource == LineExitSource::distance
          ? turnDistanceExitSpeed : turnSensorExitSpeed;
  // A stopped forward f_line has already confirmed the exit line. Continue
  // across it until the outer sensors return to white; do not search for black
  // a second time. Offset zero crossed the line inside f_line itself.
  const bool stoppedOnFrontExit = flFrMode && lastLineEndedNormally &&
      approachDirection == TravelDirection::forward &&
      previousExitSource == LineExitSource::front && !hasHandoff;
  const bool rotateImmediately = flFrMode &&
      lastLineEndedNormally &&
      ((approachDirection == TravelDirection::forward &&
        previousExitSource == LineExitSource::front && hasHandoff) ||
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
  const uint32_t intersectionLimitUs =
      intersectionHoldLimitUs(approachSpeed, approachSpeed);
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
  uint32_t previousTraceFrameAtUs = 0;
  float lastTrackedError = 0.0f;
  float lastPidError = 0.0f;
  float integral = 0.0f;
  float filteredDerivative = 0.0f;
  bool trackingInitialized = false;
  bool approachLineWasLost = false;
  TurnExitState turnExitState;
  IntersectionGuard intersectionGuard;
  bool leftOuterBlackHistory[3] = {false, false, false};
  uint8_t leftOuterBlackHistoryIndex = 0;
  bool rightOuterBlackHistory[3] = {false, false, false};
  uint8_t rightOuterBlackHistoryIndex = 0;
  bool outerWhiteHistory[3] = {false, false, false};
  uint8_t outerWhiteHistoryIndex = 0;
  // Bit 0: f0/f1 (or b0/b1); bit 1: f14/f15 (or b14/b15).
  // A prior f_line front exit keeps the existing all-four release check.
  uint8_t confirmedOuterSide = 0u;
  bool outerLineWasDetected = stoppedOnFrontExit;
  int appliedTurnLeftCommand = 0;
  int appliedTurnRightCommand = 0;

  TurnPhase phase = TurnPhase::Rotate;
  if (flFrMode && !rotateImmediately) {
    phase = stoppedOnFrontExit ? TurnPhase::CrossOuter
                              : TurnPhase::SearchOuter;
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
    const uint32_t traceFrameAtUs = lineIntersectionTraceEnabled
        ? sensorArrays.lastFrameMicros() : 0u;
    const uint32_t traceFrameDeltaUs = previousTraceFrameAtUs != 0u
        ? static_cast<uint32_t>(traceFrameAtUs - previousTraceFrameAtUs) : 0u;
    if (lineIntersectionTraceEnabled) previousTraceFrameAtUs = traceFrameAtUs;

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
      // Ignore the departure line until the motors have actually rotated.
      // White arming and black confirmation both start after this interval.
      if (elapsedTurnMs >= TURN_EXIT_MIN_ROTATION_MS &&
          updateTurnExit(turnExitState, exitWhite, exitBlack)) {
        stopAfterTurnComplete(appliedTurnLeftCommand,
                              appliedTurnRightCommand, stopPull);
        return;
      }
      motorMotion(turnLeftCommand, turnRightCommand);
      appliedTurnLeftCommand = turnLeftCommand;
      appliedTurnRightCommand = turnRightCommand;
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
      // Include the f_line stop sensor(s) when crossing a confirmed front
      // exit. All required channels must be white in the same frame.
      bool selectedExitWhite = true;
      if (stoppedOnFrontExit) {
        for (uint8_t channel = 0; channel < MyMINIConfig::SENSOR_COUNT; ++channel) {
          if ((previousFrontExitSensorMask & (1u << channel)) == 0u) continue;
          uint16_t selectedValue = 0;
          if (!normalizedTrackingSensor(false, channel, selectedValue)) {
            motor(1, 1);
            return;
          }
          if (selectedValue < LINE_EXIT_WHITE_RELEASE) selectedExitWhite = false;
        }
      }
      uint16_t turnSelectedValue = 0;
      if (confirmedOuterSide != 0u) {
        bool turnSelectedBlack = false;
        if (!readTurnApproachSelected(exitSensor, turnSelectedValue,
                                      turnSelectedBlack)) {
          motor(1, 1);
          return;
        }
      }
      // Release the same edge pair that found black. If both pairs found
      // black together, wait for both. A prior f_line exit uses all four.
      const bool leftPairWhite = f0Value >= LINE_EXIT_WHITE_RELEASE &&
          f1Value >= LINE_EXIT_WHITE_RELEASE;
      const bool rightPairWhite = f14Value >= LINE_EXIT_WHITE_RELEASE &&
          f15Value >= LINE_EXIT_WHITE_RELEASE;
      const bool matchedEdgesWhite = confirmedOuterSide == 0u
          ? leftPairWhite && rightPairWhite
          : ((confirmedOuterSide & 1u) == 0u || leftPairWhite) &&
            ((confirmedOuterSide & 2u) == 0u || rightPairWhite);
      // Match f_line's white release: calibrated white can read around 630-660.
      const bool approachExitWhite = matchedEdgesWhite &&
          (confirmedOuterSide == 0u ||
           turnSelectedValue >= LINE_EXIT_WHITE_RELEASE) &&
          selectedExitWhite;
      if (outerLineWasDetected && updateTurnApproachExit(
              approachExitWhite, outerWhiteHistory, outerWhiteHistoryIndex)) {
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
      phaseStartedAtMs = millis();  // Exclude the blocking touch brake.
      resetTurnExit(turnExitState);
      appliedTurnLeftCommand = turnMotorCommand(speed, ratio.left);
      appliedTurnRightCommand = turnMotorCommand(speed, ratio.right);
      motorMotion(appliedTurnLeftCommand, appliedTurnRightCommand);
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
        // Brake opposite the approach direction. Keep the old shared power
        // and 10 ms pulse unless directional center-brake settings are used.
        const uint8_t brakePower = turnCenterBrakeConfigured
            ? (backwardApproach ? turnCenterBackwardBrakePower
                                : turnCenterForwardBrakePower)
            : turnTouchBrake;
        const uint16_t brakeMs = turnCenterBrakeConfigured
            ? (backwardApproach ? turnCenterBackwardBrakeMs
                                : turnCenterForwardBrakeMs)
            : 10u;
        applyReverseBrake(backwardApproach, brakePower, brakeMs, true);
        phase = TurnPhase::Rotate;
        phaseStartedAtMs = millis();
        resetTurnExit(turnExitState);
        appliedTurnLeftCommand = turnMotorCommand(speed, ratio.left);
        appliedTurnRightCommand = turnMotorCommand(speed, ratio.right);
        motorMotion(appliedTurnLeftCommand, appliedTurnRightCommand);
        continue;
      }
    }

    uint16_t normalized[MyMINIConfig::SENSOR_COUNT] = {};
    bool blackByChannel[MyMINIConfig::SENSOR_COUNT] = {};
    uint16_t blackStrength[MyMINIConfig::SENSOR_COUNT] = {};
    uint16_t confirmedBlackStrength[MyMINIConfig::SENSOR_COUNT] = {};
    for (uint8_t channel = 0; channel < MyMINIConfig::SENSOR_COUNT; ++channel) {
      if (!normalizedTrackingSensor(backwardApproach, channel,
                                    normalized[channel], &blackByChannel[channel])) {
        motor(1, 1);
        return;
      }
      int32_t strength = 1000 - normalized[channel];
      if (strength < LINE_BLACK_NOISE_THRESHOLD) strength = 0;
      blackStrength[channel] = static_cast<uint16_t>(strength);
      confirmedBlackStrength[channel] = normalized[channel] < 500u
          ? static_cast<uint16_t>(1000u - normalized[channel]) : 0u;
    }

    if (centerTurnMode && phase == TurnPhase::ApproachExit &&
        centerApproachShouldDriveStraight(normalized, blackByChannel)) {
      lastTrackedError = 0.0f;
      lastPidError = 0.0f;
      integral = 0.0f;
      filteredDerivative = 0.0f;
      intersectionGuard = IntersectionGuard{};
      lastLineSeenAtUs = nowUs;
      approachLineWasLost = true;
      motorMotion(approachMotorCommand, approachMotorCommand);
      continue;
    }

    const uint32_t pidDeltaUs = static_cast<uint32_t>(nowUs - previousPidAtUs);
    float dt = static_cast<float>(pidDeltaUs) * 0.000001f;
    dt = constrain(dt, PID_MIN_DT_SECONDS, PID_MAX_DT_SECONDS);
    previousPidAtUs = nowUs;

    const float referenceError = lastTrackedError;
    const float pidPreviousBefore = lastPidError;
    float measuredError = lastTrackedError;
    uint8_t outsideBlack = 0;
    uint8_t selectedStart = 255u;
    uint8_t selectedEnd = 255u;
    const bool selectedLineFound = selectTrackedGroup(
        blackStrength, lastTrackedError, trackingInitialized, measuredError,
        outsideBlack,
        lineIntersectionTraceEnabled && centerTurnMode ? &selectedStart : nullptr,
        lineIntersectionTraceEnabled && centerTurnMode ? &selectedEnd : nullptr);
    bool lineFound = selectedLineFound;
    const float selectedMeasuredError = measuredError;
    const IntersectionGuard guardBefore = intersectionGuard;
    IntersectionGuardObservation guardObservation;
    const IntersectionGuardResult intersectionResult = centerTurnMode &&
        phase == TurnPhase::ApproachExit
        ? updateIntersectionGuard(intersectionGuard, confirmedBlackStrength,
                                  lastTrackedError, measuredError, lineFound,
                                  outsideBlack, nowUs, intersectionLimitUs,
                                  lineIntersectionTraceEnabled
                                      ? &guardObservation : nullptr)
        : IntersectionGuardResult::Follow;
    const bool crossingIntersection =
        intersectionResult == IntersectionGuardResult::Hold;
    if (crossingIntersection) {
      measuredError = intersectionGuard.heldError;
      lineFound = true;
    }
    if (lineFound) {
      if (crossingIntersection) {
        lastTrackedError = intersectionGuard.heldError;
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
    if (crossingIntersection ||
        (intersectionResult == IntersectionGuardResult::Released && lineFound)) {
      lastPidError = error;
      filteredDerivative = 0.0f;
    }
    if (approachLineWasLost) {
      // Reacquisition starts a fresh derivative sample to avoid a D spike.
      lastPidError = error;
      filteredDerivative = 0.0f;
      approachLineWasLost = false;
    }
    const float pidPreviousUsed = lastPidError;
    const float correction = calculateLinePidCorrection(
        turnApproachKp, error, dt, lineFound && !crossingIntersection,
        backwardApproach ? -1 : 1,
        backwardApproach ? 0.0f : -100.0f, approachSpeedFloat,
        approachSpeedFloat, lastPidError, integral, filteredDerivative,
        LINE_KI, crossingIntersection);
    const float motionFactor = 1.0f;  // Approach at the configured speed now.
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
    if (lineIntersectionTraceEnabled && centerTurnMode &&
        phase == TurnPhase::ApproachExit) {
      LineIntersectionTraceFrame* trace = nextLineIntersectionTraceFrame(
          mode == TurnMode::cl ? 'L' : 'R', sensorArrays.frameSequence(),
          traceFrameAtUs, traceFrameDeltaUs, pidDeltaUs, normalized,
          blackStrength, confirmedBlackStrength);
      if (trace) {
        trace->referenceError = referenceError;
        trace->measuredError = selectedMeasuredError;
        trace->trackedError = lastTrackedError;
        trace->error = error;
        trace->pidPreviousBefore = pidPreviousBefore;
        trace->pidPreviousUsed = pidPreviousUsed;
        trace->proportional = turnApproachKp * error;
        trace->integral = LINE_KI * integral;
        trace->derivative = LINE_KD * filteredDerivative;
        trace->correction = correction;
        trace->motionFactor = motionFactor;
        trace->groupStart = selectedStart;
        trace->groupEnd = selectedEnd;
        trace->outsideBlack = outsideBlack;
        trace->selectedLineFound = selectedLineFound;
        trace->finalLineFound = lineFound;
        trace->guardResult = static_cast<uint8_t>(intersectionResult);
        trace->stableBefore = guardBefore.stableFrames;
        trace->candidateBefore = guardBefore.candidateFrames;
        trace->confirmedBefore = guardBefore.confirmed;
        trace->stableAfter = intersectionGuard.stableFrames;
        trace->candidateAfter = intersectionGuard.candidateFrames;
        trace->confirmedAfter = intersectionGuard.confirmed;
        trace->clearAfter = intersectionGuard.clearFrames;
        trace->heldError = crossingIntersection
            ? intersectionGuard.heldError : guardBefore.heldError;
        const uint32_t guardStartedAtUs = intersectionGuard.startedAtUs != 0u
            ? intersectionGuard.startedAtUs : guardBefore.startedAtUs;
        trace->guardAgeUs = guardStartedAtUs != 0u
            ? static_cast<uint32_t>(nowUs - guardStartedAtUs) : 0u;
        trace->observation = guardObservation;
        const int signedLeft = backwardApproach ? -leftCommand : leftCommand;
        const int signedRight = backwardApproach ? -rightCommand : rightCommand;
        trace->leftMotor = signedLeft == 1 ? 2 : signedLeft == -1 ? -2 : signedLeft;
        trace->rightMotor = signedRight == 1 ? 2 : signedRight == -1 ? -2 : signedRight;
        finishLineIntersectionTraceFrame(*trace);
      }
    }

    if (phase == TurnPhase::SearchOuter) {
      uint16_t turnSelectedValue = 0;
      bool turnSelectedBlack = false;
      if (!readTurnApproachSelected(exitSensor, turnSelectedValue,
                                    turnSelectedBlack)) {
        motor(1, 1);
        return;
      }
      // Require both sensors of one edge pair and the chosen turn-stop
      // sensor to see black in the same frame, confirmed 2 of 3 frames.
      const bool leftTripletBlack = blackByChannel[0] &&
          blackByChannel[1] && turnSelectedBlack;
      const bool rightTripletBlack = blackByChannel[14] &&
          blackByChannel[15] && turnSelectedBlack;
      const bool leftConfirmed = updateTurnApproachExit(
          leftTripletBlack, leftOuterBlackHistory,
          leftOuterBlackHistoryIndex);
      const bool rightConfirmed = updateTurnApproachExit(
          rightTripletBlack, rightOuterBlackHistory,
          rightOuterBlackHistoryIndex);
      if (leftConfirmed || rightConfirmed) {
        confirmedOuterSide = (leftConfirmed ? 1u : 0u) |
            (rightConfirmed ? 2u : 0u);
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


