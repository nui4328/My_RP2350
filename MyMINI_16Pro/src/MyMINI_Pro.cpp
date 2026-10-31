#include "MyMINI_Pro.h"
#include "MyMINI_ProInternal.h"

#include <BNO055.h>
#include <math.h>
#include <Servo.h>
#include <Wire.h>

DualMuxSensors sensorArrays;
I2CDevices i2cDevices;
MyMINIGyro imu;
bool imuReady = false;
bool headingReferenceReady = false;
TB6612Driver motorDriver;
CalibrationManager calibrationManager(i2cDevices, motorDriver, sensorArrays);
DiagnosticsConsole diagnostics(Serial, sensorArrays, i2cDevices, motorDriver,
                               calibrationManager);
namespace {
bool latestBatteryReadingValid = false;
bool sensorApiReady = false;
int16_t latestBatteryRaw = 0;
uint8_t latestBatteryLevel = 0;
int32_t latestBatteryMillivolts = 0;
constexpr uint16_t MUX_MIN_CAL_SPAN = 1;
uint8_t bno055Address = 0x29;
constexpr uint8_t BNO085_ADDRESS = 0x4B;
constexpr uint8_t BNO055_CHIP_ID = 0xA0;
constexpr uint32_t BNO055_STARTUP_I2C_HZ = 100000;
constexpr uint32_t BNO085_I2C_HZ = 100000;
constexpr uint32_t BNO085_I2C_TIMEOUT_MS = 25;

constexpr uint8_t SERVO_PINS[] = {18, 22, 28, 0, 1};
Servo servoChannels[sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0])];
bool servoAttached[sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0])] = {};
uint16_t servoMinPulseUs[sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0])] = {};
uint16_t servoMaxPulseUs[sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0])] = {};

int8_t servoChannelForPin(uint8_t pin) {
  for (uint8_t index = 0;
       index < sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0]); ++index) {
    if (SERVO_PINS[index] == pin) return static_cast<int8_t>(index);
  }
  return -1;
}  // namespace

}  // namespace

bool set_servo_pulse_range(uint8_t pin, uint16_t minPulseUs,
                           uint16_t maxPulseUs) {
  const int8_t channel = servoChannelForPin(pin);
  if (channel < 0 || minPulseUs < 500 || maxPulseUs > 2500 ||
      minPulseUs >= maxPulseUs) return false;
  const uint8_t index = static_cast<uint8_t>(channel);
  if (servoAttached[index]) return false;  // Set before the first servo() call.
  servoMinPulseUs[index] = minPulseUs;
  servoMaxPulseUs[index] = maxPulseUs;
  return true;
}

void servo(uint8_t pin, int angle) {
  const int8_t channel = servoChannelForPin(pin);
  if (channel < 0) return;

  const uint8_t index = static_cast<uint8_t>(channel);
  if (!servoAttached[index]) {
    if (servoMinPulseUs[index] != 0) {
      if (servoChannels[index].attach(pin, servoMinPulseUs[index],
                                      servoMaxPulseUs[index]) < 0) return;
    } else {
      if (servoChannels[index].attach(pin) < 0) return;
    }
    servoAttached[index] = true;
  }
  servoChannels[index].write(constrain(angle, 0, 180));
}

namespace {

enum class BatteryWarningState : uint8_t {
  Idle,
  FirstTone,
  InterBeepGap,
  SecondTone,
  CyclePause,
  ZeroTone,
  ZeroGap
};

BatteryWarningState batteryWarningState = BatteryWarningState::Idle;
uint8_t batteryWarningLevel = 0xFF;
uint32_t batteryWarningDeadlineMs = 0;
uint32_t batteryWarningCycleStartedAtMs = 0;
bool batteryWarningToneActive = false;


// à¸—à¸”à¸ªà¸­à¸šà¹‚à¸”à¸¢à¸¢à¸à¸¥à¹‰à¸­à¹ƒà¸«à¹‰à¸¥à¸­à¸¢à¸ˆà¸²à¸à¸žà¸·à¹‰à¸™à¸à¹ˆà¸­à¸™
//
// motor(20, 20);     // à¸¡à¸­à¹€à¸•à¸­à¸£à¹Œà¸—à¸±à¹‰à¸‡à¸ªà¸­à¸‡à¹€à¸”à¸´à¸™à¸«à¸™à¹‰à¸²
// delay(1000);
// motor(0, 0);
// delay(1000);
//
// motor(-20, -20);   // à¸¡à¸­à¹€à¸•à¸­à¸£à¹Œà¸—à¸±à¹‰à¸‡à¸ªà¸­à¸‡à¸–à¸­à¸¢à¸«à¸¥à¸±à¸‡
// delay(1000);
// motor(0, 0);

// à¹€à¸”à¸´à¸™à¸•à¸²à¸¡à¹€à¸ªà¹‰à¸™à¸”à¹‰à¸§à¸¢à¹€à¸‹à¸™à¹€à¸‹à¸­à¸£à¹Œà¸«à¸™à¹‰à¸²
// à¸­à¸­à¸à¸ˆà¸²à¸à¸¥à¸¹à¸›à¹à¸¥à¸°à¸«à¸¢à¸¸à¸”à¹€à¸¡à¸·à¹ˆà¸­ Front CH0 à¸žà¸šà¹€à¸ªà¹‰à¸™à¸”à¸³
// f_line(40, 40, 0.85f, f0);
// f_line(40, 40, 0.85f, 30.0f);  // à¹€à¸”à¸´à¸™à¸•à¸²à¸¡à¹€à¸ªà¹‰à¸™à¸›à¸£à¸°à¸¡à¸²à¸“ 30 cm

bool normalizeValue(int32_t raw, int32_t minValue, int32_t maxValue,
                    uint16_t& normalized) {
  if (maxValue <= minValue) return false;
  const int32_t mapped = static_cast<int32_t>(map(
      raw, minValue, maxValue, 0L, 1000L));
  normalized = static_cast<uint16_t>(constrain(mapped, 0L, 1000L));
  return true;
}

bool calibrationBounds(LineExitSensor sensor, int32_t& minRaw,
                       int32_t& maxRaw) {
  const CalibrationData& calibration = calibrationManager.data();
  bool groupValid = false;
  if (sensor >= f0 && sensor <= f15) {
    const uint8_t channel = static_cast<uint8_t>(sensor - f0);
    minRaw = calibration.frontMin[channel];
    maxRaw = calibration.frontMax[channel];
    groupValid = calibrationManager.frontValid();
  } else if (sensor >= b0 && sensor <= b15) {
    const uint8_t channel = static_cast<uint8_t>(sensor - b0);
    minRaw = calibration.rearMin[channel];
    maxRaw = calibration.rearMax[channel];
    groupValid = calibrationManager.rearValid();
  } else if (sensor == cl || sensor == cr) {
    const uint8_t channel = sensor == cl ? 0u : 1u;
    minRaw = calibration.centerMin[channel];
    maxRaw = calibration.centerMax[channel];
    groupValid = calibrationManager.centerValid();
  } else {
    return false;
  }
  return groupValid && maxRaw > minRaw;
}

bool readLineSensorFromFrame(LineExitSensor sensor,
                             LineSensorReading& reading) {
  reading = LineSensorReading{};
  if (sensor >= f0 && sensor <= f15) {
    const uint8_t channel = static_cast<uint8_t>(sensor - f0);
    reading.rawValid = sensorArrays.frameSequence() != 0;
    if (reading.rawValid) reading.raw = sensorArrays.frontRaw()[channel];
  } else if (sensor >= b0 && sensor <= b15) {
    const uint8_t channel = static_cast<uint8_t>(sensor - b0);
    reading.rawValid = sensorArrays.frameSequence() != 0;
    if (reading.rawValid) reading.raw = sensorArrays.rearRaw()[channel];
  } else if (sensor == cl || sensor == cr) {
    const uint8_t channel = sensor == cl ? 0u : 1u;
    const uint8_t adsChannel = channel == 0u
        ? MyMINIConfig::ADS_CHANNEL_CENTER_LEFT
        : MyMINIConfig::ADS_CHANNEL_CENTER_RIGHT;
    int16_t raw = 0;
    reading.rawValid = i2cDevices.readAds1115SingleEnded(adsChannel, raw);
    if (reading.rawValid) reading.raw = raw;
  } else {
    return false;
  }

  int32_t minRaw = 0;
  int32_t maxRaw = 0;
  reading.calibrationValid = calibrationBounds(sensor, minRaw, maxRaw);
  if (reading.calibrationValid) {
    reading.minRaw = minRaw;
    reading.maxRaw = maxRaw;
  }

  if (reading.rawValid && reading.calibrationValid) {
    reading.normalizedValid = normalizeValue(
        reading.raw, reading.minRaw, reading.maxRaw, reading.normalized);
  }
  return reading.rawValid;
}

void printLineSensorRow(Print& output, LineExitSensor sensor) {
  LineSensorReading reading;
  readLineSensorFromFrame(sensor, reading);
  if (sensor >= f0 && sensor <= f15) {
    output.print('F');
    output.print(static_cast<uint8_t>(sensor - f0));
  } else if (sensor >= b0 && sensor <= b15) {
    output.print('B');
    output.print(static_cast<uint8_t>(sensor - b0));
  } else {
    output.print(sensor == cl ? F("CL") : F("CR"));
  }
  output.print('\t');
  if (reading.rawValid) output.print(reading.raw);
  else output.print(F("ERR"));
  output.print('\t');
  if (reading.calibrationValid) output.print(reading.minRaw);
  else output.print(F("N/A"));
  output.print('\t');
  if (reading.calibrationValid) output.print(reading.maxRaw);
  else output.print(F("N/A"));
  output.print('\t');
  if (reading.normalizedValid) output.print(reading.normalized);
  else output.print(F("N/A"));
  output.print('\t');
  output.println(reading.calibrationValid ? F("YES") : F("NO"));
}

int32_t readNormalizedMux(uint8_t channel, bool rear) {
  if (!sensorApiReady || channel >= MyMINIConfig::SENSOR_COUNT) {
    return ADC_VALUE_INVALID;
  }
  const LineExitSensor sensor = static_cast<LineExitSensor>(
      (rear ? b0 : f0) + channel);
  int32_t minRaw = 0;
  int32_t maxRaw = 0;
  if (!calibrationBounds(sensor, minRaw, maxRaw)) return ADC_VALUE_INVALID;
  while (!sensorArrays.update(micros())) {}
  const uint16_t filtered = rear ? sensorArrays.rearFiltered()[channel]
                                 : sensorArrays.frontFiltered()[channel];
  uint16_t normalized = 0;
  if (!normalizeValue(filtered, minRaw, maxRaw, normalized)) {
    return ADC_VALUE_INVALID;
  }
  return normalized;
}

int32_t normalizedLimit(LineExitSensor sensor, bool upper) {
  if (!sensorApiReady) return ADC_VALUE_INVALID;
  int32_t minRaw = 0;
  int32_t maxRaw = 0;
  return calibrationBounds(sensor, minRaw, maxRaw)
      ? (upper ? 1000 : 0) : ADC_VALUE_INVALID;
}

bool frontRangesValid(const CalibrationData& calibration) {
  for (uint8_t i = 0; i < MyMINIConfig::SENSOR_COUNT; ++i) {
    const uint16_t minValue = calibration.frontMin[i];
    const uint16_t maxValue = calibration.frontMax[i];
    if (maxValue <= minValue ||
        (maxValue - minValue) < MUX_MIN_CAL_SPAN) {
      return false;
    }
  }
  return true;
}

bool rearRangesValid(const CalibrationData& calibration) {
  for (uint8_t i = 0; i < MyMINIConfig::SENSOR_COUNT; ++i) {
    const uint16_t minValue = calibration.rearMin[i];
    const uint16_t maxValue = calibration.rearMax[i];
    if (maxValue <= minValue ||
        (maxValue - minValue) < MUX_MIN_CAL_SPAN) {
      return false;
    }
  }
  return true;
}

bool centerRangesValid(const CalibrationData& calibration) {
  return calibration.centerMax[0] > calibration.centerMin[0] &&
         calibration.centerMax[1] > calibration.centerMin[1];
}

// Separate warning mode from the normal 0..8 LED level.
constexpr uint8_t BATTERY_WARNING_CRITICAL_LEVEL = 0xFE;
constexpr int32_t BATTERY_START_LOCK_MAX_MV = 10850;
constexpr int32_t BATTERY_ZERO_LED_MAX_MV = 11175;
constexpr uint32_t BATTERY_CRITICAL_STEP_MS = 100;

bool batteryStartBlocked() {
  return latestBatteryReadingValid &&
         latestBatteryMillivolts >= MyMINIConfig::BATTERY_WARNING_MIN_MV &&
         latestBatteryMillivolts <= BATTERY_START_LOCK_MAX_MV;
}

struct BatteryLedAnimation {
  bool critical = false;
  uint8_t phase = 0;
};

uint8_t batteryLedLevel(int32_t batteryMillivolts) {
  if (batteryMillivolts <= BATTERY_ZERO_LED_MAX_MV) return 0;
  const int32_t mapped = static_cast<int32_t>(map(
      batteryMillivolts, MyMINIConfig::BATTERY_LED_MIN_MV,
      MyMINIConfig::BATTERY_LED_MAX_MV, 0L, 8L));
  return static_cast<uint8_t>(constrain(mapped, 0L, 8L));
}

void stopBatteryWarning() {
  if (batteryWarningToneActive) noTone(MyMINIConfig::PIN_BUZZER);
  batteryWarningState = BatteryWarningState::Idle;
  batteryWarningLevel = 0xFF;
  batteryWarningToneActive = false;
}

void startBatteryWarningTone(uint16_t frequencyHz, uint32_t durationMs,
                             uint32_t nowMs) {
  tone(MyMINIConfig::PIN_BUZZER, frequencyHz);
  batteryWarningToneActive = true;
  batteryWarningDeadlineMs = nowMs + durationMs;
}

void startBatteryWarningPattern(uint8_t batteryLevel, uint32_t nowMs) {
  batteryWarningCycleStartedAtMs = nowMs;
  if (batteryLevel == BATTERY_WARNING_CRITICAL_LEVEL) {
    startBatteryWarningTone(MyMINIConfig::BATTERY_WARNING_ZERO_LED_HZ,
                            BATTERY_CRITICAL_STEP_MS, nowMs);
    batteryWarningState = BatteryWarningState::ZeroTone;
  } else if (batteryLevel == 2) {
    startBatteryWarningTone(MyMINIConfig::BATTERY_WARNING_TWO_LED_HZ, 50, nowMs);
    batteryWarningState = BatteryWarningState::FirstTone;
  } else if (batteryLevel == 1) {
    startBatteryWarningTone(MyMINIConfig::BATTERY_WARNING_ONE_LED_HZ, 180, nowMs);
    batteryWarningState = BatteryWarningState::FirstTone;
  } else {
    startBatteryWarningTone(MyMINIConfig::BATTERY_WARNING_ZERO_LED_HZ, 1000,
                            nowMs);
    batteryWarningState = BatteryWarningState::ZeroTone;
  }
}

void updateBatteryWarning(uint8_t batteryLevel, uint32_t nowMs) {
  if (batteryLevel >= 3 && batteryLevel != BATTERY_WARNING_CRITICAL_LEVEL) {
    stopBatteryWarning();
    return;
  }

  if (batteryWarningLevel != batteryLevel) {
    stopBatteryWarning();
    batteryWarningLevel = batteryLevel;
    startBatteryWarningPattern(batteryLevel, nowMs);
    return;
  }

  if (static_cast<int32_t>(nowMs - batteryWarningDeadlineMs) < 0) return;

  switch (batteryWarningState) {
    case BatteryWarningState::FirstTone:
      noTone(MyMINIConfig::PIN_BUZZER);
      batteryWarningToneActive = false;
      batteryWarningDeadlineMs = nowMs + 100;
      batteryWarningState = BatteryWarningState::InterBeepGap;
      break;
    case BatteryWarningState::InterBeepGap:
      startBatteryWarningTone(
          batteryLevel == 2 ? MyMINIConfig::BATTERY_WARNING_TWO_LED_HZ
                            : MyMINIConfig::BATTERY_WARNING_ONE_LED_HZ,
          batteryLevel == 2 ? 50 : 180, nowMs);
      batteryWarningState = BatteryWarningState::SecondTone;
      break;
    case BatteryWarningState::SecondTone:
      noTone(MyMINIConfig::PIN_BUZZER);
      batteryWarningToneActive = false;
      batteryWarningDeadlineMs = batteryWarningCycleStartedAtMs +
          (batteryLevel == 2 ? 2000U : 1000U);
      batteryWarningState = BatteryWarningState::CyclePause;
      break;
    case BatteryWarningState::CyclePause:
      startBatteryWarningPattern(batteryLevel, nowMs);
      break;
    case BatteryWarningState::ZeroTone:
      noTone(MyMINIConfig::PIN_BUZZER);
      batteryWarningToneActive = false;
      batteryWarningDeadlineMs = nowMs +
          (batteryLevel == BATTERY_WARNING_CRITICAL_LEVEL
               ? BATTERY_CRITICAL_STEP_MS : 1000U);
      batteryWarningState = BatteryWarningState::ZeroGap;
      break;
    case BatteryWarningState::ZeroGap:
      startBatteryWarningPattern(batteryLevel, nowMs);
      break;
    case BatteryWarningState::Idle:
      break;
  }
}

void updateBatteryLed(uint32_t nowMs, uint32_t& lastUpdateAtMs,
                      bool& hasWrittenMask, uint8_t& lastLogicalMask,
                      BatteryLedAnimation& animation) {
  const bool critical = batteryStartBlocked();
  const uint32_t interval = critical ? BATTERY_CRITICAL_STEP_MS
                                    : MyMINIConfig::BATTERY_LED_UPDATE_MS;
  if (critical != animation.critical) {
    animation.critical = critical;
    animation.phase = 0;
    hasWrittenMask = false;
    lastUpdateAtMs = nowMs - interval;  // Show mode changes immediately.
  }
  if (static_cast<uint32_t>(nowMs - lastUpdateAtMs) < interval) {
    return;
  }
  lastUpdateAtMs = nowMs;

  uint8_t logicalMask = 0;
  if (critical) {
    // One LED travels 0..7..0, without repeating an end position.
    const uint8_t position = animation.phase <= 7
                                 ? animation.phase : 14 - animation.phase;
    logicalMask = static_cast<uint8_t>(1U << position);
  } else if (latestBatteryReadingValid) {
    logicalMask = latestBatteryLevel == 0
                      ? 0x00U
                      : static_cast<uint8_t>(0xFFU << (8U - latestBatteryLevel));
  }

  if (hasWrittenMask && logicalMask == lastLogicalMask) return;

  if (i2cDevices.writePcf8574(logicalMask)) {
    lastLogicalMask = logicalMask;
    hasWrittenMask = true;
    if (critical) animation.phase = (animation.phase + 1U) % 14U;
  }
}

bool readBatteryVoltage() {
  int16_t raw = 0;
  latestBatteryReadingValid = i2cDevices.readAds1115SingleEnded(
      MyMINIConfig::ADS_CHANNEL_BATTERY, raw);
  if (!latestBatteryReadingValid) return false;
  latestBatteryRaw = raw;
  latestBatteryMillivolts = static_cast<int32_t>(
      i2cDevices.batteryVoltsFromRaw(raw) * 1000.0f + 0.5f);
  latestBatteryLevel = batteryLedLevel(latestBatteryMillivolts);
  return true;
}

void printSetupValues() {
  // Show the values used by line following; sensor_diag still shows true raw.
  const uint16_t* front = sensorArrays.frontFiltered();
  const uint16_t* rear = sensorArrays.rearFiltered();
  const CalibrationData& calibration = calibrationManager.data();
  uint16_t normalized = 0;

  Serial.print(F("F:"));
  if (!calibrationManager.frontValid() ||
      !frontRangesValid(calibration)) {
    Serial.print(F("NA"));
  } else {
    for (uint8_t channel = 0; channel < MyMINIConfig::SENSOR_COUNT; ++channel) {
      normalizeValue(static_cast<int32_t>(front[channel]),
                     static_cast<int32_t>(calibration.frontMin[channel]),
                     static_cast<int32_t>(calibration.frontMax[channel]),
                     normalized);
      if (channel > 0) Serial.print(',');
      Serial.print(normalized);
    }
  }

  Serial.print(F(" B:"));
  if (!calibrationManager.rearValid() ||
      !rearRangesValid(calibration)) {
    Serial.print(F("NA"));
  } else {
    for (uint8_t channel = 0; channel < MyMINIConfig::SENSOR_COUNT; ++channel) {
      normalizeValue(static_cast<int32_t>(rear[channel]),
                     static_cast<int32_t>(calibration.rearMin[channel]),
                     static_cast<int32_t>(calibration.rearMax[channel]),
                     normalized);
      if (channel > 0) Serial.print(',');
      Serial.print(normalized);
    }
  }

  int16_t raw = 0;
  Serial.print(F(" adcL:"));
  if (!calibrationManager.centerValid() || !centerRangesValid(calibration)) {
    Serial.print(F("NA"));
  } else if (!i2cDevices.readAds1115SingleEnded(
                 MyMINIConfig::ADS_CHANNEL_CENTER_LEFT, raw)) {
    Serial.print(F("ERR"));
  } else if (normalizeValue(static_cast<int32_t>(raw),
                            static_cast<int32_t>(calibration.centerMin[0]),
                            static_cast<int32_t>(calibration.centerMax[0]),
                            normalized)) {
    Serial.print(normalized);
  } else {
    Serial.print(F("NA"));
  }

  Serial.print(F(" adcR:"));
  if (!calibrationManager.centerValid() || !centerRangesValid(calibration)) {
    Serial.print(F("NA"));
  } else if (!i2cDevices.readAds1115SingleEnded(
                 MyMINIConfig::ADS_CHANNEL_CENTER_RIGHT, raw)) {
    Serial.print(F("ERR"));
  } else if (normalizeValue(static_cast<int32_t>(raw),
                            static_cast<int32_t>(calibration.centerMin[1]),
                            static_cast<int32_t>(calibration.centerMax[1]),
                            normalized)) {
    Serial.print(normalized);
  } else {
    Serial.print(F("NA"));
  }

  // wait_button() zeros once before this display loop. Read subsequent yaw
  // changes without repeatedly resetting or probing in estimation mode.
  Serial.print(F(" gyro:"));
  float yaw = 0.0f;
  float rateX = 0.0f;
  float rateY = 0.0f;
  float rateZ = 0.0f;
  if (gyro_estimation_active()) {
    Serial.print(F("EST"));
  } else if (!gyro_ready()) {
    Serial.print(F("NA"));
  } else if (gyro_read(yaw, rateX, rateY, rateZ)) {
    Serial.print(yaw, 2);
  } else {
    Serial.print(F("ERR"));
  }

  Serial.print(F(" vbat:"));
  if (readBatteryVoltage()) {
    Serial.print(i2cDevices.batteryVoltsFromRaw(latestBatteryRaw), 2);
  } else {
    Serial.print(F("ERR"));
  }
  Serial.println();
}

// Distinct confirmation for a loaded or newly saved BNO055 rate offset.
void playGyroCalibrationMelody() {
  const uint16_t notes[] = {1319, 1760, 2349};
  for (uint8_t i = 0; i < 3; ++i) {
    tone(MyMINIConfig::PIN_BUZZER, notes[i], 55);
    delay(70);
  }
  noTone(MyMINIConfig::PIN_BUZZER);
}

}  // namespace

void wait_button() {
  // Establish yaw zero once before the first sensor/gyro display.
  // The bounded reset also establishes estimated zero if real gyro is absent.
  // Keep it outside the display loop so rotations remain visible while waiting.
  gyro_reset();
  const bool gyroPrepared = gyro_start_ready();
  if (gyroPrepared && gyro_offset_calibrated()) playGyroCalibrationMelody();
  motor(0, 0);
  uint32_t lastPrintAtMs = millis();
  uint32_t lastBatteryLedUpdateAtMs = lastPrintAtMs;
  bool hasWrittenBatteryLedMask = false;
  uint8_t lastBatteryLedLogicalMask = 0;
  BatteryLedAnimation batteryLedAnimation;
  calibrationManager.startWaitButtonSignal(lastPrintAtMs);

  for (;;) {
    const uint32_t nowMs = millis();
    sensorArrays.update(micros());
    calibrationManager.update(nowMs);
    diagnostics.update(nowMs);
    const bool startRequested = calibrationManager.consumeShortPressEvent();
    // Check a fresh sample even if START arrives before the first periodic read.
    // Consume blocked presses, so voltage recovery requires a new START press.
    if (startRequested) readBatteryVoltage();
    if (startRequested && !batteryStartBlocked()) {
      // Every new start press establishes the shared heading origin used by
      // turn_gyro(), fw_gyro(), bw_gyro(), and absolute pivot rotations.
      motor(0, 0);
      stopBatteryWarning();
      i2cDevices.writePcf8574(0x00);
      calibrationManager.stopBuzzer();
      noTone(MyMINIConfig::PIN_BUZZER);
      tone(MyMINIConfig::PIN_BUZZER, 3500, 400);
      const uint32_t beepStartedAtMs = millis();

      // The START beep completes before the shared two-second reset budget.
      const uint32_t beepElapsedMs = millis() - beepStartedAtMs;
      if (beepElapsedMs < 500u) delay(500u - beepElapsedMs);
      if (gyroPrepared) {
        // START, rather than the earlier preparation, defines mission zero.
        // If the gyro has failed while waiting, bounded reset selects fallback.
        gyro_reset();
      } else {
        // Preparation already used its recovery budget. Do not retry here.
        motor(1, 1);
        headingReferenceReady = false;
        motion_heading_zero(false);
        Serial.println(F("START: estimated heading zero; mission may continue"));
      }
      return;
    }

    if (static_cast<uint32_t>(nowMs - lastPrintAtMs) >= 100U) {
      lastPrintAtMs = nowMs;
      printSetupValues();
    }

    const uint32_t batteryNowMs = millis();
    updateBatteryLed(batteryNowMs, lastBatteryLedUpdateAtMs,
                     hasWrittenBatteryLedMask, lastBatteryLedLogicalMask,
                     batteryLedAnimation);

    if (calibrationManager.isBusy()) {
      batteryWarningState = BatteryWarningState::Idle;
      batteryWarningLevel = 0xFF;
      batteryWarningToneActive = false;
    } else if (!latestBatteryReadingValid ||
               latestBatteryMillivolts < MyMINIConfig::BATTERY_WARNING_MIN_MV) {
      stopBatteryWarning();
    } else {
      updateBatteryWarning(batteryStartBlocked() ? BATTERY_WARNING_CRITICAL_LEVEL
                                                : latestBatteryLevel,
                           batteryNowMs);
    }
  }
  delay(300);
}

void robot_begin() {
  sensorApiReady = false;
  Serial.begin(MyMINIConfig::SERIAL_BAUD);

  pinMode(MyMINIConfig::PIN_BUZZER, OUTPUT);
  digitalWrite(MyMINIConfig::PIN_BUZZER, LOW);

  motorDriver.begin();        // Always stop outputs before other initialization.
  sensorArrays.begin();
  i2cDevices.begin();
  // Gyro initialization/recovery belongs to explicit gyro_reset()/gyro_set().
  imuReady = false;
  headingReferenceReady = false;
  calibrationManager.begin();
  diagnostics.begin();
  motor(0, 0);

  tone(MyMINIConfig::PIN_BUZZER, 1047);
  delay(90);
  noTone(MyMINIConfig::PIN_BUZZER);
  delay(30);
  tone(MyMINIConfig::PIN_BUZZER, 1319);
  delay(90);
  noTone(MyMINIConfig::PIN_BUZZER);
  delay(30);
  tone(MyMINIConfig::PIN_BUZZER, 1568);
  delay(90);
  noTone(MyMINIConfig::PIN_BUZZER);
  delay(30);
  tone(MyMINIConfig::PIN_BUZZER, 2093);
  delay(150);
  noTone(MyMINIConfig::PIN_BUZZER);
  delay(500);
  motor(0, 0);
  sensorApiReady = true;
}

void robot_update() {
  sensorArrays.update(micros());
  const uint32_t nowMs = millis();
  calibrationManager.update(nowMs);
  diagnostics.update(nowMs);
}

bool read_line_sensor(LineExitSensor sensor, LineSensorReading& reading) {
  reading = LineSensorReading{};
  if (!sensorApiReady || sensor > cr) return false;
  if (sensor <= b15) {
    while (!sensorArrays.update(micros())) {}
  }
  return readLineSensorFromFrame(sensor, reading);
}

int32_t readADC_F(uint8_t channel) {
  return readNormalizedMux(channel, false);
}

int32_t readADC_F() {
  if (!sensorApiReady) return ADC_VALUE_INVALID;
  int16_t raw = 0;
  return i2cDevices.readAds1115SingleEnded(MyMINIConfig::ADS_CHANNEL_AUX, raw)
      ? static_cast<int32_t>(raw) : ADC_VALUE_INVALID;
}

int32_t minADC_F(uint8_t channel) {
  return channel < MyMINIConfig::SENSOR_COUNT
      ? normalizedLimit(static_cast<LineExitSensor>(f0 + channel), false)
      : ADC_VALUE_INVALID;
}

int32_t maxADC_F(uint8_t channel) {
  return channel < MyMINIConfig::SENSOR_COUNT
      ? normalizedLimit(static_cast<LineExitSensor>(f0 + channel), true)
      : ADC_VALUE_INVALID;
}

int32_t readADC_B(uint8_t channel) {
  return readNormalizedMux(channel, true);
}

int32_t minADC_B(uint8_t channel) {
  return channel < MyMINIConfig::SENSOR_COUNT
      ? normalizedLimit(static_cast<LineExitSensor>(b0 + channel), false)
      : ADC_VALUE_INVALID;
}

int32_t maxADC_B(uint8_t channel) {
  return channel < MyMINIConfig::SENSOR_COUNT
      ? normalizedLimit(static_cast<LineExitSensor>(b0 + channel), true)
      : ADC_VALUE_INVALID;
}

int32_t readADC_CL() {
  LineSensorReading reading;
  return read_line_sensor(cl, reading) && reading.normalizedValid
      ? reading.normalized : ADC_VALUE_INVALID;
}

int32_t minADC_CL() { return normalizedLimit(cl, false); }
int32_t maxADC_CL() { return normalizedLimit(cl, true); }

int32_t readADC_CR() {
  LineSensorReading reading;
  return read_line_sensor(cr, reading) && reading.normalizedValid
      ? reading.normalized : ADC_VALUE_INVALID;
}

int32_t minADC_CR() { return normalizedLimit(cr, false); }
int32_t maxADC_CR() { return normalizedLimit(cr, true); }

void read_rowSensor() {
  if (!sensorApiReady) {
    Serial.println(F("Raw sensors unavailable: call robot_begin() first"));
    return;
  }
  // Complete one shared front/rear frame before printing either array.
  while (!sensorArrays.update(micros())) {}
  const uint16_t* front = sensorArrays.frontRaw();
  const uint16_t* rear = sensorArrays.rearRaw();

  Serial.print(F("F:"));
  for (uint8_t channel = 0; channel < MyMINIConfig::SENSOR_COUNT; ++channel) {
    if (channel > 0u) Serial.print(',');
    Serial.print(front[channel]);
  }
  Serial.print(F(" B:"));
  for (uint8_t channel = 0; channel < MyMINIConfig::SENSOR_COUNT; ++channel) {
    if (channel > 0u) Serial.print(',');
    Serial.print(rear[channel]);
  }

  int16_t raw = 0;
  Serial.print(F(" adcL:"));
  if (i2cDevices.readAds1115SingleEnded(MyMINIConfig::ADS_CHANNEL_CENTER_LEFT, raw))
    Serial.print(raw);
  else
    Serial.print(F("ERR"));
  Serial.print(F(" adcR:"));
  if (i2cDevices.readAds1115SingleEnded(MyMINIConfig::ADS_CHANNEL_CENTER_RIGHT, raw))
    Serial.print(raw);
  else
    Serial.print(F("ERR"));
  Serial.println();
}

void print_line_sensor_table(Print& output) {
  if (!sensorApiReady) {
    output.println(F("Sensor table unavailable: call robot_begin() first"));
    return;
  }
  while (!sensorArrays.update(micros())) {}
  output.println(F("Sensor\tnow_raw\tmin_raw\tmax_raw\tnormalized\tvalid"));
  for (uint8_t channel = 0; channel < MyMINIConfig::SENSOR_COUNT; ++channel) {
    printLineSensorRow(output, static_cast<LineExitSensor>(f0 + channel));
  }
  for (uint8_t channel = 0; channel < MyMINIConfig::SENSOR_COUNT; ++channel) {
    printLineSensorRow(output, static_cast<LineExitSensor>(b0 + channel));
  }
  printLineSensorRow(output, cl);
  printLineSensorRow(output, cr);
}

bool configure_sensor_reading(uint16_t muxSettleUs, uint8_t adcDiscardReads,
                              uint8_t adcAverageReads,
                              uint8_t smoothingDivisor) {
  return sensorArrays.configureRead(muxSettleUs, adcDiscardReads,
                                    adcAverageReads, smoothingDivisor);
}

bool gyro_ready() {
  return imuReady;
}

bool set_bno055_address(uint8_t address) {
  if (sensorApiReady || (address != 0x28 && address != 0x29)) return false;
  bno055Address = address;
  return true;
}

bool select_gyro(GyroModel model, int8_t yawSign, TwoWire& bno085Wire) {
  if (&bno085Wire != &Wire && &bno085Wire != &Wire1) return false;
  return imu.select(model, yawSign, bno085Wire);
}

GyroModel selected_gyro() {
  return imu.model();
}

bool gyro_start_ready() {
  return imuReady && headingReferenceReady && !gyro_estimation_active();
}

bool gyro_read(float& yawDeg, float& rateX, float& rateY, float& rateZ) {
  if (gyro_estimation_active() || !imuReady || !imu.update()) return false;
  const float yaw = imu.yaw();
  const float x = imu.gyro('x');
  const float y = imu.gyro('y');
  const float z = imu.gyro('z');
  if (!isfinite(yaw) || !isfinite(x) || !isfinite(y) || !isfinite(z))
    return false;
  yawDeg = yaw;
  rateX = x;
  rateY = y;
  rateZ = z;
  return true;
}

bool gyro_read_raw(float& yawDeg, float& rateX, float& rateY, float& rateZ) {
  if (!gyro_read(yawDeg, rateX, rateY, rateZ)) return false;
  yawDeg = imu.yawRaw();
  return isfinite(yawDeg);
}

namespace {
constexpr uint32_t GYRO_RESET_BUDGET_MS = 2000u;

uint32_t gyroResetRemaining(uint32_t startedMs) {
  const uint32_t elapsed = millis() - startedMs;
  return elapsed < GYRO_RESET_BUDGET_MS ? GYRO_RESET_BUDGET_MS - elapsed : 0u;
}

void playGyroReadyMelody() {
  const uint16_t notes[] = {2093, 2637, 3136};
  for (uint8_t i = 0; i < 3; ++i) {
    tone(MyMINIConfig::PIN_BUZZER, notes[i], 65);
    const uint32_t started = millis();
    while (static_cast<uint32_t>(millis() - started) < 80u) {
      sensorArrays.update(micros());
      // Service normal reports; never recover or change zero here.
      if (imu.model() == GyroModel::BNO085) imu.update();
      delay(5);
    }
  }
  noTone(MyMINIConfig::PIN_BUZZER);
}
}

bool gyro_reset() {
  motor(1, 1);
  headingReferenceReady = false;
  const uint32_t startedMs = millis();
  imu.setOperationBudget(startedMs, GYRO_RESET_BUDGET_MS);
  TwoWire& bus = imu.bus();
  const uint8_t address = imu.model() == GyroModel::BNO055
      ? bno055Address : BNO085_ADDRESS;
  bool busRestarted = false;
  bool verified = false;

  while (gyroResetRemaining(startedMs) >= 4u) {
    // Reinitialization is confined to this explicit reset call. Its driver
    // transactions, delays and SH2 loops share this same deadline.
    if (!imuReady) {
      if (!busRestarted) {
        bus.end();
#if defined(ARDUINO_ARCH_RP2040)
        if (&bus == &Wire1) { bus.setSDA(26); bus.setSCL(27); }
#endif
        bus.begin();
        busRestarted = true;
      }
      bus.setClock(100000);
      const uint32_t remaining = gyroResetRemaining(startedMs);
      if (remaining < 4u) break;
      bus.setTimeout(min(5u, remaining / 3u), true);
      bus.beginTransmission(address);
      if (bus.endTransmission() == 0 && gyroResetRemaining(startedMs) >= 4u)
        imuReady = imu.begin(address, bus);
      if (imuReady) imu.setLPF(0.75f);
    }
    if (imuReady && imu.update() && isfinite(imu.yawRaw()) &&
        gyroResetRemaining(startedMs) > 240u && imu.resetAngles()) {
      // Keep the robot still and consume reports after setting the new zero.
      const uint32_t settledAt = millis();
      while (static_cast<uint32_t>(millis() - settledAt) < 200u &&
             gyroResetRemaining(startedMs) >= 4u) {
        sensorArrays.update(micros());
        if (imu.model() == GyroModel::BNO085) imu.update();
        delay(min(5u, gyroResetRemaining(startedMs)));
      }
      verified = gyroResetRemaining(startedMs) >= 4u && imu.update() &&
          isfinite(imu.yawRaw()) && isfinite(imu.gyro('z')) &&
          gyroResetRemaining(startedMs) > 0u;
      if (verified) break;
    }
    // BNO055 fusion may establish an absolute heading shortly after NDOF
    // starts. Rebase only during explicit reset, then require the usual
    // settled verification; restarting the chip here repeats its startup.
    if (imuReady && imu.model() == GyroModel::BNO055 &&
        imu.lastUpdateError() == BNO055UpdateError::ImplausibleHeading &&
        gyroResetRemaining(startedMs) > 260u && imu.resetAngles()) {
      Serial.println(F("BNO055: heading changed during startup; verifying again"));
      delay(min(20u, gyroResetRemaining(startedMs)));
      continue;
    }
    // BNO085 may need a few reports after begin; allow them before reopening.
    if (imuReady && imu.model() == GyroModel::BNO085) {
      const uint32_t reportStarted = millis();
      while (static_cast<uint32_t>(millis() - reportStarted) < 100u &&
             gyroResetRemaining(startedMs) >= 4u) {
        if (imu.update() && isfinite(imu.yawRaw())) break;
        delay(min(5u, gyroResetRemaining(startedMs)));
      }
      if (gyroResetRemaining(startedMs) > 240u && imu.update() &&
          isfinite(imu.yawRaw())) continue;
    }
    imuReady = false;
    delay(min(20u, gyroResetRemaining(startedMs)));
  }
  // A sub-transaction guard may stop a few milliseconds before the deadline.
  if (!verified) delay(gyroResetRemaining(startedMs));
  imu.clearOperationBudget();
  bus.setTimeout(25, true);
  bus.setClock(imu.model() == GyroModel::BNO085 ? BNO085_I2C_HZ
                                              : MyMINIConfig::I2C_CLOCK_HZ);
  headingReferenceReady = verified;
  motion_heading_zero(verified);
  Serial.print(imu.model() == GyroModel::BNO055 ? F("BNO055: ") : F("BNO085: "));
  if (verified) {
    Serial.println(F("GYRO_READY heading zero; real gyro mode"));
    playGyroReadyMelody();
  } else {
    Serial.println(F("GYRO_ESTIMATED after 2000 ms; mission may continue"));
  }
  // Success means a motion reference is established, including estimation.
  // gyro_start_ready() remains false when no physical gyro is available.
  return motion_heading_ready();
}

bool gyro_set() {
  // Shared bounded preparation; usable real or estimated reference is success.
  return gyro_reset();
}

bool gyro_recover() {
  // Compatibility entry point delegates to the one bounded reset workflow.
  gyro_reset();
  return gyro_start_ready();
}

bool gyro_calibrate(bool forceNew) {
  if (!sensorApiReady || imu.model() != GyroModel::BNO055) return false;
  motor(1, 1);
  if (!gyro_start_ready()) {
    gyro_reset();
    if (!gyro_start_ready()) return false;
  }
  const bool calibrated = imu.calibrate(MyMINIConfig::PIN_BUZZER, forceNew);
  // Re-establish a shared motion heading reference even if calibration fails.
  gyro_reset();
  const bool success = calibrated && gyro_start_ready() && imu.gyroOffsetReady();
  if (success) playGyroCalibrationMelody();
  return success;
}

bool gyro_offset_calibrated() {
  return gyro_start_ready() && imu.gyroOffsetReady();
}

bool gyro_calibration_status(uint8_t& system, uint8_t& gyro,
                             uint8_t& accel, uint8_t& mag) {
  BNO055CalibrationStatus status;
  if (!imuReady || !imu.calibrationStatus(status)) return false;
  system = status.system;
  gyro = status.gyro;
  accel = status.accel;
  mag = status.mag;
  return true;
}

bool save_gyro_calibration() {
  motor(0, 0);
  return imuReady && imu.saveFusionCalibrationProfile();
}


