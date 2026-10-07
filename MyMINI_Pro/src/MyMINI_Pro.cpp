#include "MyMINI_Pro.h"
#include "MyMINI_ProInternal.h"

#include <BNO055.h>
#include <math.h>
#include <Servo.h>
#include <Wire.h>

DualMuxSensors sensorArrays;
I2CDevices i2cDevices;
BNO055 imu;
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
constexpr uint8_t BNO055_ADDRESS = 0x29;
constexpr uint8_t BNO055_CHIP_ID = 0xA0;
constexpr uint32_t BNO055_STARTUP_I2C_HZ = 100000;

constexpr uint8_t SERVO_PINS[] = {18, 22, 28, 0, 1};
Servo servoChannels[sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0])];
bool servoAttached[sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0])] = {};

int8_t servoChannelForPin(uint8_t pin) {
  for (uint8_t index = 0;
       index < sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0]); ++index) {
    if (SERVO_PINS[index] == pin) return static_cast<int8_t>(index);
  }
  return -1;
}  // namespace

}  // namespace

void servo(uint8_t pin, int angle) {
  const int8_t channel = servoChannelForPin(pin);
  if (channel < 0) return;

  const uint8_t index = static_cast<uint8_t>(channel);
  if (!servoAttached[index]) {
    servoChannels[index].attach(pin);
    servoAttached[index] = true;
  }
  servoChannels[index].write(constrain(angle, 0, 180));
}

namespace {

bool readBno055ChipId(uint8_t& chipId) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(BNO055_ADDRESS, static_cast<uint8_t>(1)) != 1 ||
      Wire.available() < 1) {
    return false;
  }
  chipId = static_cast<uint8_t>(Wire.read());
  return true;
}

void initializeBno055() {
  imuReady = false;
  headingReferenceReady = false;
  bool addressFound = false;
  bool validChipIdSeen = false;
  uint8_t invalidChipId = 0xFF;

  Wire.setClock(BNO055_STARTUP_I2C_HZ);
  delay(700);
  for (uint8_t attempt = 0; attempt < 3u; ++attempt) {
    Wire.beginTransmission(BNO055_ADDRESS);
    if (Wire.endTransmission() == 0) {
      addressFound = true;
      uint8_t chipId = 0xFF;
      if (readBno055ChipId(chipId) && chipId == BNO055_CHIP_ID) {
        validChipIdSeen = true;
        if (imu.begin(BNO055_ADDRESS, Wire)) {
          imu.setLPF(0.75f);
          if (!imu.calibrate(11, false)) {
            Serial.println(F("BNO055 host gyro offset unavailable; fusion heading remains usable"));
          }
          imuReady = true;
          BNO055CalibrationStatus status;
          if (imu.calibrationStatus(status)) {
            Serial.printf("BNO055 calibration SYS=%u GYR=%u ACC=%u MAG=%u\n",
                          status.system, status.gyro, status.accel, status.mag);
          }
          Serial.println(F("BNO055 Ready address=0x29 chipId=0xA0"));
          break;
        }
      } else {
        invalidChipId = chipId;
      }
    }
    if (attempt < 2u) delay(200);
  }
  Wire.setClock(MyMINIConfig::I2C_CLOCK_HZ);

  if (imuReady) return;

  if (!addressFound) {
    Serial.println(F("BNO055 address 0x29 not found"));
  } else if (!validChipIdSeen) {
    Serial.print(F("BNO055 invalid Chip ID: 0x"));
    if (invalidChipId < 0x10u) Serial.print('0');
    Serial.println(invalidChipId, HEX);
  } else {
    Serial.println(F("BNO055 begin failed after 3 attempts"));
  }
  motor(0, 0);
}

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

uint8_t batteryLedLevel(int32_t batteryMillivolts) {
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
  if (batteryLevel == 2) {
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
  if (batteryLevel >= 3) {
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
      batteryWarningDeadlineMs = nowMs + 1000;
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
                      bool& hasWrittenMask, uint8_t& lastLogicalMask) {
  if (static_cast<uint32_t>(nowMs - lastUpdateAtMs) <
      MyMINIConfig::BATTERY_LED_UPDATE_MS) {
    return;
  }
  lastUpdateAtMs = nowMs;

  uint8_t logicalMask = 0;
  if (latestBatteryReadingValid) {
    logicalMask = latestBatteryLevel == 0
                      ? 0x00U
                      : static_cast<uint8_t>(0xFFU << (8U - latestBatteryLevel));
  }

  if (hasWrittenMask && logicalMask == lastLogicalMask) return;

  if (i2cDevices.writePcf8574(logicalMask)) {
    lastLogicalMask = logicalMask;
    hasWrittenMask = true;
  }
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

  Serial.print(F(" vbat:"));
  if (i2cDevices.readAds1115SingleEnded(
          MyMINIConfig::ADS_CHANNEL_BATTERY, raw)) {
    latestBatteryRaw = raw;
    latestBatteryReadingValid = true;
    latestBatteryMillivolts = static_cast<int32_t>(
        i2cDevices.batteryVoltsFromRaw(raw) * 1000.0f + 0.5f);
    latestBatteryLevel = batteryLedLevel(latestBatteryMillivolts);
    Serial.print(i2cDevices.batteryVoltsFromRaw(raw), 2);
  } else {
    latestBatteryReadingValid = false;
    Serial.print(F("ERR"));
  }
  Serial.println();
}

}  // namespace

void wait_button() {
  headingReferenceReady = false;
  motor(0, 0);
  uint32_t lastPrintAtMs = millis();
  uint32_t lastBatteryLedUpdateAtMs = lastPrintAtMs;
  bool hasWrittenBatteryLedMask = false;
  uint8_t lastBatteryLedLogicalMask = 0;
  calibrationManager.startWaitButtonSignal(lastPrintAtMs);

  for (;;) {
    const uint32_t nowMs = millis();
    sensorArrays.update(micros());
    calibrationManager.update(nowMs);
    diagnostics.update(nowMs);

    if (calibrationManager.consumeShortPressEvent()) {
      // Every new start press establishes the shared heading origin used by
      // turn_gyro(), fw_gyro(), bw_gyro(), and absolute pivot rotations.
      motor(0, 0);
      stopBatteryWarning();
      i2cDevices.writePcf8574(0x00);
      calibrationManager.stopBuzzer();
      noTone(MyMINIConfig::PIN_BUZZER);
      tone(MyMINIConfig::PIN_BUZZER, 3500, 400);
      const uint32_t beepStartedAtMs = millis();

      // Save a complete fusion profile during the first 500 ms wait.
      BNO055CalibrationStatus gyroStatus;
      if (imuReady && imu.calibrationStatus(gyroStatus)) {
        Serial.printf("START gyro calib SYS=%u GYR=%u ACC=%u MAG=%u\n",
                      gyroStatus.system, gyroStatus.gyro,
                      gyroStatus.accel, gyroStatus.mag);
        if (gyroStatus.fullyCalibrated() &&
            !imu.saveFusionCalibrationProfile(false)) {
          Serial.println(F("BNO055 fusion profile save failed"));
        }
      }
      const uint32_t beepElapsedMs = millis() - beepStartedAtMs;
      if (beepElapsedMs < 500u) delay(500u - beepElapsedMs);

      // Zero yaw after the beep and the first wait, then hold still for 200 ms.
      if (imuReady && imu.update() && isfinite(imu.yawRaw()) &&
          imu.resetAngles()) {
        headingReferenceReady = true;
        Serial.println(F("START gyro heading ready"));
      } else {
        Serial.println(F("START heading reference unavailable: gyro not ready"));
      }
      motor(0, 0);
      delay(200);
      return;
    }

    if (static_cast<uint32_t>(nowMs - lastPrintAtMs) >= 100U) {
      lastPrintAtMs = nowMs;
      printSetupValues();
    }

    updateBatteryLed(nowMs, lastBatteryLedUpdateAtMs,
                     hasWrittenBatteryLedMask, lastBatteryLedLogicalMask);

    if (calibrationManager.isBusy()) {
      batteryWarningState = BatteryWarningState::Idle;
      batteryWarningLevel = 0xFF;
      batteryWarningToneActive = false;
    } else if (!latestBatteryReadingValid ||
               latestBatteryMillivolts < MyMINIConfig::BATTERY_WARNING_MIN_MV) {
      stopBatteryWarning();
    } else {
      updateBatteryWarning(latestBatteryLevel, nowMs);
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
  initializeBno055();
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
  // Establish zero yaw when initialization is complete, before START is pressed.
  // wait_button() will establish a fresh origin again at the START press.
  if (imuReady && !imu.resetAngles()) {
    Serial.println(F("BNO055 startup yaw reset failed"));
  }
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

bool gyro_start_ready() {
  return imuReady && headingReferenceReady && imu.update() &&
         isfinite(imu.yawRaw());
}

bool gyro_recover() {
  motor(1, 1);
  headingReferenceReady = false;
  delay(100);  // Let any residual turn settle before choosing a new zero.

  const auto tryReset = []() {
    if (!imuReady || !imu.update() || !isfinite(imu.yawRaw()) ||
        !imu.resetAngles()) return false;
    delay(200);
    return imu.update() && isfinite(imu.yawRaw());
  };

  for (uint8_t attempt = 0; imuReady && attempt < 3u; ++attempt) {
    if (tryReset()) {
      headingReferenceReady = true;
      Serial.println(F("GYRO_RECOVERED heading ready"));
      return true;
    }
    delay(100);
  }

  Serial.println(F("GYRO_RECOVERY reinitializing BNO055"));
  initializeBno055();
  motor(1, 1);
  for (uint8_t attempt = 0; imuReady && attempt < 3u; ++attempt) {
    if (tryReset()) {
      headingReferenceReady = true;
      Serial.println(F("GYRO_RECOVERED after BNO055 reinitialization"));
      return true;
    }
    delay(100);
  }

  motor(1, 1);
  Serial.println(F("GYRO_RECOVERY_FAILED"));
  return false;
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


