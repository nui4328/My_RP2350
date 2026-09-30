#include "MyMINI_Pro.h"
#include "MyMINI_ProInternal.h"

#include <BNO055.h>
#include <Servo.h>
#include <Wire.h>

DualMuxSensors sensorArrays;
I2CDevices i2cDevices;
BNO055 imu;
bool imuReady = false;
TB6612Driver motorDriver;
DiagnosticsConsole diagnostics(Serial, sensorArrays, i2cDevices, motorDriver);
CalibrationManager calibrationManager(i2cDevices, motorDriver, sensorArrays);
namespace {
bool latestBatteryReadingValid = false;
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
          imu.calibrate(11, false);
          imu.resetAngles();
          imuReady = true;
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
  const uint16_t* front = sensorArrays.frontRaw();
  const uint16_t* rear = sensorArrays.rearRaw();
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
  const bool previousBrakeMode = get_motor_brake_at_zero();
  set_motor_brake_at_zero(false);
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
      stopBatteryWarning();
      i2cDevices.writePcf8574(0x00);
      calibrationManager.stopBuzzer();
      noTone(MyMINIConfig::PIN_BUZZER);
      tone(MyMINIConfig::PIN_BUZZER, 3500, 400);
      motor(0, 0);
      set_motor_brake_at_zero(previousBrakeMode);
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
  const bool previousBrakeMode = get_motor_brake_at_zero();
  set_motor_brake_at_zero(false);
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
  set_motor_brake_at_zero(previousBrakeMode);
}

void robot_update() {
  sensorArrays.update(micros());
  const uint32_t nowMs = millis();
  calibrationManager.update(nowMs);
  diagnostics.update(nowMs);
}

bool gyro_ready() {
  return imuReady;
}


