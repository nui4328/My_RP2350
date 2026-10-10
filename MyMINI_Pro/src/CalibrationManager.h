#pragma once

#include <Arduino.h>
#include "DualMuxSensors.h"
#include "I2CDevices.h"
#include "TB6612Driver.h"

struct __attribute__((packed)) CalibrationData {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint8_t validMask;
  uint8_t reserved[3];
  uint16_t frontMin[16];
  uint16_t frontMax[16];
  uint16_t rearMin[16];
  uint16_t rearMax[16];
  int16_t centerMin[2];
  int16_t centerMax[2];
  uint16_t crc16;
};

static_assert(sizeof(CalibrationData) == 150,
              "CalibrationData EEPROM layout must remain 150 bytes");

class CalibrationManager {
public:
  CalibrationManager(I2CDevices& i2c, TB6612Driver& motors,
                     DualMuxSensors& sensors)
      : i2c_(i2c), motors_(motors), sensors_(sensors) {}

  void begin();
  void update(uint32_t nowMs);
  bool consumeShortPressEvent();
  void startWaitButtonSignal(uint32_t nowMs);
  void stopBuzzer();
  bool loadCalibration();
  bool saveCalibration();
  bool frontValid() const;
  bool rearValid() const;
  bool centerValid() const;
  bool isBusy() const;
  const CalibrationData& data() const { return data_; }

private:
  enum class CalibrationSource : uint8_t { None, Front, Rear, Center };

  void handleButtonStateChange_(bool pressed, uint32_t nowMs);
  void checkCalibrationInputs_(uint32_t nowMs);
  void startCalibration_(CalibrationSource source, uint32_t nowMs);
  void finishCalibration_(uint32_t nowMs);
  void resetCapture_();
  void captureCalibrationSamples_(uint32_t nowMs);
  bool commitCalibration_();
  void updateBuzzer_(uint32_t nowMs);
  void startTone_(uint16_t frequencyHz, uint32_t durationMs, uint32_t nowMs);
  static uint16_t crc16_(const uint8_t* bytes, uint16_t length);
  static bool sensorRangeValid_(uint16_t minValue, uint16_t maxValue);
  static bool centerRangeValid_(int16_t minValue, int16_t maxValue);

  I2CDevices& i2c_;
  TB6612Driver& motors_;
  DualMuxSensors& sensors_;
  CalibrationData data_ = {};

  bool rawButtonPressed_ = false;
  bool debouncedButtonPressed_ = false;
  bool buttonArmed_ = false;
  bool pressTracking_ = false;
  bool shortPressEvent_ = false;
  bool frontCalLatched_ = false;
  bool adcCalLatched_ = false;
  uint32_t rawButtonChangedAtMs_ = 0;
  uint32_t pressedAtMs_ = 0;
  uint32_t nextAdcCalReadAtMs_ = 0;
  uint32_t nextCenterCaptureAtMs_ = 0;
  uint32_t calibrationEndsAtMs_ = 0;
  uint32_t buzzerDeadlineMs_ = 0;
  uint32_t nextCalibrationToneAtMs_ = 0;
  bool calibrationToneActive_ = false;
  enum class BuzzerState : uint8_t {
    Idle,
    WaitFirstTone,
    WaitGap,
    WaitSecondTone,
    CompletionFirstTone,
    CompletionGap,
    CompletionSecondTone
  };
  BuzzerState buzzerState_ = BuzzerState::Idle;
  CalibrationSource calibrationSource_ = CalibrationSource::None;
  uint16_t captureMin_[MyMINIConfig::SENSOR_COUNT] = {};
  uint16_t captureMax_[MyMINIConfig::SENSOR_COUNT] = {};
  int16_t centerCaptureMin_[2] = {};
  int16_t centerCaptureMax_[2] = {};
  uint32_t sensorCaptureCount_ = 0;
  uint8_t centerCaptureMask_ = 0;
};
