#include "CalibrationManager.h"

#include <stddef.h>
#include <string.h>

#include "RobotConfig.h"

using namespace MyMINIConfig;

namespace {
constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;
constexpr uint32_t SHORT_PRESS_MIN_MS = 30;
constexpr uint32_t SHORT_PRESS_MAX_MS = 800;
constexpr uint32_t LONG_PRESS_MS = 3000;
constexpr uint32_t CALIBRATION_DURATION_MS = 5000;
constexpr uint32_t ADCCAL_POLL_INTERVAL_MS = 100;
constexpr int16_t ADCCAL_ACTIVE_THRESHOLD = 512;
constexpr uint16_t CALIBRATION_TONE_HZ = 3000;
constexpr uint16_t COMPLETE_TONE_HZ = 3500;
constexpr uint32_t CALIBRATION_TONE_MS = 50;
constexpr uint32_t CALIBRATION_TONE_INTERVAL_MS = 300;
constexpr uint32_t COMPLETE_TONE_MS = 250;
constexpr uint32_t COMPLETE_TONE_GAP_MS = 150;
constexpr uint32_t WAIT_TONE_MS = 120;
constexpr uint32_t WAIT_TONE_GAP_MS = 100;
constexpr uint16_t CALIBRATION_EEPROM_ADDRESS = 0x0000;
constexpr uint32_t CALIBRATION_MAGIC = 0x4D4D5031UL;
constexpr uint16_t CALIBRATION_VERSION = 1;
constexpr uint8_t VALID_FRONT = 0x01;
constexpr uint8_t VALID_REAR = 0x02;
constexpr uint8_t VALID_CENTER = 0x04;
constexpr uint8_t VALID_MASK_ALL = VALID_FRONT | VALID_REAR | VALID_CENTER;
constexpr uint32_t CENTER_CAPTURE_INTERVAL_MS = 100;
} // namespace

void CalibrationManager::begin() {
  pinMode(PIN_START_BUTTON, INPUT_PULLUP);

  const uint32_t nowMs = millis();
  rawButtonPressed_ = digitalRead(PIN_START_BUTTON) == LOW;
  debouncedButtonPressed_ = rawButtonPressed_;
  buttonArmed_ = !debouncedButtonPressed_;
  rawButtonChangedAtMs_ = nowMs;
  nextAdcCalReadAtMs_ = nowMs;
  loadCalibration();
}

void CalibrationManager::update(uint32_t nowMs) {
  updateBuzzer_(nowMs);

  const bool buttonPressed = digitalRead(PIN_START_BUTTON) == LOW;
  if (buttonPressed != rawButtonPressed_) {
    rawButtonPressed_ = buttonPressed;
    rawButtonChangedAtMs_ = nowMs;
  }

  if (rawButtonPressed_ != debouncedButtonPressed_ &&
      static_cast<uint32_t>(nowMs - rawButtonChangedAtMs_) >= BUTTON_DEBOUNCE_MS) {
    debouncedButtonPressed_ = rawButtonPressed_;
    handleButtonStateChange_(debouncedButtonPressed_, nowMs);
  }

  if (pressTracking_ &&
      static_cast<uint32_t>(nowMs - pressedAtMs_) >= LONG_PRESS_MS) {
    pressTracking_ = false;
    buttonArmed_ = false;
    startCalibration_(CalibrationSource::Center, nowMs);
  }

  if (shortPressEvent_) return;

  if (calibrationSource_ != CalibrationSource::None) {
    captureCalibrationSamples_(nowMs);
    if (static_cast<int32_t>(nowMs - calibrationEndsAtMs_) >= 0) {
      finishCalibration_(nowMs);
    }
    return;
  }

  checkCalibrationInputs_(nowMs);
}

bool CalibrationManager::loadCalibration() {
  CalibrationData loaded = {};
  if (!i2c_.readEepromBlock(CALIBRATION_EEPROM_ADDRESS,
                            reinterpret_cast<uint8_t*>(&loaded),
                            sizeof(loaded)) ||
      loaded.magic != CALIBRATION_MAGIC ||
      loaded.version != CALIBRATION_VERSION ||
      loaded.size != sizeof(CalibrationData) ||
      (loaded.validMask & static_cast<uint8_t>(~VALID_MASK_ALL)) != 0 ||
      loaded.reserved[0] != 0 || loaded.reserved[1] != 0 ||
      loaded.reserved[2] != 0 ||
      loaded.crc16 != crc16_(reinterpret_cast<const uint8_t*>(&loaded),
                             offsetof(CalibrationData, crc16))) {
    memset(&data_, 0, sizeof(data_));
    return false;
  }

  data_ = loaded;
  return true;
}

bool CalibrationManager::saveCalibration() {
  data_.magic = CALIBRATION_MAGIC;
  data_.version = CALIBRATION_VERSION;
  data_.size = sizeof(CalibrationData);
  memset(data_.reserved, 0, sizeof(data_.reserved));
  data_.crc16 = crc16_(reinterpret_cast<const uint8_t*>(&data_),
                       offsetof(CalibrationData, crc16));
  return i2c_.writeEepromBlock(CALIBRATION_EEPROM_ADDRESS,
                                reinterpret_cast<const uint8_t*>(&data_),
                                sizeof(data_));
}

bool CalibrationManager::frontValid() const {
  return (data_.validMask & VALID_FRONT) != 0;
}

bool CalibrationManager::rearValid() const {
  return (data_.validMask & VALID_REAR) != 0;
}

bool CalibrationManager::centerValid() const {
  return (data_.validMask & VALID_CENTER) != 0;
}

bool CalibrationManager::isBusy() const {
  return calibrationSource_ != CalibrationSource::None ||
         buzzerState_ != BuzzerState::Idle;
}

bool CalibrationManager::consumeShortPressEvent() {
  if (!shortPressEvent_) return false;
  shortPressEvent_ = false;
  return true;
}

void CalibrationManager::startWaitButtonSignal(uint32_t nowMs) {
  if (calibrationSource_ != CalibrationSource::None) return;
  startTone_(COMPLETE_TONE_HZ, WAIT_TONE_MS, nowMs);
  buzzerState_ = BuzzerState::WaitFirstTone;
}

void CalibrationManager::stopBuzzer() {
  noTone(PIN_BUZZER);
  calibrationToneActive_ = false;
  buzzerState_ = BuzzerState::Idle;
}

void CalibrationManager::handleButtonStateChange_(bool pressed, uint32_t nowMs) {
  if (pressed) {
    if (buttonArmed_ && calibrationSource_ == CalibrationSource::None) {
      pressedAtMs_ = nowMs;
      pressTracking_ = true;
    }
    return;
  }

  if (pressTracking_) {
    const uint32_t heldMs = nowMs - pressedAtMs_;
    if (heldMs >= SHORT_PRESS_MIN_MS && heldMs <= SHORT_PRESS_MAX_MS &&
        calibrationSource_ == CalibrationSource::None) {
      shortPressEvent_ = true;
    }
  }
  pressTracking_ = false;
  buttonArmed_ = true;
}

void CalibrationManager::checkCalibrationInputs_(uint32_t nowMs) {
  const bool frontCalActive = digitalRead(PIN_FRONT_CAL) == LOW;
  if (!frontCalActive) {
    frontCalLatched_ = false;
  } else if (!frontCalLatched_) {
    frontCalLatched_ = true;
    startCalibration_(CalibrationSource::Front, nowMs);
    return;
  }

  if (static_cast<int32_t>(nowMs - nextAdcCalReadAtMs_) < 0) return;
  nextAdcCalReadAtMs_ = nowMs + ADCCAL_POLL_INTERVAL_MS;

  int16_t adcCalRaw = 0;
  uint8_t adcCalConfig = 0;
  if (!i2c_.readMcp3421Raw(adcCalRaw, adcCalConfig)) {
    return;
  }

  const bool adcCalActive = adcCalRaw <= ADCCAL_ACTIVE_THRESHOLD;
  if (!adcCalActive) {
    adcCalLatched_ = false;
  } else if (!adcCalLatched_) {
    adcCalLatched_ = true;
    startCalibration_(CalibrationSource::Rear, nowMs);
  }
}

void CalibrationManager::startCalibration_(CalibrationSource source, uint32_t nowMs) {
  if (calibrationSource_ != CalibrationSource::None) return;

  calibrationSource_ = source;
  calibrationEndsAtMs_ = nowMs + CALIBRATION_DURATION_MS;
  
  noTone(PIN_BUZZER);
  calibrationToneActive_ = false;
  nextCalibrationToneAtMs_ = nowMs;
  buzzerState_ = BuzzerState::Idle;
  resetCapture_();
}

void CalibrationManager::finishCalibration_(uint32_t nowMs) {
  
  noTone(PIN_BUZZER);
  calibrationToneActive_ = false;
  const bool calibrationSucceeded = commitCalibration_();
  calibrationSource_ = CalibrationSource::None;
  if (calibrationSucceeded) {
    startTone_(COMPLETE_TONE_HZ, COMPLETE_TONE_MS, nowMs);
    buzzerState_ = BuzzerState::CompletionFirstTone;
  } else {
    buzzerState_ = BuzzerState::Idle;
  }
}

void CalibrationManager::resetCapture_() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    captureMin_[i] = 0xFFFFU;
    captureMax_[i] = 0;
  }
  centerCaptureMin_[0] = 32767;
  centerCaptureMin_[1] = 32767;
  centerCaptureMax_[0] = -32768;
  centerCaptureMax_[1] = -32768;
  sensorCaptureCount_ = 0;
  centerCaptureMask_ = 0;
  nextCenterCaptureAtMs_ = millis();
}

void CalibrationManager::captureCalibrationSamples_(uint32_t nowMs) {
  if (calibrationSource_ == CalibrationSource::Front ||
      calibrationSource_ == CalibrationSource::Rear) {
    const uint16_t* raw = calibrationSource_ == CalibrationSource::Front
                              ? sensors_.frontRaw()
                              : sensors_.rearRaw();
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      if (raw[i] < captureMin_[i]) captureMin_[i] = raw[i];
      if (raw[i] > captureMax_[i]) captureMax_[i] = raw[i];
    }
    ++sensorCaptureCount_;
    return;
  }

  if (calibrationSource_ != CalibrationSource::Center ||
      static_cast<int32_t>(nowMs - nextCenterCaptureAtMs_) < 0) {
    return;
  }
  nextCenterCaptureAtMs_ = nowMs + CENTER_CAPTURE_INTERVAL_MS;

  int16_t raw = 0;
  if (i2c_.readAds1115SingleEnded(ADS_CHANNEL_CENTER_LEFT, raw)) {
    if (raw < centerCaptureMin_[0]) centerCaptureMin_[0] = raw;
    if (raw > centerCaptureMax_[0]) centerCaptureMax_[0] = raw;
    centerCaptureMask_ |= 0x01;
  }
  if (i2c_.readAds1115SingleEnded(ADS_CHANNEL_CENTER_RIGHT, raw)) {
    if (raw < centerCaptureMin_[1]) centerCaptureMin_[1] = raw;
    if (raw > centerCaptureMax_[1]) centerCaptureMax_[1] = raw;
    centerCaptureMask_ |= 0x02;
  }
}

bool CalibrationManager::commitCalibration_() {
  const CalibrationSource completedSource = calibrationSource_;
  CalibrationData previous = data_;

  if (completedSource == CalibrationSource::Front ||
      completedSource == CalibrationSource::Rear) {
    if (sensorCaptureCount_ == 0) return false;
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      if (!sensorRangeValid_(captureMin_[i], captureMax_[i])) return false;
    }
    if (completedSource == CalibrationSource::Front) {
      for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        data_.frontMin[i] = captureMin_[i];
        data_.frontMax[i] = captureMax_[i];
      }
      data_.validMask |= VALID_FRONT;
    } else {
      for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        data_.rearMin[i] = captureMin_[i];
        data_.rearMax[i] = captureMax_[i];
      }
      data_.validMask |= VALID_REAR;
    }
  } else if (completedSource == CalibrationSource::Center) {
    if (centerCaptureMask_ != 0x03 ||
        !centerRangeValid_(centerCaptureMin_[0], centerCaptureMax_[0]) ||
        !centerRangeValid_(centerCaptureMin_[1], centerCaptureMax_[1])) {
      return false;
    }
    for (uint8_t i = 0; i < 2; ++i) {
      data_.centerMin[i] = centerCaptureMin_[i];
      data_.centerMax[i] = centerCaptureMax_[i];
    }
    data_.validMask |= VALID_CENTER;
  } else {
    return false;
  }

  if (saveCalibration()) return true;
  data_ = previous;
  return false;
}

void CalibrationManager::updateBuzzer_(uint32_t nowMs) {
  if (calibrationSource_ != CalibrationSource::None) {
    if (calibrationToneActive_ &&
        static_cast<int32_t>(nowMs - buzzerDeadlineMs_) >= 0) {
      noTone(PIN_BUZZER);
      calibrationToneActive_ = false;
    }
    if (!calibrationToneActive_ &&
        static_cast<int32_t>(nowMs - nextCalibrationToneAtMs_) >= 0) {
      startTone_(CALIBRATION_TONE_HZ, CALIBRATION_TONE_MS, nowMs);
      calibrationToneActive_ = true;
      nextCalibrationToneAtMs_ = nowMs + CALIBRATION_TONE_INTERVAL_MS;
    }
    return;
  }

  if (static_cast<int32_t>(nowMs - buzzerDeadlineMs_) < 0) return;

  switch (buzzerState_) {
    case BuzzerState::WaitFirstTone:
      noTone(PIN_BUZZER);
      buzzerDeadlineMs_ = nowMs + WAIT_TONE_GAP_MS;
      buzzerState_ = BuzzerState::WaitGap;
      break;
    case BuzzerState::WaitGap:
      startTone_(COMPLETE_TONE_HZ, WAIT_TONE_MS, nowMs);
      buzzerState_ = BuzzerState::WaitSecondTone;
      break;
    case BuzzerState::WaitSecondTone:
      noTone(PIN_BUZZER);
      buzzerState_ = BuzzerState::Idle;
      break;
    case BuzzerState::CompletionFirstTone:
      noTone(PIN_BUZZER);
      buzzerDeadlineMs_ = nowMs + COMPLETE_TONE_GAP_MS;
      buzzerState_ = BuzzerState::CompletionGap;
      break;
    case BuzzerState::CompletionGap:
      startTone_(COMPLETE_TONE_HZ, COMPLETE_TONE_MS, nowMs);
      buzzerState_ = BuzzerState::CompletionSecondTone;
      break;
    case BuzzerState::CompletionSecondTone:
      noTone(PIN_BUZZER);
      buzzerState_ = BuzzerState::Idle;
      break;
    case BuzzerState::Idle:
      break;
  }
}

void CalibrationManager::startTone_(uint16_t frequencyHz, uint32_t durationMs,
                                    uint32_t nowMs) {
  tone(PIN_BUZZER, frequencyHz);
  buzzerDeadlineMs_ = nowMs + durationMs;
}

uint16_t CalibrationManager::crc16_(const uint8_t* bytes, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(bytes[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0 ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                                  : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

bool CalibrationManager::sensorRangeValid_(uint16_t minValue,
                                           uint16_t maxValue) {
  return maxValue > minValue;
}

bool CalibrationManager::centerRangeValid_(int16_t minValue,
                                           int16_t maxValue) {
  return maxValue > minValue;
}
