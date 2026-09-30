#pragma once

#include <Arduino.h>
#include "RobotConfig.h"

class DualMuxSensors {
public:
  void begin();
  bool update(uint32_t nowUs);

  const uint16_t* frontRaw() const { return frontRaw_; }
  const uint16_t* rearRaw() const { return rearRaw_; }
  uint32_t frameSequence() const { return frameSequence_; }
  uint32_t lastFrameMicros() const { return lastFrameMicros_; }
  void printFrame(Print& output) const;

private:
  enum class ScanState : uint8_t { SelectChannel, WaitForSettling };

  void selectChannel_(uint8_t channel);
  uint16_t readAveraged_(uint8_t pin) const;

  uint16_t frontRaw_[MyMINIConfig::SENSOR_COUNT] = {};
  uint16_t rearRaw_[MyMINIConfig::SENSOR_COUNT] = {};
  uint8_t channel_ = 0;
  ScanState state_ = ScanState::SelectChannel;
  uint32_t selectedAtUs_ = 0;
  uint32_t frameSequence_ = 0;
  uint32_t lastFrameMicros_ = 0;
};
