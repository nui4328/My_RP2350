#pragma once

#include <Arduino.h>
#include "RobotConfig.h"

class DualMuxSensors {
public:
  void begin();
  bool update(uint32_t nowUs);

  const uint16_t* frontRaw() const { return frontRaw_; }
  const uint16_t* rearRaw() const { return rearRaw_; }
  const uint16_t* frontFiltered() const { return frontFiltered_; }
  const uint16_t* rearFiltered() const { return rearFiltered_; }
  uint32_t frameSequence() const { return frameSequence_; }
  uint32_t lastFrameMicros() const { return lastFrameMicros_; }
  bool configureRead(uint16_t settleUs, uint8_t discardReads,
                     uint8_t averageReads, uint8_t smoothingDivisor);
  void setSettleMicros(uint16_t settleUs) { settleUs_ = settleUs; }
  uint16_t settleMicros() const { return settleUs_; }
  uint8_t discardReads() const { return discardReads_; }
  uint8_t averageReads() const { return averageReads_; }
  uint8_t smoothingDivisor() const { return smoothingDivisor_; }
  void printFrame(Print& output) const;

private:
  enum class ScanState : uint8_t { SelectChannel, WaitForSettling };

  void selectChannel_(uint8_t channel);
  uint16_t readAveraged_(uint8_t pin) const;
  void updateFiltered_(uint8_t channel);

  uint16_t frontRaw_[MyMINIConfig::SENSOR_COUNT] = {};
  uint16_t rearRaw_[MyMINIConfig::SENSOR_COUNT] = {};
  uint16_t frontFiltered_[MyMINIConfig::SENSOR_COUNT] = {};
  uint16_t rearFiltered_[MyMINIConfig::SENSOR_COUNT] = {};
  int32_t frontSmoothQ8_[MyMINIConfig::SENSOR_COUNT] = {};
  int32_t rearSmoothQ8_[MyMINIConfig::SENSOR_COUNT] = {};
  bool filteredInitialized_[MyMINIConfig::SENSOR_COUNT] = {};
  uint8_t channel_ = 0;
  ScanState state_ = ScanState::SelectChannel;
  uint32_t selectedAtUs_ = 0;
  uint32_t frameSequence_ = 0;
  uint32_t lastFrameMicros_ = 0;
  uint16_t settleUs_ = MyMINIConfig::MUX_SETTLE_US;
  uint8_t discardReads_ = MyMINIConfig::MUX_DISCARD_READS;
  uint8_t averageReads_ = MyMINIConfig::MUX_AVERAGE_SAMPLES;
  uint8_t smoothingDivisor_ = MyMINIConfig::MUX_SMOOTHING_DIVISOR;
};
