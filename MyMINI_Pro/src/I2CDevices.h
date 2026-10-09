#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "RobotConfig.h"

class I2CDevices {
public:
  explicit I2CDevices(TwoWire& wire = Wire) : wire_(wire) {}

  void begin();
  bool probe(uint8_t address);
  void scanBus(Print& output);
  void printExpectedDevices(Print& output);

  bool readAds1115SingleEnded(uint8_t channel, int16_t& raw);
  float adsRawToVolts(int16_t raw) const;
  float batteryVoltsFromRaw(int16_t raw) const;

  bool readMcp3421Raw(int16_t& raw, uint8_t& configByte);
  float mcp3421RawToVolts(int16_t raw) const;

  bool writePcf8574(uint8_t value);

  bool readEepromByte(uint16_t address, uint8_t& value);
  bool writeEepromByte(uint16_t address, uint8_t value);
  bool readEepromBlock(uint16_t address, uint8_t* buffer, uint16_t length);
  bool writeEepromBlock(uint16_t address, const uint8_t* buffer,
                        uint16_t length);
  bool runPreservingEepromTest(Print& output);

private:
  bool setAdsRegisterPointer_(uint8_t registerAddress);
  bool readAdsRegister_(uint8_t registerAddress, uint16_t& value);
  bool waitForEepromReady_(uint16_t timeoutMs);

  TwoWire& wire_;
};
