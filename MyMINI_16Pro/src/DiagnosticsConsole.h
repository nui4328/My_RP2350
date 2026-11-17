#pragma once

#include <Arduino.h>
#include "CalibrationManager.h"
#include "DualMuxSensors.h"
#include "I2CDevices.h"
#include "TB6612Driver.h"

class DiagnosticsConsole {
public:
  DiagnosticsConsole(Stream& serial, DualMuxSensors& sensors,
                     I2CDevices& i2c, TB6612Driver& motors,
                     CalibrationManager& calibration)
      : serial_(serial), sensors_(sensors), i2c_(i2c), motors_(motors),
        calibration_(calibration) {}

  void begin();
  void update(uint32_t nowMs);

private:
  void consumeSerial_();
  void executeLine_();
  void printHelp_();
  void printStatus_();
  void printAds_();
  void printAdcCal_();
  void printSensorDiagnostics_();
  void startLedChase_(uint32_t nowMs);
  void updateLedChase_(uint32_t nowMs);
  void handleMotorCommand_(char* sideToken, char* powerToken, uint32_t nowMs);
  void stopMotorTest_(const __FlashStringHelper* reason);
  static bool parseInt16_(const char* text, int16_t& value);

  Stream& serial_;
  DualMuxSensors& sensors_;
  I2CDevices& i2c_;
  TB6612Driver& motors_;
  CalibrationManager& calibration_;

  char line_[80] = {};
  uint8_t lineLength_ = 0;
  bool motorTestActive_ = false;
  uint32_t motorStopAtMs_ = 0;
  bool ledChaseActive_ = false;
  uint8_t ledIndex_ = 0;
  uint32_t nextLedAtMs_ = 0;
};
