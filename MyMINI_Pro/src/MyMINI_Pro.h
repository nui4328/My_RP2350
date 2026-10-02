#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "RobotConfig.h"
#include "MotorDriver.h"
#include "LineFollower.h"

// -----------------------------------------------------------------------------
// Robot initialization
// -----------------------------------------------------------------------------

// Select the motor driver before robot_begin(). Existing sketches do not need
// this: TB6612FNG is selected by default. DRV8874 mapping is library-owned;
// motor_driver_status() reports DRV8874_NOT_CONFIGURED for an invalid internal
// pin configuration.
void robot_begin();
void wait_button();
void robot_update();
bool gyro_ready();

// Configure the production MUX reader after robot_begin() and before
// wait_button()/motion. Defaults: 4 us, 1 discard, 2 averaged, no smoothing.
// Divisor 1 disables smoothing; 2, 4, and 8 are progressively slower.
// Returns false for unsupported values without changing the current settings.
bool configure_sensor_reading(uint16_t muxSettleUs, uint8_t adcDiscardReads,
                              uint8_t adcAverageReads,
                              uint8_t smoothingDivisor);

// Raw F/B readings come from a fresh existing MUX frame; CL/CR come from the
// existing ADS1115 reader. Calibration min/max are raw EEPROM values loaded
// by robot_begin(). normalized maps the current raw reading to 0..1000.
// Check rawValid and calibrationValid before using the corresponding fields.
struct LineSensorReading {
  bool rawValid = false;
  bool calibrationValid = false;
  bool normalizedValid = false;
  int32_t raw = 0;
  int32_t minRaw = 0;
  int32_t maxRaw = 0;
  uint16_t normalized = 0;
};

// Uses the existing f0..f15, b0..b15, cl and cr identifiers. Returns false
// for a read failure, invalid identifier, or a call before robot_begin().
// A missing calibration does not prevent a successful raw read.
bool read_line_sensor(LineExitSensor sensor, LineSensorReading& reading);
void print_line_sensor_table(Print& output = Serial);

// readADC_* returns the 0..1000 normalized value shown by wait_button().
// F/B use its filtered MUX sample; CL/CR use the ADS1115 raw conversion.
// minADC_*/maxADC_* return the normalized limits 0 and 1000 when that channel
// has valid calibration. Raw EEPROM min/max remain in LineSensorReading.
// An invalid channel, missing calibration, failed read, or call before
// robot_begin() returns ADC_VALUE_INVALID, never a fake zero.
constexpr int32_t ADC_VALUE_INVALID = INT32_MIN;
int32_t readADC_F(uint8_t channel);
int32_t minADC_F(uint8_t channel);
int32_t maxADC_F(uint8_t channel);
int32_t readADC_B(uint8_t channel);
int32_t minADC_B(uint8_t channel);
int32_t maxADC_B(uint8_t channel);
int32_t readADC_CL();
int32_t minADC_CL();
int32_t maxADC_CL();
int32_t readADC_CR();
int32_t minADC_CR();
int32_t maxADC_CR();

// -----------------------------------------------------------------------------
// Servo control
// -----------------------------------------------------------------------------

void servo(uint8_t pin, int angle);
