#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "RobotConfig.h"
#include "MotorDriver.h"
#include "LineFollower.h"

// -----------------------------------------------------------------------------
// Robot initialization
// -----------------------------------------------------------------------------

// Select the motor driver before robot_begin() to override the DRV8874 default.
// DRV8874 mapping is library-owned;
// motor_driver_status() reports DRV8874_NOT_CONFIGURED for an invalid internal
// pin configuration.
// Zeros the BNO055 yaw after initialization; wait_button() zeros it again
// when START is pressed so motion uses the robot's start orientation.
void robot_begin();
void wait_button();
void robot_update();
bool gyro_ready();
// Call after wait_button(): START yaw reset must have succeeded and a fresh
// BNO055 heading sample must be readable.
bool gyro_start_ready();
// Call after START if gyro_start_ready() is false. Brakes, retries the heading
// read/reset, then reinitializes BNO055 once if needed. Establishes a new zero
// at the robot's current orientation. Returns false if recovery fails.
bool gyro_recover();
// Reports the BNO055's own fusion calibration levels (0..3).
bool gyro_calibration_status(uint8_t& system, uint8_t& gyro,
                             uint8_t& accel, uint8_t& mag);
// Call while motors are stopped after all four levels reach 3.
bool save_gyro_calibration();

// Optionally override the production MUX reader after robot_begin() and before
// wait_button()/motion. Defaults: 40 us, 1 discard, 2 averaged, divisor 4.
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
