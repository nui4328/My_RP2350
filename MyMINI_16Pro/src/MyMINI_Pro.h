#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

#include "RobotConfig.h"
#include "MotorDriver.h"
#include "LineFollower.h"
#include "GyroModel.h"

// Call before robot_begin(). BNO055 at 0x29 uses Wire (GP4/GP5).
// BNO085 at 0x4B defaults to Wire1 (GP26/GP27). Pass Wire as the third
// argument when the BNO085 is wired to GP4/GP5 instead.
// yawSign is +1 or -1 for BNO085, to match the robot's turn convention.
bool select_gyro(GyroModel model, int8_t yawSign = 1,
                 TwoWire& bno085Wire = Wire1);
GyroModel selected_gyro();

// -----------------------------------------------------------------------------
// Robot initialization
// -----------------------------------------------------------------------------

// Select the motor driver before robot_begin() to override the DRV8874 default.
// DRV8874 mapping is library-owned;
// motor_driver_status() reports DRV8874_NOT_CONFIGURED for an invalid internal
// pin configuration.
// Initializes motors, sensors and peripherals. Gyro initialization and zero
// happen only in gyro_reset()/gyro_set(). wait_button() calls gyro_reset()
// once before displaying readings, then zeros at START if the gyro was ready.
void robot_begin();
void wait_button();
void robot_update();
bool gyro_ready();
// Cached real-gyro readiness; does not probe/recover. False in estimation mode.
bool gyro_start_ready();
// True when movement is using wheel commands/time instead of a physical gyro.
bool gyro_estimation_active();
// Read heading and gyro rates. BNO085 heading wraps to [0, 360) degrees;
// use gyro_read_raw() for the continuous signed heading used by motion control.
// Returns false when the selected gyro is unavailable or a sample is invalid.
bool gyro_read(float& yawDeg, float& rateX, float& rateY, float& rateZ);
// Read the same unwrapped yaw used by f_line(gyro_angle), turn_gyro and
// fw_gyro/bw_gyro, with rates from the same selected gyro update.
bool gyro_read_raw(float& yawDeg, float& rateX, float& rateY, float& rateZ);
// Brake and try to establish real gyro zero within one 2000 ms budget.
// On success play a 240 ms ready melody. On timeout establish estimated zero.
// Returns true for either usable motion reference; query gyro_start_ready()
// or gyro_estimation_active() to distinguish real gyro from estimation.
// Only an explicit reset/set may leave estimation mode.
bool gyro_reset();
// Prepare through the same bounded reset. True also permits estimated mode.
// Optional explicit preparation; wait_button() calls gyro_reset() directly.
bool gyro_set();
// Compatibility wrapper around gyro_reset(); returns real gyro readiness.
bool gyro_recover();
// Reports BNO055 fusion calibration levels (0..3); false for BNO085.
bool gyro_calibration_status(uint8_t& system, uint8_t& gyro,
                             uint8_t& accel, uint8_t& mag);
// BNO055 only: call while motors are stopped after all four levels reach 3.
// Returns false for BNO085, whose onboard calibration is managed by the chip.
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

// Print one fresh raw sensor row to Serial: F: f0..f15, B: b0..b15,
// adcL: CL, adcR: CR. Call after robot_begin(); no calibration is required.
// F/B use the ADC-averaged MUX readings before smoothing/normalization.
// CL/CR use ADS1115 raw counts; a failed center read prints ERR.
// Returns after one row; call repeatedly in loop() for continuous display.
void read_rowSensor();

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

// Set the 0/180-degree pulse widths before the first servo() call on this pin.
// Supported pins: 18, 22, 28, 0, 1. Valid range: 500..2500 us.
// Returns false for an invalid setting or an already attached servo.
bool set_servo_pulse_range(uint8_t pin, uint16_t minPulseUs,
                           uint16_t maxPulseUs);
void servo(uint8_t pin, int angle);
