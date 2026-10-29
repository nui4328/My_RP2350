#pragma once

#include <Arduino.h>

constexpr bool MOTOR_LEFT_INVERT = false;
constexpr bool MOTOR_RIGHT_INVERT = false;
constexpr bool MOTOR_BRAKE_AT_ZERO = false;  // Legacy constant; zero now coasts.

// DRV8874 is the default. Call select_motor_driver(MotorDriverType::TB6612FNG) before
// robot_begin() for a TB6612FNG robot. DRV8874 PH/EN and nSLEEP pins are
// configured in RobotConfig.
enum class MotorDriverType : uint8_t { TB6612FNG, DRV8874 };
enum class MotorDriverStatus : uint8_t {
  TB6612_READY,
  DRV8874_READY,
  DRV8874_NOT_CONFIGURED,
};

bool select_motor_driver(MotorDriverType type);
MotorDriverType selected_motor_driver();
MotorDriverStatus motor_driver_status();
bool motor_driver_ready();

void motor_begin();
// (0,0) coasts; (1,1) and (-1,-1) brake. Other +/-1 values become +/-2;
// a zero wheel in a mixed command brakes while the other wheel drives.
void motor(int sl, int sr);
void motor_stop();
// Kept for source compatibility; zero always coasts and the getter is false.
void set_motor_brake_at_zero(bool enabled);
bool get_motor_brake_at_zero();
