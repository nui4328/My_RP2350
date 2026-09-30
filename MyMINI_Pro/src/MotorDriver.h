#pragma once

#include <Arduino.h>

constexpr bool MOTOR_LEFT_INVERT = false;
constexpr bool MOTOR_RIGHT_INVERT = false;
constexpr bool MOTOR_BRAKE_AT_ZERO = true;

// TB6612FNG remains the default so existing sketches retain their original
// wiring and behavior. DRV8874 PH/EN and nSLEEP configuration is owned by
// RobotConfig; select the desired type before robot_begin().
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
void motor(int sl, int sr);
void motor_stop();
void set_motor_brake_at_zero(bool enabled);
bool get_motor_brake_at_zero();
