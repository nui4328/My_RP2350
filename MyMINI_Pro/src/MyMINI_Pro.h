#pragma once

#include <Arduino.h>

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

// -----------------------------------------------------------------------------
// Servo control
// -----------------------------------------------------------------------------

void servo(uint8_t pin, int angle);
