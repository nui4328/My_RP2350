#pragma once
#include <stdint.h>

// Internal motion source; public gyro_read* continues to report real IMU only.
bool motion_heading_ready();
bool motion_heading_update();
float motion_heading_yaw();
float motion_heading_rate();
void motion_heading_zero(bool useGyro);
void motion_heading_command(int left, int right);
void motion_heading_calibrate(uint16_t spinMs, uint16_t pivotMs, uint8_t speed);
bool gyro_estimation_active();
