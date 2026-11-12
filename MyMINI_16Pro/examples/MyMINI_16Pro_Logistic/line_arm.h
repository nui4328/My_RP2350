#pragma once

#include <Arduino.h>

// Arm motion runs on Pico 2 core 1 while the original five-argument
// f_line()/b_line() follows the line on core 0.
// armMotionMs: 0 = lower to the GP1 limit,
// positive = raise for that many ms, negative = lower for its absolute ms.
// Servo 22 uses 180/up, 0/down, and 90/stop.
void f_line(int sl, int sr, float kp, float distanceCm,
            uint8_t stopPull, int armMotionMs);
void b_line(int sl, int sr, float kp, float distanceCm,
            uint8_t stopPull, int armMotionMs);
