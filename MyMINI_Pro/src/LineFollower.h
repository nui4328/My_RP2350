#pragma once

#include <Arduino.h>

// -----------------------------------------------------------------------------
// Sensor and turn types
// -----------------------------------------------------------------------------

enum LineExitSensor : uint8_t {
  f0 = 0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14, f15,
  b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14, b15,
  cl, cr
};

enum class TurnMode : uint8_t { fl, fr, cl, cr, l, r };

static constexpr TurnMode tfl = TurnMode::fl;
static constexpr TurnMode tfr = TurnMode::fr;
static constexpr TurnMode tcl = TurnMode::cl;
static constexpr TurnMode tcr = TurnMode::cr;
static constexpr TurnMode tl = TurnMode::l;
static constexpr TurnMode tr = TurnMode::r;

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

constexpr float LINE_KI = 0.0f;
constexpr float LINE_KD = 0.012f;
constexpr bool LINE_DEBUG = false;
// Enable verbose line exit diagnostics only for a supervised bench run.
void set_line_diagnostics(bool enabled);
constexpr uint32_t LINE_TIMEOUT_MS = 15000;
constexpr uint32_t LINE_ACCEL_TIME_MS = 400;
constexpr float LINE_MM_PER_SECOND_AT_100 = 900.0f;
constexpr float LINE_DECEL_DISTANCE_MM = 120.0f;
constexpr float LINE_MIN_DECEL_FACTOR = 0.25f;
constexpr int16_t TRACK_WINDOW_RADIUS = 22;
constexpr float MAX_ERROR_RATE_PER_SECOND = 800.0f;

// f_line()/b_line() only. Ramp: 0..LINE_TIMEOUT_MS ms (0 disables it).
// PID: kpScale 0..10, ki 0..10; invalid values leave the settings unchanged.
bool set_line_ramp_ms(uint16_t rampMs);
bool set_line_pid_tuning(float kpScale, float ki);

// -----------------------------------------------------------------------------
// Forward line following
// -----------------------------------------------------------------------------

void f_line(int sl, int sr, float kp, LineExitSensor exitSensor,
            uint8_t stopPull = 0);
void f_line(int sl, int sr, float kp, float distanceCm,
            uint8_t stopPull = 0);

// -----------------------------------------------------------------------------
// Backward line following
// -----------------------------------------------------------------------------

void b_line(int sl, int sr, float kp, LineExitSensor exitSensor,
            uint8_t stopPull = 0);
void b_line(int sl, int sr, float kp, float distanceCm,
            uint8_t stopPull = 0);

// -----------------------------------------------------------------------------
// Forward gyro movement
// -----------------------------------------------------------------------------

void fw_gyro(float targetAngle, uint8_t speed, float kp, float distanceCm,
             uint8_t stopPull = 0);

// -----------------------------------------------------------------------------
// Backward gyro movement
// -----------------------------------------------------------------------------

void bw_gyro(float targetAngle, uint8_t speed, float kp, float distanceCm,
             uint8_t stopPull = 0);

// -----------------------------------------------------------------------------
// Intersection turn
// -----------------------------------------------------------------------------

void turn(TurnMode mode, uint8_t speed, LineExitSensor exitSensor,
          uint8_t stopPull = 0);

// Absolute heading from the most recent wait_button() press: right positive,
// left negative, target -180..180 degrees. Uses turn() approach, rotates at
// the requested speed, and brakes 20 degrees early (at most half a short turn).
// stopPull is accepted for API compatibility but ignored.
void turn_gyro(TurnMode mode, uint8_t speed, float targetDeg,
               uint8_t stopPull = 0);

// -----------------------------------------------------------------------------
// Spin rotation
// -----------------------------------------------------------------------------

bool rotate_spin(float angleDeg, uint8_t speed, uint8_t stopPull = 0);

// -----------------------------------------------------------------------------
// Forward pivot rotation
// -----------------------------------------------------------------------------

bool rotateFW_pivot(float angleDeg, uint8_t speed, uint8_t stopPull = 0);

// -----------------------------------------------------------------------------
// Backward pivot rotation
// -----------------------------------------------------------------------------

bool rotateBW_pivot(float angleDeg, uint8_t speed, uint8_t stopPull = 0);

// -----------------------------------------------------------------------------
// Motor, turn, and rotate configuration
// -----------------------------------------------------------------------------

bool set_turn_motor(TurnMode mode, int8_t leftRatio, int8_t rightRatio);
void set_turn_overshoot(uint16_t overshootMs);
void set_turn_timeout(uint16_t timeoutMs);
void set_turn_approach_kp(float kp);
bool set_turn_approach(uint8_t forwardSpeed, uint8_t touchBrake);
bool set_turn_approach(uint8_t sensorExitSpeed, uint8_t distanceExitSpeed,
                       uint8_t touchBrake);
void set_turn_touch_brake_ms(uint16_t durationMs);
bool set_turn_line_search(uint8_t searchSpeed, uint16_t fastTurnMs);
bool set_rotate_fallback(uint16_t spin90Ms, uint16_t pivot90Ms,
                         uint8_t calibrationSpeed = 50);
