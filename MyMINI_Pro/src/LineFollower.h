#pragma once

#include <Arduino.h>
#include <type_traits>

// -----------------------------------------------------------------------------
// Sensor and turn types
// -----------------------------------------------------------------------------

enum LineExitSensor : uint8_t {
  f0 = 0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14, f15,
  b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14, b15,
  cl, cr
};

struct LineExitPair {
  LineExitSensor first;
  LineExitSensor second;
};

// f0 & f15 and f0 & 15 both select the front sensors 0 and 15.
constexpr LineExitPair operator&(LineExitSensor first, LineExitSensor second) {
  return {first, second};
}

constexpr LineExitPair operator&(LineExitSensor first, int secondChannel) {
  return {first, secondChannel >= 0 && secondChannel <= 15
      ? static_cast<LineExitSensor>(
            (first >= b0 && first <= b15 ? static_cast<int>(b0)
                                        : static_cast<int>(f0)) + secondChannel)
      : cr};
}

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
// Print only the signed PID error, one number per line, at 50 Hz while
// f_line()/b_line() runs. Enabling this disables verbose diagnostics.
void set_line_error_monitor(bool enabled);
// Buffer the latest control frames for f_line()/b_line() and the cl/cr turn
// approach without printing inside the sensor loop. A large error/measurement
// change or wide off-track black pattern freezes the trace after 48 more frames.
// Call print_line_intersection_trace() only
// after the blocking motion command returns; enabling resets the buffer and
// disables the live Serial diagnostics above. No sensor is read a second time.
void set_line_intersection_trace(bool enabled);
void print_line_intersection_trace(Print& output = Serial);
constexpr uint32_t LINE_TIMEOUT_MS = 15000;
constexpr uint32_t LINE_ACCEL_TIME_MS = 400;
constexpr float LINE_MM_PER_SECOND_AT_100 = 900.0f;
constexpr float LINE_DECEL_DISTANCE_MM = 120.0f;
constexpr float LINE_MIN_DECEL_FACTOR = 0.25f;
constexpr int16_t TRACK_WINDOW_RADIUS = 22;
constexpr float MAX_ERROR_RATE_PER_SECOND = 800.0f;

// f_line()/b_line() only. Default ramp: 0 to requested speed in 200 ms.
// Set the starting speed (0..100) in setup(); the faster requested wheel
// starts at min(startSpeed, requestedSpeed), and the other keeps its ratio.
// Both requested wheel speeds below 40 skip the ramp. A ramp time of 0 also
// disables the ramp and immediately uses the requested speed.
// The one-argument setting controls only f_line()/b_line() Kd (0..10):
// Kp comes from each command and Ki is zero. The two-argument overload remains
// available for sketches using the previous kpScale/ki API.
bool set_line_ramp_ms(uint16_t rampMs);
bool set_line_ramp_start_speed(int startSpeed);
// Scale the open-loop distance estimate for distance-ended f_line()/b_line().
// Default 1.0. If a 20 cm command travels 35 cm, start with 35/20 = 1.75.
// Valid finite range: greater than 0 through 5.0. Tune on the actual robot.
bool set_line_distance_scale(float scale);
// Distance-ended f_line()/b_line(). Pass centimeters x 100 as an integer:
// 200 = 2.00 cm, 150 = 1.50 cm. Default is 200 (2.00 cm);
// 0 disables the slowdown. Values longer than the target cover the whole
// command. Negative values are rejected without changing the setting.
bool set_line_decel_ramp_cm(int distanceCmTimes100);
// Reject old decimal calls: 2.0f must not silently become 0.02 cm.
bool set_line_decel_ramp_cm(float) = delete;
bool set_line_decel_ramp_cm(double) = delete;
bool set_line_pid_tuning(float kd);
bool set_line_pid_tuning(float kpScale, float ki);

// -----------------------------------------------------------------------------
// Forward line following
// -----------------------------------------------------------------------------
// One-shot PID target for the next f_line() or b_line() call, 0..100.
// 10 holds the line near the left sensors, 50 at center, 90 near the right.
// Invalid values act as 50. Every call restores positoin_error to 50 when it
// returns, including invalid-argument and sensor-error exits.
extern int positoin_error;

// For kp < 0.85, f_line()/b_line() cross the specified side-line patterns
// (left: 2..7 black, 14..15 white; right: 9..15 black, 0..1 white)
// with a +10 left-pattern or -10 right-pattern steering correction while the
// pattern is visible. If all sensors go white, pivot toward the side that most
// recently saw black until the main line is reacquired or the crossing times out.
// The selected exit sensor takes priority and is checked on every frame.
// Distance-ended commands cross either side.

void f_line(int sl, int sr, float kp, LineExitSensor exitSensor,
            uint8_t stopPull = 0);
// Exit when both selected sensors see black in the same frame, confirmed in
// two of three frames. Pair detection starts with the first sensor frame.
// Examples: f_line(60, 60, 2.0f, f0 & f15, 10);
//           f_line(60, 60, 2.0f, f0 & 15, 10);
void f_line(int sl, int sr, float kp, LineExitPair exitSensors,
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

// stopPull is the reverse motor pulse duration in milliseconds (0 skips it).
void fw_gyro(float targetAngle, uint8_t speed, float kp, float distanceCm,
             uint8_t stopPull = 0);

// -----------------------------------------------------------------------------
// Backward gyro movement
// -----------------------------------------------------------------------------

// stopPull is the reverse motor pulse duration in milliseconds (0 skips it).
void bw_gyro(float targetAngle, uint8_t speed, float kp, float distanceCm,
             uint8_t stopPull = 0);

// D gain for gyro heading hold, in motor-command units per degree/second.
// Defaults to 0.035 for both directions. One argument sets both; two let
// forward and backward motion use different values. Returns false for invalid
// values without changing either setting.
bool set_gyro_kd(float kd);
bool set_gyro_kd(float forwardKd, float backwardKd);

// -----------------------------------------------------------------------------
// Intersection turn
// -----------------------------------------------------------------------------

// The selected exit sensor must first clear the line (it and its inward
// neighbor read white in two frames), then see black in two of three frames.
// Only the selected sensor can stop the turn.
// stopPull is the reverse-brake duration in milliseconds (0..255); each
// wheel brakes with the opposite of its current turn motor command.
void turn(TurnMode mode, uint8_t speed, LineExitSensor exitSensor,
          uint8_t stopPull = 0);

// Absolute heading from the most recent wait_button() press: right positive,
// left negative, target -180..180 degrees. For fl/fr, follows the line until
// the crossing is confirmed, then advances for the configured overshoot time
// without further line checks. Slows near the target, then checks the settled
// heading and makes bounded corrections.
// brakeLeadDeg caps the predicted stopping lead (1..100 degrees); 0 uses the
// library cap of 20 degrees. It is not a fixed early-stop angle.
void turn_gyro(TurnMode mode, uint8_t speed, float targetDeg,
               uint8_t brakeLeadDeg = 0);

// -----------------------------------------------------------------------------
// Spin rotation
// angleDeg is the target heading (-180..180 degrees) from the latest
// successful wait_button()/gyro_reset() zero. Takes the shortest route;
// at an exact 180-degree tie, the target's sign selects right/left.
// A 0-degree target returns to the start heading. Requires a ready gyro and
// heading reference; returns false and brakes if either is unavailable.
// stopPull is the reverse-brake duration in milliseconds (0..100).
// The reverse pulse uses the opposite of the last motor command, then brakes.
// With IMU, returns immediately after braking; no 110 ms settle wait or
// post-brake correction. Crossing the target also brakes without reversing.
// -----------------------------------------------------------------------------

bool rotate_spin(uint8_t speed, float angleDeg, uint8_t stopPull = 0);

// -----------------------------------------------------------------------------
// Forward pivot rotation
// With IMU ready, angleDeg is an absolute heading from the latest
// wait_button() start. Without IMU, the timed fallback turns relatively.
// stopPull is the reverse-brake duration in milliseconds (0..100).
// The reverse pulse uses the opposite of the last motor command, then brakes.
// With IMU, this forward pivot returns as soon as the motor brakes; it does
// not wait 110 ms, make post-brake angle corrections, or reverse direction
// within the same call when a gyro sample crosses the target.
// -----------------------------------------------------------------------------

bool rotateFW_pivot(uint8_t speed, float angleDeg, uint8_t stopPull = 0);

// -----------------------------------------------------------------------------
// Backward pivot rotation
// With IMU ready, angleDeg is an absolute heading from the latest
// wait_button() start. Without IMU, the timed fallback turns relatively.
// stopPull is the reverse-brake duration in milliseconds (0..100).
// The reverse pulse uses the opposite of the last motor command, then brakes.
// -----------------------------------------------------------------------------

bool rotateBW_pivot(uint8_t speed, float angleDeg, uint8_t stopPull = 0);

// Old angle-first calls must fail rather than silently swap speed and angle.
template <typename OldAngle>
typename std::enable_if<std::is_floating_point<OldAngle>::value, bool>::type
rotate_spin(OldAngle, uint8_t, uint8_t = 0) = delete;
template <typename OldAngle>
typename std::enable_if<std::is_floating_point<OldAngle>::value, bool>::type
rotateFW_pivot(OldAngle, uint8_t, uint8_t = 0) = delete;
template <typename OldAngle>
typename std::enable_if<std::is_floating_point<OldAngle>::value, bool>::type
rotateBW_pivot(OldAngle, uint8_t, uint8_t = 0) = delete;

// -----------------------------------------------------------------------------
// Motor, turn, and rotate configuration
// -----------------------------------------------------------------------------

bool set_turn_motor(TurnMode mode, int8_t leftRatio, int8_t rightRatio);
void set_turn_overshoot(uint16_t overshootMs);
void set_turn_timeout(uint16_t timeoutMs);
void set_turn_approach_kp(float kp);
// Speeds apply immediately during the line-following approach. touchBrake
// is the reverse brake power before rotation, opposite the approach direction.
bool set_turn_approach(uint8_t forwardSpeed, uint8_t touchBrake);
bool set_turn_approach(uint8_t sensorExitSpeed, uint8_t distanceExitSpeed,
                       uint8_t touchBrake);
// Applies to fl/fr and turn_gyro(); turn(tcl/tcr) uses a fixed 10 ms pulse
// until set_turn_center_brake() configures its forward/backward pulses.
void set_turn_touch_brake_ms(uint16_t durationMs);
// turn(tcl/tcr): brake opposite the approach direction after cl/cr sees black.
// Set separate power (0..100) and duration (ms) for forward and backward
// approaches. With no call, both use set_turn_approach() power for 10 ms.
bool set_turn_center_brake(uint8_t forwardPower, uint16_t forwardMs,
                           uint8_t backwardPower, uint16_t backwardMs);
// turn() uses the requested speed for the first fastTurnMs, then caps it at
// searchSpeed while reading the stop sensor.
// Defaults: 40 and 60 ms.
bool set_turn_line_search(uint8_t searchSpeed, uint16_t fastTurnMs);
bool set_rotate_fallback(uint16_t spin90Ms, uint16_t pivot90Ms,
                          uint8_t calibrationSpeed = 50);
// Shared gyro PD tuning for rotate_spin(), rotateFW_pivot(), rotateBW_pivot().
// P uses angle error (degrees); D uses measured yaw rate (degrees/second).
// Defaults: kp=0.90, kd=0.035. Returns false for invalid values.
bool set_rotate_pid(float kp, float kd);
// rotate_spin() only: start reducing its speed cap when measured yaw rate
// predicts the target is slowdownMs away. Ramp smoothly toward slowSpeed
// during that time; 0 ms disables the extra slowdown. slowSpeed is 2..100.
// The command's own speed remains the upper cap. Default: disabled.
bool set_rotate_spin_slowdown(uint16_t slowdownMs, uint8_t slowSpeed);
