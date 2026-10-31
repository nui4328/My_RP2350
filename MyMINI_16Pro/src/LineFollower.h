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

// Explicit heading target; a plain float remains distance in centimeters.
// Target an absolute heading relative to the mission gyro zero, modulo 360.
// Positive yaw turns right; +180 and -180 select the same heading.
// The bool is kept for compatibility: true checks a heading crossing.
struct GyroAngle { float degrees; bool nearestEquivalent; };
constexpr GyroAngle gyro_angle(float degrees) { return {degrees, true}; }
// Use a specific cumulative yaw target, including turns beyond 360 degrees.
constexpr GyroAngle gyro_angle_raw(float degrees) { return {degrees, false}; }

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

// f_line() and b_line() follow the tracked black group with PID. After two
// narrow frames, the branch filter can exclude a new side wing from tracking
// until the original line returns for two narrow frames. Steering is bounded
// while that filter is active. Exit detection uses the sensor readings before
// branch filtering and takes priority over recovery when its condition is met.
// Sensor- and distance-ended f_line() share the following tracking behavior;
// b_line() and gyro-ended f_line() retain their separate recovery behavior.
// Outside recovery, black with no selected group produces zero PID error.
// With Kp > 0.95, a dominant corner arm or sole visible group can take over
// tracking when the old selection misses it. When all 16 sensors
// see white, it steers toward the most recently seen f0 or f15 when exactly
// one edge sensor was black. Inner-only and both-edge frames keep that
// direction. An all-white gap gets 30 ms of straight approach before turning,
// with sensor scanning continuing throughout. Capture starts only after a
// turn command and a group no wider than four channels; that clears the
// remembered edge. Kp captures with 25% base speed. Full PID resumes after
// three consecutive narrow frames within 10 units of the target position,
// and base speed returns over 80 ms. Wide patches and white gaps preserve
// the capture turn without another straight approach.

// All sensor-ended f_line calls count black from the first fresh sensor frame,
// including after any turn, without first requiring white. An existing black
// line can therefore finish the command after two confirmed frames.
// With f0 or f15, the selected edge and its inward neighbor (f0/f1 or f15/f14)
// must see black in the same frame; the exit needs two of three confirmed frames.
void f_line(int sl, int sr, float kp, LineExitSensor exitSensor,
            uint8_t stopPull = 0);
// Exit when both selected sensors see black in the same frame, confirmed in
// two of three frames. Pair detection starts with the first sensor frame.
// Examples: f_line(60, 60, 2.0f, f0 & f15, 10);
// Center pair: f_line(30, 30, 0.5f, cl & cr, 1); requires both black.
//           f_line(60, 60, 2.0f, f0 & 15, 10);
void f_line(int sl, int sr, float kp, LineExitPair exitSensors,
            uint8_t stopPull = 0);
void f_line(int sl, int sr, float kp, float distanceCm,
            uint8_t stopPull = 0);
// Per-call distance mode: follow the nearest group within the tracking window.
// Branch checks start disabled. After a valid tracked PID error first reaches
// [-10,+10], enable them from the next frame until this call returns.
// A no-line zero error does not enable branch checks.
// Refresh the main footprint from the nearest group within 22 position units,
// including a shifted group, before checking for branches. Connected distant
// edge arms cannot enlarge the remembered main footprint.
// A branch means this latest/nearby main group is black together with
// (f1 AND f2) on the left, or (f13 AND f14) on the right.
// f0/f15 alone do not confirm a branch.
// Required channels must be black in the same frame and lie outside the
// tracked main group on that side. All main positions are eligible; there is
// no f5/f10 cutoff. Follow only the main
// footprint in that frame. Group width alone does not reject a line. Remember
// f0/f15 and recover after an all-white gap using twice the steering gains.
// On all-white, short-brake for 20 ms while scanning, then spin with zero
// forward base speed. With no remembered edge, hold brake until line/timeout.
// Narrow capture and three centered frames apply;
// normal gains resume on confirmation. Timeout/distance/braking still apply.
// Without the old main footprint, use normal nearest-group selection.
// The next five-argument call automatically uses normal behavior.
enum class LineCrossingMode : uint8_t { crossing };
constexpr LineCrossingMode crossing_mode = LineCrossingMode::crossing;
void f_line(int sl, int sr, float kp, float distanceCm,
            uint8_t stopPull, LineCrossingMode mode);
// Follow the line until the requested heading is crossed in either direction.
// If the command starts at that heading, it first waits to leave by 5 degrees.
// gyro_angle(180) and gyro_angle(-180) stop at the same orientation.
// Use gyro_angle_raw() to target an exact unwrapped yaw instead.
// Uses the shared motion heading: real gyro, or calibrated wheel estimation
// selected by gyro_reset()/gyro_set(). A failed real read latches estimation.
// In estimated mode, completion is approximate; gyro_estimation_active()
// identifies this mode. Invalid arguments and motion timeouts still stop.
// Example: f_line(50, 50, 1.5f, gyro_angle(135.0f), 1);
void f_line(int sl, int sr, float kp, GyroAngle target,
            uint8_t stopPull = 0);
// True when the latest gyro-ended f_line reached its measured/estimated angle.
// This does not certify a physically measured angle in estimation mode.
bool f_line_gyro_succeeded();

// -----------------------------------------------------------------------------
// Backward line following
// -----------------------------------------------------------------------------

void b_line(int sl, int sr, float kp, LineExitSensor exitSensor,
            uint8_t stopPull = 0);
void b_line(int sl, int sr, float kp, float distanceCm,
            uint8_t stopPull = 0);

// -----------------------------------------------------------------------------
// All gyro movement uses measured or estimated heading after gyro_reset().
// Estimation integrates wheel commands/time; it cannot observe slip or drift.
// No readiness probes/recovery occur while estimation is active.
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

// For tfl/tfr approaching a new line, f0/f1 or f14/f15 and the chosen
// turn-stop sensor must see black together in 2 of 3 frames, then the
// confirmed pair and chosen sensor must see white in 2 of 3 frames.
// During rotation, the selected exit sensor and its inward neighbor first
// clear the line in two frames. For front/rear exits, the preceding sensor
// in the turn direction must then see black in two of three frames, without
// first requiring white. It must be seen black before the selected sensor.
// The selected sensor can then stop the turn on black in two of three frames.
// Alternatively, the selected sensor and its inward neighbor black together
// in two of three frames stop the turn without confirming the predecessor.
// For left b7, the predecessor is b6; for right b8, it is b9. End channels
// without a preceding sensor and center exits retain the selected-only rule.
// For tcl/tcr, cl/cr must see black in two of three frames before rotation.
// During tcl/tcr rotation, front/rear exits require the selected sensor and
// its inward neighbor black together in two of three frames; no predecessor.
// stopPull is the reverse-brake duration in milliseconds (0..255); each
// wheel brakes with the opposite of its current turn motor command. After
// that pulse (or immediately when stopPull is zero), both motors coast.
void turn(TurnMode mode, uint8_t speed, LineExitSensor exitSensor,
          uint8_t stopPull = 0);

// Absolute heading from the most recent wait_button() press: right positive,
// left negative, target -180..180 degrees. For fl/fr, follows the line until
// the crossing is confirmed, then advances for the configured overshoot time
// without further line checks. Slows near the target, then checks the settled
// heading and makes bounded corrections.
// brakeLeadDeg caps the predicted stopping lead (1..100 degrees); 0 uses the
// library cap of 20 degrees. It is not a fixed early-stop angle.
// Uses measured heading, or shared wheel estimation after reset timeout/read
// failure. Estimated success does not certify a physically measured angle.
// Returning hardware is used only after an explicit gyro_reset()/gyro_set().
void turn_gyro(TurnMode mode, uint8_t speed, float targetDeg,
               uint8_t brakeLeadDeg = 0);
// Result of the latest turn_gyro() call. Check before continuing a mission.
bool turn_gyro_succeeded();

// -----------------------------------------------------------------------------
// Spin rotation
// angleDeg is the target heading (-360..360 degrees) from the latest
// successful wait_button()/gyro_reset() zero. Equivalent headings such as
// 225 and -135 take the same shortest route;
// at an exact 180-degree tie, the target's sign selects right/left.
// A 0-degree target returns to the start heading. If the gyro or heading
// reference has never been established, the angle is a relative timed turn.
// After gyro_reset()/wait_button(), estimation retains an absolute reference
// shared across all motor commands, even when no physical gyro is available.
// stopPull is the reverse-brake duration in milliseconds (0..100).
// Runs at the requested speed for the first 70% of the turn, then at 40%
// (minimum motor drive 2). The gyro measures heading but does not tune speed.
// In the slow phase, spin starts braking early according to measured yaw rate
// and an estimated 50 ms stopping response; motor speed remains fixed.
// The reverse pulse uses the opposite of the slow speed, then brakes.
// With IMU, returns immediately after braking; no post-brake correction.
// A gyro read failure latches calibrated wheel estimation from the last
// heading. Timing is calibrated with set_rotate_fallback().
// -----------------------------------------------------------------------------

bool rotate_spin(uint8_t speed, float angleDeg, uint8_t stopPull = 0);

// -----------------------------------------------------------------------------
// Forward pivot rotation
// With a ready IMU, angleDeg is an absolute heading from the latest
// wait_button() start. Estimated mode retains that absolute reference.
// Only a call made before establishing any reference uses a relative turn.
// stopPull is the reverse-brake duration in milliseconds (0..100).
// Runs at the requested speed for the first 70% of the turn, then at 40%
// (minimum motor drive 2). The gyro measures heading but does not tune speed.
// The reverse pulse uses the opposite of the slow speed, then brakes.
// Without IMU, the same two phases use calibrated elapsed time. With IMU,
// the brake begins early based on yaw rate and estimated stopping response;
// this forward pivot returns immediately after braking, with no correction.
// A gyro read failure continues from the shared wheel heading estimate.
// -----------------------------------------------------------------------------

bool rotateFW_pivot(uint8_t speed, float angleDeg, uint8_t stopPull = 0);

// -----------------------------------------------------------------------------
// Backward pivot rotation
// With IMU ready, angleDeg is an absolute heading from the latest
// wait_button() start. Estimated mode retains that absolute reference.
// Only a call made before establishing any reference uses a relative turn.
// stopPull is the reverse-brake duration in milliseconds (0..100).
// Runs at the requested speed for the first 70% of the turn, then at 40%
// (minimum motor drive 2). The gyro measures heading but does not tune speed.
// The reverse pulse uses the opposite of the slow speed, then brakes.
// Without IMU, the same two phases use calibrated elapsed time. With IMU,
// the brake begins early based on yaw rate and estimated stopping response;
// returns immediately after braking, with no post-brake correction.
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
// Used for turn() line approaches except immediately after distance-ended
// f_line(), which inherits the effective gains from its final slowed frame.
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
// Legacy compatibility setter. All three rotate commands now use fixed
// 100%/40% speed phases, so these PID values no longer affect them.
// Returns false for invalid values.
bool set_rotate_pid(float kp, float kd);
// Legacy compatibility setter. Its values no longer affect rotate_spin(),
// which always switches to 40% speed with 30% of its turn remaining.
bool set_rotate_spin_slowdown(uint16_t slowdownMs, uint8_t slowSpeed);
