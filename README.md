# Pico2 My Mini Pro

Arduino library for a robot using two 74HC4067 16-channel analog multiplexers:

- a 16-channel front line-sensor array;
- a 16-channel rear line-sensor array;
- CAT24C128 external I2C EEPROM for calibration values;
- one direct, active-low front calibration button; and
- one optional rear calibration button read through an MCP3421.

## Install

Place this complete folder in the Arduino `libraries` directory, restart Arduino
IDE, then open **File > Examples > Pico2 My Mini Pro > MyMiniPro**.

The single bundled example has two tabs:

- `MyMiniPro.ino`: starts the robot, waits for Start, and reads/displays Front,
  Rear, and underbody sensors every 100 ms in `loop()`. Motors remain stopped;
  servos are not attached. Open Serial Monitor at 115200 baud.
- `Functions.ino`: commented, copy-ready commands for sensors, calibration,
  EEPROM, motors, servos, Start, buzzer, battery LEDs, and voltage compensation.
  This tab does not execute any commands automatically.

Previous examples have been removed. For the supplied Pico 2 wiring, declare:

```cpp
#include <Pico2_MyMiniPro.h>

Pico2MyMiniPro robot;
```

`robot.begin()` configures the two MUXes, I2C on GP4/GP5, EEPROM, buzzer on
GP9, Start button on GP2, front calibration button on GP3, and the rear MCP3421
calibration button.

## Motors

`robot.begin()` configures the VNH7070ASTR motor-driver control pins and 20 kHz
PWM. On Pico 2, the PWM duty has 12-bit resolution (raw range `0` to `4095`),
while the `Motor()` API remains percentage-based:

```cpp
robot.Motor(60, 60);    // both motors forward at 60%
robot.Motor(-50, -50);  // both motors reverse at 50%
robot.Motor(0, 0);      // stop both motors
```

The command is `Motor(left, right)` and each value is limited to `-100…100`.
The magnitude is mapped linearly to raw PWM duty: `0` maps to `0`, `50` maps
to approximately `2047`, and `100` maps to `4095`; the sign controls direction.
In Arduino-Pico, `analogWriteResolution(12)` is the equivalent of setting
`analogWriteRange(4095)`. At the core's standard 133 MHz RP2040 and 150 MHz
RP2350 system clocks, that range is attainable with the 20 kHz PWM request, so
the core does not reduce the 12-bit duty range. If a sketch lowers the system
clock below about 82 MHz, Arduino-Pico preserves the requested frequency by
reducing the hardware range internally; the `Motor()` input scale still remains
`-100` to `100`.

The PWM divider has finite fractional precision, so 20 kHz is the configured
target rather than an exact waveform-frequency guarantee. Do not call
`analogWriteFreq()`, `analogWriteRange()`, or `analogWriteResolution()` later
for another purpose unless you intend to reconfigure the motor PWM globally.

For the supplied Pico 2 map, left uses GP19/GP20/GP21 and right uses
GP6/GP7/GP8 as `PWM`/`INA`/`INB`; confirm those connections against the actual
VNH7070ASTR boards before running motors.

## Servos

`Pico2MyMiniPro` provides five fixed Servo GPIOs. They use the Arduino-Pico
core's PIO-backed `Servo` support, so their control pulses are separate from
the motor driver's 20 kHz PWM.

For a single Front/Rear line-sensor channel, use the simple forms
`robot.readSensorFront(channel)`, `robot.minSensorFront(channel)`,
`robot.maxSensorFront(channel)`, and their `Rear` equivalents. Valid channel
numbers are `0` through `15`; an invalid channel returns `0`.

| Servo GPIO | GPIO |
| --- | --- |
| `18` | GP18 |
| `22` | GP22 |
| `28` | GP28 |
| `0` | GP0 |
| `1` | GP1 |

No servo is attached in `robot.begin()`: its GPIO remains inactive until the
first `servo()` call for that GPIO. This makes servo use opt-in and prevents a
boot-time movement command. GP0 and GP1 are user-confirmed wiring; do not
reuse either pin for other hardware, and verify them on the assembled robot.

```cpp
// Optional: set calibrated endpoints before the first angle command.
robot.setServoPulseLimits(18, 1000, 2000);
robot.servo(18, 90);   // attaches GP18, then commands 90°

robot.servo(22, 0);    // GP22
robot.detachServo(18); // stop sending pulses to GP18
```

Only GPIO `18`, `22`, `28`, `0`, and `1` are accepted. `servo(gpio, degrees)`
returns `false` and does nothing for another GPIO; otherwise it clamps degrees
below `0` to `0` and above `180` to `180`, then returns `true`. It also returns
`false` without driving the pin if the Arduino-Pico Servo backend cannot obtain
a PIO state machine.
`detachServo(gpio)` also returns `false` for an unconfigured GPIO.
`servoAttached(gpio)` and `servoAngle(gpio)` (returns `-1` while detached or
for an unconfigured GPIO) expose the current state safely.

The installed Arduino-Pico Servo defaults are 1000–2000 microseconds. To use a servo's
calibrated safe pulse range, call `setServoPulseLimits(gpio, minimumUs,
maximumUs)` before it is attached (or after `detachServo()`). It returns false
for an unconfigured GPIO, equal/reversed limits, or an attached servo. Power
servos from a regulated external supply sized for their stall current, connect
its ground to Pico GND, and confirm that the servo accepts a 3.3 V control
signal; never power a servo from the Pico 3V3 rail.

### Motor voltage compensation

Pico 2 enables compensation with a 12.4 V reference. Call this once per main
loop, outside time-critical sensor sampling:

```cpp
robot.serviceMotorVoltageCompensation();
```

It starts an ADS1115 conversion at most every 500 ms, then polls for completion
on a later pass. `Motor()` neither reads battery voltage nor scales its input.
The cached gain is `12.4/Vfiltered`, never less than 1.0; invalid readings and
USB readings below 6.5 V use a gain of 1.0. Apply that gain to only the base
speed before sending motor commands:

```cpp
const int compensatedBase = robot.compensateBaseMotorSpeed(baseSpeed);
robot.Motor(compensatedBase, compensatedBase);
```

`Motor()` still clamps each final command to ±100. You can change the reference
or a longer sample interval with
`setMotorVoltageCompensation(true, referenceVolts, sampleIntervalMs,
maximumCompensationPercent)`. The default maximum compensation is deliberately
conservative at 5% (`maximumCompensationPercent = 5`); intervals below 500 ms are
intentionally raised to 500 ms. This is an open-loop approximation, so tune the
cap from real driving tests rather than assuming it holds motor RPM under load.

The service uses a single short I2C transaction to start conversion, then polls
at least 2 ms later and reads the result only when ready; it contains no
conversion wait loop or delay. I2C transactions themselves remain synchronous,
so call the service after the sensor sampling rather than inside that hot path.

The library currently provides sensor acquisition, calibration, and direct motor
commands. Automatic line-following controllers have been removed.

## Battery voltage

ADS1115 AIN0 measures the battery-divider output. The supplied Pico 2 setup
uses the calibrated divider ratio `4.0`. It also applies a two-point correction
based on 6.0 V reading as 5.71 V and 12.4 V reading as 12.35 V. Read the
actual battery voltage with:

```cpp
float batteryVoltage = robot.readBatteryVoltage();
```

To calibrate the divider ratio again with a multimeter, call
`robot.setBatteryDividerRatio(actualBatteryVolts / measuredAin0Volts)`.
To set a gain and offset correction, call
`robot.setBatteryCalibration(gain, offsetVolts)`.

### ADS1115 auxiliary inputs

The battery divider uses ADS1115 `AIN0`. Two of the remaining single-ended
inputs are the underbody white/black sensors beside the left and right wheels.
The library returns their raw signed conversion counts, not a white/black
decision:

```cpp
int16_t left = robot.readAdcL();                  // AIN1, left-wheel sensor
int16_t right = robot.readAdcR();                 // AIN2, right-wheel sensor
int16_t ain3 = robot.readAds1115Ain3Raw();      // AIN3, no board-specific role
```

Each call uses the same single-shot ADS1115 configuration as `AIN0`: ±4.096 V,
860 samples/s, and the configured I2C address (default `0x48`). Despite the
short method names, `readAdcL()` and `readAdcR()` return signed raw ADC codes,
not a white/black decision. They return `INT16_MIN` if I2C fails or the
conversion is not ready within the synchronous 10 ms timeout. Establish the
white/black polarity and threshold from readings on the actual robot before
using either underbody value in control logic.

### Underbody-sensor calibration

While `waitButton()` is active, a short press of the Start button starts the
robot normally. Hold that same button continuously for 5 seconds to begin a
non-blocking AIN1/AIN2 calibration instead. Keep both sensors moving across
their intended white and black surfaces during the configured calibration time
(5 seconds in the Pico 2 setup). The motors remain stopped while the press is
debounced, while it is held, and during calibration. After a long press starts
calibration, release the button and press it again briefly to start the robot.
The Front and Rear calibration buttons retain their original behavior.

For a later manual re-calibration, the same non-blocking programmatic workflow
remains available:

```cpp
robot.startUnderbodyCalibration();

// Call once per loop. Keep motor control paused while this returns true.
if (robot.serviceUnderbodyCalibration()) return;

if (robot.hasUnderbodyCalibration()) {
  uint16_t left = robot.readAdcLNormalized();
  uint16_t right = robot.readAdcRNormalized();
}
```

`readAdcLNormalized()` and `readAdcRNormalized()` return `0` through `1000`,
clamped at their saved endpoints. Without a valid saved range, or when the ADS
read fails, they return `MyMiniPro::kUncalibratedNormalizedValue` (`65535`),
so no division by zero occurs. Retrieve the saved raw extrema with
`adcLMinimum()`, `adcLMaximum()`, `adcRMinimum()`, and `adcRMaximum()`; each
returns `INT16_MIN` if the paired calibration is not valid.

The ranges use a separate `UBL1` record in CAT24C128 immediately after the
existing `CAL1` Front/Rear record. Older firmware ignores that extension;
newer firmware continues loading valid legacy Front/Rear data unchanged. A
bad or absent `UBL1` record marks only the underbody calibration invalid.
If the long-press pass receives invalid readings or its EEPROM save fails, the
Serial Monitor reports `ADC CAL: REQUIRED`; it never starts the motors by
itself. The user can still observe the raw ADC values, use the existing
Front/Rear calibration controls, and choose when to make a normal short Start
press.

### PCF8574P battery LED bar

The library can drive eight battery LEDs connected to PCF8574P P0 through P7
without an additional PCF8574 library. The PCF8574 shares the I2C bus already
configured on Pico 2 GP4/GP5. After `robot.begin()`, configure the board with
its actual A0-A2 address and LED polarity:

```cpp
robot.beginBatteryLevelLeds(0x20, true);
//                      address  ^ LEDs turn on when a PCF pin is LOW
```

`PCF8574P` addresses are normally `0x20` through `0x27`, depending on A0-A2;
use `0x20` when all three address pins are connected to GND.
Do not assume `0x20`, or active-low versus active-high, without checking the
specific board's wiring. Active-low is common when the PCF8574 sinks LED
current, but it is not universal.

Call either form below regularly:

```cpp
robot.updateBatteryLevelLeds();                 // reads battery voltage itself
robot.updateBatteryLevelLeds(batteryVoltage);   // uses an existing reading
```

The display uses P0 first: below 11.0 V all LEDs are off; 11.0 V lights P0;
each additional 0.2 V lights one more LED; and 12.4 V or above lights P0-P7.
`beginBatteryLevelLeds()` displays the first reading immediately. During
`waitButton()`, the bar refreshes every 500 ms. After Start, call
`updateBatteryLevelLeds()` from your sketch. The default waiting setup uses
address `0x20` and active-high LEDs. From 10.5
V to below 11.2 V, the configured buzzer sounds a 200 ms, 1.2 kHz warning every
1.5 seconds. Below 10.5 V, it instead repeats a longer 700 ms warning every
1.5 seconds. The pattern changes immediately at the threshold and re-arms after
the voltage returns to 11.2 V or higher. A reading below 6.5 V is treated as
USB power: it keeps the existing LED mapping but suppresses and resets all
battery-warning beeps.

## Fast operation

Call `lineSensors.scan(Direction::Forward)` while moving forward, and
`lineSensors.scan(Direction::Reverse)` while reversing. Only the active
16-channel MUX is scanned. Avoid `Serial.print()` in the motor control loop.

Call `lineSensors.serviceCalibration()` at the start of every `loop()`. One
button press starts a timed calibration; the library records minimum and
maximum values and stores them in CAT24C128 after it completes.

The Pico 2 configuration uses a 5-second calibration. Front, rear, and underbody
calibration use 100 ms progress beeps at 1800 Hz every 350 ms, extended from
25 ms to make them easier to hear without blocking sensor sampling. Two longer
beeps confirm that the values have been saved. Longer beeps do not increase
the GPIO signal amplitude; increasing drive voltage requires an appropriate
external buzzer driver rather than a software volume setting.

## Start button

For a button connected between Pico 2 GP2 and GND, add this in `setup()`:

```cpp
lineSensors.beginStartButton(2);
```

Then call `serviceStartButton()` in every `loop()`. `robotStarted()` is false
after boot and becomes true after one press, following a 300 ms confirmation
beep. Use the waiting mode below to keep motors stopped until Start.

For the complete wait mode, use this one line at the start of `loop()`:

```cpp
if (!lineSensors.waitButton(Serial, 100)) return;
```

It prints both arrays while waiting, accepts either calibration button, and
continues to the robot-control code after the GP2 Start button is pressed.

## Serial Monitor waiting mode

Before Start, `waitButton(Serial, 100)` prints one row every 100 ms in this form:

```text
F: 512,498,...,530 | R: 476,501,...,489 | ADC NL: 524 | ADC NR: 481
```

`F` contains front channels 0–15 and `R` contains rear channels 0–15, each
mapped from its saved minimum and maximum to `0–1000`. `ADC NL` and `ADC NR`
are the normalized ADS1115 readings from AIN1 (left underbody sensor) and
AIN2 (right underbody sensor); raw MCP3421 and ADS1115 readings are not printed
on the recurring line. During an active programmatic underbody calibration the row
shows `ADC CAL: RUNNING`; while Start is being held it shows
`ADC CAL HOLD: n/5s`; after calibration is valid, it also shows `ADC NL` and
`ADC NR` as normalized 0-1000 values. Otherwise it shows
`ADC CAL: REQUIRED`. The Front or Rear calibration button can be pressed while
this display is running; the selected array calibrates for 5 seconds, saves to
EEPROM, then the live display resumes. After GP2 Start is pressed,
`waitButton()` returns true. Your sketch can then scan the array needed for
the driving direction.

For the supplied MCP3421 button circuit, the expected readings are about `1`
when pressed and `2047` when released. The library therefore uses `1000` as
the rear-button threshold: a value below it starts rear calibration.

## Values for motor control

`values(array, channel)` returns, in order:

1. `current` — current raw ADC value.
2. `maximum` — calibration maximum.
3. `minimum` — calibration minimum.

`normalized(array, channel)` maps the calibrated result to 0–1000.

## Passive buzzer

For a passive buzzer wired to Pico 2 GP9, add this in `setup()` after
`lineSensors.begin(Wire)`:

```cpp
lineSensors.beginBuzzer(9);
lineSensors.playStartupMelody();  // Short non-blocking startup melody
lineSensors.beep(1500, 100);      // 1.5 kHz for 100 ms, non-blocking
```

Use `playTone(frequency)` for a continuous tone and `stopBuzzer()` to stop it.
Timed `beep()` calls stop automatically when `serviceCalibration()` is called
regularly from `loop()`.

The startup melody uses a higher octave for better perceived loudness. Software
cannot increase the GPIO voltage; for substantially more volume, use a
transistor driver and power the buzzer from its rated supply voltage.

## Forward PD line following: F_Line

**Gain migration:** F_Line now controls with position error **-50..+50**.
Divide every old F_Line KP, KD and recovery KP threshold by50 before using this
library. There is no automatic legacy-gain detection. For example5/0.05/4 becomes
0.1/0.001/0.08. Old values are still interpreted
as NEW gains and can make control50 times stronger; migrate external sketches too.
KP, KD and recovery threshold accept every finite signed float with no tuning
ceiling. Defaults are KD0.001 and threshold0.4. NaN/Inf KP returns InvalidArgument
with stopped motors; NaN/Inf setters return false and retain the previous setting.

```cpp
using Sensor = MyMiniProForward::Sensor;
MyMiniPro::ForwardSettings cfg;
cfg.maximumRunMs = 3000;
robot.setFLineKd(0.001); // setup;0 disables D
// Replace zero with measured cm/s divided by command percent before distance use.
cfg.centimetersPerSecondPerPercent = 0.0;
robot.setForwardSettings(cfg);
auto reason = robot.F_Line(20, 20, 0.1, 10);          // estimated 10 cm
reason = robot.F_Line(20, 20, 0.1, Sensor::f0);       // next f0 line edge
reason = robot.F_Line(20, 20, 0.1, Sensor::cl);       // calibrated ADC L
reason = robot.F_Line(20, 20, 0.1, Sensor::b15);      // Rear channel 15
```

There are exactly four arguments. Numeric integers/floats mean centimeters;
`Sensor` is a scoped enum, so sensor names cannot accidentally become distances.
F0..F15 map to Front 0..15, CL/CR to ADS1115 AIN1/AIN2 with saved ADC L/R
calibration, and B0..B15 to Rear 0..15. A sensor exit needs no distance model.
The result may be ignored by ordinary calling code.

Setup and semantics:

- Call `begin()` and load/calibrate sensors first. All 16 front channels require
  a span of at least 100 ADC counts; rear exits also require that span on the
  selected channel. CL/CR require valid saved underbody calibration.
- Defaults assume a low reading on the line. Set `frontLineHigh`, `rearLineHigh`,
  and `centerLineHigh` for your surfaces. Stored extrema alone do not identify
  which surface is the line. `frontMap` lists physical left-to-right channels;
  the default is 0..15. Confirm physical mapping and forward motor polarity.
- Existing smoothed readings feed a forward-only group tracker which reuses the weighted line locator for each visible group.
  Control position error is -50..+50; positive means line on the right. Left PWM increases and
  right decreases by the PD correction described below. Wheel commands clamp to -100..100 before voltage
  compensation; no reversal. Input speeds are 1..100; kp accepts every finite signed float.
- A sensor must first read clear (line strength <=400) continuously for20ms.
  Then default line confirmation0ms accepts the first scan with strength>=600.
  Set `setFLineSensorDebounceMs(ms)` for additional stable-line confirmation;
  `ForwardSettings::debounceMs` controls clear arming only. Intermediate readings
  reset the current stability timer. Starting on a line never ends immediately:
  leave it before detecting the next line. F7/F8 may remain on the tracked line
  and never rearm. Each exit detector uses its own channel's calibrated
  raw sample captured during the same scan; steering continues using its EMA.
- Every four-argument return stops both motors. Reasons are `DistanceReached`, `SensorDetected`,
  `InvalidArgument`, `CalibrationNotReady`, `DistanceNotCalibrated`, `LineLost`,
  `SensorError`, `Timeout`, `Stopped`, and `MotorsNotReady` (enum order).
  Temporary line loss now enters recovery and continues scanning; see phase two below.
  Junctions retain continuity. Exit gates remain independent of recovery.
- Runs are blocking. Numeric distance uses `maximumRunMs` (default 3000, allowed 1..60000). Sensor mode has no default elapsed-time limit (`sensorMaximumRunMs=0`). For a bounded bench test only, set `sensorMaximumRunMs` to 1..60000 explicitly.
  Hardware Start stops after its initial release; the initial held launch press
  is ignored. `stopRequested` is an optional fast callback for serial/other stop
  input. This call does not wait for Start or service calibration buttons.
  Timeouts/stops are checked between synchronous sensor operations, so latency
  can include an ADC/array scan. The caller must arm each movement explicitly.

Distance is an estimate without encoders. No guessed default coefficient is
provided: a positive `centimetersPerSecondPerPercent` must be measured at the
intended operating speed, surface, load and compensation configuration. Measure
straight travel distance/time, then divide cm/s by the nominal percent command.
A single linear coefficient is approximate, especially near motor deadband,
curves, slip, or PWM saturation. Validate several short distances on the floor.
`F_Line` services battery compensation each iteration and explicitly compensates
both motor commands because `Motor()` does not do this itself. It integrates the
previous effective mean command (after clipping, divided by cached voltage gain)
against elapsed time, avoiding counting increased compensation as extra travel.
USB/invalid battery readings retain the library's gain=1 fallback. Distance tests
with lifted wheels validate timing only. Zero cm stops immediately after setup
validation; an unavailable distance calibration is still reported.

See `examples/ForwardLineTest/ForwardLineTest.ino`: no movement at boot, 20 percent
base speed, KP=0.1 and KD=0.001 (wheel commands include PD and voltage correction), 1.5-second cap, one command
per run, serial `x` or Start to stop. First test with wheels lifted and a valid
front line; manually move the selected exit sensor clear then onto the line.
Test each sensor command separately. Distance mode stays disabled until the
coefficient is measured and entered. Floor tests require a clear short track.


Including `Pico2_MyMiniPro.h` exposes typed constexpr short names F0..F15, CL, CR,
and B0..B15: `robot.F_Line(40, 40, 0.1, f15)`. Existing `Sensor::` names still work.
Arduino's binary names B0/B1/B10/B11 collide with rear names. Only these four use
token macros pointing to typed constexpr aliases (also preserving `Sensor::B0`);
use standard binary literals `0b0`, `0b1`, `0b10`, `0b11` for numbers instead.
Numeric final arguments still select distance mode. `MyMiniPro/Functions` contains copyable blocks with global/setup/loop
placement. Synchronous ADC reads cancel a pending battery conversion so a center
reading cannot be mistaken for battery voltage; sampling resumes at the existing
rate limit.

### Diagnosing early sensor exits

`MyMiniProForward::resultName(result)` returns a readable reason for diagnostic
sketches; the main MyMiniPro example is silent.
`SensorDetected` means an eligible exit sensor confirmed clear then line. `LineLost`
remains an enum for source compatibility, but temporary line loss now recovers.
`CalibrationNotReady`, `SensorError`, `InvalidArgument`, and
`MotorsNotReady` identify setup or read faults. `Stopped` means Start or the stop
callback requested a stop. Sensor mode no longer ends after 3 seconds by default.
`Timeout` in sensor mode requires a nonzero explicit `sensorMaximumRunMs`.
The dedicated ForwardLineTest enables a 1500ms limit for lifted-wheel testing;
the main MyMiniPro sketch uses the default sensor behavior, with no time limit.

### Front exit channel identity

`Sensor::F0` means the same MUX channel as `readSensorFront(0)` and the first
number in the `F:` live-reading list; `Sensor::F15` means channel 15/the last
number. `frontMap` changes the left-to-right steering calculation only, never
the exit condition. The recorded RobotConfig profile says channel 0 is leftmost
and 15 rightmost; physical labels/wiring on a different assembly must be checked
against live readings rather than reversing the channel numbers by guesswork.

Use ForwardLineTest for selected-channel, result and cached-reading diagnostics.
`SensorDetected` confirms the selected exit; the main example has no Serial output.
With wheels lifted, expose one outer channel at a time. Moving the whole array
off the line now exercises recovery, so stop via Start or the stop callback.

### 400us diagnostic test profile

Both MyMiniPro and ForwardLineTest now call `setMuxSettleMicroseconds(400)`
before F_Line. This matches the existing PID tuner profile and gives the analog
signal more time after switching, including the scan wrap from channel15 to0.
The global library default remains12us. This is a diagnostic mitigation, not
proof that crosstalk caused the reported stop; no channel numbers are reversed.

Recalibrate FRONT at400us before testing: open MyMiniPro, keep motors stopped
in waitButton mode, run the front calibration button sequence and sweep both
surfaces, then press Start. For ForwardLineTest, first save that front calibration
using MyMiniPro at400us, then upload the dedicated test. Rear exits should also
have their rear calibration checked/repeated at400us. Changing the settle time
does not invalidate or label saved EEPROM ranges automatically, so this manual
step is required. Keep the front tracking array on a valid line during the test.
A front scan now includes at least6.4ms of settling; exit debounce still uses
elapsed milliseconds. No extra ADC read is discarded: this small change reuses
the existing settling API and measured-profile workflow first.

### Side branches and crossing lines

F_Line now uses a forward-only continuity tracker; the PID tuner locator is
unchanged. Adjacent sensors with line strength >200 form candidate groups. Each
group uses the existing weighted centroid and strength>=600 validity thresholds.
The nearest group to the previous line position is followed; interval distance includes a one-sensor-spacing switching penalty for groups not overlapping the previous group. Centroid distance breaks ties, then physical scan order. On startup, a narrow group nearest center is chosen. A broad group with
more than8 active sensors can retain the previous heading within its visible
span, but a broad startup with no previous heading is rejected. History is local
to each F_Line call and does not carry stale directions between runs.

Acceptance sequence: choose F0, establish a centered main line, cross the F15
side branch first, continue along the visible original path, then cross F0.
Only F0's stable clear-to-line gate returns SensorDetected. Multiple groups or
a broad junction retain visible-path continuity. When no group remains visible,
the recovery policy uses the last normal error without changing the tracker's
branch history. This rule favors the nearest existing trajectory rather than
choosing a named route at complex junctions.

### KD without changing the four-argument call

```cpp
// setup:
robot.setFLineKd(0.001);
Serial.println(robot.fLineKd());
// Existing call shape is unchanged:
auto reason = robot.F_Line(20, 20, 0.1, Sensor::f0);
```

`setFLineKd(kd)` returns true for every finite signed float; NaN/Inf return false
and leave the previous setting unchanged. Default KD is0.001. Use KD=0 for the
previous P-only output; source compatibility is preserved, but nonzero defaultD
intentionally changes transient control. KD is captured once per F_Line call,
independent of setForwardSettings. No derivative history carries across calls.

The tracker keeps its internal normalized-1..+1 geometry. At the control boundary,
F_Line multiplies that position by50, so recovery and PD use-50..+50. At front-scan completion,
F_Line timestamps it with micros() and computes:

- dt = elapsed microseconds /1000000
- slope = (error - previousError) /dt, in position units per second
- filteredD += dt/(0.020 +dt) * (slope -filteredD)
- correction = kp*error +kd*filteredD
- left = leftSpeed +correction; right = rightSpeed -correction

KD units are PWM percentage-point seconds per scale-50 position unit. The20ms
filter and50ms maximum sample gap are unchanged. The separate PID tuner keeps
its own normalized scale and gains; its gains are not directly interchangeable. A rising positive error increases correction;
when the error falls toward center, D opposes the P correction. Filtering has a
time constant, while dividing by actual dt keeps derivative units independent
of scan rate.

First sample, elapsed time below100us (including repeated timestamps), or a gap
over50ms uses P only and resets D at the current error/time. The next normal
sample resumes without a stale-history kick; timer wrap is supported. Wheels
clamp to-100..100 before existing voltage compensation and final PWM clipping,
allowing a reverse command during following. Loss/reacquisition resets D to avoid
a synthetic-error kick; sensor exits and junction continuity remain independent.

### Avoiding the initial pull toward a side branch

The tracker remembers the previously selected group and separate branch groups.
If they merge through weak support (>200), it holds the main-path heading even
when fewer than9 sensors exceed the strong-line threshold. The hold remains
while the group is wide and releases when a narrow group reappears (pre-merge
width plus at most one sensor, with no new merge/broad pattern). The held heading
is constrained to the visible group's span. This prevents a changing merged
centroid from pulling steering into a side branch before the full intersection.

Regression sequence now includes F15, then F14, then F13 becoming dark while
the center stays visible; weak bridge readings connect the groups next; a broad
crossing follows; finally F0 becomes dark. At KP0.3/KD0.001 the simulated commands
remain20/20 through branch onset/merge and only F0 ends the call. The unguarded
merged centroid in this fixture shifts right by more than0.2 normalized error,
so the test exercises an actual steering disturbance. Recovery tests verify that
after a narrow path returns, an ordinary curve is followed again. All-white
continues to fault. The smoothing rule still needs validation on the real robot.

### Fast sensor exits without reducing MUX settling

```cpp
robot.setMuxSettleMicroseconds(400); // microseconds; recalibrate at this setting
robot.setFLineSensorDebounceMs(0);   // milliseconds;0=first qualifying scan
Serial.println(robot.fLineSensorDebounceMs());
```

The confirmation setter accepts0..60000ms, rejects larger values without changing
its previous value, and is captured per F_Line call. It is independent of KD,
MUX settling, setForwardSettings, and sensor-mode timeout. Clear arming still
requires20ms by default. Once detected, the event is latched until the call ends.

At400us per channel,16 channels require at least6.4ms plus ADC/loop overhead.
A20ms line-confirmation period needs several scans and can miss a narrow crossing
at speed40. A two-scan requirement can also miss a one-scan pulse, so the new
fast default is0ms. Tracking EMA50% can reduce a single full-strength pulse to
only500 (below the600 threshold); exit detection therefore captures the selected and eligible inward-neighbor
raw conversion before EMA, with no extra ADC reads. It applies the same saved
calibration and polarity; PD/junction tracking retain their filtered readings.

`lastFLineSensorStrength()` reports the selected fresh calibrated strength:
0=clear,1000=line,65535=no valid selected sample. It can differ from the cached
tracking `normalized()` value; it remains the selected value even if its neighbor triggers. Diagnostic examples print MUX us, confirmation ms,
KP/KD, and this strength. Subthreshold noise does not trigger, and clear arming
prevents an initial line from triggering. A full-strength one-scan noise spike
cannot be distinguished from a real one-scan crossing; choose a nonzero
confirmation time if needed, accepting the narrow-pulse tradeoff. Pulses shorter
than the sampling interval can still fall between samples; verify on the robot.

### Starting a distance-mode test

Yes: `robot.F_Line(40,40, 0.3,20)` requests approximately20cm. A numeric argument
selects distance mode; a scoped Sensor value selects sensor mode. First configure
a positive measured coefficient; its default0 returns DistanceNotCalibrated.

```cpp
// In setup after begin(), before the run:
MyMiniPro::ForwardSettings distanceSettings;
distanceSettings.centimetersPerSecondPerPercent = measuredCmPerSecondPerPercent;
distanceSettings.maximumRunMs = 1500;
robot.setForwardSettings(distanceSettings);
// After the normal Start/calibration gate, run once:
auto result = robot.F_Line(40,40, 0.3,2.0); // first test:2cm
robot.stopMotors();
Serial.println(MyMiniProForward::resultName(result));
```

Declare `constexpr float measuredCmPerSecondPerPercent=0.0;` globally and replace
zero only with a measured value. The Functions tab has the full copyable blocks.
The zero placeholder deliberately prevents an uncalibrated distance run.

Measure physical travelD(cm) and running timet(seconds) using the working sensor
mode on a straight line at the intended nominal speed. For equal40/40 commands,
coefficient=D/(t*40). Example arithmetic24cm over2s at40 gives0.30; this is not a
calibration value for this robot. Capture millis immediately before/after F_Line
and measure the chassis stop point before any subsequent reverse/braking pulse.
Repeat measurements, then validate short runs. Different speeds, acceleration,
curves and slip limit a single linear estimate; prefer the same operating speed.

Keep battery compensation configured the same during measurement and use. F_Line
services/applies it itself; supply nominal PWM percentages and do not multiply
them or the coefficient by its gain. Internally the estimate integrates the mean
applied command divided by cached gain, accounting for clipping without counting
compensation as extra travel. Calibration with plain Motor() alone would need the
same service/compensation logic and otherwise does not match this path.

### Guided measurement example (46mm wheels)

Open `ForwardDistanceCalibration`, Serial115200 with newline. After the normal
Front calibration/Start gate, send `m` for an explicit bounded0.5second run at40/40,
measure the chassis travel, then send `c <actual cm>`. It prints the coefficient
constant to copy into MyMiniPro. Send `d 2` to test2cm (1.5second maximum), or `x`
to stop. The coefficient starts at0 and numeric mode cannot run until calibrated.
It is RAM-only until copied; no guessed value is installed.

`MyMiniProForward::distanceCoefficientFromMeasurement(cm, elapsedMs, meanPercent)`
is the pure calculation helper; it returns NAN for invalid input. MyMiniPro uses
literal settings in setup(), runs once after Start and emits no Serial output.
A silent Stream discards waitButton telemetry while keeping calibration handling.
This robot's initial45cm in500ms measurement gave2.25, but
the20cm test traveled15cm. MyMiniPro now uses the measured correction
`settings.centimetersPerSecondPerPercent=1.6875`:2.25*(15/20), in cm/s/percent.
A numeric20cm call at40/40 has nominal constant-speed duration296.3ms (with startup/deceleration disabled); the3000ms distance safety
limit allows normal `DistanceReached` completion. The equivalent baseline example instead
calls `robot.F_Line(40, 40, 0.1, f15, fr, 90, f11, 40)`; the action owns turning and
braking, with no manual post-pulse. Sensor mode has no default timeout; distance settings
do not cap it. KD0.001, MUX100us, smoothing50 and confirmation0 are currently configured.
The pulse affects the final physical stopping position.
This coefficient is specific to this robot and its test conditions. Repeat3 runs
and average their calculated coefficients later; use actual elapsed time for each.

The supplied46mm wheel diameter corresponds to about144.513mm/revolution. Without
encoder counts or measured RPM, that is not enough to infer distance from PWM.
The calibration example displays this geometry but derives k from measured D/t.
See its README for the complete procedure and compensation notes.


## F_Line distance actions: NS, CS, CP, FS and FP

F_Line's PD output permits either wheel to reverse: left = baseLeft + correction,
right = baseRight - correction, each limited to -100..100 before voltage
compensation. Negative values follow the calculated P+D output, not a fixed
reverse power or a Kp=4 switch. For base 40/40, Kp=5, error=+10 and D=0, outputs
are 90/-10; error=-10 gives -10/90. Kp=20 at error=+10 reaches 100/-100.
The same bounds apply during numeric approach/crossing and lost-line recovery.
Recovery still generates its existing synthetic error; at Kp=0.85 and base40/40,
loss with a known positive direction yields about82/-2 after integer truncation.
Battery compensation preserves sign and clips to -100..100. Distance estimation
uses the signed mean wheel command, so equal opposing outputs add no distance.
B_Line's legacy relative wheel limits and explicit turn/brake behavior are unchanged.

Set the speed used after the requested distance, while approaching/crossing a line,
in setup after begin():

```cpp
robot.setFLineApproachSpeed(30, 30); // Left/right nominal percent, each 1..100.
// robot.setFLineApproachSpeed(30); // Set both wheels equally.
// robot.setFLineApproachSpeed(0);  // Restore F_Line's supplied speeds (default).
```

For `F_Line(40,40,0.3,20.0,fr,50,f5,40)`, the distance phase uses 40/40,
the subsequent approach and end-pair crossing use 30/30, and the turn uses 50.
This setting covers numeric CS/CP/FS/FP/CL/CR/FL/FR and legacy numeric CROSS.
All numeric approach/crossing phases use front-array PD steering around the new
nominal speeds, with the same Kp, Kd, tracker and lost-line recovery as the initial
distance phase. This includes CS/CP/FS/FP, CL/CR center approach, FL/FR approach
and end-pair crossing, and numeric CROSS. Turn commands still use turnSpeed.
Sensor-exit calls and B_Line are unaffected; NS/NL/NR have no approach phase.
CP/FP/CROSS with brake 0 retain the approach outputs when returning.
Invalid values return false and preserve the previous pair; only (0,0) disables
the override. Settings are captured once per call and battery compensation applies
normally. Sketches that omit the setter retain their existing speeds.

Numeric-distance calls also support these six-argument finish actions:

```cpp
robot.F_Line(40, 40, 0.3, 20.0, ns, 40);
robot.F_Line(40, 40, 0.3, 20.0, cs, 40);
robot.F_Line(40, 40, 0.3, 20.0, cp, 40);
robot.F_Line(40, 40, 0.3, 20.0, cp, 0);
robot.F_Line(40, 40, 0.3, 20.0, fs, 40);
robot.F_Line(40, 40, 0.3, 20.0, fp, 40);
robot.F_Line(40, 40, 0.3, 20.0, fp, 0);
```

After the distance-follow phase, NS brakes and stops immediately (like STOP).
CS follows the line until either CL or CR confirms black, then brakes and stops.
CP follows the line until both CL and CR have independently confirmed black,
then both are white together for debounceMs. They need not see black simultaneously.
FS follows the line until any of F0/F1/F14/F15 confirms black, then brakes and stops.
FP follows the line until any of those four confirms black, then all four are
white together for debounceMs. Black already present on entering this phase counts.
Each black confirmation uses setFLineSensorDebounceMs; white is strength <=400
and black is strength >=600, after calibration and polarity conversion.

NS/CS/FS return StopCompleted and always stop. CP/FP return CrossCompleted;
brake >0 brakes and stops, while brake 0 retains the forward motor outputs.
All watched sensors must be calibrated. These new actions are only accepted for
F_Line numeric-distance finish calls, not sensor-exit calls or B_Line.
The approach/crossing phase uses sensorMaximumRunMs (default 0 disables timeout),
while the initial distance phase still uses maximumRunMs. Stop requests and faults
stop the motors in every phase.

Numeric FL/FR now follow the line after the distance until any F0/F1/F14/F15
confirms black, then perform the existing F turn: brake, advance until the
direction-end pair has each seen black and is stably white, turn, and stop on the
selected turn sensor's new white-to-black edge. Sensor-exit FL/FR are unchanged.
NL/NR still spin immediately after distance; CL/CR still follow to their respective
center sensor before spinning. Their eight-argument forms remain unchanged:

```cpp
robot.F_Line(40, 40, 0.3, 20.0, nl, 50, f5, 40);
robot.F_Line(40, 40, 0.3, 20.0, nr, 50, f5, 40);
robot.F_Line(40, 40, 0.3, 20.0, cl, 50, f5, 40);
robot.F_Line(40, 40, 0.3, 20.0, cr, 50, f5, 40);
robot.F_Line(40, 40, 0.3, 20.0, fl, 50, f5, 40);
robot.F_Line(40, 40, 0.3, 20.0, fr, 50, f5, 40);
```

## F_Line phase two: recovery, turns, STOP and CROSS

The four-argument sensor/distance calls remain available and stop on return.
All forms now recover from temporary loss instead of immediately returning
LineLost. B_Line is not implemented by this change.

### Recovery while following (including center approach)

```cpp
robot.setFLineRecoveryKpThreshold(0.4);
// robot.getFLineRecoveryKpThreshold();
```

Default threshold is0.4. The setter accepts every finite signed float and returns false without
changing the previous value for NaN or infinity.
KP **greater than** the configured threshold enables snap; equality is mild.
For mild recovery, the last valid error remains clamped to[-50,50]. Snap uses the
last nonzero error direction and an extreme error that yields at least one base
speed of steering correction (signed wheel bounds-100..100 apply).
Before a direction is known, the fallback is straight; it never guesses left/right.
D is reset during loss and on reacquisition, then normal PD resumes. Visible
junction/branch continuity and independent exit-edge detection are retained.

Sensor mode has no default time limit, including recovery, approach, turns and
crossing. Start after release or stopRequested can stop every stage. Explicit
sensorMaximumRunMs is an optional bench limit per sensor stage; the default0
keeps it disabled. maximumRunMs bounds only the initial numeric-distance phase.

### Copyable calls

Choose one movement per desired step; these examples are alternatives, not a route.
Main MyMiniPro uses the first call, after calibration/Start, with no Serial output.

```cpp
robot.setFLineFRMotors(100, -30);
robot.setFLineFLMotors(-30, 100);
robot.F_Line(40, 40, 0.1, f5, fr, 90, f11, 40);
robot.F_Line(40, 40, 0.1, f15, fl, 50, f5, 40);
robot.F_Line(40, 40, 0.1, f0, fr, 50, b5, 40);
robot.F_Line(40, 40, 0.1, f15, cl, 50, f5, 40);
robot.F_Line(40, 40, 0.1, f15, cr, 50, b5, 40);
robot.F_Line(40, 40, 0.1, 20.0, nl, 50, f5, 40);
robot.F_Line(40, 40, 0.1, 20.0, nr, 50, b5, 40);
robot.F_Line(40, 40, 0.1, cl, cl, 50, f5, 40);
robot.F_Line(40, 40, 0.1, cr, cr, 50, b5, 40);
robot.F_Line(40, 40, 0.1, f15, stop, 40);
robot.F_Line(40, 40, 0.1, 20.0, stop, 0);
robot.F_Line(40, 40, 0.1, f15, cross, 0);
robot.F_Line(40, 40, 0.1, b15, cross, 40);
robot.F_Line(40, 40, 0.1, cl, cross, 40);
robot.F_Line(40, 40, 0.1, 20.0, cross, f5, 0);
robot.F_Line(40, 40, 0.1, 20.0, cross, b5, 40);
```

| Initial exit | Allowed actions | CROSS form |
| --- | --- | --- |
| Numeric centimeters | FL, FR, CL, CR, NL, NR, NS, CS, CP, FS, FP, STOP, CROSS |7 arguments for legacy CROSS, explicit F/B crossing sensor |
| F0..F15 | FL, FR, CL, CR, NL, NR, STOP, CROSS |6 arguments, same exit sensor |
| CL/CR | CL, CR, STOP, CROSS |6 arguments, same center sensor |
| B0..B15 | STOP, CROSS |6 arguments, same rear sensor |

Turn forms have8 arguments; turnStopSensor must be F0..F15 or B0..B15.
Numeric CROSS explicitly accepts F/B only, not CL/CR. The ambiguous six-argument
numeric CROSS returns InvalidCrossSensor and stops before movement.
CL/CR remain typed sensor constants; ActionChoice interprets them as actions only
in the action parameter. Scoped Sensor:: and MyMiniProForward::Action:: names also work.

### State transitions and return behavior

- FL/FR: brake on exit, advance straight at the original left/right speeds until
  both direction-end sensors have independently seen black and are stably white
  together (FL: F0/F1; FR: F14/F15), then turn to the selected stop sensor. See the motor setters and detailed sequence below.
- CL/CR: if the initial exit is not a center sensor, keep following at the same
  left/right speeds until the direction-corresponding center sensor sees black.
  It accepts that center line without requiring a second clear-arm. Either center
  exit already locates the center, so it skips approach and spins immediately.
- NL/NR: spin immediately at the exit. Left spin is(-speed,+speed), right is(+speed,-speed).
- CL/CR/NL/NR start a fresh clear-arm on the exact selected F/B stop sensor:
  calibrated strength<=400 for debounceMs(default20), then>=600 for the configured
  setFLineSensorDebounceMs(default0). Old black and other sensors cannot end a turn.
  Selected samples are fresh before EMA; polarity/calibration follows that array.
  FR/FL use the same fresh post-turn white-to-black stop gate, with no minimum turn time.
- STOP: brake then stop at the successful initial exit. STOP0 simply stops.
- Sensor CROSS: if the selected exit itself confirms the edge, continue until it
  stays white for debounceMs. If only its inward neighbor triggers, advance at
  the original speeds until the selected sensor confirms black, then stable white.
  Neighbor history never substitutes for selected black; missing selected black
  keeps CROSS running until stop/fault or an enabled sensor timeout.
- Numeric CROSS: after distance completion, drive forward until the explicitly
  selected sensor confirms black, then stable white. Black already present at the
  distance boundary is accepted; no unnecessary new black edge is required.
- CROSS with brake0 returns without a stop or output change at return. Outputs
  remain active until the caller changes them; follow with the next command or
  robot.stopMotors(). CP/FP with brake0 also retain outputs.
- All faults, invalid combinations, timeout and user stop paths stop the motors,
  including CROSS0. Calibration is checked before movement. Callback/hardware
  stop is checked between synchronous sensor operations and every brake tick.

### Brake mapping and results

turnSpeed and brake are integer percentages, clamped to0..100. For brake level b>0,
each wheel receives the opposite of its incoming signed output with magnitude
ceil(abs(output)*b/100), capped at100. Duration is5+ceil(25*b/100) milliseconds
(6..30ms). A stopped wheel gets0; both stopped wheels produce no pulse. Higher
levels never reduce counter impulse. The implementation polls stop/fault once per
1ms brake tick; it does not use one long blocking delay. Brake0 has no active pulse.
Turns and STOP always stop at completion; CROSS stops only with nonzero brake.

Success results: TurnCompleted, StopCompleted, CrossCompleted. Four-argument
success remains SensorDetected or DistanceReached. InvalidAction covers a wrong
exit/action pairing or argument form; InvalidTurnSensor and InvalidCrossSensor
identify wrong selected-sensor types. Existing calibration/read/stop/timeout errors
remain. Physical turn angle, braking travel and the single-run distance model
still need validation on this robot; native tests validate logic and motor commands.

### FR/FL wheel profiles and brake-clear-turn sequence (2026-09-16)

```cpp
robot.setFLineFRMotors(100, -30);
robot.setFLineFLMotors(-30, 100);
robot.F_Line(40, 40, 0.1, f5, fr, 90, f11, 40);
```

Each setter takes **signed left/right percentages of turnSpeed**, clamped to
-100..100. Effective wheel command is `turnSpeed * percentage / 100`, truncated
 toward zero, then the existing voltage compensation is applied. Thus FR(100,-30)
at90 commands(90,-27) before compensation; FL(-30,100) at90 commands(-27,90).
Opposing the inner wheel can tighten the turn. These are example tuning values,
not a measured optimal profile. The caller controls both wheel signs, so a profile
can also reverse the expected yaw if set incorrectly. FR and FL settings are
independent, retained in RAM, and snapshotted for each call. They affect no other
actions. Defaults remain FR(100,0), FL(0,100), preserving the former wheel powers.

For **F_Line only**, both FR and FL perform:

1. Follow until the initial exit succeeds (F5 in the example).
2. Apply the supplied brake level to incoming wheel outputs, then stop. Brake0
   has no counter pulse but still stops before advancing.
3. Advance at the original left/right speeds, with voltage compensation and no PD.
   FL watches physical channels **F0 and F1**; FR watches **F14 and F15**, independent
   of frontMap. Each channel must separately confirm black (strength>=600 for
   setFLineSensorDebounceMs, default0). Starting black counts after confirmation;
   starting white must first reach black. Neither the exit nor the other channel
   supplies its history. History begins after pre-braking.
4. After both have seen black, require **both currently white** (strength<=400)
   together for debounceMs(default20). A black re-touch or ambiguous reading on
   either channel restarts this shared white interval. Earlier white on one side
   cannot be combined with later white on the other while the first is black.
5. Start the configured FR/FL motor profile on the next loop, without extra
   clearance distance or minimum turn time. The exact selected TURN-STOP (F11 in
   the example) must newly confirm stable white after turn entry, then confirmed
   black. Old black at turn entry cannot immediately stop the turn.
6. Counter-brake the actual turn outputs, stop, and return TurnCompleted.

For a diagonal line, F14 may cross before F15. F15's initial white does not mean
it passed the line: the robot waits for F15's own black and then white. On a
perpendicular line both may already be black when advance begins.

The pair uses freshly captured raw MUX samples before EMA, each channel's saved
calibration and frontLineHigh polarity. Turn-stop F/B sensors use their own array
calibration/polarity. Scans are sequential, not physically simultaneous; the
shared white interval is evaluated on each pair of fresh samples. Stop callback,
Start button and calibration checks remain active in brake, advance and turn.
Optional sensorMaximumRunMs bounds the pair phase as a whole and the turn phase;
default0 has no timeout. Missing crossings keep advancing until stop or timeout.
There is no additional distance-model requirement for sensor-triggered turns.
Numeric exits still require the calibrated distance coefficient.

### Retired clearance/time compatibility API

`setFLineTurnClearanceCm(value)` and `setFLineMinTurnMs(value)` remain callable for
source compatibility, but only zero returns true. Nonzero, negative or nonfinite
clearance values return false; nonzero milliseconds return false. They store no
settings, have no motion effect, and both getters always return zero. Remove
legacy calls that requested1cm/80ms; the actual MyMiniPro example contains neither.
This pair-based sequence replaces those guards rather than silently retaining them.

Wheel-profile setters and CL/CR/NL/NR/STOP/CROSS are unchanged. B_Line remains
paused and incomplete. No physical trial or upload was performed; sensing alone
cannot prove which physical line produced a later white-to-black transition.
### F exit with inward neighbor (2026-09-16)

During the initial F_Line follow phase, an F sensor exit now accepts either the
selected physical channel or its immediate neighbor toward the middle:

| Selected F channel | Eligible neighbor |
| --- | --- |
| F0, F1, F2, F3, F4, F5, F6, F7 | F1, F2, F3, F4, F5, F6, F7, F8 respectively |
| F8, F9, F10, F11, F12, F13, F14, F15 | F7, F8, F9, F10, F11, F12, F13, F14 respectively |

For index i, neighbor is i+1 when i<8, otherwise i-1. Thus F4 means F4 OR F5,
and F11 means F11 OR F10; frontMap does not remap these physical channel IDs.
Each has its own clear-arming and black-confirmation history. A clear selected
channel cannot arm a neighbor that has stayed black from startup. Either valid
white-to-black edge exits follow. Both raw values are captured before EMA in
one existing 16-channel scan, using each calibration and frontLineHigh polarity.
Calibration failures stop motion. lastFLineSensorStrength still reports the
selected channel, so it can be white when the neighbor triggers.

The equivalent baseline uses `robot.F_Line(40,40, 0.1,f15,fr,90,f11,40)`: F15 OR F14 exits
follow; FR preparation still requires F14 AND F15 each to cross black and then
both stay white; only F11 ends the turn after its fresh white-to-black edge.
No extra distance or minimum turn time is added. B and center exits, numeric
exits and center-approach gates retain their existing single-sensor behavior.
CROSS still crosses the selected sensor's actual line as described above; a
neighbor exit alone does not count as selected black. No robot upload performed.
### Scale-50 control boundary and equivalence

The tracker still uses normalized geometry and existing branch/hysteresis rules.
F_Line converts its measured error by50 immediately before recovery and PD.
Their stored error/derivative history uses the new units; this is not a display-only
change. Under identity frontMap, Fi has position `50*(2*i/15-1)`: F0=-50,
F7=-3.33333, F8=+3.33333, F15=+50; one channel pitch is6.66667. Weighted centroids
interpolate between positions. Sensor strength thresholds0..1000 are unchanged.

For new KP0.1, an error of6.66667 produces P=0.666667 percentage points, equal to
old KP5 on normalized error0.133333. KD0.001 on the scaled derivative likewise
matches old KD0.05. The20ms derivative filter and timestamp/reset rules are unchanged.
No integral term is added. Mild recovery clamps measured history to[-50,+50].
Aggressive loss recovery uses sign*max(50,max(baseL,baseR)/max(abs(KP),0.000002));
direction deadband is0.005. Main KP0.1/base40 yields synthetic error400, hence
P40, equivalent to old synthetic8 times KP5. Equality with threshold stays mild.
The paused B_Line draft retains scale1, its old gain limits/defaults and recovery;
the separate PID tuner is unchanged.

Independent frozen scale1 reference tests compare384030 visible/lost/reacquired
samples, derivative timings/wrap, gains, threshold boundaries, clamping and motor
quantization. Floating-point rescaling is algebraically equivalent but not bitwise:
observed correction difference was at most0.000023 percentage points. Values
straddling an integer truncation boundary can differ by1 motor-command point before
voltage gain; compensation can amplify that quantization difference. This is not
an intentional increase in gain. Hardware behavior has not been retested here.
### Unrestricted finite gains and current upload settings

KP, KD and recovery threshold have no tuning ceiling or nonnegative restriction.
Negative KP reverses the proportional correction; negative KD reverses damping.
The signed comparison KP>threshold selects aggressive recovery; equality is mild.
Recovery uses abs(KP) only when calculating synthetic magnitude. Zero/tiny gains
use the numerical denominator floor stated above; this does not clamp stored gains.
NaN/Inf are rejected. P/D products and their sum use double precision for F_Line,
then bound the result to finite float range before the signed-100..100 wheel clamps.
Sensor checks, calibration, stop handling and output motor limits remain active.

The following tuning example uses KP0.45, KD0.021 and recovery threshold0.4:

```cpp
robot.setFLineKd(0.021);
robot.setFLineRecoveryKpThreshold(0.4);
robot.F_Line(40, 40, 0.45, f15, fr, 90, f11, 40);
```

KD0.021 is accepted and used; it does not fall back to the default. These user-tuned
gains are different from the equivalent baseline0.1/0.001/0.08. Update the entire
library and sketch together; external scale1 gains must be divided by50 manually.

## F_Line startup and distance speed profiles

ForwardSettings now defaults to a300ms startup ramp and a5cm distance slowdown
window with an end-speed ratio of20% of the requested speed:

```cpp
MyMiniPro::ForwardSettings settings;
settings.centimetersPerSecondPerPercent = 1.6875; // retain your measured value
settings.maximumRunMs = 3000;
settings.startupRampMs = 300;          // ms;0 disables startup ramp
settings.distanceDecelerationCm = 5;  // cm;0 disables distance slowdown
settings.distanceEndSpeedPercent = 20; // percent OF target,1..100
robot.setForwardSettings(settings);
```

For each wheel whose requested translation speed is at least25, the first command
is zero. Its envelope rises linearly to1
in startupRampMs, timed from that first command. Each F_Line call has fresh local
state, including after stop/fault/completion. Settings snapshot on entry. The
factor scales the complete bounded motor command AFTER P+D/recovery and voltage
compensation, so a large steering correction cannot bypass the startup envelope.
Signs are preserved. Relative wheel commands are preserved when both wheels
receive the same factor, apart from integer quantization;
PD gains, tracking and recovery rules do not change. Ramping only the base speed
would still allow abrupt D/recovery output, so both complete wheel outputs ramp.

Only the numeric distance Follow phase additionally uses remaining modeled cm.
For the default5cm window, its factor decreases from1 at5cm remaining to0.2 at1cm,
then holds0.2 through the final1cm. On a centered line at target40/40 and gain1,
that means approximately8/8 until the distance completes. Changing
 distanceEndSpeedPercent changes the low-speed ratio; the hold region is always
 the final fifth of the configured window. Targets too small to represent the
ratio in integer percent commands use a per-wheel minimum factor of1/target.
Startup and slowdown combine using the smaller factor. Short moves can therefore
finish without reaching full target speed. The end factor stays positive rather
than asymptotically approaching zero.

Distance integration uses the commands actually sent during the profile, divided
by compensation gain as before. This remains an encoderless estimate; a wheel
may not move at low PWM. Tune ramp duration, slowdown distance and end ratio on
the real vehicle, and calibrate the distance coefficient for its usable speeds.
The20% default is a ratio, not an absolute20-percent motor command. PD and voltage
compensation can make actual wheel commands differ from the centered-line example.

Distance slowdown does not apply to sensor-exit commands or post-distance
approach/crossing phases. Those phases keep their existing configured approach
speed and sensor conditions. Startup is continuous across translation phases and
is not restarted at phase changes. Explicit turn profiles and counter-braking
bypass the envelope; braking uses the actual incoming (possibly slowed) output.
Stop/Start/calibration checks remain polled with no ramp delay loop. B_Line ignores
these added settings. Invalid numeric slowdown distance (negative/NaN/Inf), or
end ratio outside1..100 when slowdown is enabled, stops with InvalidArgument.

To reproduce immediate-speed behavior set startupRampMs=0 and
 distanceDecelerationCm=0. Constant-speed distance calibration examples explicitly
 disable both profiles so their measured-cm/time/nominal-command calculation stays
 valid. Other examples inherit the defaults unless configured otherwise. Longer
 travel time may require adjusting the existing numeric maximumRunMs timeout.
No hardware upload or robot motion was performed while implementing this feature.
### Low-speed wheel bypass (24/25 boundary)

Each wheel with requested translation target below25 skips BOTH startup and
distance slowdown;25 is included in the profiles. For example24/40 starts with
left24 immediately while right ramps from0 toward40; near the distance endpoint,
left stays24 and right reaches about8. Both24/24 respond immediately throughout;
both25/25 ramp and end around5/5. This intentionally changes the wheel ratio during
the profile when only one wheel qualifies. Eligibility uses requested target speed,
not the magnitude/sign of the PD-corrected output. A negative correction output
keeps its sign; a low-target wheel still bypasses even if PD reverses that wheel.
F_Line SPL/SPR arguments themselves still accept only1..100, so negative arguments
return InvalidArgument before movement. The current post-distance approach target
also determines bypass eligibility; configured15/15 therefore bypasses the profiles.
## Lowercase F_Line arguments

Use lowercase values in new sketches; the method name remains `F_Line`:

```cpp
robot.F_Line(60, 60, 0.85, 40, ns, 40);
robot.F_Line(40, 40, 0.1, 20.0, fs, 40);
robot.F_Line(40, 40, 0.1, f15, fr, 90, f11, 40);
robot.F_Line(40, 40, 0.1, cl, cr, 50, b5, 40);
robot.F_Line(40, 40, 0.1, 20.0, cross, f5, 0);
```

- Actions: `fl`, `fr`, `cl`, `cr`, `nl`, `nr`, `stop`, `cross`, `ns`, `cs`, `cp`, `fs`, `fp`.
- Sensors: `f0` through `f15`, `b0` through `b15`, plus `cl` and `cr`.
- Scoped forms also work: `MyMiniProForward::Sensor::f15` and
  `MyMiniProForward::Action::fr` (or `Sensor::f15` with the existing type alias).

These are typed aliases for the original enum values. Uppercase constants and
scoped names remain supported with the same IDs, overloads and behavior; no old
names were removed. Bare cl/cr remain sensor-typed and ActionChoice interprets
these two as turn actions only in the action argument, just like CL/CR. Lowercase
rear aliases use no binary-literal macros. Function/setter names and gain units
are unchanged. Existing argument validity and safety rules still apply.

This naming-only update was inspected at source level; compile/upload were not
run, as requested. Recompile your sketch with the updated library yourself.