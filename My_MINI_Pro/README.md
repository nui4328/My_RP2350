# Pico2 My Mini Pro

Arduino library for a robot using two 74HC4067 16-channel analog multiplexers:

- a 16-channel front line-sensor array;
- a 16-channel rear line-sensor array;
- CAT24C128 external I2C EEPROM for calibration values;
- one direct, active-low front calibration button; and
- one optional rear calibration button read through an MCP3421.

## Install

Place this complete folder in the Arduino `libraries` directory, restart Arduino
IDE, then open **File > Examples > Pico2 My Mini Pro >
Pico2_MyMiniPro**.

For the supplied Pico 2 wiring, the example only needs:

```cpp
#include <Pico2_MyMiniPro.h>

Pico2MyMiniPro robot;
```

`robot.begin()` configures the two MUXes, I2C on GP4/GP5, EEPROM, buzzer on
GP9, Start button on GP2, front calibration button on GP3, and the rear MCP3421
calibration button.

## Motors

`robot.begin()` configures the VNH7070ASTR motor-driver control pins and 20 kHz
PWM:

```cpp
robot.Motor(60, 60);    // both motors forward at 60%
robot.Motor(-50, -50);  // both motors reverse at 50%
robot.Motor(0, 0);      // stop both motors
```

The command is `Motor(left, right)` and each value is limited to `-100…100`.
For the supplied Pico 2 map, left uses GP19/GP20/GP21 and right uses
GP6/GP7/GP8 as `PWM`/`INA`/`INB`; confirm those connections against the actual
VNH7070ASTR boards before running motors.

## Servos

`Pico2MyMiniPro` provides five fixed Servo GPIOs. They use the Arduino-Pico
core's PIO-backed `Servo` support, so their control pulses are separate from
the motor driver's 20 kHz PWM.

For the clean in-library starting point, open **Pico2_ServoRobotExample**. It
uses `Pico2MyMiniPro`, suppresses the short startup melody after the reset beep
with `robot.begin(false)`, and
commands `robot.servo(18, 90)` with no Serial, test flags, sensor, motor, or
interactive logic in the sketch.

For a two-tab learning/reference sketch, open **Pico2_LeadMe**. Its first tab
initializes the robot and waits for Start; its **Functions_List_Me** tab lists
copy-ready `robot.*` commands for Front/Rear line sensors and their
current/min/max values, ADC L/R values and extrema, Start-button waiting,
buzzer control, Servo control, and motor commands.

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

### Test each servo safely

Use the separate **Pico2_ServoGpioTest** example to verify wiring without
moving every servo together. It starts with all Servo outputs detached and
suppresses the short startup melody; the standard single reset acknowledgement
still sounds at boot. First
check that the mechanism can move freely, connect an external servo supply and
common ground, then set `kEnableServoMotionTest` to `true` in that sketch and
upload it. Open Serial Monitor at 115200 baud and send `1`, `2`, `3`, `4`, or
`5`: it tests exactly one GPIO (GP18, GP22, GP28, GP0, or GP1 respectively),
using the small sequence 90° → 80° → 100° → 90°, then detaches it. Set the
flag back to `false` after wiring verification. The test sketch does not run a
motion sequence unless both the flag is enabled and one of those Serial
commands is sent.

### Diagnose the GP18 signal path

For an advanced direct-Servo comparison outside this library, open
**Pico2_DirectServoGpio18Minimal**. It uses only `Servo.h`, attaches GP18 with
1000–2000 microsecond bounds, and continuously commands 90°; it has no robot
library, Serial, flags, motors, sensors, buzzer, or interactive logic.

First use **Pico2_DirectServoGpio18Test** when checking whether the
Arduino-Pico `Servo.h` backend itself produces a signal. This standalone sketch
does not include or instantiate `Pico2MyMiniPro`, so it excludes the robot's
buzzer, motors, MUXes, I2C, and Servo wrapper. Set its
`kEnableServoPulse = true`, upload it, then open Serial Monitor at 115200 baud.
It directly executes:

```cpp
servoOnGp18.attach(18, 1000, 2000);
servoOnGp18.write(90);
```

On `ATTACH OK`, probe GP18 relative to Pico GND: the output should continue at
3.3 V, 50 Hz, with an approximately 1.5 ms HIGH pulse. `ATTACH FAILED` means
the Arduino-Pico Servo backend could not allocate a PIO state machine. Its
default flag is false, so it emits no Servo pulse until deliberately enabled.

If a Servo on GP18 still has no observed signal, use the separate
**Pico2_Gpio18HardwareDiagnostic** example. It needs no Serial command. With
its default `kEnableServoPulsePhase = false`, it does not attach a Servo: after
boot it produces three directly driven GP18 cycles of 500 ms HIGH / 500 ms LOW,
then makes GP18 an input. Probe GP18 relative to Pico GND (disconnect the
servo signal/power during this first phase). Seeing that 3.3 V digital pattern
proves the Pico pin, probe point, and carrier trace; not seeing it points to a
physical net, connector, or measurement problem.

Only after that phase is confirmed, set `kEnableServoPulsePhase = true` and
upload again. The sketch repeats the digital probe, then attaches GP18 at 90°
for ten seconds. The expected Servo signal is 3.3 V at 50 Hz with an
approximately 1.5 ms HIGH pulse. Its Serial output explicitly distinguishes a
PIO attach failure from an active pulse phase. The diagnostic uses
`begin(false)`, stops motors, and suppresses the short startup melody.

### Motor voltage compensation

Pico 2 enables compensation with a 12.4 V reference. Call this once per main
loop, outside the time-critical line-sensor/PID section:

```cpp
robot.serviceMotorVoltageCompensation();
```

It starts an ADS1115 conversion at most every 500 ms, then polls for completion
on a later pass. `Motor()` neither reads battery voltage nor scales its input.
The cached gain is `12.4/Vfiltered`, never less than 1.0; invalid readings and
USB readings below 6.5 V use a gain of 1.0. Apply that gain to only the base
speed before mixing in the unscaled PID steering correction:

```cpp
const int compensatedBase = robot.compensateBaseMotorSpeed(baseSpeed);
robot.Motor(compensatedBase + correction, compensatedBase - correction);
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
so call the service after the sensor/PID work rather than inside that hot path.

### Forward line follower

The example enables a forward-only controller after the Start button. It scans
only the Front MUX signal on GP27, treats channel 0 as left and channel 15 as
right, and converts calibrated readings to blackness with `1000 - normalized`.
It then uses a darkness-weighted centroid around centre position 7.5 for the
line error.

The defaults are deliberately conservative and must be tuned on the real robot:
base speed 35, `kp=9.0`, `ki=0`, `kd=0.08`, a 5 ms control interval, a line
detection sum of 1200, and steering limited to 35. A missing Front calibration
or a missing line stops both motors rather than searching blindly. If a positive
correction turns away from the line, set `steeringSign` to `-1`.

Configure this feature with `ForwardLineFollowerSettings`, then call
`serviceForwardLineFollower()` once per control loop. The service applies
voltage compensation only to the base speed; it leaves the PID steering
correction unscaled before sending the final left/right motor commands.

The two-tab **Pico2_LeadMe** example is a ready-to-run Front-only PID
starting point. It waits for GP2 Start, then scans only the Front 16-channel
array once every control pass; its second tab lists the line-follower methods.
It deliberately does not use the Rear array, turning logic, or a line-search
routine yet.

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
`beginBatteryLevelLeds()` displays the first reading immediately. While the
example is in `waitButton()` waiting for Start, it refreshes the bar every
500 ms; after Start, the example refreshes it with its existing 60 ms battery
sample. The supplied setup uses address `0x20` and active-high LEDs. From 10.5
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

The supplied Pico 2 example uses a 5-second calibration: short periodic beeps
indicate calibration is still running, then two longer beeps confirm that the
values have been saved. A Pico GPIO produces a fixed 3.3V signal, so the
periodic beeps are perceived as gentler because they are very short; actual
volume needs a transistor driver.

## Start button

For a button connected between Pico 2 GP2 and GND, add this in `setup()`:

```cpp
lineSensors.beginStartButton(2);
```

Then call `serviceStartButton()` in every `loop()`. `robotStarted()` is false
after boot and becomes true after one press. The supplied example keeps the
robot inactive until that happens and plays one 300 ms confirmation beep.

For the complete wait mode, use this one line at the start of `loop()`:

```cpp
if (!lineSensors.waitButton(Serial, 100)) return;
```

It prints both arrays while waiting, accepts either calibration button, and
continues to the robot-control code after the GP2 Start button is pressed.

## Serial Monitor waiting mode

Before the Start button is pressed, the Pico 2 example prints one row every
100 ms in this form:

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
EEPROM, then the live display resumes. After GP2 Start is pressed, the example
stops this serial output and scans only the array needed for the driving
direction.

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
