# MyMINI_Pro Arduino Library

Arduino library for the MyMINI Pro competition line-following robot. It targets
the Raspberry Pi Pico 2 / RP2350 using the Arduino-Pico core.

## Install

1. Install the external libraries required by the firmware: the BNO055 library
   used by this robot and the Arduino Servo library compatible with Arduino-Pico.
2. In Arduino IDE, choose **Sketch > Include Library > Add .ZIP Library...**.
3. Select `MyMINI_Pro.zip`.
4. Select **Raspberry Pi Pico 2** and open an example from **File > Examples >
   MyMINI_Pro**.

The library includes no guessed `depends=` entry because the installed BNO055
library name is environment-specific.

## Quick start

```cpp
#include <MyMINI_Pro.h>

void setup() {
  robot_begin();  // Initialize motors, sensors, calibration, and gyro.
  wait_button();  // Wait for START and set the gyro's zero heading.
}

void loop() {
  robot_update(); // Keep background robot services running.
}
```

`robot_begin()` initializes motors, I2C, sensors, calibration, diagnostics,
buzzer, and BNO055. `wait_button()` runs setup/calibration mode until a short
press on GP2. Call `robot_update()` continuously from `loop()`.

## Example `robot_setup()` settings

The complete setup and a short explanation of **every value** are in
[`examples/MyMINI_Pro/set_robo.ino`](examples/MyMINI_Pro/set_robo.ino) and
[its README](examples/MyMINI_Pro/README.md). Key units and argument order:

| Example call | Meaning |
| --- | --- |
| `configure_sensor_reading(40, 1, 2, 4)` | MUX wait in µs, discarded ADC reads, averaged ADC reads, frame smoothing divisor |
| `set_line_distance_scale(1.75f)` | Distance correction: measured 35 cm for a 20 cm command gives 35/20 |
| `set_turn_overshoot(20)` | `fl/fr` travel 20 ms beyond the crossing before braking |
| `set_turn_line_search(60, 100)` | Search-speed cap 60 after 100 ms at the requested turn speed |
| `set_line_ramp_start_speed(5)`, `set_line_ramp_ms(200)` | Start line motion at speed 5 and ramp for 200 ms when ramping applies |
| `set_line_decel_ramp_cm(100)` | Slow over the last **1.00 cm** of distance-ended line motion; 0 disables it |
| `set_line_pid_tuning(0.0125f)`, `set_gyro_kd(0.00850f, 0.0125f)` | Line Kd; forward/backward gyro-straight Kd |
| `set_rotate_fallback(300, 550, 50)` | Timed 90° spin/pivot: 300/550 ms at motor speed 50; also used by `turn_gyro` |
| `set_turn_approach(25, 25, 15)` | Approach speeds after sensor/distance exit, then reverse-brake power |
| `set_turn_center_brake(25, 10, 20, 60)` | `tcl/tcr` reverse-brake power/time after forward, then backward approach |
| `set_turn_motor(mode, left, right)` | Signed left/right wheel ratios, as percentages of requested turn speed |

## Hardware

- Board: Raspberry Pi Pico 2 / RP2350
- Motor driver: DRV8874 by default (TB6612FNG can be selected before `robot_begin()`); `motor(left, right)` range `-100..100`
- I2C: BNO055 `0x29`, ADS1115 `0x48`, PCF8574 `0x20`, CAT24C128 `0x50`,
  MCP3421 `0x68`
- Calibration and pin assignments are preserved in
  [`extras/HARDWARE_CONTRACT.md`](extras/HARDWARE_CONTRACT.md).

### Servo pins

| GPIO | Use |
| --- | --- |
| GP18 | Servo channel 1 |
| GP22 | Servo channel 2 |
| GP28 | Servo channel 3 |
| GP0 | Spare servo or UART0 TX |
| GP1 | Spare servo or UART0 RX |

`servo(pin, angle)` clamps angles to `0..180` and attaches each allowed pin
only on its first use. Arduino-Pico defaults to 1000..2000 us for 0..180°.
Call `set_servo_pulse_range(pin, minUs, maxUs)` before the first `servo()`
for that pin to widen the pulse range; the example sets all five supported
pins (GP18, GP22, GP28, GP0, GP1) to 500..2500 us.
The setting accepts 500..2500 us and returns false after attachment.
Check the physical servo and mechanism's safe endpoints before use.
GP0 and GP1 must not be used for servo while UART0 is in
use.

## Public API

| Group | API |
| --- | --- |
| Robot | `robot_begin`, `wait_button`, `robot_update`, `gyro_ready` |
| Sensors | `read_line_sensor`, `print_line_sensor_table`, `readADC_*`, `minADC_*`, `maxADC_*` |
| Direct control | `motor`, `servo`, `set_servo_pulse_range` |
| Line | `f_line`, `b_line` |
| Gyro straight | `fw_gyro`, `bw_gyro` |
| Turns | `turn`, `turn_gyro` |
| Rotation | `rotate_spin`, `rotateFW_pivot`, `rotateBW_pivot` |
| Settings | `configure_sensor_reading`, `set_line_ramp_start_speed`, `set_line_ramp_ms`, `set_line_distance_scale`, `set_line_decel_ramp_cm`, `set_line_pid_tuning`, `set_gyro_kd`, `set_turn_motor`, `set_turn_overshoot`, `set_turn_touch_brake_ms`, `set_turn_timeout`, `set_turn_approach_kp`, `set_turn_approach`, `set_turn_center_brake`, `set_turn_line_search`, `set_rotate_fallback` |

`stopPull` is the optional reverse-brake duration in milliseconds for the
rotation commands (`0..100`); their reverse pulse uses the opposite of the
slowed motor command. Straight distance commands estimate
travel from motor command and time because this robot has no encoder.

### Sensor values

```cpp
LineSensorReading value;
if (read_line_sensor(f0, value)) { // Read F0; enter only when a fresh reading succeeds.
  Serial.print(value.raw);
  if (value.normalizedValid) Serial.println(value.normalized);
}
read_line_sensor(b0, value); // Read rear sensor B0 into value.
read_line_sensor(cl, value); // Read center-left sensor CL.
read_line_sensor(cr, value); // Read center-right sensor CR.
print_line_sensor_table();   // Print all sensor readings to Serial.
```

`read_line_sensor(LineExitSensor sensor, LineSensorReading& reading)` reads one
fresh F/B MUX frame or one CL/CR ADS1115 conversion. It returns `false` when
the raw read fails or the selector is invalid. `reading.rawValid` reports the
raw read; `reading.calibrationValid` reports the channel calibration;
`reading.normalizedValid` is true only when both are usable. The table prints
all 34 channels with `N/A` for uncalibrated min/max/normalized values and
`ERR` when a raw read fails. See `examples/08_SensorValues` for a complete
sketch that prints the table every 200 ms and reads F0, B0, CL and CR singly.

Calibration min/max are **raw ADC values saved in EEPROM** and loaded by
`robot_begin()`. The normalized value maps the current raw reading into
`0..1000`; these two scales must not be compared directly. F/B line-following
uses its existing filtered readings, while this diagnostic API normalizes the
raw frame so `raw` and `normalized` describe the same sample.

For direct `0..1000` values matching `wait_button()`, use:

```cpp
int32_t f = readADC_F(0);   // F0; channels 0..15.
int32_t b = readADC_B(0);   // Read normalized B0; channels 0..15.
int32_t l = readADC_CL();   // Read normalized center-left sensor.
int32_t r = readADC_CR();   // Read normalized center-right sensor.
int32_t low = minADC_F(0); // 0 when F0 calibration is valid.
int32_t high = maxADC_F(0); // 1000 when F0 calibration is valid.
if (f != ADC_VALUE_INVALID) Serial.println(f);
```

Each `readADC_*` value is normalized `0..1000`, with F/B using the filtered
MUX sample just like `wait_button()`. Each `minADC_*` returns `0` and each
`maxADC_*` returns `1000` only when that channel has valid calibration; these
are normalized scale limits, **not the raw EEPROM calibration bounds**. All
12 functions return `ADC_VALUE_INVALID` if their value is unavailable. Use
`LineSensorReading::minRaw` and `maxRaw` or the table for raw EEPROM bounds.

`f_line()` and `b_line()` use the tracked sensor group and PID through
curves and junctions. After two narrow frames establish the main line, a new
side wing covering at least two sensors is excluded from steering and from
the remembered edge side. Steering is limited while that wing is visible;
the original line must return as a narrow group for two frames before normal
PID resumes. The side wing does not end a distance command or trigger a new
error stop. Distance commands end at their estimated target; sensor commands
keep reading their selected exit sensors without this filter. A persistent
black group outside the tracking window can otherwise be reacquired. On an
all-white row, the robot searches toward its last detected line side (or
away from the rejected side branch when no edge side was recorded), then
stops with `LINE_LOST_TIMEOUT` if no black returns within three seconds.

`gyro_angle(degrees)` selects an absolute heading (modulo 360 degrees)
relative to the mission gyro zero. Positive yaw turns right; negative yaw
turns left. For example, `gyro_angle(180.0f)` and
`gyro_angle(-180.0f)` select the same heading on every lap. The
line command accepts a crossing from either direction. If it starts within
5 degrees of that heading, it waits to leave that band before accepting
another crossing. Use `gyro_angle_raw(degrees)` for an exact
cumulative yaw target relative to the last gyro reset.
A plain float in the fourth `f_line()` argument still means distance in cm.
The robot follows the line with PID; the gyro only decides when to exit.
The target must be reached along the line's turn direction. An invalid gyro
reading brakes the motors and ends the command.

## Motion examples

```cpp
motor(40, 40);                  // Drive both wheels forward at speed 40.
servo(18, 90);                  // Move the servo on GP18 to 90 degrees.
f_line(40, 40, 0.85f, f0, 0);  // Follow the front line until the F0/F1 edge pair sees black.
f_line(50, 50, 1.5f, gyro_angle(135.0f), 1); // Follow the line until gyro yaw reaches +135°.
b_line(40, 40, 0.85f, b0, 0);  // Follow the rear line until the B0 exit is detected.
fw_gyro(0.0f, 70, 0.80f, 40.0f, 10); // Drive forward 40 cm at heading 0; reverse-brake 10 ms.
bw_gyro(0.0f, 70, 0.80f, 30.0f, 10); // Drive backward 30 cm at heading 0; reverse-brake 10 ms.
turn(tcr, 60, f11, 5);         // Approach CR, turn right to F11, then reverse-brake 5 ms.
turn_gyro(tfr, 60, 90.0f, 5); // Approach the turn and reach absolute +90; 5 deg caps brake lead.
turn_gyro(tfl, 60, 0.0f, 5);  // Approach the turn and return to heading 0.
rotate_spin(60, 90.0f, 10);    // Spin to heading +90; reverse-brake 10 ms.
rotate_spin(60, 0.0f, 10);     // Spin back to heading 0.
rotateFW_pivot(60, 60.0f, 10); // Forward-wheel pivot to +60; timed relative turn if gyro fails.
rotateBW_pivot(60, 0.0f, 10);  // Backward-wheel pivot to heading 0.
```

`positoin_error` (spelled this way in the API) selects the line position for
the next `f_line()` or `b_line()` call. It defaults to 50 and returns to 50
when that call finishes, including an error exit. Values 0..100 map across
the sensor row: 10 is near the left sensors, 50 is the center, and 90 is near
the right sensors. Invalid values use the center. This changes the PID target
only; line selection and lost-line recovery still use actual sensor positions.

```cpp
positoin_error = 10;             // Put the line near the left sensors for the next command.
f_line(30, 30, 1.5f, 20.0f, 1); // Drive forward 20 cm; the target resets to center on return.
positoin_error = 90;             // Put the line near the right sensors for the next command.
b_line(30, 30, 1.5f, 20.0f, 1); // Drive backward 20 cm; the target resets to center on return.
// The next f_line()/b_line() uses 50 unless assigned again.
```

For sensor-ended motion, choose an exit sensor that does not stay on the
tracked line. For example, a left target near f1 may overlap the f0/f1 exit
pair and prevent arming or end a command early.

For `f_line(..., f0, ...)` and `f_line(..., f15, ...)`, both sensors in the
respective edge pair (f0/f1 or f15/f14) must see black together in two of
three frames before the command detects its exit line.

When `turn(tfl/tfr/tcl/tcr, ...)` immediately follows a distance-ended
`f_line()`, its line approach uses the effective Kp, Ki, and Kd from the last
driven `f_line()` frame, after the distance slowdown factor. The first fresh
line frame resets the derivative baseline. Approach speed still comes from
`set_turn_approach()`; turns without this handoff use `set_turn_approach_kp()`
and the default turn Ki/Kd. The rotation and stop-sensor phases are unchanged.

For `turn(tcl/tcr, ...)`, `set_turn_center_brake(forwardPower, forwardMs,
backwardPower, backwardMs)` sets the reverse brake after `cl/cr` detects black.
The selected center sensor must report black in two of three sensor frames
before this brake and the rotation begin.
Forward and backward refer to the line-following approach before rotation.
For example, `set_turn_center_brake(30, 10, 20, 15)` brakes a forward approach
with power 30 for 10 ms and a backward approach with power 20 for 15 ms.
Without this call, both directions use the brake power from
`set_turn_approach()` for a fixed 10 ms.

`rotate_spin()`, `rotateFW_pivot()`, and `rotateBW_pivot()` take a
target heading relative to the latest successful `wait_button()` or
`gyro_reset()` zero. Their valid range is -360..360 degrees; equivalent
headings take the shortest route. A target of 0 returns to the zero heading.
If the gyro or heading reference is unavailable before the turn, the same
argument becomes a **relative, timed wheel turn from the current direction**.
Without a gyro, an absolute heading cannot be verified.

These three commands use the requested motor speed for the first 70% of the
turn, then 40% for the last 30% (minimum drive command 2). The gyro decides
when to slow and stop; it does not adjust the speed within either phase.
The final argument, such as `10`, drives the wheels in the opposite direction
for 10 ms at the slow speed, then brakes. During gyro operation, measured yaw
rate can start braking early to limit overrun. A gyro read failure, implausible
jump, reverse heading, or missing progress switches to wheel timing for the
estimated **remaining** angle. `set_rotate_fallback(spin90Ms, pivot90Ms,
calibrationSpeed)` sets the 90-degree timing used by spin and pivot; measure
these values on the actual robot. A `true` return after timed fallback means
the timed command completed, not that the final angle was measured.

The legacy `set_rotate_pid(kp, kd)` and
`set_rotate_spin_slowdown(ms, speed)` settings remain callable for source
compatibility but no longer change the three rotate commands.

`turn_gyro(mode, speed, targetDeg, brakeLeadDeg)` accepts an absolute heading
from the latest START press in -180..180 degrees; right is positive and left
negative. Its `fl/fr/tcl/tcr` modes keep their line-approach phases and use
the wheel ratios from `set_turn_motor()`. The gyro rotation uses P control,
slows near the target, checks the settled angle, and can make bounded small
corrections. The last argument caps the **predicted** brake lead in degrees;
0 uses the default cap of 20 degrees. It is not a fixed early-stop angle or
a delay. A gyro failure uses timed wheel rotation with the configured ratios;
when a valid yaw was read, only the estimated remaining angle is commanded.
If no valid yaw was ever read, `targetDeg` is relative to the current direction.
Other failures, such as invalid mode, missing approach calibration, or timeout,
brake and report `turn_gyro failure:` on Serial. Sensor-based `turn()` uses
its final argument as a reverse-brake duration in milliseconds.

Before running a mission, complete sensor calibration in `wait_button()` and
test motor direction with the diagnostics console. The console is available at
115200 baud and supports `help`, `status`, `i2c`, `mux`, `ads`, `adccal`,
`frontcal`, `leds`, `motor`, `stop`, and `eeprom_test YES`.

## Motor driver selection

DRV8874 is the default. The example calls `select_motor_driver()` explicitly
to show the active driver. To select DRV8874 explicitly, call it **before** `robot_begin()`:

```cpp
void setup() {
  select_motor_driver(MotorDriverType::DRV8874); // Select the PH/EN driver.
  robot_begin();                                 // Initialize the selected driver and sensors.
}
```

The library owns the confirmed motor mapping and polarity: left EN/PH/nSLEEP
is `GP19/GP20/GP21`; right EN/PH/nSLEEP is `GP6/GP7/GP8`. `nSLEEP` is separate
Pico-controlled GPIO for each module, not hard-wired high. Selecting DRV8874
initializes only those six signals and never falls back to TB6612FNG.

For DRV8874, PMODE must be physically tied to GND. `motor(0, 0)` coasts both
wheels (EN low, nSLEEP low); `motor(1, 1)` or `motor(-1, -1)` brakes both
(EN low, nSLEEP high). The driver waits 1 ms after each sleep/wake transition.
TB6612FNG coasts with IN1/IN2 low and PWM high, and brakes with IN1/IN2 high
and PWM high; its STBY is hard-wired high. Drive commands begin at magnitude 2.
In a mixed command, +/-1 becomes +/-2 and a zero wheel brakes to keep pivots.
The old `set_motor_brake_at_zero()` API is retained as a no-op for source
compatibility; `get_motor_brake_at_zero()` always returns false. IMODE, VREF,
and IPROPI remain hardware-configured.
