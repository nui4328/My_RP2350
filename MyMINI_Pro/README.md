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
  robot_begin();
  wait_button();
}

void loop() {
  robot_update();
}
```

`robot_begin()` initializes motors, I2C, sensors, calibration, diagnostics,
buzzer, and BNO055. `wait_button()` runs setup/calibration mode until a short
press on GP2. Call `robot_update()` continuously from `loop()`.

## Hardware

- Board: Raspberry Pi Pico 2 / RP2350
- Motor driver: TB6612FNG by default, `motor(left, right)` range `-100..100`
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
only on its first use. GP0 and GP1 must not be used for servo while UART0 is in
use.

## Public API

| Group | API |
| --- | --- |
| Robot | `robot_begin`, `wait_button`, `robot_update`, `gyro_ready` |
| Sensors | `read_line_sensor`, `print_line_sensor_table`, `readADC_*`, `minADC_*`, `maxADC_*` |
| Direct control | `motor`, `servo` |
| Line | `f_line`, `b_line` |
| Gyro straight | `fw_gyro`, `bw_gyro` |
| Turns | `turn` |
| Rotation | `rotate_spin`, `rotateFW_pivot`, `rotateBW_pivot` |
| Settings | `set_motor_brake_at_zero`, `get_motor_brake_at_zero`, `set_turn_motor`, `set_turn_overshoot`, `set_turn_timeout`, `set_turn_approach_kp`, `set_turn_approach`, `set_turn_line_search`, `set_rotate_fallback` |

`stopPull` is the optional reverse-direction braking strength (`0..100`) used
when a supported motion command finishes. Straight distance commands estimate
travel from motor command and time because this robot has no encoder.

### Sensor values

```cpp
LineSensorReading value;
if (read_line_sensor(f0, value)) {
  Serial.print(value.raw);
  if (value.normalizedValid) Serial.println(value.normalized);
}
read_line_sensor(b0, value);
read_line_sensor(cl, value);
read_line_sensor(cr, value);
print_line_sensor_table();  // Serial by default.
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
int32_t b = readADC_B(0);   // B0; channels 0..15.
int32_t l = readADC_CL();
int32_t r = readADC_CR();
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

## Motion examples

```cpp
motor(40, 40);                 // Forward.
servo(18, 90);                 // Servo angle.
f_line(40, 40, 0.85f, f0, 0); // Follow front line to f0.
b_line(40, 40, 0.85f, b0, 0); // Follow rear line to b0.
fw_gyro(0.0f, 70, 0.80f, 40.0f, 10);
bw_gyro(0.0f, 70, 0.80f, 30.0f, 10);
turn(tcr, 60, f11, 5);
turn_gyro(tfr, 60, 90.0f, 5); // Absolute +90 deg from START.
turn_gyro(tfl, 60, 0.0f, 5);  // Return to START heading.
rotate_spin(90.0f, 60, 10);    // Relative angle.
rotateFW_pivot(60.0f, 60, 10); // Absolute heading when gyro is ready.
rotateBW_pivot(0.0f, 60, 10);  // Return to heading zero when gyro is ready.
```

Positive rotation means right and negative means left. `rotate_spin()` uses a
relative angle. The two pivot functions use an absolute gyro heading when
`gyro_ready()` is true; without gyro, their timed fallback keeps the original
relative-angle behavior. A zero pivot argument without gyro stops successfully
without rotating.

`turn_gyro(TurnMode mode, uint8_t speed, float targetDeg, uint8_t stopPull = 0)`
uses an absolute heading from the latest `wait_button()` start press. Right is
positive, left negative, and targets must be within -180..180 degrees. It
retains each `turn()` mode's line approach, motor ratios, and speed stages;
only the rotating exit uses the gyro. Sensor readings
do not stop the rotating phase. Failures brake both wheels and print a
`turn_gyro failure:` reason on Serial. `tfl`/`tfr` still need their normal
calibrated line approach unless handed off from a line command. See
`examples/07_TurnGyro` for the start-to-+90-to-start sequence.
The gyro rotation keeps the requested motor speed until it reaches an early
brake point, then calls `motor(1, 1)` and returns. The brake lead is 20 degrees:
starting at 0 degrees with a +90 degree target brakes near +70 degrees. For a
short turn the lead is limited to half the turn angle. There is no near-target
slowdown, reverse pulse, or post-turn correction. `stopPull` remains
in the API for existing sketches but has no effect in `turn_gyro()`; sensor
based `turn()` keeps its original `stopPull` behavior. Measure the final angle
on the robot because the 20 degree lead depends on speed, battery, and grip.

Before running a mission, complete sensor calibration in `wait_button()` and
test motor direction with the diagnostics console. The console is available at
115200 baud and supports `help`, `status`, `i2c`, `mux`, `ads`, `adccal`,
`frontcal`, `leds`, `motor`, `stop`, and `eeprom_test YES`.

## Optional DRV8874 dual-motor driver

Existing sketches stay on TB6612FNG. To use two DRV8874 modules, select the
library-owned PH/EN configuration **before** `robot_begin()`:

```cpp
void setup() {
  select_motor_driver(MotorDriverType::DRV8874);
  robot_begin();
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
