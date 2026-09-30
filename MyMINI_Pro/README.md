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
| Direct control | `motor`, `servo` |
| Line | `f_line`, `b_line` |
| Gyro straight | `fw_gyro`, `bw_gyro` |
| Turns | `turn` |
| Rotation | `rotate_spin`, `rotateFW_pivot`, `rotateBW_pivot` |
| Settings | `set_motor_brake_at_zero`, `get_motor_brake_at_zero`, `set_turn_motor`, `set_turn_overshoot`, `set_turn_timeout`, `set_turn_approach_kp`, `set_turn_approach`, `set_turn_line_search`, `set_rotate_fallback` |

`stopPull` is the optional reverse-direction braking strength (`0..100`) used
when a supported motion command finishes. Straight distance commands estimate
travel from motor command and time because this robot has no encoder.

## Motion examples

```cpp
motor(40, 40);                 // Forward.
servo(18, 90);                 // Servo angle.
f_line(40, 40, 0.85f, f0, 0); // Follow front line to f0.
b_line(40, 40, 0.85f, b0, 0); // Follow rear line to b0.
fw_gyro(0.0f, 70, 0.80f, 40.0f, 10);
bw_gyro(0.0f, 70, 0.80f, 30.0f, 10);
turn(tcr, 60, f11, 5);
rotate_spin(90.0f, 60, 10);    // Relative angle.
rotateFW_pivot(60.0f, 60, 10); // Absolute heading when gyro is ready.
rotateBW_pivot(0.0f, 60, 10);  // Return to heading zero when gyro is ready.
```

Positive rotation means right and negative means left. `rotate_spin()` uses a
relative angle. The two pivot functions use an absolute gyro heading when
`gyro_ready()` is true; without gyro, their timed fallback keeps the original
relative-angle behavior. A zero pivot argument without gyro stops successfully
without rotating.

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

For DRV8874, PMODE must be physically tied to GND. `motor(0, 0)` means normal
PH/EN brake (EN low); it does not sleep the bridge. IMODE, VREF, and IPROPI
remain hardware-configured because this library has no verified GPIO for them.
`set_motor_brake_at_zero(false)` retains its legacy TB6612 coast behavior, but
DRV8874 PH/EN still brakes at zero because coast requires a different hardware
control mode and the library never uses nSLEEP as a routine stop.
