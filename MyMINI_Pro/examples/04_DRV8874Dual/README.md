# DRV8874 dual-motor example

This example drives one brushed motor per DRV8874 module in PH/EN mode.
`PMODE` on both modules must be wired to **GND** before `nSLEEP` is brought
high: this latches PH/EN mode. The code drives `EN/IN1` with PWM and `PH/IN2`
with direction. It never uses `nSLEEP` as an ordinary stop command: in PH/EN,
`EN = 0` is the normal low-side brake; `nSLEEP = 0` places the bridge in Hi-Z
sleep.

## Confirmed MyMINI_Pro signals

| Logical motor command | DRV8874 pin | Pico GPIO |
| --- | --- | ---: |
| `motor(left, ...)` | EN/IN1 (PWM) | GP19 |
| `motor(left, ...)` | PH/IN2 (direction) | GP20 |
| `motor(..., right)` | EN/IN1 (PWM) | GP6 |
| `motor(..., right)` | PH/IN2 (direction) | GP7 |

The library hardware contract confirms these are existing TB6612 output pins;
they are not sensor or servo pins. They must be rewired from the TB6612
modules to the two DRV8874 modules, not driven by both types at once.

The installed-module wiring has nSLEEP under Pico control: left nSLEEP is
**GP21** and right nSLEEP is **GP8**. They are configured internally by the
library; this example does not need to set either pin. The two lines are
separate, so the driver holds both low during setup, then releases both high
after the PH/EN pins are configured. Do not change them to hard-wired-high
without updating `RobotConfig.h` and verifying the physical wiring.

`IMODE`, `VREF`, and `IPROPI` are board-level current-limit/current-sense
connections and this example does not claim MCU GPIO for them. Configure them
from the DRV8874 hardware design and datasheet before powering the motors.

Connect motor supply `VM` for each module according to its voltage/current
requirements, and connect Pico ground, motor supply ground, and both DRV8874
grounds together. Do not power a motor from the Pico 3.3-V rail. Test with the
wheels off the ground first, and set each `invertDirection` flag only if its
actual motor polarity is opposite to the `motor(left, right)` contract.
