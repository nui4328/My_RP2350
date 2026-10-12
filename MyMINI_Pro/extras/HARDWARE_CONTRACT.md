# MyMINI_Pro hardware contract — phase 1

This file is the source of truth for the first Arduino hardware-diagnostics
firmware. GPIO numbers are Raspberry Pi Pico 2 GP numbers.

## Sensor arrays

| Function | Front J13 | Rear J5 |
| --- | ---: | ---: |
| MUX S0 | GP17 | GP10 |
| MUX S1 | GP16 | GP11 |
| MUX S2 | GP15 | GP12 |
| MUX S3 | GP14 | GP13 |
| MUX analog signal | GP27 | GP26 |
| Calibration request | GP3, active LOW | ADCcal through MCP3421 |

- Both arrays: CH0 is left and CH15 is right when viewed from above.
- Black line produces a lower ADC reading; white floor produces a higher ADC
  reading.
- 16 sensors at 6 mm pitch; total center-to-center span is 90 mm.
- Front row is +90 mm from the wheel axle; rear row is -70 mm from the axle.
- Sensor height is 5 mm; competition line width is 20 mm.

## I2C bus

SDA is GP4, SCL is GP5, and the initial bus rate is 400 kHz.

| Address | Device | Role |
| ---: | --- | --- |
| 0x20 | PCF8574 | Eight status LEDs |
| 0x48 | ADS1115 | AIN0 battery, AIN1 center-left, AIN2 center-right, AIN3 auxiliary |
| 0x50 | CAT24C128 | Calibration/settings EEPROM |
| 0x68 | MCP3421A0 | Rear ADCcal input |

ADS1115 AIN0 sees the battery through a 20 kOhm / 6.8 kOhm divider. The
firmware uses a divider multiplier of approximately 3.941176. ADS1115 AIN3 is
named `ADC_AUX` in software even though the schematic net is `adc_F`.

## TB6612FNG motor outputs

| Motor | PWM | IN1 | IN2 |
| --- | ---: | ---: | ---: |
| Left | GP6 | GP8 | GP7 |
| Right | GP19 | GP21 | GP20 |

Each TB6612FNG module uses its A/B channels in parallel for one motor. STBY is
hard-wired high, so software stops motors using PWM and direction inputs.

Motor commands are percentages from -100 to 100. PWM is configured with
`analogWriteResolution(12)` and `analogWriteFreq(15000)`, with duty values
from 0 to 4095. Diagnostics accept `motor L|R [-30..30]`, default to 25%
when power is omitted, and automatically stop after 1000 ms.

## Optional DRV8874 motor outputs

The existing board contract assigns the TB6612 outputs above. If those modules
are physically replaced by two DRV8874 modules, the verified PH/EN signal
reuse is:

| Logical `motor()` side | EN/IN1 PWM | PH/IN2 direction | nSLEEP |
| --- | ---: | ---: | ---: |
| Left | GP19 | GP20 | GP21 |
| Right | GP6 | GP7 | GP8 |

This mapping is intentionally separate from the TB6612 configuration, whose
logical routing is preserved unchanged for legacy sketches. The installed
DRV8874 modules use separate Pico-controlled nSLEEP lines (not hard-wired
high). IMODE, VREF, and IPROPI remain board-level connections without MCU GPIO
assignment. PMODE must be tied to GND to select PH/EN mode. Do not connect
TB6612 and DRV8874 inputs to these signals simultaneously.

The installed coreless motors are rated 12.4 V, 500 RPM, with a reported 5 A
stall current. This exceeds the TB6612FNG continuous-current capability. Phase
1 therefore limits manual diagnostic commands to 30% and stops them after one
second. Tests must be performed with wheels off the ground; software is not a
substitute for current limiting.

## Phase boundary

Phase 1 contains only safe hardware diagnostics. It intentionally does not
contain line normalization, calibration capture, PID control, racing states,
or automatic motor motion.
