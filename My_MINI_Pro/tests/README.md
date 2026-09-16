# Native ADS1115 tests

The test uses minimal Arduino and `TwoWire` stubs so it can exercise the
library's I2C configuration without a board or firmware upload. From the
library root, run:

```powershell
g++ -std=c++17 -Itests/arduino_stubs -Isrc tests/ads1115_raw_test.cpp src/Pico2_MyMiniPro.cpp -o tests/ads1115_raw_test.exe
./tests/ads1115_raw_test.exe
```

It verifies the `readAdcL()` (AIN1), `readAdcR()` (AIN2), and AIN3 wrappers'
ADS1115 input selection, shared single-shot configuration, signed raw
conversion values, I2C failure, and a conversion-ready timeout. It also
verifies underbody calibration extrema, duration/completion, endpoint and
clamped normalization, EEPROM persistence/corruption handling, and legacy
Front/Rear CAL1 compatibility. It also covers debounced short Start presses,
the five-second Start hold that enters underbody calibration, the required
release-and-new-short-press sequence, and invalid-calibration safety.
The live-reading test also locks down the Serial contract: a calibrated
regular line contains only normalized underbody fields named `ADC NL` and
`ADC NR`, never the MCP3421 or raw `ADC L`/`ADC R` values.
It additionally verifies the fixed Pico 2 Servo GPIO allowlist, lazy
attachment, detach behavior, invalid-GPIO rejection, and 0–180 degree
clamping without requiring a board. It also verifies that stopping the Pico 2
startup melody leaves the buzzer silent on later service passes and that the
default `Pico2MyMiniPro::begin()` plays its reset acknowledgement followed by
the short startup melody.
