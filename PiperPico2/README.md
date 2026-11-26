# PiperPico2

Hardware library for Piper Robot on Raspberry Pi Pico 2, using the Earle Philhower Arduino-Pico core (`rp2040` architecture). Version 0.6.1.

## Layout

- `PiperPico2.h`: public API and board pin assignments.
- `PiperPico2.cpp`: startup, sensor reads, battery, OLED and sound. Includes per-channel Servo pulse limits.
- `PiperPico2Motor.cpp`: motor direction caching and drive shutdown.
- `PiperPico2Encoder.cpp`: encoder interrupts owned by this library.
- `PiperPico2Calibration.cpp`: incremental calibration and checked EEPROM operations.
- `example/`: existing application sketches, intentionally unchanged.
- `examples/PiperPico2_Basic/`: new starter sketch with exactly three Arduino tabs: `PiperPico2_Basic`, `README`, and `set_bord`.
- `tests/`: host behavior tests and a Pico 2 compile-only smoke sketch.

Install Adafruit GFX Library, Adafruit SSD1306, and Adafruit PWM Servo Driver Library through Arduino Library Manager. Their transitive dependencies are also required. The main library no longer depends on the two local EncoderLibrarys classes. Existing application sketches still require their own additional libraries.

## Three-tab starter example

Open `examples/PiperPico2_Basic/PiperPico2_Basic.ino` in Arduino IDE, or use File > Examples > PiperPico2 > PiperPico2_Basic after library installation. All three `.ino` files belong to the same sketch:

- `PiperPico2_Basic`: startup menu loop, one-time user program after RUN in setup(), and repeating user program/Serial telemetry in loop(). Add your robot program here.
- `README`: Thai explanations and commented function usage snippets; they do not run automatically.
- `set_bord`: choose the driver and direction for A–D, set pulse endpoints for Servo 0–5 in microseconds, swap Encoder A/B pairs, and configure I2C and motor PWM frequencies.

The default selects LEGACY for every motor and configures illustrative 1000–2000 us Servo endpoints; replace these with the actual hardware limits. The starter does not command motor rotation or Servo movement. It uses only the library's internal encoder counters and stops application work if board initialization fails. The OLED UI adapted from `example/piper_spt/UI.ino` provides RUN, READSENSOR (ADC/COLOR/GYRO/ENCODER), and CALIBRATESENSOR (ADC-Cal/TCS-Cal), with separate TCS0-Cal and TCS1-Cal choices inside TCS-Cal. A non-blocking splash plays a one-time ascending C-D-E-G-E-D-C startup melody (about one second) before the menu. Edit `BOARD_BOOT_MELODY_ENABLED`, `BOARD_BOOT_MELODY` and `BOARD_BOOT_NOTE_GAP_MS` in `set_bord` to disable or customize it. The menu waits for the melody to finish; the separate RUN beep remains 500 ms. Turn the MUX-10 knob to select, short-click to enter, and hold the button for 3 seconds to go back. Short-clicking RUN sounds one 500 ms beep, then exits the menu loop in setup() and hands control to the user program. The main loop no longer calls the UI. Button presses and long holds after RUN do not stop motors or return to the menu. The user may place one-time robot commands immediately after the setup() menu loop, and repeating commands in loop(). Four ADC menu items scroll within the three available rows. ADC calibration uses the incremental API and checks the EEPROM result. Voltage status and an optional rate-limited asynchronous beep replace blocking warnings.

COLOR is enabled by default (`BOARD_ENABLE_COLOR=1`) and uses the installed `my_TCS` and `SoftwareWire` libraries. Select CALIBRATESENSOR > TCS-Cal > TCS0-Cal or TCS1-Cal. Each short click captures one prompted color (RED, GREEN, BLUE, YELLOW, WHITE, BLACK) for the selected sensor only. After all six colors, it verifies flags and save status, displays the result, and returns to TCS-Cal. It never starts or saves the other sensor automatically. Long-hold cancellation reloads only the selected sensor; read/low-light errors retain the current step for retry. Existing `my_TCS` APIs for complete calibration and save status are required. TCS EEPROM regions (0x0080–0x00A4 and 0x00C0–0x00E4) do not overlap current line calibration (0x0000–0x0027); other EEPROM formats must be checked separately. Color capture/storage still block within that external driver.

GYRO now defaults to the installed local **BNO055** library (`BOARD_ENABLE_GYRO=1`, `BOARD_GYRO_TYPE=BOARD_GYRO_BNO055`). Set `BOARD_BNO055_ADDRESS` to the module's address (the currently configured 0x29 matches the user-reported hardware; shared-bus use with TCS0 requires physically selecting 0x28 and updating the constant). It shares Wire on SDA4/SCL5. If COLOR is enabled, do not assign BNO055 the same 0x29 address as TCS0 on that bus. The current configuration enables both features but BNO055 and TCS0 are configured at 0x29 on Wire. The UI detects that conflict, skips both conflicting devices, and displays ADDR CONFLICT 0x29. TCS1 at 0x29 on Wire1 (SDA26/SCL27) can still initialize. Changing the software constant alone does not change a module's physical address. The reader shows filtered accumulated yaw in degrees relative to startup, Z rotation rate in degrees/second, and calibration levels System/Gyro/Accel/Mag (0–3). It loads any existing stored gyro offset but does not start calibration or write EEPROM automatically. Read failures display an error without stale readings; rejected headings and unavailable calibration status are distinguished. Display-only yaw limits are +/-99999.9 degrees. The supplied local BNO055 library must support `loadStoredGyroCalibration()`, `calibrationStatus()` and `lastUpdateError()`.

Select `BOARD_GYRO_TYPE=BOARD_GYRO_BMI160` to retain the prior `my_BMI160` backend, or `BOARD_ENABLE_GYRO=0` to disable it. BMI160's `gyro('z')` returns an angle in degrees and its driver has no per-read error flag. Sensor initialization can block. After RUN, the UI does not poll either gyro; robot control code must explicitly update the selected sensor when needed.

`getDisplay()` exposes the library's initialized OLED for this UI; it reuses the existing buffer and does not call OLED begin again. The example keeps exactly three tabs, with UI/configuration in `set_bord` and user program hooks in the main tab. Compile for `rp2040:rp2040:rpipico2`.

## Startup and status

Create one long-lived `PiperPico2 robot;` object. `robot.begin()` initializes the hardware and returns false if PWM, OLED, encoder ownership or bus recovery fails. Check `pwmReady()`, `displayReady()` and `motorOK()` for component status. A missing or invalid EEPROM calibration does not fail startup: the default sensor range remains 0–4095 and `calibrationReady()` is false.

Startup disables all four GPIO motor PWM outputs before bus initialization. The library leaves Serial setup and startup tones to the application. It waits 250 ms for power settling and retries bus/device initialization over a nominal 2-second window, plus per-attempt I/O time. Each I2C transaction has a 25 ms timeout with controller reset enabled. Start Serial in your own sketch if needed.

The bus defaults to 100 kHz. `setI2CClock(400000)` selects 400 kHz for the library, including OLED transfers and restoration afterwards; only 100–400 kHz is accepted. Choose a speed supported by all attached devices and wiring. Other code changing Wire or creating its own OLED instance can still override the shared bus clock.

## Motor control

`motor('A', speed)` through `'D'` accepts -100 to 100. The default `MotorDriver::LEGACY` retains the existing two-direction-input interface. Configure the type before the first `begin()`; configuration is locked once startup is attempted. No commands or wiring changes are issued by selecting a type.

### Selecting DRV8874

The attached board schematic ties PMODE to GND, selecting **PH/EN mode**. EN uses the existing GPIO PWM pin; PH and nSLEEP use PCA9685 channels. The mapping below interprets schematic nets `ledN` as PCA9685 channel N.

| Motor | EN / GPIO PWM | PH / PCA9685 | nSLEEP / PCA9685 | Mapping basis |
|---|---:|---:|---:|---|
| A | 8 | 0 | 1 | U2 in the supplied schematic |
| C | 7 | 2 | 3 | U9 in the supplied schematic |
| B | 9 | 4 | 5 | Extension of existing paired channels; not shown in supplied schematic |
| D | 6 | 6 | 7 | Extension of existing paired channels; not shown in supplied schematic |

For a board with DRV8874 on all four motors, select it at construction:

```cpp
PiperPico2 robot(PiperPico2::MotorDriver::DRV8874);
```

For the two drivers shown in the schematic, keep other motors on their existing type:

```cpp
PiperPico2 robot;
// In setup(), before robot.begin():
robot.setMotorDriver('A', PiperPico2::MotorDriver::DRV8874);
robot.setMotorDriver('C', PiperPico2::MotorDriver::DRV8874);
robot.begin();
// Existing calls remain unchanged:
robot.motor('A', 50);
robot.motor('C', -50);
robot.stopMotors();
```

`setMotorDriver(id, type, reverseDirection)` returns false for an invalid ID/type or changes after startup. Its optional third argument defaults to false; pass true to reverse the motor's speed sign. This works with either type. Positive DRV8874 commands set PH HIGH (OUT1 to OUT2); negative commands set PH LOW. The schematic connects MR1 to OUT2 and MR2 to OUT1, so choose reversal according to your robot's required wheel direction rather than assuming chassis-forward from signal names.

DRV8874 initialization raises nSLEEP and waits 1 ms before EN can drive. Speed zero sets EN LOW while keeping nSLEEP HIGH, giving low-side braking. The driver stays awake between commands to avoid repeated startup delays. Unknown or invalidated cached states reassert nSLEEP and wait again before enabling PWM. Direction changes only update PH; same-direction speed changes use GPIO PWM alone. This implementation supports the shown PMODE=GND wiring; it does not provide the alternate PMODE control modes, sleep/coast selection, current sensing or nFAULT monitoring. nFAULT is unconnected in the supplied schematic.

Control truth table and wake timing are based on [TI DRV8874 datasheet, sections 6.5 and 7.3.2.1](https://www.ti.com/lit/ds/symlink/drv8874.pdf). Current-limit configuration remains a hardware responsibility; in the supplied schematic IPROPI is grounded.

For both types, direction changes first disable GPIO PWM before writing PCA9685 control signals. A failed control write leaves the affected motor PWM off, invalidates its cache and sets `motorOK()` false; the next command retries. `stopMotors()` disables all drive PWM; it requests neutral direction inputs for LEGACY and braking for DRV8874. PWM scaling uses integer arithmetic so commands of ±100 reach the configured maximum exactly.

`getPWM()` invalidates direction caches when returning the raw driver. If you retain that reference and later change motor channels 0–7 directly, call `invalidateMotorCache()` afterwards. An external PCA9685 reset also requires cache invalidation. Runtime bus-disconnect detection is limited to actual I2C transactions; cached commands do not probe the bus.

## Encoders and existing applications

The library now initializes its own four rising-edge interrupt counters. One count per A rising edge preserves the counting convention of the former external drivers. Default A/B pairs are (20,19), (21,22), (16,17), (10,18). Before the first `begin()`, call `setEncoderPins(index, pinA, pinB)` to select either order of an existing pair. Indices 0, 1, 2, 3 map to `enc1Left()`, `enc1Right()`, `enc2Left()`, `enc2Right()`. For example, `setEncoderPins(0, 19, 20)` reverses the first counter relative to the same physical rotation. Only the original two pins for that counter are allowed; invalid indices/pairs and configuration after startup return false without changing the settings. The first pin receives the rising-edge interrupt; the second is sampled for direction. This swaps the counting edge as well as reversing the sign, so a partial rotation can differ by a count. It does not reverse motor output. Settings survive repeated `begin()` calls in RAM and must be set again after reboot. In the three-tab starter, edit the corresponding row of `BOARD_ENCODER_PINS` in `set_bord`. Reset preserves the calling core's interrupt state. Use the API and reset from one core; cross-core synchronization is not provided.

Only one live object may own the encoder interrupts. Object destruction releases them. External code attaching interrupts to the same pins replaces these callbacks; do not mix the two owners.

**Deferred application migration:** the current `example/` sketches create their own encoders and attach interrupts after `robot.begin()`. Those external counters remain the application's source of truth; the robot's internal encoder getters will not follow them. When editing those sketches later, use `begin(false)` for external ownership, or remove their external encoders and consistently use the robot getters. The ST example also uses A/B=(19,20) for its first encoder, opposite to the library pin order; verify physical wiring and distance polarity during migration. Application timeouts, derived distance constants and duplicated sketch logic are also deferred.

## Calibration and EEPROM

Existing APIs `calibrate()`, `loadCalibration()` and `saveCalibration()` remain available. EEPROM address is 0x50 and the existing 40-byte little-endian min/max layout is preserved. Loading checks complete reads and requires `0 <= min < max <= 4095` for every sensor before replacing any active values. Saving validates all channels, ACK-polls with a 20 ms deadline per write, then reads back each write. A failure returns false. The EEPROM layout has no checksum or atomic dual-slot commit: a power interruption during saving can still leave a mixed but numerically valid record.

For incremental sampling call `startCalibration(durationMs)`, then call `updateCalibration()` regularly while `isCalibrating()` is true. Sampling runs at most once per 50 ms call interval. Invalid or cancelled scans retain previous live calibration. `calibrationSaved()` reports the last save result. The final update performs EEPROM saving synchronously; it is not a hard real-time operation. `calibrate()` remains a blocking compatibility wrapper.

`playToneAsync(frequency, durationMs)` delegates timing to the core tone implementation. `playTone()` retains its blocking duration for compatibility. `updateBattery()` samples the MUX voltage input; `getBatteryVoltage()` returns that cached sample; `bat_control()` is a compatibility alias for sampling, without an automatic motor cutoff. `getVoltage()` performs an immediate read. `showVoltageUntilButton()` remains a blocking UI method, stops motors first and returns immediately if the display is unavailable.

## Per-channel Servo pulse limits

`setservo(channel, angle)` uses logical channels 0–5 (PCA9685 channels 15, 14, 13, 12, 11, 10). Angles are clamped to 0–180 and mapped into that channel's configured pulse interval. The frequency remains 60 Hz. Invalid channels, invalid legacy defaults and calls before successful PWM initialization are ignored.

Configure a Servo's measured endpoints before its next movement command. Both setters return bool; false leaves the previous interval unchanged. They can be called before or after `begin()`. Setting a range sends no I2C command and does not change an already active output; the new interval takes effect on the next `setservo()` call.

- `setServoPulseRangeUs(channel, minUs, maxUs)` accepts microseconds. Use pulse endpoints appropriate for the actual Servo and mechanism. Conversion uses the nominal 60 Hz frequency; the lower endpoint rounds up and the upper endpoint rounds down to PCA9685 ticks. Unrepresentable intervals and intervals exceeding the PWM counter range are rejected.
- `setServoPulseRange(channel, minTicks, maxTicks)` accepts raw PCA9685 counts, matching the units of the existing `SERVO_MIN/MAX`. Valid ranges satisfy `1 <= minTicks < maxTicks <= 4095`. Values 0 and 4096 are not accepted as endpoints.

For example, the library call `robot.setServoPulseRangeUs(0, 1000, 2000)` assigns channel 0 a nominal 1000–2000 microsecond interval. At 60 Hz this becomes counts 246–491. Then `robot.setservo(0, 0)` uses the lower endpoint, `robot.setservo(0, 180)` uses the upper endpoint, and intermediate angles interpolate between them. These numbers illustrate the API; determine actual endpoints for your hardware. Oscillator accuracy affects the actual pulse width, so nominal conversion does not replace measurement.

Each channel is independent. Channels without an override continue to use the public `SERVO_MIN=90` and `SERVO_MAX=760` defaults in raw PWM counts, retaining the previous behavior. Updating those public defaults only affects channels without overrides. Repeated `begin()` calls preserve the configured ranges in RAM but turn all Servo outputs off during initialization. The ranges are not saved to EEPROM and must be configured again after reboot.

The range maps the existing logical 0–180 command scale onto the configured mechanical travel; it does not measure actual shaft angle. There is no speed ramp or position feedback. Direct writes through `getPWM()` bypass these limits. The existing `example/` sketches remain unchanged. The new `examples/PiperPico2_Basic/` starter demonstrates per-channel limits in its `set_bord` tab.

## Verification

Run `powershell -ExecutionPolicy Bypass -File tests/run-host-tests.ps1` from the library root with MinGW g++ installed. Tests use simulated hardware to exercise production C++ sources. They cover both driver modes, mixed configurations, pin mapping, direction inversion, full duty, startup/wake timing, cache invalidation and I2C failures, in addition to encoder, calibration and per-channel Servo limits (including invalid inputs, unit conversion, channel isolation and legacy defaults). They validate control flow and data handling, not electrical behavior, ADC accuracy or actual latency.

Compile `tests/CompileSmoke` with board `rp2040:rp2040:rpipico2` and this library, using your own Arduino CLI configuration. This sketch is for compile/link verification only; its setup issues motor commands, so do not upload it as a neutral hardware test.

On hardware, measure `micros()` around your actual full control loop, recording mean, maximum and variation without printing each iteration. Compare same-direction motor updates and direction changes separately, then repeat with OLED updates and sensor reads enabled. Verify both encoder polarities, failed-sensor handling in the application, EEPROM disconnect behavior, and motor stop before deployment. Actual speed improvements require these measurements.

## KLJ-7525-3627 buzzer tuning

The starter uses 2093–3136 Hz melody notes and `BOARD_RUN_TONE_HZ=2731`. These frequencies target the specified 2731 Hz operating frequency instead of the earlier 1047–2093 Hz melody; any loudness improvement requires listening/measuring on the actual board. This is frequency tuning, not an amplitude/volume increase.

The [manufacturer datasheet](https://datasheet.lcsc.com/datasheet/pdf/34bf2407de16d1edc14e00e246bd5b06.pdf?productCode=C189208) specifies 3.6 Vp-p square-wave drive at 50% duty, 2.5–4.5 Vp-p operation, and up to 90 mA at rated voltage, with a transistor driver circuit. The GPIO supplies the control signal to an appropriate driver; do not drive that coil directly from a Pico GPIO. The example does not change GPIO drive current or supply voltage.

## Color-first classification

The installed sibling `my_TCS/my_TCS.cpp` now classifies RED/YELLOW/GREEN/BLUE before either achromatic class. Existing chromatic predicates and the distance fallback run first; the fallback only compares those four colors. If none matches, `c < 180` selects BLACK and `c > 1800` selects WHITE, subject to the respective calibration flags. Intermediate brightness returns UNKNOWN. WHITE/BLACK no longer participate in ratio-distance selection. Dark readings, including c=0, can reach BLACK without a divide by zero. EEPROM layout and the existing calibration data are unchanged, and both TCS instances use this shared implementation.

`python tests/test_tcs_color.py` extracts and compiles the actual installed getColor() body with a minimal classification fixture. It covers bright/dim primary colors, C-only white/black, zero clear, threshold boundaries and calibration flags for two instances. It does not emulate optical behavior or I2C.

## Encoder OLED page

READSENSOR > ENCODER displays `enc1Left()` / `enc1Right()` on the first data row and `enc2Left()` / `enc2Right()` on the second, in two 64-pixel columns. Labels are 1L/1R and 2L/2R. The page refreshes with the UI and long-hold returns to READSENSOR without resetting counts. READSENSOR scrolls to reveal its fourth item. Counts too wide for a cell are abbreviated in millions (M); full stored counts remain unchanged and available through the getters/Serial. Layout tests cover both signs and the full signed 32-bit extremes.

## Cold-power startup recovery

The former one-shot initialization could fail if SDA/SCL or an I2C device was not ready immediately after power-on; the sketch then remained in its failure path until a manual reset. This is a software failure path consistent with the reported symptom, not a measurement of the physical root cause. Startup now keeps local motor PWM disabled, waits 250 ms, checks/reinitializes the bus, retries PCA9685/OLED setup, and returns failure after a nominal 2-second retry window (plus I/O time) instead of hanging indefinitely. OLED readiness is probed before and after initialization; an ACK still cannot prove that the panel is visibly displaying correctly. The OLED has no connected hardware reset pin in this configuration.

If PWM initialization fails but the OLED works, the starter displays BOARD INIT FAILED. Serial diagnostics repeat every second when board initialization fails. The device-address conflict guard is independent of power stabilization. Host tests simulate delayed device availability, temporarily held SDA, permanently held SCL, missing OLED and visible failure diagnostics. Actual power rails, reset timing and repeated power-off/on tests still require the physical board; no firmware was uploaded automatically.
