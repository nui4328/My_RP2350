#pragma once

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

/**
 * A reusable Arduino library for a robot with two 16-channel 74HC4067
 * line-sensor arrays.  It deliberately does not print to Serial, keeping the
 * scan path fast enough for line-following code.
 */
class MyMiniPro {
 public:
  static constexpr uint8_t kSensorCount = 16;

  struct MuxPins {
    uint8_t s0;
    uint8_t s1;
    uint8_t s2;
    uint8_t s3;
    uint8_t signal;
  };

  struct MotorPins {
    uint8_t pwm;
    uint8_t in1;
    uint8_t in2;
  };

  struct Settings {
    uint8_t frontCalibrationPin;  // Button to GND; 255 disables it.
    uint8_t eepromAddress;         // CAT24C128 normally 0x50.
    uint8_t mcp3421Address;        // MCP3421A0 normally 0x68.
    int32_t rearButtonThreshold;   // Rear button is pressed below this value.
    uint16_t adcMaximum;           // 4095 for Pico 2 12-bit ADC; 1023 for UNO.
    uint16_t muxSettleMicroseconds;
    uint32_t calibrationTimeMs;
    bool rearButtonEnabled;
  };

  struct SensorValues {
    uint16_t current;
    uint16_t maximum;
    uint16_t minimum;
  };

  // Forward-only line follower for a black line on a white surface. Sensor
  // channel 0 is the physical left edge and channel 15 is the right edge.
  // kp/ki/kd produce motor-command units; start conservatively and tune on the
  // actual robot. steeringSign reverses the correction if motor wiring makes
  // a positive correction turn the wrong way.
  struct ForwardLineFollowerSettings {
    int baseSpeed = 35;
    float kp = 9.0f;
    float ki = 0.0f;
    float kd = 0.08f;
    float integralLimit = 5.0f;
    uint16_t minimumDarknessSum = 1200;
    int maxSteering = 35;
    uint32_t controlIntervalUs = 5000;
    int8_t steeringSign = 1;
  };

  enum class Direction : uint8_t { Forward, Reverse };
  enum class Array : uint8_t { Front, Rear };

  MyMiniPro(const MuxPins &frontMux, const MuxPins &rearMux,
                    const Settings &settings);

  // Configure GPIO and retain the I2C bus. Configure custom I2C pins before
  // this call when the selected Arduino board supports that feature.
  void begin(TwoWire &wire = Wire);

  // Passive-buzzer support. Call beginBuzzer(9) once in setup() for the
  // buzzer wired to Pico 2 GP9. Timed beeps are non-blocking.
  void beginBuzzer(uint8_t pin);
  void playStartupMelody();
  void playTone(uint16_t frequency);
  void beep(uint16_t frequency, uint16_t durationMs);
  void stopBuzzer();
  void updateBuzzer();
  bool buzzerActive() const;

  // Start button is active-low (button wired between GPIO and GND).
  // The robot remains stopped after boot until this button is pressed once.
  void beginStartButton(uint8_t pin);
  bool serviceStartButton();
  bool robotStarted() const;
  void stopRobot();

  // Idle/setup mode: handles both calibration buttons, prints front and rear
  // values while waiting, and returns true only after the Start button is hit.
  bool waitButton(Stream &output, uint32_t monitorIntervalMs = 100);
  // Convenience form for the standard Arduino Serial Monitor.
  bool waitButton(uint32_t monitorIntervalMs = 100);

  // Motor command range: -100 (full reverse) to +100 (full forward).
  // Parameter order is Motor(left, right).
  void beginMotors(const MotorPins &leftMotor, const MotorPins &rightMotor,
                   uint32_t pwmFrequency = 20000);
  void Motor(int left, int right);
  void stopMotors();

  // Samples a filtered, cached battery voltage for optional motor-voltage
  // compensation. Call serviceMotorVoltageCompensation() from loop(); it
  // rate-limits ADS1115 sampling. Motor() is deliberately not scaled, so PID
  // steering corrections retain their tuned gain. Invalid readings and USB
  // readings below 6.5 V use a gain of 1.0.
  // maximumCompensationPercent is limited to 100; 5 allows at most 5% extra PWM.
  void setMotorVoltageCompensation(bool enabled, float referenceVoltage = 12.4f,
                                   uint32_t sampleIntervalMs = 500,
                                   uint8_t maximumCompensationPercent = 5);
  void serviceMotorVoltageCompensation();
  float motorVoltageCompensationGain() const;

  // Apply the cached gain to a base/forward command before mixing PID steering
  // correction into left/right commands. The signed result is limited to +/-100.
  int compensateBaseMotorSpeed(int baseSpeed) const;

  // The service scans only Front, calculates a black-line centroid, and drives
  // the motors. It will not run without valid Front calibration. A lost line
  // stops both motors; it never searches blindly. Call from the control loop.
  void configureForwardLineFollower(const ForwardLineFollowerSettings &settings);
  void setForwardLineFollowerEnabled(bool enabled);
  bool forwardLineFollowerEnabled() const;
  bool forwardLineDetected() const;
  float forwardLineError() const;
  bool serviceForwardLineFollower();

  // Compact forward line-following interface. Call FLine() repeatedly from
  // loop(); leftSpeed/rightSpeed are independent base speeds and kp is the
  // proportional gain to use for this call. Kd and Ki retain the values set
  // by setFLinePID().
  void FLine(int leftSpeed, int rightSpeed, float kp);

  // Set the D and I gains retained by FLine(). Kp is intentionally supplied
  // by FLine() itself. Ki defaults to zero for ordinary PD control.
  void setFLinePID(float kd, float ki = 0.0f);

  // Call once at boot. False means calibration has not been saved yet.
  bool loadCalibration();
  bool saveCalibration();
  bool hasCalibration(Array array) const;

  // Scan only the requested 16-sensor array. This is the normal fast path.
  void scan(Direction direction);
  void scanFront();
  void scanRear();

  // Simple single-channel aliases. channel is 0 through 15; an invalid channel
  // returns zero. The read methods scan their array before returning its value.
  uint16_t readSensorFront(uint8_t channel);
  uint16_t minSensorFront(uint8_t channel) const;
  uint16_t maxSensorFront(uint8_t channel) const;
  uint16_t readSensorRear(uint8_t channel);
  uint16_t minSensorRear(uint8_t channel) const;
  uint16_t maxSensorRear(uint8_t channel) const;

  // Intended for setup/testing mode only. It scans both arrays and emits one
  // line of calibrated 0..1000 values: F: 16 values | R: 16 values.
  void printLiveReadings(Stream &output);

  // Raw MCP3421 value used by the rear calibration button.
  int32_t rearCalibrationButtonValue();

  // Returns current, maximum, minimum in that order.
  SensorValues values(Array array, uint8_t channel) const;

  // Calibration arrays loaded from CAT24C128. Index 0..15 is the sensor
  // channel. The returned arrays are read-only to protect calibration data.
  const uint16_t *minValues(Array array) const;
  const uint16_t *maxValues(Array array) const;

  // ADS1115 analog inputs. ADS1115ADDR defaults to 0x48.
  static constexpr uint16_t kUncalibratedNormalizedValue = UINT16_MAX;
  void setAds1115Address(uint8_t address);
  int16_t readAds1115Raw(uint8_t channel);
  // Convenience accessors for the non-battery ADC inputs. They return the
  // signed ADS1115 conversion count, or INT16_MIN when a conversion cannot be
  // completed. AIN1 and AIN2 are the left and right underbody white/black
  // sensors. Their polarity and operating threshold are hardware-dependent.
  int16_t readAdcL();  // AIN1, underbody sensor by the left wheel.
  int16_t readAdcR();  // AIN2, underbody sensor by the right wheel.
  int16_t readAds1115Ain3Raw();     // AIN3, no board-specific role.

  // Programmatic, non-blocking calibration for the AIN1/AIN2 underbody
  // sensors. Call startUnderbodyCalibration(), then call
  // serviceUnderbodyCalibration() regularly until it returns false and
  // underbodyCalibrationActive() is false. The existing calibration duration
  // setting is used. Both sensors must observe a non-zero range before the
  // result is considered valid and saved.
  void startUnderbodyCalibration();
  bool serviceUnderbodyCalibration();
  bool underbodyCalibrationActive() const;
  bool hasUnderbodyCalibration() const;
  int16_t adcLMinimum() const;
  int16_t adcLMaximum() const;
  int16_t adcRMinimum() const;
  int16_t adcRMaximum() const;
  // Short aliases for the saved underbody ADC calibration extrema.
  int16_t minAdcL() const;
  int16_t maxAdcL() const;
  int16_t minAdcR() const;
  int16_t maxAdcR() const;

  // Return a live 0..1000 reading when the corresponding underbody range is
  // valid. Return kUncalibratedNormalizedValue if calibration is unavailable,
  // invalid, or the ADS1115 read fails.
  uint16_t readAdcLNormalized();
  uint16_t readAdcRNormalized();
  float readAds1115Voltage(uint8_t channel);

  // AIN0 battery measurement. The Pico2MyMiniPro default ratio is 4.0.
  void setBatteryDividerRatio(float dividerRatio);
  void setBatteryCalibration(float gain, float offsetVolts);
  float readBatteryVoltage();
  float readBatteryVoltage(float dividerRatio);

  // PCF8574P battery-level LED bar on P0..P7. The caller supplies the
  // hardware address (normally 0x20..0x27, set by A0..A2) and whether an LED
  // turns on when its PCF pin is LOW. beginBatteryLevelLeds() immediately
  // displays the current battery voltage.
  bool beginBatteryLevelLeds(uint8_t pcf8574Address, bool ledsActiveLow);

  // Reads readBatteryVoltage() then displays it on P0..P7. The float overload
  // is useful when the voltage has already been sampled. Below 11.0 V clears
  // all LEDs; 11.0 V lights P0; every additional 0.2 V lights one more LED;
  // 12.4 V and above lights P0..P7. Both return false on an I2C error or if
  // beginBatteryLevelLeds() has not been called.
  bool updateBatteryLevelLeds();
  bool updateBatteryLevelLeds(float batteryVoltage);
  static uint8_t batteryLevelLedCount(float batteryVoltage);

  // Converts the calibrated range to 0..1000. Returns zero without a valid
  // calibration range.
  uint16_t normalized(Array array, uint8_t channel) const;

  // Begin a 10-second (or configured duration) calibration explicitly.
  void startCalibration(Array array);

  // Call regularly from loop(). Returns true when calibration is active or a
  // calibration button has just started it. While true, do not run motor PID.
  bool serviceCalibration();
  bool calibrationActive() const;
  Array calibrationArray() const;

 private:
  struct Storage {
    uint32_t magic;
    uint8_t validFlags;
    uint8_t reserved;
    uint16_t frontMin[kSensorCount];
    uint16_t frontMax[kSensorCount];
    uint16_t rearMin[kSensorCount];
    uint16_t rearMax[kSensorCount];
    uint16_t checksum;
  };

  // This separate record deliberately begins after the legacy CAL1 record.
  // It leaves existing Front/Rear calibration bytes untouched and can be
  // ignored safely by older library versions.
  struct UnderbodyStorage {
    uint32_t magic;
    uint8_t validFlags;
    uint8_t reserved;
    int16_t adcLMinimum;
    int16_t adcLMaximum;
    int16_t adcRMinimum;
    int16_t adcRMaximum;
    uint16_t checksum;
  };

  static constexpr uint32_t kMagic = 0x43414C31UL;  // "CAL1"
  static constexpr uint8_t kFrontValid = 0x01;
  static constexpr uint8_t kRearValid = 0x02;
  static constexpr uint32_t kUnderbodyMagic = 0x55424C31UL;  // "UBL1"
  static constexpr uint8_t kUnderbodyValid = 0x01;
  static constexpr uint16_t kUnderbodyStorageAddress = sizeof(Storage);
  static constexpr uint8_t kMcp3421Config = 0x90;  // continuous, 12-bit, x1

  const MuxPins frontMux_;
  const MuxPins rearMux_;
  const Settings settings_;
  TwoWire *wire_ = nullptr;
  Storage storage_{};
  UnderbodyStorage underbodyStorage_{};
  uint16_t frontCurrent_[kSensorCount]{};
  uint16_t rearCurrent_[kSensorCount]{};
  bool calibrationActive_ = false;
  Array calibrationArray_ = Array::Front;
  uint32_t calibrationStartedAt_ = 0;
  bool frontButtonWasPressed_ = false;
  bool rearButtonWasPressed_ = false;
  uint32_t lastRearButtonPollAt_ = 0;
  bool underbodyCalibrationActive_ = false;
  uint32_t underbodyCalibrationStartedAt_ = 0;
  uint32_t lastUnderbodyCalibrationProgressBeepAt_ = 0;
  uint8_t buzzerPin_ = 255;
  bool buzzerActive_ = false;
  uint32_t buzzerStopsAt_ = 0;
  bool startupMelodyActive_ = false;
  uint8_t startupMelodyStep_ = 0;
  uint32_t startupMelodyNextAt_ = 0;
  bool calibrationSecondBeepPending_ = false;
  uint32_t calibrationSecondBeepAt_ = 0;
  uint32_t lastCalibrationProgressBeepAt_ = 0;
  uint8_t startButtonPin_ = 255;
  bool startButtonWasPressed_ = false;
  bool robotStarted_ = false;
  bool waitStartButtonInitialized_ = false;
  bool waitStartButtonRawPressed_ = false;
  bool waitStartButtonStablePressed_ = false;
  bool waitStartButtonLongPressStarted_ = false;
  uint32_t waitStartButtonChangedAt_ = 0;
  uint32_t waitStartButtonPressedAt_ = 0;
  uint32_t lastWaitingMonitorPrintAt_ = 0;
  MotorPins leftMotorPins_ = {255, 255, 255};
  MotorPins rightMotorPins_ = {255, 255, 255};
  uint16_t pwmOutputMaximum_ = 255;
  uint8_t ads1115Address_ = 0x48;
  float batteryDividerRatio_ = 4.0f;
  float batteryCalibrationGain_ = 0.9638554f;
  float batteryCalibrationOffset_ = 0.4963855f;
  bool motorVoltageCompensationEnabled_ = false;
  float motorVoltageReference_ = 12.4f;
  float filteredMotorVoltage_ = 0.0f;
  bool motorVoltageSampleValid_ = false;
  float motorVoltageCompensationGain_ = 1.0f;
  float motorVoltageMaximumGain_ = 1.05f;
  uint32_t motorVoltageSampleIntervalMs_ = 500;
  uint32_t lastMotorVoltageSampleAt_ = 0;
  uint32_t lastMotorVoltagePollAt_ = 0;
  bool motorVoltageConversionPending_ = false;
  ForwardLineFollowerSettings forwardLineFollowerSettings_{};
  bool forwardLineFollowerEnabled_ = false;
  bool forwardLineDetected_ = false;
  bool forwardLinePreviousErrorValid_ = false;
  float forwardLineError_ = 0.0f;
  float forwardLinePreviousError_ = 0.0f;
  float forwardLineIntegral_ = 0.0f;
  uint32_t lastForwardLineControlAt_ = 0;
  int forwardLineLeftBaseSpeed_ = 35;
  int forwardLineRightBaseSpeed_ = 35;
  uint8_t batteryLedPcf8574Address_ = 0;
  bool batteryLedsActiveLow_ = false;
  bool batteryLevelLedsEnabled_ = false;
  uint32_t lastBatteryLevelLedsUpdateAt_ = 0;
  uint32_t lastLowBatteryWarningAt_ = 0;
  bool lowBatteryWarningActive_ = false;
  bool lowBatteryWarningIsCritical_ = false;

  void driveMotor(const MotorPins &pins, int speed);
  void resetForwardLineFollowerState();
  bool startAds1115SingleShot(uint8_t channel);
  void applyMotorVoltageSample(float batteryVoltage);

  void setupMux(const MuxPins &mux);
  void selectChannel(const MuxPins &mux, uint8_t channel);
  uint16_t readChannel(const MuxPins &mux, uint8_t channel);
  void scanMux(const MuxPins &mux, uint16_t readings[kSensorCount]);

  uint16_t *minimum(Array array);
  uint16_t *maximum(Array array);
  const uint16_t *minimum(Array array) const;
  const uint16_t *maximum(Array array) const;
  const uint16_t *current(Array array) const;

  bool frontButtonPressedEdge();
  bool rearButtonPressedEdge();
  int32_t readMcp3421();
  bool writeAds1115Register(uint8_t reg, uint16_t value);
  bool readAds1115Register(uint8_t reg, uint16_t &value);
  bool writeBatteryLevelLeds(uint8_t litLedCount);
  void configureMcp3421();
  void updateCalibration();
  void playCalibrationFinishedBeep();
  void resetStorage();
  void resetUnderbodyStorage();
  uint16_t underbodyChecksum() const;
  bool loadUnderbodyCalibration();
  bool saveUnderbodyCalibration();
  uint16_t normalizeUnderbody(int16_t value, int16_t minimum,
                              int16_t maximum) const;

  uint16_t checksum() const;
  bool eepromWriteByte(uint16_t address, uint8_t value);
  bool eepromReadByte(uint16_t address, uint8_t &value);
  bool eepromWriteBlock(uint16_t address, const uint8_t *data, size_t size);
  bool eepromReadBlock(uint16_t address, uint8_t *data, size_t size);
};

// Ready-to-use configuration for the user's Pico 2 robot wiring.
// It keeps MyMiniPro available above for other Arduino boards and
// custom pin maps, while making the Pico 2 example very small.
class Pico2MyMiniPro : public MyMiniPro {
 public:
  static constexpr uint8_t kServoCount = 5;

  Pico2MyMiniPro();
  // Every boot begins with an immediate single reset beep. The default follows
  // it with the short startup melody; pass false to suppress only that melody.
  // playStartupMelody() also remains available for manual use later.
  void begin();
  void begin(bool playStartupTone);

  // Servo GPIOs are fixed at 18, 22, 28, 0, and 1. They are deliberately not
  // attached during begin(), so no servo is driven until servo() is explicitly
  // called. Degrees outside 0..180 are safely clamped. These methods return
  // false for an unconfigured GPIO (or when pulse limits are changed after
  // that servo has been attached).
  bool servo(uint8_t gpio, int degrees);
  bool setServoPulseLimits(uint8_t gpio, uint16_t minimumPulseUs,
                           uint16_t maximumPulseUs);
  bool detachServo(uint8_t gpio);
  bool servoAttached(uint8_t gpio) const;
  int servoAngle(uint8_t gpio) const;

 private:
  static bool servoGpioIndex(uint8_t gpio, uint8_t &index);

  Servo servos_[kServoCount];
  uint16_t servoMinimumPulseUs_[kServoCount] = {1000, 1000, 1000, 1000, 1000};
  uint16_t servoMaximumPulseUs_[kServoCount] = {2000, 2000, 2000, 2000, 2000};
  bool servoIsAttached_[kServoCount] = {false, false, false, false, false};
  int16_t servoAngles_[kServoCount] = {-1, -1, -1, -1, -1};
};
