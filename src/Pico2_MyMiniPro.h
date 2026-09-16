#pragma once

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "MyMiniProForward.h"

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
  // On RP2040/RP2350, these percent commands map to 12-bit PWM duty values
  // 0..4095; Arduino-Pico can rescale the hardware output if a custom system
  // clock cannot sustain that range at the configured frequency. Parameter
  // order is Motor(left, right).
  void beginMotors(const MotorPins &leftMotor, const MotorPins &rightMotor,
                   uint32_t pwmFrequency = 20000);
  void Motor(int left, int right);
  int leftMotorCommand() const { return lastLeftMotorCommand_; }
  int rightMotorCommand() const { return lastRightMotorCommand_; }
  void stopMotors();

  // Blocking forward PD following. Numeric exits are centimeters; typed sensor
  // exits require clear -> line. Every return stops both motors.
  using ForwardResult = MyMiniProForward::Result;
  using ForwardSettings = MyMiniProForward::Settings;
  using LineSensor = MyMiniProForward::Sensor;
  void setForwardSettings(const ForwardSettings &settings) { forwardSettings_ = settings; }
  // F_Line D gain: percent-command seconds per [-50,+50] position unit. Default0.001.
  // Accepts every finite signed float; NaN/Inf return false and keep prior value.
  bool setFLineKd(float kd);
  float fLineKd() const { return fLineKd_; }
  // Numeric F_Line approach/crossing speed after distance, before turning.
  // 1..100 per wheel; (0,0) restores the call's speeds (default). Invalid keeps prior values.
  bool setFLineApproachSpeed(int leftPercent, int rightPercent);
  bool setFLineApproachSpeed(int percent) { return setFLineApproachSpeed(percent, percent); }
  // Signed wheel percentages of turnSpeed, clamped -100..100. Defaults FR(100,0), FL(0,100).
  void setFLineFRMotors(int leftPercent, int rightPercent);
  void setFLineFLMotors(int leftPercent, int rightPercent);
  // Retired compatibility API: only zero is accepted; nonzero returns false.
  // No motion effect. End-pair crossing replaces distance/minimum-time guards.
  bool setFLineTurnClearanceCm(float centimeters);
  float fLineTurnClearanceCm() const { return 0; }
  bool setFLineMinTurnMs(uint32_t milliseconds);
  uint32_t fLineMinTurnMs() const { return 0; }
  // Snap only when signed KP exceeds this threshold. Any finite float; default0.4.
  bool setFLineRecoveryKpThreshold(float threshold);
  float getFLineRecoveryKpThreshold() const { return fLineRecoveryKpThreshold_; }
  // Line confirmation only:0(default)=first qualifying scan, finite0..60000ms.
  // Clear-to-arm still uses ForwardSettings::debounceMs(default20ms).
  bool setFLineSensorDebounceMs(uint32_t milliseconds);
  uint16_t fLineSensorDebounceMs() const { return fLineSensorDebounceMs_; }
  // Last selected sensor's fresh calibrated line strength,0..1000 (high=line).
  // UINT16_MAX means no valid selected sample for this call.
  uint16_t lastFLineSensorStrength() const { return lastFLineSensorStrength_; }
  ForwardResult F_Line(int leftSpeed, int rightSpeed, float kp, float centimeters);
  ForwardResult F_Line(int leftSpeed, int rightSpeed, float kp, LineSensor sensor);
  using LineAction = MyMiniProForward::ActionChoice;
  // Action forms:6 args STOP/sensor CROSS;8 args turn;7 args numeric CROSS+sensor.
  // Only successful CROSS with brake0 retains motor output at return.
  ForwardResult F_Line(int left, int right, float kp, float cm, LineAction action, int brake);
  ForwardResult F_Line(int left, int right, float kp, LineSensor exit, LineAction action, int brake);
  ForwardResult F_Line(int left, int right, float kp, float cm, LineAction action,
                       int turnSpeed, LineSensor turnSensor, int brake);
  ForwardResult F_Line(int left, int right, float kp, LineSensor exit, LineAction action,
                       int turnSpeed, LineSensor turnSensor, int brake);
  ForwardResult F_Line(int left, int right, float kp, float cm, LineAction action,
                       LineSensor crossSensor, int brake);

  // Reverse following uses positive speed magnitudes and its own rear settings.
  using BackwardSettings = MyMiniProForward::Settings;
  using BackwardResult = MyMiniProForward::Result;
  void setBackwardSettings(const BackwardSettings &settings) { backwardSettings_=settings; }
  bool setBLineKd(float kd);
  float bLineKd() const { return bLineKd_; }
  bool setBLineRecoveryKpThreshold(float threshold);
  float getBLineRecoveryKpThreshold() const { return bLineRecoveryKpThreshold_; }
  bool setBLineSensorDebounceMs(uint32_t milliseconds);
  uint16_t bLineSensorDebounceMs() const { return bLineSensorDebounceMs_; }
  uint16_t lastBLineSensorStrength() const { return lastBLineSensorStrength_; }
  BackwardResult B_Line(int left,int right,float kp,float centimeters);
  BackwardResult B_Line(int left,int right,float kp,LineSensor sensor);
  BackwardResult B_Line(int left,int right,float kp,float cm,LineAction action,int brake);
  BackwardResult B_Line(int left,int right,float kp,LineSensor exit,LineAction action,int brake);
  BackwardResult B_Line(int left,int right,float kp,float cm,LineAction action,
                        int turnSpeed,LineSensor turnSensor,int brake);
  BackwardResult B_Line(int left,int right,float kp,LineSensor exit,LineAction action,
                        int turnSpeed,LineSensor turnSensor,int brake);
  BackwardResult B_Line(int left,int right,float kp,float cm,LineAction action,
                        LineSensor crossSensor,int brake);

  // Samples a filtered, cached battery voltage for optional motor-voltage
  // compensation. Call serviceMotorVoltageCompensation() from loop(); it
  // rate-limits ADS1115 sampling. Motor() uses the supplied commands directly.
  // Invalid readings and USB
  // readings below 6.5 V use a gain of 1.0.
  // maximumCompensationPercent is limited to 100; 5 allows at most 5% extra PWM.
  void setMotorVoltageCompensation(bool enabled, float referenceVoltage = 12.4f,
                                   uint32_t sampleIntervalMs = 500,
                                   uint8_t maximumCompensationPercent = 5);
  void serviceMotorVoltageCompensation();
  float motorVoltageCompensationGain() const;

  // Apply the cached voltage gain to a motor command.
  // The signed result is limited to +/-100.
  int compensateBaseMotorSpeed(int baseSpeed) const;

  // Call once at boot. False means calibration has not been saved yet.
  bool loadCalibration();
  bool saveCalibration();
  bool hasCalibration(Array array) const;

  // Scan only the requested 16-sensor array. This is the normal fast path.
  void scan(Direction direction);
  void scanFront();
  void scanRear();

  // Smooth each sensor with an exponential moving average without adding
  // extra ADC reads. 100 means raw/unfiltered; 40 is a fast, stable default.
  // Smaller percentages are smoother but react more slowly to a line edge.
  void setSensorSmoothing(uint8_t newReadingPercent = 40);
  // Apply between scans. Extrema are retained, but their accuracy at a new
  // settling time must be checked against the actual surfaces. Not persisted.
  bool setMuxSettleMicroseconds(uint16_t microseconds);
  uint16_t muxSettleMicroseconds() const { return settings_.muxSettleMicroseconds; }

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
  // calibration button has just started it. While true, keep motors stopped.
  bool serviceCalibration();
  bool calibrationActive() const;
  Array calibrationArray() const;

 private:
  ForwardSettings forwardSettings_;
  BackwardSettings backwardSettings_;
  float bLineKd_=0.05f;
  float bLineRecoveryKpThreshold_=4.0f;
  uint16_t bLineSensorDebounceMs_=0;
  uint16_t lastBLineSensorStrength_=UINT16_MAX;
  float fLineKd_ = 0.001f;
  int fLineApproachLeft_=0, fLineApproachRight_=0;
  int fLineFRLeftPercent_=100, fLineFRRightPercent_=0;
  int fLineFLLeftPercent_=0, fLineFLRightPercent_=100;
  float fLineRecoveryKpThreshold_ = 0.4f;
  uint16_t fLineSensorDebounceMs_ = 0;
  uint16_t lastFLineSensorStrength_ = UINT16_MAX;
  struct ForwardMotion {
    MyMiniProForward::Action action;
    int turnSpeed, brake;
    LineSensor selected;
    enum class Form { Finish, Turn, Cross } form;
  };
  ForwardResult runForward(int left, int right, float kp, bool sensorExit,
                           float centimeters, LineSensor sensor, const ForwardMotion *motion=nullptr,
                           bool reverse=false);
  bool forwardSensorReady(LineSensor sensor) const;
  uint16_t forwardSensorStrength(LineSensor sensor, const ForwardSettings &cfg, uint16_t &lastStrength);
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
  Settings settings_;
  TwoWire *wire_ = nullptr;
  Storage storage_{};
  UnderbodyStorage underbodyStorage_{};
  uint16_t frontCurrent_[kSensorCount]{};
  uint16_t rearCurrent_[kSensorCount]{};
  uint8_t sensorSmoothingPercent_ = 40;
  bool frontSensorFilterInitialized_ = false;
  bool rearSensorFilterInitialized_ = false;
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
  int lastLeftMotorCommand_ = 0;
  int lastRightMotorCommand_ = 0;
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
  uint8_t batteryLedPcf8574Address_ = 0;
  bool batteryLedsActiveLow_ = false;
  bool batteryLevelLedsEnabled_ = false;
  bool waitBatteryLedsInitAttempted_ = false;
  uint32_t lastBatteryLevelLedsUpdateAt_ = 0;
  uint32_t lastLowBatteryWarningAt_ = 0;
  bool lowBatteryWarningActive_ = false;
  bool lowBatteryWarningIsCritical_ = false;

  void driveMotor(const MotorPins &pins, int speed);
  bool startAds1115SingleShot(uint8_t channel);
  void applyMotorVoltageSample(float batteryVoltage);

  void setupMux(const MuxPins &mux);
  void selectChannel(const MuxPins &mux, uint8_t channel);
  uint16_t readChannel(const MuxPins &mux, uint8_t channel);
  void scanMux(const MuxPins &mux, uint16_t readings[kSensorCount],
               bool &filterInitialized, uint8_t captureChannel = 255,
               uint16_t *capturedRaw = nullptr, uint8_t secondCaptureChannel = 255,
               uint16_t *secondCapturedRaw = nullptr);

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




// Short sensor names retain the scoped enum type and select the sensor overload.
constexpr MyMiniProForward::Sensor F0 = MyMiniProForward::Sensor::F0;
constexpr MyMiniProForward::Sensor F1 = MyMiniProForward::Sensor::F1;
constexpr MyMiniProForward::Sensor F2 = MyMiniProForward::Sensor::F2;
constexpr MyMiniProForward::Sensor F3 = MyMiniProForward::Sensor::F3;
constexpr MyMiniProForward::Sensor F4 = MyMiniProForward::Sensor::F4;
constexpr MyMiniProForward::Sensor F5 = MyMiniProForward::Sensor::F5;
constexpr MyMiniProForward::Sensor F6 = MyMiniProForward::Sensor::F6;
constexpr MyMiniProForward::Sensor F7 = MyMiniProForward::Sensor::F7;
constexpr MyMiniProForward::Sensor F8 = MyMiniProForward::Sensor::F8;
constexpr MyMiniProForward::Sensor F9 = MyMiniProForward::Sensor::F9;
constexpr MyMiniProForward::Sensor F10 = MyMiniProForward::Sensor::F10;
constexpr MyMiniProForward::Sensor F11 = MyMiniProForward::Sensor::F11;
constexpr MyMiniProForward::Sensor F12 = MyMiniProForward::Sensor::F12;
constexpr MyMiniProForward::Sensor F13 = MyMiniProForward::Sensor::F13;
constexpr MyMiniProForward::Sensor F14 = MyMiniProForward::Sensor::F14;
constexpr MyMiniProForward::Sensor F15 = MyMiniProForward::Sensor::F15;
constexpr MyMiniProForward::Sensor CL = MyMiniProForward::Sensor::CL;
constexpr MyMiniProForward::Sensor CR = MyMiniProForward::Sensor::CR;
constexpr MyMiniProForward::Sensor MyMiniPro_B0 = MyMiniProForward::Sensor::B0;
constexpr MyMiniProForward::Sensor MyMiniPro_B1 = MyMiniProForward::Sensor::B1;
constexpr MyMiniProForward::Sensor B2 = MyMiniProForward::Sensor::B2;
constexpr MyMiniProForward::Sensor B3 = MyMiniProForward::Sensor::B3;
constexpr MyMiniProForward::Sensor B4 = MyMiniProForward::Sensor::B4;
constexpr MyMiniProForward::Sensor B5 = MyMiniProForward::Sensor::B5;
constexpr MyMiniProForward::Sensor B6 = MyMiniProForward::Sensor::B6;
constexpr MyMiniProForward::Sensor B7 = MyMiniProForward::Sensor::B7;
constexpr MyMiniProForward::Sensor B8 = MyMiniProForward::Sensor::B8;
constexpr MyMiniProForward::Sensor B9 = MyMiniProForward::Sensor::B9;
constexpr MyMiniProForward::Sensor MyMiniPro_B10 = MyMiniProForward::Sensor::B10;
constexpr MyMiniProForward::Sensor MyMiniPro_B11 = MyMiniProForward::Sensor::B11;
constexpr MyMiniProForward::Sensor B12 = MyMiniProForward::Sensor::B12;
constexpr MyMiniProForward::Sensor B13 = MyMiniProForward::Sensor::B13;
constexpr MyMiniProForward::Sensor B14 = MyMiniProForward::Sensor::B14;
constexpr MyMiniProForward::Sensor B15 = MyMiniProForward::Sensor::B15;
// ArduinoCore-API declares these four names as global binary enum values.
// Token aliases are necessary; both bare B0 and Sensor::B0 remain sensor-typed.
#define B0 MyMiniPro_B0
#define B1 MyMiniPro_B1
#define B10 MyMiniPro_B10
#define B11 MyMiniPro_B11

constexpr MyMiniProForward::Action FL = MyMiniProForward::Action::FL;
constexpr MyMiniProForward::Action FR = MyMiniProForward::Action::FR;
constexpr MyMiniProForward::Action NL = MyMiniProForward::Action::NL;
constexpr MyMiniProForward::Action NR = MyMiniProForward::Action::NR;
constexpr MyMiniProForward::Action STOP = MyMiniProForward::Action::STOP;
constexpr MyMiniProForward::Action CROSS = MyMiniProForward::Action::CROSS;
constexpr MyMiniProForward::Action NS = MyMiniProForward::Action::NS;
constexpr MyMiniProForward::Action CS = MyMiniProForward::Action::CS;
constexpr MyMiniProForward::Action CP = MyMiniProForward::Action::CP;
constexpr MyMiniProForward::Action FS = MyMiniProForward::Action::FS;
constexpr MyMiniProForward::Action FP = MyMiniProForward::Action::FP;


// Lowercase F_Line arguments. Keep enum types so sensor/distance overloads stay distinct.
constexpr MyMiniProForward::Sensor f0 = MyMiniProForward::Sensor::f0;
constexpr MyMiniProForward::Sensor f1 = MyMiniProForward::Sensor::f1;
constexpr MyMiniProForward::Sensor f2 = MyMiniProForward::Sensor::f2;
constexpr MyMiniProForward::Sensor f3 = MyMiniProForward::Sensor::f3;
constexpr MyMiniProForward::Sensor f4 = MyMiniProForward::Sensor::f4;
constexpr MyMiniProForward::Sensor f5 = MyMiniProForward::Sensor::f5;
constexpr MyMiniProForward::Sensor f6 = MyMiniProForward::Sensor::f6;
constexpr MyMiniProForward::Sensor f7 = MyMiniProForward::Sensor::f7;
constexpr MyMiniProForward::Sensor f8 = MyMiniProForward::Sensor::f8;
constexpr MyMiniProForward::Sensor f9 = MyMiniProForward::Sensor::f9;
constexpr MyMiniProForward::Sensor f10 = MyMiniProForward::Sensor::f10;
constexpr MyMiniProForward::Sensor f11 = MyMiniProForward::Sensor::f11;
constexpr MyMiniProForward::Sensor f12 = MyMiniProForward::Sensor::f12;
constexpr MyMiniProForward::Sensor f13 = MyMiniProForward::Sensor::f13;
constexpr MyMiniProForward::Sensor f14 = MyMiniProForward::Sensor::f14;
constexpr MyMiniProForward::Sensor f15 = MyMiniProForward::Sensor::f15;
constexpr MyMiniProForward::Sensor cl = MyMiniProForward::Sensor::cl;
constexpr MyMiniProForward::Sensor cr = MyMiniProForward::Sensor::cr;
constexpr MyMiniProForward::Sensor b0 = MyMiniProForward::Sensor::b0;
constexpr MyMiniProForward::Sensor b1 = MyMiniProForward::Sensor::b1;
constexpr MyMiniProForward::Sensor b2 = MyMiniProForward::Sensor::b2;
constexpr MyMiniProForward::Sensor b3 = MyMiniProForward::Sensor::b3;
constexpr MyMiniProForward::Sensor b4 = MyMiniProForward::Sensor::b4;
constexpr MyMiniProForward::Sensor b5 = MyMiniProForward::Sensor::b5;
constexpr MyMiniProForward::Sensor b6 = MyMiniProForward::Sensor::b6;
constexpr MyMiniProForward::Sensor b7 = MyMiniProForward::Sensor::b7;
constexpr MyMiniProForward::Sensor b8 = MyMiniProForward::Sensor::b8;
constexpr MyMiniProForward::Sensor b9 = MyMiniProForward::Sensor::b9;
constexpr MyMiniProForward::Sensor b10 = MyMiniProForward::Sensor::b10;
constexpr MyMiniProForward::Sensor b11 = MyMiniProForward::Sensor::b11;
constexpr MyMiniProForward::Sensor b12 = MyMiniProForward::Sensor::b12;
constexpr MyMiniProForward::Sensor b13 = MyMiniProForward::Sensor::b13;
constexpr MyMiniProForward::Sensor b14 = MyMiniProForward::Sensor::b14;
constexpr MyMiniProForward::Sensor b15 = MyMiniProForward::Sensor::b15;
constexpr MyMiniProForward::Action fl = MyMiniProForward::Action::fl;
constexpr MyMiniProForward::Action fr = MyMiniProForward::Action::fr;
constexpr MyMiniProForward::Action nl = MyMiniProForward::Action::nl;
constexpr MyMiniProForward::Action nr = MyMiniProForward::Action::nr;
constexpr MyMiniProForward::Action stop = MyMiniProForward::Action::stop;
constexpr MyMiniProForward::Action cross = MyMiniProForward::Action::cross;
constexpr MyMiniProForward::Action ns = MyMiniProForward::Action::ns;
constexpr MyMiniProForward::Action cs = MyMiniProForward::Action::cs;
constexpr MyMiniProForward::Action cp = MyMiniProForward::Action::cp;
constexpr MyMiniProForward::Action fs = MyMiniProForward::Action::fs;
constexpr MyMiniProForward::Action fp = MyMiniProForward::Action::fp;
// cl/cr are Sensor aliases, like CL/CR; ActionChoice also accepts them as turn actions.
