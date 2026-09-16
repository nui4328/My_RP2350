#include "Pico2_MyMiniPro.h"

namespace {
uint16_t normalizeReading(uint16_t reading, uint16_t minimum, uint16_t maximum) {
  if (maximum<=minimum) return 0;
  const long value=map(reading,minimum,maximum,0,1000);
  return static_cast<uint16_t>(constrain(value,0L,1000L));
}
// Short bright major-key startup melody. It begins with the melody itself;
// there is no separate long cue beep before it.
constexpr uint16_t kStartupNotes[] = {1568, 2093, 2637, 3136};
constexpr uint16_t kStartupDurations[] = {70, 70, 90, 140};
constexpr uint8_t kStartupNoteCount = sizeof(kStartupNotes) / sizeof(kStartupNotes[0]);
constexpr uint16_t kCalibrationProgressFrequency = 1800;
// Longer shared progress beeps improve audibility without blocking sampling.
constexpr uint16_t kCalibrationProgressDurationMs = 100;
constexpr uint16_t kCalibrationProgressIntervalMs = 350;
constexpr uint16_t kCalibrationFinishedFrequency = 3100;
constexpr uint16_t kCalibrationFinishedDurationMs = 150;
constexpr uint16_t kStartConfirmationFrequency = 3100;
constexpr uint16_t kStartConfirmationDurationMs = 300;
constexpr uint16_t kResetAcknowledgementFrequency = 3100;
constexpr uint16_t kResetAcknowledgementDurationMs = 60;
constexpr uint32_t kBatteryLevelLedsUpdateIntervalMs = 500;
constexpr uint16_t kLowBatteryWarningFrequency = 1200;
constexpr uint16_t kLowBatteryWarningDurationMs = 200;
constexpr uint16_t kCriticalBatteryWarningDurationMs = 700;
constexpr uint32_t kLowBatteryWarningIntervalMs = 1500;
constexpr uint16_t kMotorPwmOutputMaximum = 4095;

const MyMiniPro::MuxPins kPico2FrontMux = {17, 16, 15, 14, 27};
const MyMiniPro::MuxPins kPico2RearMux = {10, 11, 12, 13, 26};
const MyMiniPro::Settings kPico2Settings = {
    3, 0x50, 0x68, 1000, 4095, 12, 5000, true};
const MyMiniPro::MotorPins kPico2LeftMotor = {19, 20, 21};
const MyMiniPro::MotorPins kPico2RightMotor = {6, 7, 8};
constexpr uint8_t kPico2ServoPins[Pico2MyMiniPro::kServoCount] = {
    18, 22, 28, 0, 1};
constexpr float kAds1115LsbVolts = 0.000125f;  // ±4.096 V range
}

Pico2MyMiniPro::Pico2MyMiniPro()
    : MyMiniPro(kPico2FrontMux, kPico2RearMux, kPico2Settings) {}

void Pico2MyMiniPro::begin() { begin(true); }

void Pico2MyMiniPro::begin(bool playStartupTone) {
  // Make the reset acknowledgement the first audible action. Configuring GP9
  // before I2C, sensors, and motors lets the user hear the reset immediately.
  beginBuzzer(9);
  beep(kResetAcknowledgementFrequency, kResetAcknowledgementDurationMs);
  delay(kResetAcknowledgementDurationMs);
  stopBuzzer();
  if (playStartupTone) playStartupMelody();

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
  analogReadResolution(12);
  Wire.setSDA(4);
  Wire.setSCL(5);
#endif
  Wire.begin();
  Wire.setClock(400000);
  analogReadResolution(12);  // อ่าน ADC: 0–4095

  MyMiniPro::begin(Wire);
  beginStartButton(2);
  beginMotors(kPico2LeftMotor, kPico2RightMotor, 20000);
  setMotorVoltageCompensation(true, 12.4f, 500);
  loadCalibration();
}

bool Pico2MyMiniPro::servoGpioIndex(uint8_t gpio, uint8_t &index) {
  for (uint8_t candidate = 0; candidate < kServoCount; ++candidate) {
    if (kPico2ServoPins[candidate] == gpio) {
      index = candidate;
      return true;
    }
  }
  return false;
}

bool Pico2MyMiniPro::servo(uint8_t gpio, int degrees) {
  uint8_t index = 0;
  if (!servoGpioIndex(gpio, index)) return false;

  const int boundedDegrees = degrees < 0 ? 0 : (degrees > 180 ? 180 : degrees);
  if (!servoIsAttached_[index]) {
    // Servo is deliberately attached only on its first explicit command. This
    // keeps all five signal pins inactive during boot and robot setup.
    const int attachResult =
        servos_[index].attach(kPico2ServoPins[index], servoMinimumPulseUs_[index],
                              servoMaximumPulseUs_[index]);
    if (attachResult < 0 || !servos_[index].attached()) return false;
    servoIsAttached_[index] = true;
  }

  servos_[index].write(boundedDegrees);
  servoAngles_[index] = boundedDegrees;
  return true;
}

bool Pico2MyMiniPro::setServoPulseLimits(uint8_t gpio,
                                          uint16_t minimumPulseUs,
                                          uint16_t maximumPulseUs) {
  uint8_t index = 0;
  if (!servoGpioIndex(gpio, index) || minimumPulseUs >= maximumPulseUs ||
      servoIsAttached_[index]) {
    return false;
  }

  servoMinimumPulseUs_[index] = minimumPulseUs;
  servoMaximumPulseUs_[index] = maximumPulseUs;
  return true;
}

bool Pico2MyMiniPro::detachServo(uint8_t gpio) {
  uint8_t index = 0;
  if (!servoGpioIndex(gpio, index)) return false;

  if (servoIsAttached_[index]) {
    servos_[index].detach();
    servoIsAttached_[index] = false;
  }
  servoAngles_[index] = -1;
  return true;
}

bool Pico2MyMiniPro::servoAttached(uint8_t gpio) const {
  uint8_t index = 0;
  return servoGpioIndex(gpio, index) && servoIsAttached_[index];
}

int Pico2MyMiniPro::servoAngle(uint8_t gpio) const {
  uint8_t index = 0;
  if (!servoGpioIndex(gpio, index)) return -1;
  return servoAngles_[index];
}

MyMiniPro::MyMiniPro(const MuxPins &frontMux,
                                     const MuxPins &rearMux,
                                     const Settings &settings)
    : frontMux_(frontMux), rearMux_(rearMux), settings_(settings) {}

void MyMiniPro::begin(TwoWire &wire) {
  wire_ = &wire;
  setupMux(frontMux_);
  setupMux(rearMux_);

  if (settings_.frontCalibrationPin != 255) {
    pinMode(settings_.frontCalibrationPin, INPUT_PULLUP);
  }

  selectChannel(frontMux_, 0);
  selectChannel(rearMux_, 0);

  if (settings_.rearButtonEnabled) {
    configureMcp3421();
  }
}

void MyMiniPro::beginBuzzer(uint8_t pin) {
  stopBuzzer();
  buzzerPin_ = pin;
  pinMode(buzzerPin_, OUTPUT);
  digitalWrite(buzzerPin_, LOW);
}

void MyMiniPro::playStartupMelody() {
  if (buzzerPin_ == 255) return;

  startupMelodyActive_ = true;
  startupMelodyStep_ = 0;
  buzzerStopsAt_ = 0;
  tone(buzzerPin_, kStartupNotes[startupMelodyStep_]);
  buzzerActive_ = true;
  startupMelodyNextAt_ = millis() + kStartupDurations[startupMelodyStep_];
}

void MyMiniPro::playTone(uint16_t frequency) {
  if (buzzerPin_ == 255 || frequency == 0) return;
  startupMelodyActive_ = false;
  tone(buzzerPin_, frequency);
  buzzerActive_ = true;
  buzzerStopsAt_ = 0;  // Zero means play until stopBuzzer() is called.
}

void MyMiniPro::beep(uint16_t frequency, uint16_t durationMs) {
  if (buzzerPin_ == 255 || frequency == 0 || durationMs == 0) return;
  startupMelodyActive_ = false;
  // Supplying duration makes the Arduino core stop the passive buzzer even
  // when user code has already left waitButton() and entered loop().
  tone(buzzerPin_, frequency, durationMs);
  buzzerActive_ = true;
  buzzerStopsAt_ = millis() + durationMs;
}

void MyMiniPro::stopBuzzer() {
  if (buzzerPin_ != 255) {
    noTone(buzzerPin_);
    digitalWrite(buzzerPin_, LOW);
  }
  buzzerActive_ = false;
  buzzerStopsAt_ = 0;
  startupMelodyActive_ = false;
  startupMelodyStep_ = 0;
  startupMelodyNextAt_ = 0;
}

void MyMiniPro::updateBuzzer() {
  if (startupMelodyActive_) {
    if (static_cast<int32_t>(millis() - startupMelodyNextAt_) < 0) return;

    ++startupMelodyStep_;
    if (startupMelodyStep_ >= kStartupNoteCount) {
      stopBuzzer();
      return;
    }

    tone(buzzerPin_, kStartupNotes[startupMelodyStep_]);
    startupMelodyNextAt_ = millis() + kStartupDurations[startupMelodyStep_];
    return;
  }

  if (calibrationSecondBeepPending_ &&
      static_cast<int32_t>(millis() - calibrationSecondBeepAt_) >= 0) {
    calibrationSecondBeepPending_ = false;
    tone(buzzerPin_, kCalibrationFinishedFrequency);
    buzzerActive_ = true;
    buzzerStopsAt_ = millis() + kCalibrationFinishedDurationMs;
  }

  if (!buzzerActive_ || buzzerStopsAt_ == 0) return;
  if (static_cast<int32_t>(millis() - buzzerStopsAt_) >= 0) {
    stopBuzzer();
  }
}

bool MyMiniPro::buzzerActive() const { return buzzerActive_; }

void MyMiniPro::beginStartButton(uint8_t pin) {
  startButtonPin_ = pin;
  startButtonWasPressed_ = false;
  robotStarted_ = false;
  waitStartButtonInitialized_ = false;
  waitStartButtonRawPressed_ = false;
  waitStartButtonStablePressed_ = false;
  waitStartButtonLongPressStarted_ = false;
  waitStartButtonChangedAt_ = 0;
  waitStartButtonPressedAt_ = 0;
  pinMode(startButtonPin_, INPUT_PULLUP);
}

bool MyMiniPro::serviceStartButton() {
  updateBuzzer();
  if (startButtonPin_ == 255) return false;

  const bool pressed = digitalRead(startButtonPin_) == LOW;
  const bool pressedEdge = pressed && !startButtonWasPressed_;
  startButtonWasPressed_ = pressed;
  if (!pressedEdge) return false;

  beep(kStartConfirmationFrequency, kStartConfirmationDurationMs);
  // Start is only allowed after the confirmation tone finishes. This happens
  // in setup() before the motor-control loop, so it cannot slow the robot.
  delay(kStartConfirmationDurationMs);
  robotStarted_ = true;
  return true;
}

bool MyMiniPro::robotStarted() const { return robotStarted_; }

void MyMiniPro::stopRobot() {
  robotStarted_ = false;
}

bool MyMiniPro::waitButton(uint32_t monitorIntervalMs) {
  
  return waitButton(Serial, monitorIntervalMs);
}

void MyMiniPro::beginMotors(const MotorPins &leftMotor,
                             const MotorPins &rightMotor,
                             uint32_t pwmFrequency) {
  leftMotorPins_ = leftMotor;
  rightMotorPins_ = rightMotor;

  const MotorPins motors[] = {leftMotorPins_, rightMotorPins_};
  for (const MotorPins &motor : motors) {
    pinMode(motor.pwm, OUTPUT);
    pinMode(motor.in1, OUTPUT);
    pinMode(motor.in2, OUTPUT);
  }

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
  analogWriteFreq(pwmFrequency);
  // Keep Motor() in percent while requesting the Pico PWM's full 12-bit duty
  // range. Arduino-Pico reduces the hardware range only if the active system
  // clock cannot support this range at the requested frequency.
  // In Arduino-Pico, analogWriteResolution(12) configures a range of 4095.
  analogWriteResolution(12);
  pwmOutputMaximum_ = kMotorPwmOutputMaximum;
#else
  (void)pwmFrequency;
  pwmOutputMaximum_ = 255;
#endif

  stopMotors();
}

void MyMiniPro::setMotorVoltageCompensation(bool enabled,
                                             float referenceVoltage,
                                             uint32_t sampleIntervalMs,
                                             uint8_t maximumCompensationPercent) {
  motorVoltageCompensationEnabled_ = enabled;
  motorVoltageReference_ = referenceVoltage > 0.0f ? referenceVoltage : 12.4f;
  motorVoltageSampleIntervalMs_ = sampleIntervalMs < 500 ? 500 : sampleIntervalMs;
  const uint8_t limitedPercent = maximumCompensationPercent > 100
                                     ? 100
                                     : maximumCompensationPercent;
  const float allowedCompensation = limitedPercent / 100.0f;
  motorVoltageMaximumGain_ = 1.0f + allowedCompensation;
  motorVoltageSampleValid_ = false;
  motorVoltageCompensationGain_ = 1.0f;
  motorVoltageConversionPending_ = false;
  lastMotorVoltagePollAt_ = 0;
  // Service routine takes the first sample after one full interval, keeping
  // startup and the fast Motor() path free of ADS1115 conversion latency.
  lastMotorVoltageSampleAt_ = millis();
}

float MyMiniPro::motorVoltageCompensationGain() const {
  return motorVoltageCompensationGain_;
}

void MyMiniPro::serviceMotorVoltageCompensation() {
  if (!motorVoltageCompensationEnabled_) return;

  constexpr uint32_t kAds1115ConversionPollIntervalMs = 2;
  constexpr uint32_t kAds1115ConversionTimeoutMs = 20;

  if (motorVoltageConversionPending_) {
    if (millis() - lastMotorVoltageSampleAt_ >= kAds1115ConversionTimeoutMs) {
      motorVoltageConversionPending_ = false;
      lastMotorVoltageSampleAt_ = millis();
      applyMotorVoltageSample(NAN);
      return;
    }
    if (millis() - lastMotorVoltagePollAt_ < kAds1115ConversionPollIntervalMs) {
      return;
    }

    lastMotorVoltagePollAt_ = millis();
    uint16_t status = 0;
    if (!readAds1115Register(0x01, status)) {
      motorVoltageConversionPending_ = false;
      lastMotorVoltageSampleAt_ = millis();
      applyMotorVoltageSample(NAN);
      return;
    }
    if ((status & 0x8000U) == 0) return;

    uint16_t raw = 0;
    motorVoltageConversionPending_ = false;
    lastMotorVoltageSampleAt_ = millis();
    if (!readAds1115Register(0x00, raw)) {
      applyMotorVoltageSample(NAN);
      return;
    }

    const float adcVoltage = static_cast<int16_t>(raw) * kAds1115LsbVolts;
    const float batteryVoltage = adcVoltage * batteryDividerRatio_ *
                                 batteryCalibrationGain_ + batteryCalibrationOffset_;
    applyMotorVoltageSample(batteryVoltage);
    return;
  }

  if (millis() - lastMotorVoltageSampleAt_ < motorVoltageSampleIntervalMs_) return;

  lastMotorVoltageSampleAt_ = millis();
  if (!startAds1115SingleShot(0)) {
    applyMotorVoltageSample(NAN);
    return;
  }
  motorVoltageConversionPending_ = true;
  lastMotorVoltagePollAt_ = millis();
}

void MyMiniPro::applyMotorVoltageSample(float batteryVoltage) {
  if (isnan(batteryVoltage) || batteryVoltage < 6.5f) {
    motorVoltageSampleValid_ = false;
    filteredMotorVoltage_ = 0.0f;
    motorVoltageCompensationGain_ = 1.0f;
    return;
  }

  constexpr float kMotorVoltageFilterAlpha = 0.2f;
  if (!motorVoltageSampleValid_) {
    filteredMotorVoltage_ = batteryVoltage;
    motorVoltageSampleValid_ = true;
  } else {
    filteredMotorVoltage_ +=
        (batteryVoltage - filteredMotorVoltage_) * kMotorVoltageFilterAlpha;
  }

  const float requestedGain = motorVoltageReference_ / filteredMotorVoltage_;
  const float nonReducingGain = requestedGain > 1.0f ? requestedGain : 1.0f;
  motorVoltageCompensationGain_ = nonReducingGain < motorVoltageMaximumGain_
                                      ? nonReducingGain
                                      : motorVoltageMaximumGain_;
}

int MyMiniPro::compensateBaseMotorSpeed(int baseSpeed) const {
  const int speed = constrain(baseSpeed, -100, 100);
  const int scaledSpeed = static_cast<int>(speed * motorVoltageCompensationGain_);
  return constrain(scaledSpeed, -100, 100);
}

void MyMiniPro::driveMotor(const MotorPins &pins, int speed) {
  if (pins.pwm == 255) return;

  speed = constrain(speed, -100, 100);
  // The public Motor() command stays in percent; analogWrite() receives the
  // raw duty value for the configured PWM range (0..4095 on Pico).
  const uint16_t power = map(abs(speed), 0, 100, 0, pwmOutputMaximum_);

  if (speed > 0) {
    digitalWrite(pins.in1, HIGH);
    digitalWrite(pins.in2, LOW);
  } else if (speed < 0) {
    digitalWrite(pins.in1, LOW);
    digitalWrite(pins.in2, HIGH);
  } else {
    digitalWrite(pins.in1, LOW);
    digitalWrite(pins.in2, LOW);
  }

  analogWrite(pins.pwm, power);
}

void MyMiniPro::Motor(int left, int right) {
  lastLeftMotorCommand_ = constrain(left, -100, 100);
  lastRightMotorCommand_ = constrain(right, -100, 100);
  driveMotor(leftMotorPins_, lastLeftMotorCommand_);
  driveMotor(rightMotorPins_, lastRightMotorCommand_);
}

void MyMiniPro::stopMotors() { Motor(0, 0); }

bool MyMiniPro::waitButton(Stream &output,
                                   uint32_t monitorIntervalMs) {
  // waitButton is the safe pre-start state. Keep both motors stopped while
  // detecting a Start press, running underbody calibration, and waiting.
  if (!robotStarted_) stopMotors();
  // Let an explicitly requested boot melody finish before reading Start.
  if (startupMelodyActive_) {
    updateBuzzer();
    return false;
  }

  // Default board: ADS1115 battery input and active-high PCF8574 at 0x20.
  // Attempt once so a missing LED board cannot delay every button poll.
  // Respect an LED configuration already supplied by the sketch.
  if (!waitBatteryLedsInitAttempted_ && !underbodyCalibrationActive_) {
    waitBatteryLedsInitAttempted_ = true;
    if (!batteryLevelLedsEnabled_ && !beginBatteryLevelLeds(0x20, false)) {
      output.print(F("Battery LED bar unavailable (PCF8574 0x20)"));
      output.println();
    }
  }

  if (underbodyCalibrationActive_) {
    updateBuzzer();
    serviceUnderbodyCalibration();
    if (millis() - lastWaitingMonitorPrintAt_ >= monitorIntervalMs) {
      lastWaitingMonitorPrintAt_ = millis();
      printLiveReadings(output);
    }
    // A calibration completion is also a waiting-state pass: require a new
    // short Start press after the user releases the long press.
    return false;
  }

  // Keep the battery bar visible during boot/start-button wait without
  // repeatedly sampling ADS1115 on every pass through this fast loop.
  if (batteryLevelLedsEnabled_ &&
      millis() - lastBatteryLevelLedsUpdateAt_ >=
          kBatteryLevelLedsUpdateIntervalMs) {
    lastBatteryLevelLedsUpdateAt_ = millis();
    const float batteryVoltage = readBatteryVoltage();
    updateBatteryLevelLeds(batteryVoltage);
  }

  // Calibration has priority over starting the robot. It returns to this
  // waiting state automatically after calibration finishes.
  if (serviceCalibration()) return false;

  if (startButtonPin_ != 255) {
    constexpr uint32_t kStartButtonDebounceMs = 30;
    constexpr uint32_t kUnderbodyCalibrationHoldMs = 5000;
    const uint32_t now = millis();
    const bool rawPressed = digitalRead(startButtonPin_) == LOW;

    if (!waitStartButtonInitialized_) {
      waitStartButtonInitialized_ = true;
      waitStartButtonRawPressed_ = rawPressed;
      waitStartButtonStablePressed_ = false;
      waitStartButtonChangedAt_ = now;
    } else if (rawPressed != waitStartButtonRawPressed_) {
      waitStartButtonRawPressed_ = rawPressed;
      waitStartButtonChangedAt_ = now;
    }

    if (waitStartButtonRawPressed_ != waitStartButtonStablePressed_ &&
        now - waitStartButtonChangedAt_ >= kStartButtonDebounceMs) {
      waitStartButtonStablePressed_ = waitStartButtonRawPressed_;
      if (waitStartButtonStablePressed_) {
        waitStartButtonPressedAt_ = now;
        waitStartButtonLongPressStarted_ = false;
      } else if (!waitStartButtonLongPressStarted_) {
        // A debounced release before five seconds is the normal Start action.
        beep(kStartConfirmationFrequency, kStartConfirmationDurationMs);
        delay(kStartConfirmationDurationMs);
        robotStarted_ = true;
        return true;
      } else {
        // The long press has already started calibration; release only arms
        // the next, separate short Start press.
        waitStartButtonLongPressStarted_ = false;
      }
    }

    if (waitStartButtonStablePressed_ &&
        !waitStartButtonLongPressStarted_ &&
        now - waitStartButtonPressedAt_ >= kUnderbodyCalibrationHoldMs) {
      waitStartButtonLongPressStarted_ = true;
      startUnderbodyCalibration();
      return false;
    }
  }

  if (millis() - lastWaitingMonitorPrintAt_ >= monitorIntervalMs) {
    lastWaitingMonitorPrintAt_ = millis();
    printLiveReadings(output);
  }
  return false;
}

void MyMiniPro::setupMux(const MuxPins &mux) {
  pinMode(mux.s0, OUTPUT);
  pinMode(mux.s1, OUTPUT);
  pinMode(mux.s2, OUTPUT);
  pinMode(mux.s3, OUTPUT);
}

void MyMiniPro::selectChannel(const MuxPins &mux, uint8_t channel) {
  digitalWrite(mux.s0, bitRead(channel, 0));
  digitalWrite(mux.s1, bitRead(channel, 1));
  digitalWrite(mux.s2, bitRead(channel, 2));
  digitalWrite(mux.s3, bitRead(channel, 3));
}

uint16_t MyMiniPro::readChannel(const MuxPins &mux, uint8_t channel) {
  selectChannel(mux, channel);
  delayMicroseconds(settings_.muxSettleMicroseconds);
  return analogRead(mux.signal);
}

void MyMiniPro::scanMux(const MuxPins &mux,
                        uint16_t readings[kSensorCount],
                        bool &filterInitialized, uint8_t captureChannel,
                        uint16_t *capturedRaw, uint8_t secondCaptureChannel,
                        uint16_t *secondCapturedRaw) {
  for (uint8_t channel = 0; channel < kSensorCount; ++channel) {
    const uint16_t rawReading = readChannel(mux, channel);
    if (capturedRaw && channel==captureChannel) *capturedRaw=rawReading;
    if (secondCapturedRaw && channel==secondCaptureChannel) *secondCapturedRaw=rawReading;
    if (!filterInitialized || sensorSmoothingPercent_ == 100) {
      readings[channel] = rawReading;
    } else {
      // Integer EMA: new=40% raw + 60% previous by default. It costs no
      // second ADC conversion; smoothing still adds response lag.
      readings[channel] = static_cast<uint16_t>(
          (static_cast<uint32_t>(rawReading) * sensorSmoothingPercent_ +
           static_cast<uint32_t>(readings[channel]) *
               (100 - sensorSmoothingPercent_) +
           50) /
          100);
    }
  }
  filterInitialized = true;
}

void MyMiniPro::scan(Direction direction) {
  if (direction == Direction::Forward) {
    scanFront();
  } else {
    scanRear();
  }
}

void MyMiniPro::scanFront() {
  scanMux(frontMux_, frontCurrent_, frontSensorFilterInitialized_);
}

void MyMiniPro::scanRear() {
  scanMux(rearMux_, rearCurrent_, rearSensorFilterInitialized_);
}

void MyMiniPro::setSensorSmoothing(uint8_t newReadingPercent) {
  sensorSmoothingPercent_ = constrain(static_cast<int>(newReadingPercent), 1, 100);
  // Start from fresh readings after changing the filter strength.
  frontSensorFilterInitialized_ = false;
  rearSensorFilterInitialized_ = false;
}

bool MyMiniPro::setMuxSettleMicroseconds(uint16_t microseconds) {
  if (microseconds < 1 || microseconds > 2000 || calibrationActive_ ||
      underbodyCalibrationActive_) return false;
  settings_.muxSettleMicroseconds = microseconds;
  frontSensorFilterInitialized_ = rearSensorFilterInitialized_ = false;
  return true;
}

uint16_t MyMiniPro::readSensorFront(uint8_t channel) {
  scanFront();
  return values(Array::Front, channel).current;
}

uint16_t MyMiniPro::minSensorFront(uint8_t channel) const {
  return values(Array::Front, channel).minimum;
}

uint16_t MyMiniPro::maxSensorFront(uint8_t channel) const {
  return values(Array::Front, channel).maximum;
}

uint16_t MyMiniPro::readSensorRear(uint8_t channel) {
  scanRear();
  return values(Array::Rear, channel).current;
}

uint16_t MyMiniPro::minSensorRear(uint8_t channel) const {
  return values(Array::Rear, channel).minimum;
}

uint16_t MyMiniPro::maxSensorRear(uint8_t channel) const {
  return values(Array::Rear, channel).maximum;
}

void MyMiniPro::printLiveReadings(Stream &output) {
  scanFront();
  scanRear();

  output.print(F("F: "));
  for (uint8_t channel = 0; channel < kSensorCount; ++channel) {
    if (channel != 0) output.print(',');
    output.print(normalized(Array::Front, channel));
  }

  output.print(F(" | R: "));
  for (uint8_t channel = 0; channel < kSensorCount; ++channel) {
    if (channel != 0) output.print(',');
    output.print(normalized(Array::Rear, channel));
  }
  if (underbodyCalibrationActive_) {
    output.print(F(" | ADC CAL: RUNNING"));
  } else if (waitStartButtonStablePressed_ &&
             !waitStartButtonLongPressStarted_) {
    output.print(F(" | ADC CAL HOLD: "));
    output.print((millis() - waitStartButtonPressedAt_) / 1000);
    output.print(F("/5s"));
  } else if (hasUnderbodyCalibration()) {
    output.print(F(" | ADC NL: "));
    output.print(readAdcLNormalized());
    output.print(F(" | ADC NR: "));
    output.print(readAdcRNormalized());
  } else {
    output.print(F(" | ADC CAL: REQUIRED"));
  }
  // waitButton calls this only at the monitor interval, not every poll.
  const float batteryVoltage = readBatteryVoltage();
  output.print(F(" | BAT: "));
  if (isnan(batteryVoltage)) {
    output.print(F("READ ERROR"));
  } else {
    output.print(batteryVoltage);
    output.print(F(" V"));
  }
  output.println();
}

int32_t MyMiniPro::rearCalibrationButtonValue() {
  if (!settings_.rearButtonEnabled) return INT32_MAX;
  return readMcp3421();
}

uint16_t *MyMiniPro::minimum(Array array) {
  return array == Array::Front ? storage_.frontMin : storage_.rearMin;
}

uint16_t *MyMiniPro::maximum(Array array) {
  return array == Array::Front ? storage_.frontMax : storage_.rearMax;
}

const uint16_t *MyMiniPro::minimum(Array array) const {
  return array == Array::Front ? storage_.frontMin : storage_.rearMin;
}

const uint16_t *MyMiniPro::maximum(Array array) const {
  return array == Array::Front ? storage_.frontMax : storage_.rearMax;
}

const uint16_t *MyMiniPro::current(Array array) const {
  return array == Array::Front ? frontCurrent_ : rearCurrent_;
}

MyMiniPro::SensorValues MyMiniPro::values(Array array,
                                                           uint8_t channel) const {
  if (channel >= kSensorCount) return {0, 0, 0};
  return {current(array)[channel], maximum(array)[channel], minimum(array)[channel]};
}

const uint16_t *MyMiniPro::minValues(Array array) const {
  return minimum(array);
}

const uint16_t *MyMiniPro::maxValues(Array array) const {
  return maximum(array);
}

void MyMiniPro::setAds1115Address(uint8_t address) {
  ads1115Address_ = address;
}

bool MyMiniPro::writeAds1115Register(uint8_t reg, uint16_t value) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(ads1115Address_);
  wire_->write(reg);
  wire_->write(highByte(value));
  wire_->write(lowByte(value));
  return wire_->endTransmission() == 0;
}

bool MyMiniPro::readAds1115Register(uint8_t reg, uint16_t &value) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(ads1115Address_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(ads1115Address_, static_cast<uint8_t>(2)) != 2) return false;
  value = static_cast<uint16_t>((wire_->read() << 8) | wire_->read());
  return true;
}

bool MyMiniPro::startAds1115SingleShot(uint8_t channel) {
  if (channel > 3) return false;

  // Single-shot, AINx-to-GND, ±4.096 V, 860 samples/sec, comparator off.
  const uint16_t mux = static_cast<uint16_t>(0x4000U + (channel << 12));
  const uint16_t config = static_cast<uint16_t>(0x8000U | mux | 0x0200U |
                                                0x0100U | 0x00E0U | 0x0003U);
  return writeAds1115Register(0x01, config);
}

int16_t MyMiniPro::readAds1115Raw(uint8_t channel) {
  // A synchronous conversion replaces the ADS mux/configuration. Never let
  // the battery service consume this channel as a pending AIN0 conversion.
  if (motorVoltageConversionPending_) {
    motorVoltageConversionPending_ = false;
    lastMotorVoltageSampleAt_ = millis();
  }
  if (!startAds1115SingleShot(channel)) return INT16_MIN;

  uint16_t status = 0;
  const uint32_t startedAt = millis();
  do {
    if (!readAds1115Register(0x01, status)) return INT16_MIN;
  } while ((status & 0x8000U) == 0 && millis() - startedAt < 10);

  // Do not return the previous conversion if the single-shot conversion did
  // not become ready before the synchronous API timeout.
  if ((status & 0x8000U) == 0) return INT16_MIN;

  uint16_t raw = 0;
  if (!readAds1115Register(0x00, raw)) return INT16_MIN;
  return static_cast<int16_t>(raw);
}

int16_t MyMiniPro::readAdcL() { return readAds1115Raw(1); }

int16_t MyMiniPro::readAdcR() { return readAds1115Raw(2); }

int16_t MyMiniPro::readAds1115Ain3Raw() { return readAds1115Raw(3); }

void MyMiniPro::startUnderbodyCalibration() {
  underbodyStorage_.adcLMinimum = INT16_MAX;
  underbodyStorage_.adcLMaximum = INT16_MIN;
  underbodyStorage_.adcRMinimum = INT16_MAX;
  underbodyStorage_.adcRMaximum = INT16_MIN;
  underbodyStorage_.validFlags = 0;
  underbodyCalibrationActive_ = true;
  underbodyCalibrationStartedAt_ = millis();
  lastUnderbodyCalibrationProgressBeepAt_ =
      underbodyCalibrationStartedAt_ - kCalibrationProgressIntervalMs;
}

bool MyMiniPro::serviceUnderbodyCalibration() {
  if (!underbodyCalibrationActive_) return false;

  const int16_t adcL = readAdcL();
  const int16_t adcR = readAdcR();
  if (adcL != INT16_MIN) {
    if (adcL < underbodyStorage_.adcLMinimum) underbodyStorage_.adcLMinimum = adcL;
    if (adcL > underbodyStorage_.adcLMaximum) underbodyStorage_.adcLMaximum = adcL;
  }
  if (adcR != INT16_MIN) {
    if (adcR < underbodyStorage_.adcRMinimum) underbodyStorage_.adcRMinimum = adcR;
    if (adcR > underbodyStorage_.adcRMaximum) underbodyStorage_.adcRMaximum = adcR;
  }

  if (millis() - lastUnderbodyCalibrationProgressBeepAt_ >=
      kCalibrationProgressIntervalMs) {
    lastUnderbodyCalibrationProgressBeepAt_ = millis();
    beep(kCalibrationProgressFrequency, kCalibrationProgressDurationMs);
  }

  if (static_cast<uint32_t>(millis() - underbodyCalibrationStartedAt_) <
      settings_.calibrationTimeMs) {
    return true;
  }

  underbodyCalibrationActive_ = false;
  const bool rangesValid =
      underbodyStorage_.adcLMaximum > underbodyStorage_.adcLMinimum &&
      underbodyStorage_.adcRMaximum > underbodyStorage_.adcRMinimum;
  underbodyStorage_.validFlags = rangesValid ? kUnderbodyValid : 0;
  if (rangesValid && saveUnderbodyCalibration()) {
    playCalibrationFinishedBeep();
  } else {
    underbodyStorage_.validFlags = 0;
  }
  return true;
}

bool MyMiniPro::underbodyCalibrationActive() const {
  return underbodyCalibrationActive_;
}

bool MyMiniPro::hasUnderbodyCalibration() const {
  return (underbodyStorage_.validFlags & kUnderbodyValid) != 0 &&
         underbodyStorage_.adcLMaximum > underbodyStorage_.adcLMinimum &&
         underbodyStorage_.adcRMaximum > underbodyStorage_.adcRMinimum;
}

int16_t MyMiniPro::adcLMinimum() const {
  return hasUnderbodyCalibration() ? underbodyStorage_.adcLMinimum : INT16_MIN;
}

int16_t MyMiniPro::adcLMaximum() const {
  return hasUnderbodyCalibration() ? underbodyStorage_.adcLMaximum : INT16_MIN;
}

int16_t MyMiniPro::adcRMinimum() const {
  return hasUnderbodyCalibration() ? underbodyStorage_.adcRMinimum : INT16_MIN;
}

int16_t MyMiniPro::adcRMaximum() const {
  return hasUnderbodyCalibration() ? underbodyStorage_.adcRMaximum : INT16_MIN;
}

int16_t MyMiniPro::minAdcL() const { return adcLMinimum(); }

int16_t MyMiniPro::maxAdcL() const { return adcLMaximum(); }

int16_t MyMiniPro::minAdcR() const { return adcRMinimum(); }

int16_t MyMiniPro::maxAdcR() const { return adcRMaximum(); }

uint16_t MyMiniPro::normalizeUnderbody(int16_t value, int16_t minimum,
                                       int16_t maximum) const {
  if (value == INT16_MIN || maximum <= minimum) {
    return kUncalibratedNormalizedValue;
  }
  const long mapped = map(value, minimum, maximum, 0, 1000);
  return static_cast<uint16_t>(constrain(mapped, 0L, 1000L));
}

uint16_t MyMiniPro::readAdcLNormalized() {
  if (!hasUnderbodyCalibration()) return kUncalibratedNormalizedValue;
  return normalizeUnderbody(readAdcL(), underbodyStorage_.adcLMinimum,
                            underbodyStorage_.adcLMaximum);
}

uint16_t MyMiniPro::readAdcRNormalized() {
  if (!hasUnderbodyCalibration()) return kUncalibratedNormalizedValue;
  return normalizeUnderbody(readAdcR(), underbodyStorage_.adcRMinimum,
                            underbodyStorage_.adcRMaximum);
}

float MyMiniPro::readAds1115Voltage(uint8_t channel) {
  const int16_t raw = readAds1115Raw(channel);
  if (raw == INT16_MIN) return NAN;
  return raw * kAds1115LsbVolts;
}

void MyMiniPro::setBatteryDividerRatio(float dividerRatio) {
  if (dividerRatio > 0.0f) batteryDividerRatio_ = dividerRatio;
}

void MyMiniPro::setBatteryCalibration(float gain, float offsetVolts) {
  if (gain > 0.0f) batteryCalibrationGain_ = gain;
  batteryCalibrationOffset_ = offsetVolts;
}

float MyMiniPro::readBatteryVoltage() {
  const float dividedVoltage = readBatteryVoltage(batteryDividerRatio_);
  if (isnan(dividedVoltage)) return NAN;
  return dividedVoltage * batteryCalibrationGain_ + batteryCalibrationOffset_;
}

float MyMiniPro::readBatteryVoltage(float dividerRatio) {
  const float adcVoltage = readAds1115Voltage(0);
  if (isnan(adcVoltage)) return NAN;
  return adcVoltage * dividerRatio;
}

bool MyMiniPro::beginBatteryLevelLeds(uint8_t pcf8574Address,
                                      bool ledsActiveLow) {
  // PCF8574P uses the 0x20..0x27 range; accepting any valid 7-bit address
  // also supports compatible parts and boards with a different address map.
  if (wire_ == nullptr || pcf8574Address < 0x08 || pcf8574Address > 0x77) {
    return false;
  }

  batteryLedPcf8574Address_ = pcf8574Address;
  batteryLedsActiveLow_ = ledsActiveLow;
  batteryLevelLedsEnabled_ = true;
  lowBatteryWarningActive_ = false;
  lowBatteryWarningIsCritical_ = false;
  lastLowBatteryWarningAt_ = 0;
  const float batteryVoltage = readBatteryVoltage();
  if (updateBatteryLevelLeds(batteryVoltage)) {
    lastBatteryLevelLedsUpdateAt_ = millis();
    return true;
  }

  batteryLevelLedsEnabled_ = false;
  return false;
}

uint8_t MyMiniPro::batteryLevelLedCount(float batteryVoltage) {
  if (isnan(batteryVoltage) || batteryVoltage < 11.0f) return 0;
  if (batteryVoltage >= 12.4f) return 8;

  uint8_t litLedCount = 1;
  for (uint8_t led = 1; led < 8; ++led) {
    const float threshold = 11.0f + 0.2f * led;
    if (batteryVoltage >= threshold) {
      ++litLedCount;
    }
  }
  return litLedCount;
}

bool MyMiniPro::writeBatteryLevelLeds(uint8_t litLedCount) {
  if (!batteryLevelLedsEnabled_ || wire_ == nullptr) return false;

  if (litLedCount > 8) litLedCount = 8;
  const uint8_t enabledMask = litLedCount == 8
                                  ? 0xFF
                                  : static_cast<uint8_t>((1U << litLedCount) - 1U);
  const uint8_t output = batteryLedsActiveLow_
                             ? static_cast<uint8_t>(~enabledMask)
                             : enabledMask;

  wire_->beginTransmission(batteryLedPcf8574Address_);
  wire_->write(output);
  return wire_->endTransmission() == 0;
}

bool MyMiniPro::updateBatteryLevelLeds() {
  return updateBatteryLevelLeds(readBatteryVoltage());
}

bool MyMiniPro::updateBatteryLevelLeds(float batteryVoltage) {
  const uint8_t litLedCount = batteryLevelLedCount(batteryVoltage);
  if (!writeBatteryLevelLeds(litLedCount)) return false;

  // USB power reads below 6.5 V on this battery-divider path. It is not a
  // battery-low condition, so silence and reset the battery-warning state
  // before considering the normal low and critical battery thresholds.
  if (!isnan(batteryVoltage) && batteryVoltage < 6.5f) {
    lowBatteryWarningActive_ = false;
    lowBatteryWarningIsCritical_ = false;
    lastLowBatteryWarningAt_ = 0;
  // Repeat a non-blocking warning while battery voltage stays below the
  // two-LED threshold. Critical voltage gets a longer tone; changing between
  // low and critical plays the new pattern immediately. Returning to 11.2 V
  // or above arms the warning for a future drop.
  } else if (!isnan(batteryVoltage) && batteryVoltage < 11.2f) {
    const bool isCritical = batteryVoltage < 10.5f;
    if (!lowBatteryWarningActive_ || lowBatteryWarningIsCritical_ != isCritical ||
        millis() - lastLowBatteryWarningAt_ >= kLowBatteryWarningIntervalMs) {
      const uint16_t duration = isCritical ? kCriticalBatteryWarningDurationMs
                                           : kLowBatteryWarningDurationMs;
      beep(kLowBatteryWarningFrequency, duration);
      lastLowBatteryWarningAt_ = millis();
      lowBatteryWarningActive_ = true;
      lowBatteryWarningIsCritical_ = isCritical;
    }
  } else if (!isnan(batteryVoltage)) {
    lowBatteryWarningActive_ = false;
    lowBatteryWarningIsCritical_ = false;
    lastLowBatteryWarningAt_ = 0;
  }
  return true;
}

uint16_t MyMiniPro::normalized(Array array, uint8_t channel) const {
  const SensorValues sensor = values(array, channel);
  return normalizeReading(sensor.current,sensor.minimum,sensor.maximum);
}

bool MyMiniPro::hasCalibration(Array array) const {
  const uint8_t flag = array == Array::Front ? kFrontValid : kRearValid;
  return (storage_.validFlags & flag) != 0;
}

void MyMiniPro::resetStorage() {
  storage_.magic = kMagic;
  storage_.validFlags = 0;
  storage_.reserved = 0;
  for (uint8_t channel = 0; channel < kSensorCount; ++channel) {
    storage_.frontMin[channel] = settings_.adcMaximum;
    storage_.frontMax[channel] = 0;
    storage_.rearMin[channel] = settings_.adcMaximum;
    storage_.rearMax[channel] = 0;
  }
  storage_.checksum = checksum();
}

void MyMiniPro::resetUnderbodyStorage() {
  underbodyStorage_.magic = kUnderbodyMagic;
  underbodyStorage_.validFlags = 0;
  underbodyStorage_.reserved = 0;
  underbodyStorage_.adcLMinimum = INT16_MAX;
  underbodyStorage_.adcLMaximum = INT16_MIN;
  underbodyStorage_.adcRMinimum = INT16_MAX;
  underbodyStorage_.adcRMaximum = INT16_MIN;
  underbodyStorage_.checksum = underbodyChecksum();
}

void MyMiniPro::startCalibration(Array array) {
  calibrationSecondBeepPending_ = false;
  lastCalibrationProgressBeepAt_ = millis() - kCalibrationProgressIntervalMs;
  calibrationArray_ = array;
  calibrationActive_ = true;
  calibrationStartedAt_ = millis();

  uint16_t *minValues = minimum(array);
  uint16_t *maxValues = maximum(array);
  for (uint8_t channel = 0; channel < kSensorCount; ++channel) {
    minValues[channel] = settings_.adcMaximum;
    maxValues[channel] = 0;
  }
}

void MyMiniPro::playCalibrationFinishedBeep() {
  // First beep now, then a second beep after a short gap.
  beep(kCalibrationFinishedFrequency, kCalibrationFinishedDurationMs);
  calibrationSecondBeepPending_ = true;
  calibrationSecondBeepAt_ = millis() + 250;
}

bool MyMiniPro::calibrationActive() const { return calibrationActive_; }

MyMiniPro::Array MyMiniPro::calibrationArray() const {
  return calibrationArray_;
}

void MyMiniPro::updateCalibration() {
  if (calibrationArray_ == Array::Front) {
    scanFront();
  } else {
    scanRear();
  }

  const uint16_t *readings = current(calibrationArray_);
  uint16_t *minValues = minimum(calibrationArray_);
  uint16_t *maxValues = maximum(calibrationArray_);
  for (uint8_t channel = 0; channel < kSensorCount; ++channel) {
    if (readings[channel] < minValues[channel]) minValues[channel] = readings[channel];
    if (readings[channel] > maxValues[channel]) maxValues[channel] = readings[channel];
  }

  // Periodic beeps while the robot is being moved over white and black.
  if (millis() - lastCalibrationProgressBeepAt_ >= kCalibrationProgressIntervalMs) {
    lastCalibrationProgressBeepAt_ = millis();
    beep(kCalibrationProgressFrequency, kCalibrationProgressDurationMs);
  }

  if (static_cast<uint32_t>(millis() - calibrationStartedAt_) >=
      settings_.calibrationTimeMs) {
    calibrationActive_ = false;
    storage_.validFlags |= calibrationArray_ == Array::Front ? kFrontValid : kRearValid;
    saveCalibration();
    playCalibrationFinishedBeep();
  }
}

bool MyMiniPro::frontButtonPressedEdge() {
  if (settings_.frontCalibrationPin == 255) return false;
  const bool pressed = digitalRead(settings_.frontCalibrationPin) == LOW;

  const bool edge = pressed && !frontButtonWasPressed_;
  frontButtonWasPressed_ = pressed;
  return edge;
}

void MyMiniPro::configureMcp3421() {
  if (wire_ == nullptr) return;
  wire_->beginTransmission(settings_.mcp3421Address);
  wire_->write(kMcp3421Config);
  wire_->endTransmission();
}

int32_t MyMiniPro::readMcp3421() {
  if (wire_ == nullptr) return INT32_MAX;
  if (wire_->requestFrom(settings_.mcp3421Address, static_cast<uint8_t>(3)) != 3) {
    return INT32_MAX;
  }

  int16_t value = static_cast<int16_t>((wire_->read() << 8) | wire_->read());
  wire_->read();  // configuration byte
  value &= 0x0FFF;
  if (value & 0x0800) value |= 0xF000;
  return value;
}

bool MyMiniPro::rearButtonPressedEdge() {
  if (!settings_.rearButtonEnabled || millis() - lastRearButtonPollAt_ < 20) return false;
  lastRearButtonPollAt_ = millis();
  const bool pressed = readMcp3421() < settings_.rearButtonThreshold;

  const bool edge = pressed && !rearButtonWasPressed_;
  rearButtonWasPressed_ = pressed;
  return edge;
}

bool MyMiniPro::serviceCalibration() {
  updateBuzzer();

  if (calibrationActive_) {
    updateCalibration();
    return true;
  }
  if (frontButtonPressedEdge()) {
    startCalibration(Array::Front);
    return true;
  }
  if (rearButtonPressedEdge()) {
    startCalibration(Array::Rear);
    return true;
  }
  return false;
}

uint16_t MyMiniPro::checksum() const {
  const uint8_t *data = reinterpret_cast<const uint8_t *>(&storage_);
  uint16_t total = 0;
  for (size_t i = 0; i < sizeof(Storage) - sizeof(storage_.checksum); ++i) {
    total += data[i];
  }
  return total;
}

bool MyMiniPro::eepromWriteByte(uint16_t address, uint8_t value) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(settings_.eepromAddress);
  wire_->write(highByte(address));
  wire_->write(lowByte(address));
  wire_->write(value);
  if (wire_->endTransmission() != 0) return false;
  delay(5);
  return true;
}

bool MyMiniPro::eepromReadByte(uint16_t address, uint8_t &value) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(settings_.eepromAddress);
  wire_->write(highByte(address));
  wire_->write(lowByte(address));
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(settings_.eepromAddress, static_cast<uint8_t>(1)) != 1) return false;
  value = wire_->read();
  return true;
}

bool MyMiniPro::eepromWriteBlock(uint16_t address, const uint8_t *data,
                                          size_t size) {
  for (size_t i = 0; i < size; ++i) {
    if (!eepromWriteByte(address + i, data[i])) return false;
  }
  return true;
}

bool MyMiniPro::eepromReadBlock(uint16_t address, uint8_t *data, size_t size) {
  for (size_t i = 0; i < size; ++i) {
    if (!eepromReadByte(address + i, data[i])) return false;
  }
  return true;
}

bool MyMiniPro::loadCalibration() {
  const bool mainCalibrationLoaded =
      eepromReadBlock(0, reinterpret_cast<uint8_t *>(&storage_), sizeof(Storage)) &&
      storage_.magic == kMagic && storage_.checksum == checksum();
  if (!mainCalibrationLoaded) {
    resetStorage();
  }
  loadUnderbodyCalibration();
  return mainCalibrationLoaded;
}

bool MyMiniPro::saveCalibration() {
  storage_.magic = kMagic;
  storage_.checksum = checksum();
  return eepromWriteBlock(0, reinterpret_cast<const uint8_t *>(&storage_),
                          sizeof(Storage));
}

uint16_t MyMiniPro::underbodyChecksum() const {
  const uint8_t *data = reinterpret_cast<const uint8_t *>(&underbodyStorage_);
  uint16_t total = 0;
  for (size_t i = 0; i < sizeof(UnderbodyStorage) -
                             sizeof(underbodyStorage_.checksum);
       ++i) {
    total += data[i];
  }
  return total;
}

bool MyMiniPro::loadUnderbodyCalibration() {
  if (!eepromReadBlock(kUnderbodyStorageAddress,
                       reinterpret_cast<uint8_t *>(&underbodyStorage_),
                       sizeof(UnderbodyStorage)) ||
      underbodyStorage_.magic != kUnderbodyMagic ||
      underbodyStorage_.checksum != underbodyChecksum() ||
      !hasUnderbodyCalibration()) {
    resetUnderbodyStorage();
    return false;
  }
  return true;
}

bool MyMiniPro::saveUnderbodyCalibration() {
  underbodyStorage_.magic = kUnderbodyMagic;
  underbodyStorage_.checksum = underbodyChecksum();
  return eepromWriteBlock(kUnderbodyStorageAddress,
                          reinterpret_cast<const uint8_t *>(&underbodyStorage_),
                          sizeof(UnderbodyStorage));
}

#include "MyMiniProForwardTracker.h"
#include "MyMiniProForwardPD.h"

bool MyMiniPro::setFLineSensorDebounceMs(uint32_t milliseconds) {
  if (milliseconds>60000) return false;
  fLineSensorDebounceMs_=static_cast<uint16_t>(milliseconds);
  return true;
}

bool MyMiniPro::setFLineKd(float kd) {
  if (!isfinite(kd)) return false;
  fLineKd_=kd;
  return true;
}

void MyMiniPro::setFLineFRMotors(int leftPercent,int rightPercent) {
  fLineFRLeftPercent_=constrain(leftPercent,-100,100);
  fLineFRRightPercent_=constrain(rightPercent,-100,100);
}
bool MyMiniPro::setFLineApproachSpeed(int leftPercent,int rightPercent) {
  if(!((leftPercent==0 && rightPercent==0) ||
       (leftPercent>=1 && leftPercent<=100 && rightPercent>=1 && rightPercent<=100)))return false;
  fLineApproachLeft_=leftPercent;
  fLineApproachRight_=rightPercent;
  return true;
}
void MyMiniPro::setFLineFLMotors(int leftPercent,int rightPercent) {
  fLineFLLeftPercent_=constrain(leftPercent,-100,100);
  fLineFLRightPercent_=constrain(rightPercent,-100,100);
}

bool MyMiniPro::setFLineTurnClearanceCm(float centimeters) {
  return isfinite(centimeters) && centimeters==0;
}
bool MyMiniPro::setFLineMinTurnMs(uint32_t milliseconds) {
  return milliseconds==0;
}

bool MyMiniPro::setFLineRecoveryKpThreshold(float threshold) {
  if(!isfinite(threshold))return false;
  fLineRecoveryKpThreshold_=threshold;
  return true;
}

MyMiniPro::ForwardResult MyMiniPro::F_Line(int left, int right, float kp, float cm) {
  return runForward(left, right, kp, false, cm, LineSensor::F0);
}
MyMiniPro::ForwardResult MyMiniPro::F_Line(int left, int right, float kp, LineSensor sensor) {
  return runForward(left, right, kp, true, 0, sensor);
}
