#include <Pico2_MyMiniPro.h>

#include <cassert>
#include <climits>
#include <cstdint>
#include <initializer_list>

namespace {
unsigned long fakeMillis = 0;
unsigned long fakeMicros = 0;
unsigned long millisStep = 1;
int fakeDigitalRead = HIGH;
bool fakeToneActive = false;
int fakeTimedToneCount = 0;

MyMiniPro makeRobot(uint32_t calibrationTimeMs = 5000) {
  const MyMiniPro::MuxPins front = {1, 2, 3, 4, 5};
  const MyMiniPro::MuxPins rear = {6, 7, 8, 9, 10};
  const MyMiniPro::Settings settings = {
      255, 0x50, 0x68, 1000, 4095, 3, calibrationTimeMs, false};
  MyMiniPro robot(front, rear, settings);
  robot.begin(Wire);
  robot.beginStartButton(11);
  return robot;
}

void resetWire() {
  Wire.reset();
  fakeMillis = 0;
  millisStep = 1;
  fakeDigitalRead = HIGH;
  fakeToneActive = false;
  fakeTimedToneCount = 0;
}

void enqueueReadyRaw(uint16_t raw) {
  Wire.enqueueResponse({0x80, 0x00});  // Config OS bit: conversion ready.
  Wire.enqueueResponse({static_cast<uint8_t>(raw >> 8),
                        static_cast<uint8_t>(raw & 0xff)});
}

void enqueueUnderbodySample(uint16_t adcL, uint16_t adcR) {
  enqueueReadyRaw(adcL);
  enqueueReadyRaw(adcR);
}

uint16_t lastConfigWrite() {
  for (auto it = Wire.transactions.rbegin(); it != Wire.transactions.rend(); ++it) {
    if (it->bytes.size() == 3 && it->bytes[0] == 0x01) {
      return static_cast<uint16_t>((it->bytes[1] << 8) | it->bytes[2]);
    }
  }
  assert(false && "ADS1115 config write was not issued");
  return 0;
}

void testUnderbodyAndAuxiliaryChannelsSelectCorrectInputAndPreserveSign() {
  MyMiniPro robot = makeRobot();

  resetWire();
  enqueueReadyRaw(0xff9c);  // -100 in the signed ADS1115 conversion format.
  assert(robot.readAdcL() == -100);
  assert(lastConfigWrite() == 0xd3e3);  // AIN1 (underbody left)-to-GND, single-shot, ±4.096 V, 860 SPS.

  resetWire();
  enqueueReadyRaw(0x1234);
  assert(robot.readAdcR() == 0x1234);
  assert(lastConfigWrite() == 0xe3e3);  // AIN2 (underbody right)-to-GND.

  resetWire();
  enqueueReadyRaw(0x7fff);
  assert(robot.readAds1115Ain3Raw() == 0x7fff);
  assert(lastConfigWrite() == 0xf3e3);  // AIN3-to-GND.
}

void testWrapperPropagatesI2cFailure() {
  MyMiniPro robot = makeRobot();
  resetWire();
  Wire.endTransmissionResult = 4;

  assert(robot.readAdcR() == INT16_MIN);
  assert(Wire.requestRegisters.empty());
}

void testWrapperReturnsErrorWhenConversionNeverBecomesReady() {
  MyMiniPro robot = makeRobot();
  resetWire();
  millisStep = 2;
  for (int sample = 0; sample < 8; ++sample) {
    Wire.enqueueResponse({0x00, 0x00});  // Config OS bit remains clear.
  }

  assert(robot.readAds1115Ain3Raw() == INT16_MIN);
  assert(!Wire.requestRegisters.empty());
  for (uint8_t selected : Wire.requestRegisters) {
    assert(selected == 0x01);  // It must not read the conversion register.
  }
}

void testUnderbodyCalibrationNormalizationAndPersistence() {
  MyMiniPro robot = makeRobot(5);
  resetWire();

  robot.startUnderbodyCalibration();
  millisStep = 0;
  enqueueUnderbodySample(100, 200);
  assert(robot.serviceUnderbodyCalibration());
  assert(robot.underbodyCalibrationActive());
  assert(!robot.hasUnderbodyCalibration());

  fakeMillis = 10;
  enqueueUnderbodySample(300, 500);
  assert(robot.serviceUnderbodyCalibration());
  assert(!robot.underbodyCalibrationActive());
  assert(robot.hasUnderbodyCalibration());
  assert(robot.adcLMinimum() == 100);
  assert(robot.adcLMaximum() == 300);
  assert(robot.adcRMinimum() == 200);
  assert(robot.adcRMaximum() == 500);

  Wire.clearTransactions();
  enqueueReadyRaw(100);
  assert(robot.readAdcLNormalized() == 0);
  enqueueReadyRaw(300);
  assert(robot.readAdcLNormalized() == 1000);
  enqueueReadyRaw(50);
  assert(robot.readAdcLNormalized() == 0);
  enqueueReadyRaw(600);
  assert(robot.readAdcRNormalized() == 1000);
  enqueueReadyRaw(0x8000);
  assert(robot.readAdcRNormalized() ==
         MyMiniPro::kUncalibratedNormalizedValue);

  Wire.clearTransactions();  // Keep EEPROM contents while constructing a new robot.
  MyMiniPro reloaded = makeRobot(5);
  assert(!reloaded.loadCalibration());  // No legacy Front/Rear record was stored.
  assert(reloaded.hasUnderbodyCalibration());
  assert(reloaded.adcLMinimum() == 100);
  assert(reloaded.adcRMaximum() == 500);

  // The extension follows the unchanged legacy CAL1 record. A damaged
  // extension must not be treated as a valid underbody calibration.
  constexpr uint16_t kLegacyStorageSize = 136;
  Wire.writeEeprom(kLegacyStorageSize, 0x00);
  MyMiniPro corrupted = makeRobot(5);
  corrupted.loadCalibration();
  assert(!corrupted.hasUnderbodyCalibration());
  assert(corrupted.readAdcLNormalized() ==
         MyMiniPro::kUncalibratedNormalizedValue);
}

struct LegacyStorage {
  uint32_t magic;
  uint8_t validFlags;
  uint8_t reserved;
  uint16_t frontMin[MyMiniPro::kSensorCount];
  uint16_t frontMax[MyMiniPro::kSensorCount];
  uint16_t rearMin[MyMiniPro::kSensorCount];
  uint16_t rearMax[MyMiniPro::kSensorCount];
  uint16_t checksum;
};

uint16_t legacyChecksum(const LegacyStorage &storage) {
  const auto *bytes = reinterpret_cast<const uint8_t *>(&storage);
  uint16_t total = 0;
  for (size_t index = 0; index < sizeof(LegacyStorage) - sizeof(storage.checksum);
       ++index) {
    total += bytes[index];
  }
  return total;
}

void writeLegacyStorage(const LegacyStorage &storage) {
  const auto *bytes = reinterpret_cast<const uint8_t *>(&storage);
  for (size_t index = 0; index < sizeof(LegacyStorage); ++index) {
    Wire.writeEeprom(static_cast<uint16_t>(index), bytes[index]);
  }
}

void testLegacyFrontRearCalibrationRemainsReadable() {
  resetWire();
  static_assert(sizeof(LegacyStorage) == 136,
                "The compatibility fixture must match the original CAL1 record");
  LegacyStorage legacy{};
  legacy.magic = 0x43414C31UL;  // CAL1
  legacy.validFlags = 0x03;
  for (uint8_t channel = 0; channel < MyMiniPro::kSensorCount; ++channel) {
    legacy.frontMin[channel] = 100;
    legacy.frontMax[channel] = 900;
    legacy.rearMin[channel] = 200;
    legacy.rearMax[channel] = 800;
  }
  legacy.checksum = legacyChecksum(legacy);
  writeLegacyStorage(legacy);
  const uint8_t firstByteBeforeLoad = Wire.readEeprom(0);

  MyMiniPro robot = makeRobot();
  assert(robot.loadCalibration());
  assert(robot.hasCalibration(MyMiniPro::Array::Front));
  assert(robot.hasCalibration(MyMiniPro::Array::Rear));
  assert(!robot.hasUnderbodyCalibration());
  assert(Wire.readEeprom(0) == firstByteBeforeLoad);
}

void testWaitButtonShortPressStartsWithoutUnderbodyCalibration() {
  MyMiniPro robot = makeRobot(5);
  robot.beginBuzzer(9);
  resetWire();
  Stream output;

  assert(!robot.waitButton(output, 10000));  // Initialize debounced high state.
  assert(fakeTimedToneCount == 0);
  assert(!robot.waitButton(output, 10000));
  assert(fakeTimedToneCount == 0);
  fakeDigitalRead = LOW;
  assert(!robot.waitButton(output, 10000));
  fakeMillis = 40;
  assert(!robot.waitButton(output, 10000));
  assert(!robot.underbodyCalibrationActive());
  assert(!robot.robotStarted());

  fakeDigitalRead = HIGH;
  assert(!robot.waitButton(output, 10000));
  fakeMillis = 80;
  assert(robot.waitButton(output, 10000));
  assert(robot.robotStarted());
  assert(fakeTimedToneCount == 1);  // Start keeps its original single beep.
  for (const auto &transaction : Wire.transactions) {
    assert(!(transaction.address == 0x48 && transaction.bytes.size() == 3 &&
             transaction.bytes[0] == 0x01));
  }
}

void testWaitButtonLongPressCalibratesThenRequiresNewShortPress() {
  MyMiniPro robot = makeRobot(5);
  resetWire();
  Stream output;

  assert(!robot.waitButton(output, 10000));  // Initialize debounced high state.
  fakeDigitalRead = LOW;
  assert(!robot.waitButton(output, 10000));
  fakeMillis = 40;
  assert(!robot.waitButton(output, 10000));  // Debounced press.

  fakeMillis = 5040;
  assert(!robot.waitButton(output, 10000));  // Hold for five seconds.
  assert(robot.underbodyCalibrationActive());
  assert(!robot.robotStarted());

  millisStep = 0;
  enqueueUnderbodySample(100, 200);
  assert(!robot.waitButton(output, 10000));
  assert(robot.underbodyCalibrationActive());
  fakeMillis += 10;
  enqueueUnderbodySample(300, 500);
  assert(!robot.waitButton(output, 10000));
  assert(!robot.underbodyCalibrationActive());
  assert(robot.hasUnderbodyCalibration());
  assert(!robot.robotStarted());  // Long press never doubles as Start.

  fakeDigitalRead = HIGH;
  assert(!robot.waitButton(output, 10000));
  fakeMillis += 40;
  assert(!robot.waitButton(output, 10000));  // Release only arms a new press.

  fakeDigitalRead = LOW;
  assert(!robot.waitButton(output, 10000));
  fakeMillis += 40;
  assert(!robot.waitButton(output, 10000));
  fakeDigitalRead = HIGH;
  assert(!robot.waitButton(output, 10000));
  fakeMillis += 40;
  assert(robot.waitButton(output, 10000));
  assert(robot.robotStarted());

  // A completed long-press calibration is not restarted by later wait calls.
  Wire.clearTransactions();
  robot.waitButton(output, 10000);
  for (const auto &transaction : Wire.transactions) {
    assert(!(transaction.address == 0x48 && transaction.bytes.size() == 3 &&
             transaction.bytes[0] == 0x01));
  }
}

void testWaitButtonInvalidLongPressDoesNotAutoStart() {
  MyMiniPro robot = makeRobot(5);
  resetWire();
  Stream output;

  assert(!robot.waitButton(output, 10000));
  fakeDigitalRead = LOW;
  assert(!robot.waitButton(output, 10000));
  fakeMillis = 40;
  assert(!robot.waitButton(output, 10000));
  fakeMillis = 5040;
  assert(!robot.waitButton(output, 10000));
  millisStep = 0;
  enqueueUnderbodySample(100, 200);
  assert(!robot.waitButton(output, 10000));
  fakeMillis += 10;
  enqueueUnderbodySample(100, 200);  // No non-zero ranges: invalid calibration.
  assert(!robot.waitButton(output, 10000));
  assert(!robot.hasUnderbodyCalibration());
  assert(!robot.robotStarted());
}

void testLiveReadingLineShowsOnlyNormalizedUnderbodyValues() {
  MyMiniPro robot = makeRobot(5);
  resetWire();
  robot.startUnderbodyCalibration();
  millisStep = 0;
  enqueueUnderbodySample(100, 200);
  robot.serviceUnderbodyCalibration();
  fakeMillis = 10;
  enqueueUnderbodySample(300, 500);
  robot.serviceUnderbodyCalibration();
  assert(robot.hasUnderbodyCalibration());

  Wire.clearTransactions();
  enqueueReadyRaw(100);
  enqueueReadyRaw(500);
  Stream validOutput;
  robot.printLiveReadings(validOutput);
  assert(validOutput.contents.find("ADC NL: 0") != std::string::npos);
  assert(validOutput.contents.find("ADC NR: 1000") != std::string::npos);
  assert(validOutput.contents.find("MCP") == std::string::npos);
  assert(validOutput.contents.find("ADC L:") == std::string::npos);
  assert(validOutput.contents.find("ADC R:") == std::string::npos);

  MyMiniPro uncalibrated = makeRobot(5);
  resetWire();
  Stream uncalibratedOutput;
  uncalibrated.printLiveReadings(uncalibratedOutput);
  assert(uncalibratedOutput.contents.find("ADC CAL: REQUIRED") !=
         std::string::npos);
  assert(uncalibratedOutput.contents.find("ADC NL:") == std::string::npos);
  assert(uncalibratedOutput.contents.find("ADC NR:") == std::string::npos);
}

void testPico2ServoGpioMappingAndAngleBounds() {
  Pico2MyMiniPro robot;
  const uint8_t expectedPins[] = {18, 22, 28, 0, 1};

  for (uint8_t index = 0; index < Pico2MyMiniPro::kServoCount; ++index) {
    const uint8_t gpio = expectedPins[index];
    assert(!robot.servoAttached(gpio));
    assert(robot.servoAngle(gpio) == -1);
  }

  assert(robot.setServoPulseLimits(18, 600, 2300));
  assert(!robot.setServoPulseLimits(18, 2300, 600));
  assert(robot.servo(18, -25));
  assert(robot.servoAttached(18));
  assert(robot.servoAngle(18) == 0);
  assert(robot.servo(18, 999));
  assert(robot.servoAngle(18) == 180);
  assert(!robot.setServoPulseLimits(18, 700, 2200));

  assert(robot.servo(1, 90));
  assert(robot.servoAttached(1));
  assert(robot.servoAngle(1) == 90);
  assert(robot.detachServo(18));
  assert(!robot.servoAttached(18));
  assert(robot.servoAngle(18) == -1);
  assert(!robot.servo(17, 90));
  assert(!robot.servo(19, 90));
  assert(!robot.detachServo(17));

  Pico2MyMiniPro attachFailureRobot;
  Servo::forceAttachFailure = true;
  assert(!attachFailureRobot.servo(22, 90));
  assert(!attachFailureRobot.servoAttached(22));
  assert(attachFailureRobot.servoAngle(22) == -1);
  Servo::forceAttachFailure = false;
  assert(attachFailureRobot.servo(22, 90));
}

void testStopBuzzerCancelsPico2StartupMelody() {
  resetWire();
  Pico2MyMiniPro robot;
  robot.begin(true);
  assert(robot.buzzerActive());
  assert(fakeToneActive);

  robot.stopBuzzer();
  assert(!robot.buzzerActive());
  assert(!fakeToneActive);

  fakeMillis = 1000;
  robot.updateBuzzer();
  assert(!fakeToneActive);
}

void testPico2BeginCanSuppressStartupMelody() {
  resetWire();
  Pico2MyMiniPro robot;
  robot.begin(false);
  assert(fakeTimedToneCount == 1);  // Immediate reset acknowledgement.
  assert(!robot.buzzerActive());
  assert(!fakeToneActive);
}

void testPico2DefaultBeginAcknowledgesThenStartsShortMelody() {
  resetWire();
  Pico2MyMiniPro robot;
  robot.begin();
  assert(fakeTimedToneCount == 1);
  assert(robot.buzzerActive());
  assert(fakeToneActive);
}

void testWaitButtonLeavesStartupMelodyIntact() {
  resetWire();
  Pico2MyMiniPro robot;
  robot.begin(true);
  Stream output;
  fakeTimedToneCount = 0;  // Inspect only tones caused by waitButton().

  assert(!robot.waitButton(output, 10000));
  assert(robot.buzzerActive());
  assert(fakeTimedToneCount == 0);  // Waiting must not cancel the melody.

  for (unsigned long at = 1000; robot.buzzerActive(); at += 1000) {
    fakeMillis = at;
    assert(!robot.waitButton(output, 10000));
  }
  assert(!robot.buzzerActive());  // The last melody note has completed.
  assert(fakeTimedToneCount == 0);

  assert(!robot.waitButton(output, 10000));
  assert(fakeTimedToneCount == 0);
}

void testSimpleSensorAndAdcAliases() {
  resetWire();
  MyMiniPro robot = makeRobot();

  assert(robot.readSensorFront(0) == 0);
  assert(robot.readSensorRear(15) == 0);
  assert(robot.readSensorFront(MyMiniPro::kSensorCount) == 0);
  assert(robot.minSensorFront(0) == 0);
  assert(robot.maxSensorFront(0) == 0);
  assert(robot.minSensorRear(0) == 0);
  assert(robot.maxSensorRear(0) == 0);
  assert(robot.minAdcL() == INT16_MIN);
  assert(robot.maxAdcL() == INT16_MIN);
  assert(robot.minAdcR() == INT16_MIN);
  assert(robot.maxAdcR() == INT16_MIN);
  assert(!robot.waitButton());
}
}  // namespace

TwoWire Wire;
Stream Serial;

unsigned long millis() {
  fakeMillis += millisStep;
  return fakeMillis;
}
unsigned long micros() { return ++fakeMicros; }
void delay(unsigned long) {}
void delayMicroseconds(unsigned int) {}
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return fakeDigitalRead; }
int analogRead(uint8_t) { return 0; }
void analogReadResolution(int) {}
void analogWrite(uint8_t, int) {}
void analogWriteFreq(uint32_t) {}
void analogWriteRange(uint32_t) {}
void tone(uint8_t, unsigned int) { fakeToneActive = true; }
void tone(uint8_t, unsigned int, unsigned long) {
  fakeToneActive = true;
  ++fakeTimedToneCount;
}
void noTone(uint8_t) { fakeToneActive = false; }

int main() {
  testUnderbodyAndAuxiliaryChannelsSelectCorrectInputAndPreserveSign();
  testWrapperPropagatesI2cFailure();
  testWrapperReturnsErrorWhenConversionNeverBecomesReady();
  testUnderbodyCalibrationNormalizationAndPersistence();
  testLegacyFrontRearCalibrationRemainsReadable();
  testWaitButtonShortPressStartsWithoutUnderbodyCalibration();
  testWaitButtonLongPressCalibratesThenRequiresNewShortPress();
  testWaitButtonInvalidLongPressDoesNotAutoStart();
  testLiveReadingLineShowsOnlyNormalizedUnderbodyValues();
  testPico2ServoGpioMappingAndAngleBounds();
  testStopBuzzerCancelsPico2StartupMelody();
  testPico2BeginCanSuppressStartupMelody();
  testPico2DefaultBeginAcknowledgesThenStartsShortMelody();
  testWaitButtonLeavesStartupMelodyIntact();
  testSimpleSensorAndAdcAliases();
  return 0;
}
