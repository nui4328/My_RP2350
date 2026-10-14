#include "DiagnosticsConsole.h"

#include <stdlib.h>
#include <string.h>

using namespace MyMINIConfig;

void DiagnosticsConsole::begin() {
  pinMode(PIN_FRONT_CAL, INPUT_PULLUP);
  i2c_.writePcf8574(0x00);
  serial_.println();
  serial_.println(F("MyMINI_Pro hardware diagnostics"));
  serial_.println(F("Motors are stopped. Type: help"));
}

void DiagnosticsConsole::update(uint32_t nowMs) {
  consumeSerial_();
  updateLedChase_(nowMs);

  if (motorTestActive_ &&
      static_cast<int32_t>(nowMs - motorStopAtMs_) >= 0) {
    stopMotorTest_(F("motor auto-stop"));
  }
}

void DiagnosticsConsole::consumeSerial_() {
  uint8_t budget = 32;
  while (serial_.available() > 0 && budget-- > 0) {
    const char incoming = static_cast<char>(serial_.read());
    if (incoming == '\r') continue;
    if (incoming == '\n') {
      if (lineLength_ > 0) {
        line_[lineLength_] = '\0';
        executeLine_();
        lineLength_ = 0;
      }
      continue;
    }

    if (lineLength_ < sizeof(line_) - 1U) {
      line_[lineLength_++] = incoming;
    } else {
      lineLength_ = 0;
      serial_.println(F("ERR line too long"));
    }
  }
}

void DiagnosticsConsole::executeLine_() {
  char* command = strtok(line_, " ");
  if (command == nullptr) return;

  if (strcmp(command, "help") == 0) {
    printHelp_();
  } else if (strcmp(command, "status") == 0) {
    printStatus_();
  } else if (strcmp(command, "i2c") == 0) {
    i2c_.scanBus(serial_);
  } else if (strcmp(command, "mux") == 0) {
    sensors_.printFrame(serial_);
  } else if (strcmp(command, "sensor_diag") == 0) {
    printSensorDiagnostics_();
  } else if (strcmp(command, "mux_settle") == 0) {
    char* valueToken = strtok(nullptr, " ");
    int16_t settleUs = 0;
    if (!parseInt16_(valueToken, settleUs) ||
        (settleUs != 4 && settleUs != 20 && settleUs != 50)) {
      serial_.println(F("Use: mux_settle 4|20|50"));
    } else {
      sensors_.setSettleMicros(static_cast<uint16_t>(settleUs));
      serial_.print(F("mux_settle_us="));
      serial_.println(sensors_.settleMicros());
    }
  } else if (strcmp(command, "ads") == 0) {
    printAds_();
  } else if (strcmp(command, "adccal") == 0) {
    printAdcCal_();
  } else if (strcmp(command, "frontcal") == 0) {
    serial_.print(F("front calibration input="));
    serial_.println(digitalRead(PIN_FRONT_CAL) == LOW ? F("ACTIVE LOW")
                                                       : F("inactive HIGH"));
  } else if (strcmp(command, "leds") == 0) {
    startLedChase_(millis());
  } else if (strcmp(command, "motor") == 0) {
    char* side = strtok(nullptr, " ");
    char* power = strtok(nullptr, " ");
    handleMotorCommand_(side, power, millis());
  } else if (strcmp(command, "stop") == 0) {
    stopMotorTest_(F("manual stop"));
  } else if (strcmp(command, "eeprom_test") == 0) {
    char* confirmation = strtok(nullptr, " ");
    if (confirmation != nullptr && strcmp(confirmation, "YES") == 0) {
      i2c_.runPreservingEepromTest(serial_);
    } else {
      serial_.println(F("Use exact command: eeprom_test YES"));
    }
  } else {
    serial_.println(F("ERR unknown command; type help"));
  }
}

void DiagnosticsConsole::printHelp_() {
  serial_.println(F("help                 - show commands"));
  serial_.println(F("status               - expected devices and safety inputs"));
  serial_.println(F("i2c                  - scan I2C addresses"));
  serial_.println(F("mux                  - print latest front/rear 16-channel frame"));
  serial_.println(F("sensor_diag          - raw/normalized by channel, 32-frame raw range"));
  serial_.println(F("mux_settle 4|20|50   - diagnostic MUX settling comparison, default 4 us"));
  serial_.println(F("ads                  - read ADS1115 AIN0..AIN3"));
  serial_.println(F("adccal               - read rear ADCcal from MCP3421"));
  serial_.println(F("frontcal             - read active-low GP3 input"));
  serial_.println(F("leds                 - non-blocking PCF8574 LED chase"));
  serial_.println(F("motor L|R [-30..30] - 1 second wheel-off-ground test (percent, default 25%)"));
  serial_.println(F("stop                  - stop both motors"));
  serial_.println(F("eeprom_test YES       - preserve/test/restore one EEPROM byte"));
}

void DiagnosticsConsole::printStatus_() {
  serial_.println(F("--- status ---"));
  i2c_.printExpectedDevices(serial_);
  serial_.print(F("mux frame="));
  serial_.println(sensors_.frameSequence());
  serial_.print(F("front_cal="));
  serial_.println(digitalRead(PIN_FRONT_CAL) == LOW ? F("ACTIVE") : F("inactive"));
  serial_.print(F("motor L/R (%)="));
  serial_.print(motors_.leftCommand());
  serial_.print('/');
  serial_.println(motors_.rightCommand());
}

void DiagnosticsConsole::printAds_() {
  static const __FlashStringHelper* labels[4] = {
      F("battery"), F("center_left"), F("center_right"), F("aux")};
  for (uint8_t channel = 0; channel < 4; ++channel) {
    int16_t raw = 0;
    serial_.print(F("AIN"));
    serial_.print(channel);
    serial_.print(' ');
    serial_.print(labels[channel]);
    serial_.print(F(" raw="));
    if (!i2c_.readAds1115SingleEnded(channel, raw)) {
      serial_.println(F("READ_FAIL"));
      continue;
    }
    serial_.print(raw);
    serial_.print(F(" volts="));
    serial_.print(i2c_.adsRawToVolts(raw), 4);
    if (channel == ADS_CHANNEL_BATTERY) {
      serial_.print(F(" battery="));
      serial_.print(i2c_.batteryVoltsFromRaw(raw), 3);
    }
    serial_.println();
  }
}

void DiagnosticsConsole::printAdcCal_() {
  int16_t raw = 0;
  uint8_t config = 0;
  if (!i2c_.readMcp3421Raw(raw, config)) {
    serial_.println(F("ADCcal READ_FAIL"));
    return;
  }
  serial_.print(F("ADCcal raw="));
  serial_.print(raw);
  serial_.print(F(" volts="));
  serial_.print(i2c_.mcp3421RawToVolts(raw), 3);
  serial_.print(F(" config=0x"));
  if (config < 0x10) serial_.print('0');
  serial_.print(config, HEX);
  serial_.println(F(" threshold=NOT_SET"));
}

void DiagnosticsConsole::printSensorDiagnostics_() {
  const CalibrationData& data = calibration_.data();
  uint16_t rawLow[2][SENSOR_COUNT];
  uint16_t rawHigh[2][SENSOR_COUNT];
  for (uint8_t row = 0; row < 2; ++row) {
    for (uint8_t channel = 0; channel < SENSOR_COUNT; ++channel) {
      rawLow[row][channel] = 0xFFFFu;
      rawHigh[row][channel] = 0;
    }
  }
  // Diagnostic-only burst: a complete frame keeps front and rear in the same
  // scan, and min/max exposes jitter without filtering production readings.
  for (uint8_t frame = 0; frame < 32; ++frame) {
    while (!sensors_.update(micros())) {}
    for (uint8_t row = 0; row < 2; ++row) {
      const uint16_t* values = row == 0 ? sensors_.frontRaw()
                                         : sensors_.rearRaw();
      for (uint8_t channel = 0; channel < SENSOR_COUNT; ++channel) {
        if (values[channel] < rawLow[row][channel])
          rawLow[row][channel] = values[channel];
        if (values[channel] > rawHigh[row][channel])
          rawHigh[row][channel] = values[channel];
      }
    }
  }
  serial_.print(F("sensor_diag frame="));
  serial_.println(sensors_.frameSequence());
  serial_.print(F("mux_settle_us="));
  serial_.println(sensors_.settleMicros());
  serial_.print(F("adc_discard="));
  serial_.print(sensors_.discardReads());
  serial_.print(F(" adc_average="));
  serial_.print(sensors_.averageReads());
  serial_.print(F(" smooth_divisor="));
  serial_.println(sensors_.smoothingDivisor());
  for (uint8_t row = 0; row < 2; ++row) {
    const bool valid = row == 0 ? calibration_.frontValid()
                                : calibration_.rearValid();
    const uint16_t* raw = row == 0 ? sensors_.frontRaw()
                                     : sensors_.rearRaw();
    const uint16_t* filtered = row == 0 ? sensors_.frontFiltered()
                                          : sensors_.rearFiltered();
    for (uint8_t channel = 0; channel < SENSOR_COUNT; ++channel) {
      const uint16_t minValue = row == 0 ? data.frontMin[channel]
                                          : data.rearMin[channel];
      const uint16_t maxValue = row == 0 ? data.frontMax[channel]
                                          : data.rearMax[channel];
      serial_.print(row == 0 ? 'F' : 'B');
      serial_.print(channel);
      serial_.print(F(" raw="));
      serial_.print(raw[channel]);
      serial_.print(F(" filtered="));
      serial_.print(filtered[channel]);
      serial_.print(F(" raw_range="));
      serial_.print(rawLow[row][channel]);
      serial_.print(F(".."));
      serial_.print(rawHigh[row][channel]);
      const bool channelCalValid = valid && maxValue > minValue;
      serial_.print(F(" min="));
      if (channelCalValid) serial_.print(minValue);
      else serial_.print(F("N/A"));
      serial_.print(F(" max="));
      if (channelCalValid) serial_.print(maxValue);
      else serial_.print(F("N/A"));
      serial_.print(F(" normalized="));
      if (!channelCalValid) {
        serial_.println(F("NA state=INVALID_CAL"));
        continue;
      }
      const int32_t mapped = static_cast<int32_t>(map(
          raw[channel], minValue, maxValue, 0L, 1000L));
      serial_.print(constrain(mapped, 0L, 1000L));
      serial_.print(F(" filtered_norm="));
      const int32_t filteredMapped = static_cast<int32_t>(map(
          filtered[channel], minValue, maxValue,
          0L, 1000L));
      serial_.print(constrain(filteredMapped, 0L, 1000L));
      serial_.print(F(" state="));
      serial_.println(static_cast<int32_t>(raw[channel]) * 2 <
                              static_cast<int32_t>(minValue) + maxValue
                          ? F("BLACK") : F("WHITE"));
    }
  }
}

void DiagnosticsConsole::startLedChase_(uint32_t nowMs) {
  ledIndex_ = 0;
  nextLedAtMs_ = nowMs;
  ledChaseActive_ = true;
  serial_.println(F("LED chase started"));
}

void DiagnosticsConsole::updateLedChase_(uint32_t nowMs) {
  if (!ledChaseActive_ || static_cast<int32_t>(nowMs - nextLedAtMs_) < 0) {
    return;
  }

  if (ledIndex_ >= 8) {
    i2c_.writePcf8574(0x00);
    ledChaseActive_ = false;
    serial_.println(F("LED chase complete"));
    return;
  }

  const uint8_t oneHot = static_cast<uint8_t>(1U << ledIndex_);
  if (!i2c_.writePcf8574(oneHot)) {
    ledChaseActive_ = false;
    serial_.println(F("LED chase I2C_FAIL"));
    return;
  }
  ++ledIndex_;
  nextLedAtMs_ = nowMs + 100;
}

void DiagnosticsConsole::handleMotorCommand_(char* sideToken, char* powerToken,
                                              uint32_t nowMs) {
  if (sideToken == nullptr ||
      (strcmp(sideToken, "L") != 0 && strcmp(sideToken, "R") != 0)) {
    serial_.println(F("Use: motor L|R [-30..30]"));
    return;
  }

  int16_t requested = 25;
  if (powerToken != nullptr && !parseInt16_(powerToken, requested)) {
    serial_.println(F("ERR invalid motor power"));
    return;
  }

  if (requested > static_cast<int16_t>(MOTOR_DIAGNOSTIC_LIMIT)) {
    requested = MOTOR_DIAGNOSTIC_LIMIT;
  } else if (requested < -static_cast<int16_t>(MOTOR_DIAGNOSTIC_LIMIT)) {
    requested = -static_cast<int16_t>(MOTOR_DIAGNOSTIC_LIMIT);
  }

  motors_.stopAll();
  motors_.set(strcmp(sideToken, "L") == 0 ? TB6612Driver::Motor::Left
                                           : TB6612Driver::Motor::Right,
              requested);
  motorTestActive_ = true;
  motorStopAtMs_ = nowMs + MOTOR_TEST_TIMEOUT_MS;
  serial_.print(F("motor test power="));
  serial_.print(requested);
  serial_.println(F("% auto-stop=1000ms"));
}

void DiagnosticsConsole::stopMotorTest_(const __FlashStringHelper* reason) {
  motors_.stopAll();
  motorTestActive_ = false;
  serial_.print(F("motors stopped: "));
  serial_.println(reason);
}

bool DiagnosticsConsole::parseInt16_(const char* text, int16_t& value) {
  if (text == nullptr || *text == '\0') return false;
  char* end = nullptr;
  const long parsed = strtol(text, &end, 10);
  if (*end != '\0' || parsed < -32768L || parsed > 32767L) return false;
  value = static_cast<int16_t>(parsed);
  return true;
}
