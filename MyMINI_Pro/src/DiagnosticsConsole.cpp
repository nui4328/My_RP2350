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
