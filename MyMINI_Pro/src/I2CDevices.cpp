#include "I2CDevices.h"

using namespace MyMINIConfig;

namespace {
constexpr uint16_t EEPROM_MAX_SAFE_ADDRESS = 0x3FFE;
}

void I2CDevices::begin() {
#if defined(ARDUINO_ARCH_RP2040)
  wire_.setSDA(PIN_I2C_SDA);
  wire_.setSCL(PIN_I2C_SCL);
#endif
  wire_.begin();
  wire_.setClock(I2C_CLOCK_HZ);
}

bool I2CDevices::probe(uint8_t address) {
  wire_.beginTransmission(address);
  return wire_.endTransmission() == 0;
}

void I2CDevices::scanBus(Print& output) {
  uint8_t found = 0;
  output.println(F("I2C scan:"));
  for (uint8_t address = 0x08; address <= 0x77; ++address) {
    if (probe(address)) {
      output.print(F("  0x"));
      if (address < 0x10) output.print('0');
      output.println(address, HEX);
      ++found;
    }
  }
  output.print(F("devices="));
  output.println(found);
}

void I2CDevices::printExpectedDevices(Print& output) {
  const uint8_t addresses[] = {
      PCF8574_ADDRESS, ADS1115_ADDRESS, CAT24C128_ADDRESS, MCP3421_ADDRESS};
  const __FlashStringHelper* names[] = {
      F("PCF8574"), F("ADS1115"), F("CAT24C128"), F("MCP3421")};

  for (uint8_t i = 0; i < 4; ++i) {
    output.print(names[i]);
    output.print(F(" 0x"));
    if (addresses[i] < 0x10) output.print('0');
    output.print(addresses[i], HEX);
    output.println(probe(addresses[i]) ? F(" OK") : F(" MISSING"));
  }
}

bool I2CDevices::readAds1115SingleEnded(uint8_t channel, int16_t& raw) {
  if (channel > 3) return false;

  // Single-shot, AINx-GND, +/-4.096 V, 860 SPS, comparator disabled.
  const uint16_t config = static_cast<uint16_t>(
      0x8000U | ((0x04U + channel) << 12U) | 0x0200U | 0x0100U |
      0x00E0U | 0x0003U);

  wire_.beginTransmission(ADS1115_ADDRESS);
  wire_.write(static_cast<uint8_t>(0x01));
  wire_.write(static_cast<uint8_t>(config >> 8));
  wire_.write(static_cast<uint8_t>(config & 0xFF));
  if (wire_.endTransmission() != 0) return false;

  const uint32_t started = millis();
  uint16_t currentConfig = 0;
  do {
    if (!readAdsRegister_(0x01, currentConfig)) return false;
    if ((currentConfig & 0x8000U) != 0) break;
    delayMicroseconds(200);
  } while (static_cast<uint32_t>(millis() - started) < 10U);

  if ((currentConfig & 0x8000U) == 0) return false;

  uint16_t conversion = 0;
  if (!readAdsRegister_(0x00, conversion)) return false;
  raw = static_cast<int16_t>(conversion);
  return true;
}

float I2CDevices::adsRawToVolts(int16_t raw) const {
  return static_cast<float>(raw) * ADS1115_FULL_SCALE_VOLTS / 32768.0f;
}

float I2CDevices::batteryVoltsFromRaw(int16_t raw) const {
  return adsRawToVolts(raw) * BATTERY_DIVIDER_MULTIPLIER;
}

bool I2CDevices::setAdsRegisterPointer_(uint8_t registerAddress) {
  wire_.beginTransmission(ADS1115_ADDRESS);
  wire_.write(registerAddress);
  return wire_.endTransmission(false) == 0;
}

bool I2CDevices::readAdsRegister_(uint8_t registerAddress, uint16_t& value) {
  if (!setAdsRegisterPointer_(registerAddress)) return false;
  if (wire_.requestFrom(ADS1115_ADDRESS, static_cast<uint8_t>(2)) != 2) {
    return false;
  }
  value = static_cast<uint16_t>(wire_.read()) << 8;
  value |= static_cast<uint8_t>(wire_.read());
  return true;
}

bool I2CDevices::readMcp3421Raw(int16_t& raw, uint8_t& configByte) {
  // Continuous conversion, 12-bit/240 SPS, gain x1.
  wire_.beginTransmission(MCP3421_ADDRESS);
  wire_.write(static_cast<uint8_t>(0x10));
  if (wire_.endTransmission() != 0) return false;

  const uint32_t started = millis();
  do {
    delayMicroseconds(500);
    if (wire_.requestFrom(MCP3421_ADDRESS, static_cast<uint8_t>(3)) != 3) {
      return false;
    }
    const uint16_t data = static_cast<uint16_t>(wire_.read()) << 8 |
                          static_cast<uint8_t>(wire_.read());
    configByte = static_cast<uint8_t>(wire_.read());
    if ((configByte & 0x80U) == 0) {
      raw = static_cast<int16_t>(data);
      return true;
    }
  } while (static_cast<uint32_t>(millis() - started) < 20U);
  return false;
}

float I2CDevices::mcp3421RawToVolts(int16_t raw) const {
  // At 12-bit and PGA x1: 1 mV per LSB.
  return static_cast<float>(raw) * 0.001f;
}

bool I2CDevices::writePcf8574(uint8_t value) {
  wire_.beginTransmission(PCF8574_ADDRESS);
  wire_.write(value);
  return wire_.endTransmission() == 0;
}

bool I2CDevices::readEepromByte(uint16_t address, uint8_t& value) {
  wire_.beginTransmission(CAT24C128_ADDRESS);
  wire_.write(static_cast<uint8_t>(address >> 8));
  wire_.write(static_cast<uint8_t>(address & 0xFF));
  if (wire_.endTransmission(false) != 0) return false;
  if (wire_.requestFrom(CAT24C128_ADDRESS, static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  value = static_cast<uint8_t>(wire_.read());
  return true;
}

bool I2CDevices::writeEepromByte(uint16_t address, uint8_t value) {
  wire_.beginTransmission(CAT24C128_ADDRESS);
  wire_.write(static_cast<uint8_t>(address >> 8));
  wire_.write(static_cast<uint8_t>(address & 0xFF));
  wire_.write(value);
  if (wire_.endTransmission() != 0) return false;
  return waitForEepromReady_(20);
}

bool I2CDevices::readEepromBlock(uint16_t address, uint8_t* buffer,
                                  uint16_t length) {
  if (buffer == nullptr) return false;
  if (length == 0) return true;

  const uint32_t lastAddress = static_cast<uint32_t>(address) + length - 1U;
  if (lastAddress > EEPROM_MAX_SAFE_ADDRESS) {
    return false;
  }

  uint16_t offset = 0;
  while (offset < length) {
    const uint16_t remaining = length - offset;
    const uint8_t chunk = static_cast<uint8_t>(remaining > 32U ? 32U : remaining);
    const uint16_t currentAddress = address + offset;

    wire_.beginTransmission(CAT24C128_ADDRESS);
    wire_.write(static_cast<uint8_t>(currentAddress >> 8));
    wire_.write(static_cast<uint8_t>(currentAddress & 0xFF));
    if (wire_.endTransmission(false) != 0) return false;
    if (wire_.requestFrom(CAT24C128_ADDRESS, chunk) != chunk) return false;

    for (uint8_t i = 0; i < chunk; ++i) {
      buffer[offset + i] = static_cast<uint8_t>(wire_.read());
    }
    offset += chunk;
  }
  return true;
}

bool I2CDevices::writeEepromBlock(uint16_t address, const uint8_t* buffer,
                                   uint16_t length) {
  if (buffer == nullptr) return false;
  if (length == 0) return true;

  const uint32_t lastAddress = static_cast<uint32_t>(address) + length - 1U;
  if (lastAddress > EEPROM_MAX_SAFE_ADDRESS) {
    return false;
  }

  uint16_t offset = 0;
  while (offset < length) {
    const uint16_t currentAddress = address + offset;
    const uint8_t pageRemaining = static_cast<uint8_t>(
        64U - (currentAddress & 0x003FU));
    const uint16_t remaining = length - offset;
    const uint8_t chunk = static_cast<uint8_t>(
        remaining < pageRemaining ? remaining : pageRemaining);

    wire_.beginTransmission(CAT24C128_ADDRESS);
    wire_.write(static_cast<uint8_t>(currentAddress >> 8));
    wire_.write(static_cast<uint8_t>(currentAddress & 0xFF));
    if (wire_.write(buffer + offset, chunk) != chunk) return false;
    if (wire_.endTransmission() != 0) return false;
    if (!waitForEepromReady_(20)) return false;

    offset += chunk;
  }
  return true;
}

bool I2CDevices::waitForEepromReady_(uint16_t timeoutMs) {
  const uint32_t started = millis();
  do {
    if (probe(CAT24C128_ADDRESS)) return true;
  } while (static_cast<uint32_t>(millis() - started) < timeoutMs);
  return false;
}

bool I2CDevices::runPreservingEepromTest(Print& output) {
  uint8_t original = 0;
  if (!readEepromByte(EEPROM_TEST_ADDRESS, original)) {
    output.println(F("EEPROM read failed"));
    return false;
  }

  const uint8_t pattern = static_cast<uint8_t>(original ^ 0xA5U);
  if (!writeEepromByte(EEPROM_TEST_ADDRESS, pattern)) {
    output.println(F("EEPROM test write failed"));
    return false;
  }

  uint8_t verified = 0;
  const bool patternOk = readEepromByte(EEPROM_TEST_ADDRESS, verified) &&
                         verified == pattern;
  const bool restored = writeEepromByte(EEPROM_TEST_ADDRESS, original);
  uint8_t restoredValue = 0;
  const bool restoreVerified = restored &&
      readEepromByte(EEPROM_TEST_ADDRESS, restoredValue) &&
      restoredValue == original;

  output.print(F("EEPROM pattern="));
  output.print(patternOk ? F("OK") : F("FAIL"));
  output.print(F(" restore="));
  output.println(restoreVerified ? F("OK") : F("FAIL"));
  return patternOk && restoreVerified;
}
