#include <Wire.h>
#include "color_i2c_send.h"

extern uint8_t scannedColorOrder[6];
extern uint8_t scannedColorCount;

namespace {
constexpr uint8_t kColorDisplayAddress = 0x42;
constexpr uint8_t kColorPacketMarker = 0xC6;
constexpr uint8_t kOrderCommand = 1;
constexpr uint8_t kOffCommand = 2;
constexpr uint8_t kOnCommand = 3;
constexpr uint8_t kSingleCommand = 4;
uint8_t lastOrder[6] = {};
uint8_t lastSingle = 6;
uint8_t lastDisplayMode = 0;
bool displayEnabled = true;
int lastStatus = -1;

bool transmitPacket(const uint8_t* packet, size_t length) {
  Wire.beginTransmission(kColorDisplayAddress);
  Wire.write(packet, length);
  const uint8_t status = Wire.endTransmission();
  if (status != lastStatus) {
    Serial.print(F("COLOR I2C: display "));
    if (status == 0) Serial.println(F("ACK"));
    else {
      Serial.print(F("send error "));
      Serial.println(status);
    }
    lastStatus = status;
  }
  return status == 0;
}

bool transmitOrder(const uint8_t order[6]) {
  uint8_t packet[9] = {kColorPacketMarker, kOrderCommand};
  uint8_t checksum = 0xA5 ^ packet[0] ^ packet[1];
  for (uint8_t i = 0; i < 6; ++i) {
    packet[i + 2] = order[i];
    checksum ^= order[i];
  }
  packet[8] = checksum;
  return transmitPacket(packet, sizeof(packet));
}

bool transmitScreenCommand(uint8_t command) {
  const uint8_t packet[3] = {
      kColorPacketMarker, command,
      static_cast<uint8_t>(0xA5 ^ kColorPacketMarker ^ command)};
  return transmitPacket(packet, sizeof(packet));
}

bool transmitSingle(uint8_t color) {
  const uint8_t packet[4] = {
      kColorPacketMarker, kSingleCommand, color,
      static_cast<uint8_t>(0xA5 ^ kColorPacketMarker ^ kSingleCommand ^ color)};
  return transmitPacket(packet, sizeof(packet));
}

void resendLastDisplay() {
  if (lastDisplayMode == kOrderCommand) transmitOrder(lastOrder);
  else if (lastDisplayMode == kSingleCommand) transmitSingle(lastSingle);
}
}

// Wire is already the Pico 2 master on GP4/GP5 for the TCS and EEPROM.
// Packet: marker, command, six color IDs, XOR checksum. IDs match
// color_line_scan.ino: RED=0, GREEN=1, YELLOW=2, BLUE=3, WHITE=4, BLACK=5.
bool sendColorBands(BandColor first, BandColor second, BandColor third,
                    BandColor fourth, BandColor fifth, BandColor sixth) {
  const uint8_t order[6] = {first, second, third, fourth, fifth, sixth};
  uint8_t seen = 0;
  for (uint8_t i = 0; i < 6; ++i) {
    const uint8_t color = order[i];
    if (color > 5 || (seen & (1u << color))) return false;
    seen |= 1u << color;
  }
  if (seen != 0x3f) return false;
  for (uint8_t i = 0; i < 6; ++i) lastOrder[i] = order[i];
  lastDisplayMode = kOrderCommand;
  displayEnabled = true;
  return transmitOrder(lastOrder);
}

bool sendColorBands(const String& color) {
  static const char* const names[7] = {
      "RED", "GREEN", "YELLOW", "BLUE", "WHITE", "BLACK", "UNKNOWN"};
  for (uint8_t i = 0; i < 7; ++i) {
    if (!color.equalsIgnoreCase(names[i])) continue;
    lastSingle = i;
    lastDisplayMode = kSingleCommand;
    displayEnabled = true;
    return transmitSingle(i);
  }
  return false;
}

bool publishScannedColors() {
  if (scannedColorCount != 6) return false;
  return sendColorBands(
      static_cast<BandColor>(scannedColorOrder[0]),
      static_cast<BandColor>(scannedColorOrder[1]),
      static_cast<BandColor>(scannedColorOrder[2]),
      static_cast<BandColor>(scannedColorOrder[3]),
      static_cast<BandColor>(scannedColorOrder[4]),
      static_cast<BandColor>(scannedColorOrder[5]));
}

bool turnOffTft() {
  displayEnabled = false;
  return transmitScreenCommand(kOffCommand);
}

bool turnOnTft() {
  const bool acknowledged = transmitScreenCommand(kOnCommand);
  displayEnabled = true;
  resendLastDisplay();
  return acknowledged;
}

void resendColorBandsIfEnabled() {
  if (displayEnabled) resendLastDisplay();
  else if (!displayEnabled) transmitScreenCommand(kOffCommand);
}
