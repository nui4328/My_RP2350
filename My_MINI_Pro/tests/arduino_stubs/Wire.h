#pragma once

#include <Arduino.h>

#include <array>
#include <deque>
#include <vector>

class TwoWire {
 public:
  struct Transaction {
    uint8_t address;
    std::vector<uint8_t> bytes;
    bool sendStop;
  };

  TwoWire() { reset(); }

  void begin() {}
  void setClock(uint32_t clock) { clockHz = clock; }
  void setSDA(uint8_t pin) { sda = pin; }
  void setSCL(uint8_t pin) { scl = pin; }

  void beginTransmission(uint8_t address) {
    activeAddress = address;
    activeBytes.clear();
  }
  size_t write(uint8_t value) {
    activeBytes.push_back(value);
    return 1;
  }
  uint8_t endTransmission(bool sendStop = true) {
    transactions.push_back({activeAddress, activeBytes, sendStop});
    if (!activeBytes.empty()) selectedRegister = activeBytes.front();
    if (endTransmissionResult != 0) return endTransmissionResult;
    if (activeAddress == kEepromAddress) {
      if (activeBytes.size() == 2) {
        eepromPointer = static_cast<uint16_t>((activeBytes[0] << 8) | activeBytes[1]);
      } else if (activeBytes.size() == 3) {
        const uint16_t address =
            static_cast<uint16_t>((activeBytes[0] << 8) | activeBytes[1]);
        if (address < eeprom.size()) eeprom[address] = activeBytes[2];
      }
    }
    return endTransmissionResult;
  }
  uint8_t requestFrom(uint8_t address, uint8_t quantity) {
    requestAddresses.push_back(address);
    requestRegisters.push_back(selectedRegister);
    activeRead.clear();
    if (address == kEepromAddress) {
      for (uint8_t index = 0; index < quantity; ++index) {
        activeRead.push_back(eepromPointer < eeprom.size() ? eeprom[eepromPointer] : 0xff);
        ++eepromPointer;
      }
      readIndex = 0;
      return quantity;
    }
    if (responses.empty()) return 0;
    activeRead = responses.front();
    responses.pop_front();
    readIndex = 0;
    return static_cast<uint8_t>(activeRead.size());
  }
  int read() {
    return readIndex < activeRead.size() ? activeRead[readIndex++] : -1;
  }

  void enqueueResponse(std::initializer_list<uint8_t> bytes) {
    responses.emplace_back(bytes);
  }
  void reset() {
    transactions.clear();
    requestAddresses.clear();
    requestRegisters.clear();
    responses.clear();
    activeBytes.clear();
    activeRead.clear();
    readIndex = 0;
    selectedRegister = 0;
    eepromPointer = 0;
    endTransmissionResult = 0;
    eeprom.fill(0xff);
  }
  void clearTransactions() {
    transactions.clear();
    requestAddresses.clear();
    requestRegisters.clear();
    responses.clear();
    activeBytes.clear();
    activeRead.clear();
    readIndex = 0;
    selectedRegister = 0;
    endTransmissionResult = 0;
  }
  void writeEeprom(uint16_t address, uint8_t value) {
    if (address < eeprom.size()) eeprom[address] = value;
  }
  uint8_t readEeprom(uint16_t address) const {
    return address < eeprom.size() ? eeprom[address] : 0xff;
  }

  uint8_t endTransmissionResult = 0;
  uint32_t clockHz = 0;
  uint8_t sda = 0;
  uint8_t scl = 0;
  std::vector<Transaction> transactions;
  std::vector<uint8_t> requestAddresses;
  std::vector<uint8_t> requestRegisters;

 private:
  static constexpr uint8_t kEepromAddress = 0x50;
  uint8_t activeAddress = 0;
  uint8_t selectedRegister = 0;
  std::vector<uint8_t> activeBytes;
  std::deque<std::vector<uint8_t>> responses;
  std::vector<uint8_t> activeRead;
  size_t readIndex = 0;
  uint16_t eepromPointer = 0;
  std::array<uint8_t, 512> eeprom{};
};

extern TwoWire Wire;
