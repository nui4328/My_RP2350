#pragma once
#include <Arduino.h>

// Software I2C master, 7-bit addressing, open-drain emulation.
// No clock stretching support. Intended for a single master, short wires.
class SoftI2C {
public:
  SoftI2C(uint8_t sda, uint8_t scl, unsigned int halfPeriodUs = 5)
    : sda_(sda), scl_(scl), delayUs_(halfPeriodUs) {}

  void begin() {
    release(sda_);
    release(scl_);
  }

  bool start() {
    release(sda_); release(scl_); tick();
    if (!digitalRead(sda_) || !digitalRead(scl_)) return false;
    pullLow(sda_); tick(); pullLow(scl_); tick();
    return true;
  }

  void repeatedStart() {
    release(sda_); tick();
    release(scl_); tick();
    pullLow(sda_); tick();
    pullLow(scl_); tick();
  }

  void stop() {
    pullLow(sda_); tick();
    release(scl_); tick();
    release(sda_); tick();
  }

  // Returns true when slave acknowledges the byte.
  bool writeByte(uint8_t value) {
    for (uint8_t i = 0; i < 8; ++i) {
      if (value & 0x80) release(sda_); else pullLow(sda_);
      tick(); release(scl_); tick(); pullLow(scl_); tick();
      value <<= 1;
    }
    release(sda_); tick(); release(scl_); tick();
    bool ack = digitalRead(sda_) == LOW;
    pullLow(scl_); tick();
    return ack;
  }

  uint8_t readByte(bool ack) {
    uint8_t value = 0;
    release(sda_);
    for (uint8_t i = 0; i < 8; ++i) {
      tick(); release(scl_); tick();
      value = uint8_t((value << 1) | (digitalRead(sda_) ? 1 : 0));
      pullLow(scl_); tick();
    }
    if (ack) pullLow(sda_); else release(sda_);
    tick(); release(scl_); tick(); pullLow(scl_); tick();
    release(sda_);
    return value;
  }

  bool probe(uint8_t address7) {
    if (!start()) return false;
    bool found = writeByte(uint8_t(address7 << 1));
    stop();
    return found;
  }

  bool writeRegister(uint8_t address7, uint8_t reg, uint8_t value) {
    if (!start()) return false;
    bool ok = writeByte(uint8_t(address7 << 1));
    if (ok) ok = writeByte(reg);
    if (ok) ok = writeByte(value);
    stop();
    return ok;
  }

  bool readRegisters(uint8_t address7, uint8_t reg, uint8_t* out, size_t count) {
    if (!count || !out || !start()) return false;
    bool ok = writeByte(uint8_t(address7 << 1));
    if (ok) ok = writeByte(reg);
    if (ok) {
      repeatedStart();
      ok = writeByte(uint8_t((address7 << 1) | 1));
    }
    if (ok) {
      for (size_t i = 0; i < count; ++i) out[i] = readByte(i + 1 < count);
    }
    stop();
    return ok;
  }

private:
  uint8_t sda_, scl_;
  unsigned int delayUs_;
  void tick() const { delayMicroseconds(delayUs_); }
  static void pullLow(uint8_t pin) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  static void release(uint8_t pin) {
    pinMode(pin, INPUT); // external 3.3V pull-up required
  }
};
